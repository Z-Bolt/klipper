# Z-Bolt toolchanger: dual-head tool changing with real-time dock sensing
#
# Replaces the macro based toolchanger logic that used to live in
# tools.cfg / homing.cfg / parking.cfg.  This module owns:
#   - dock (and optional slider) sensors, sampled in real time
#   - the tool state machine and the fault matrix
#   - park / pickup motion, coupling, wiping and driver current boost
#   - homing recovery
#   - tool offsets and second-tool Z offset calibration
#
# Why this exists: a jinja macro renders printer['gcode_button ...'].state at
# macro expansion time, which runs ahead of the machine by the whole move
# queue (buffer_time_high, 2s by default).  The old macros compensated with a
# blind "G4 P2000" that sometimes lost the race and drove a head into a dock.
# Here every decision samples the sensors after toolhead.wait_moves(), so the
# host is synchronised with physical motion, and every edge is also watched
# from the reactor so an unexpected transition aborts the sequence at once.
#
# Install: copy or symlink this file and zbolt_toolchanger_tool.py into
# ~/klipper/klippy/extras/ then FIRMWARE_RESTART.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# config.getchoice() indexes its choices, so every choice set is a dict.
REACTIONS = {r: r for r in
             ['ignore', 'warn', 'pause', 'error', 'shutdown']}
COUPLINGS = {c: c for c in ['solenoid', 'latch']}
SPEED_CHOICES = {s: s for s in ['slow', 'fast']}
BOOST_PHASES = {p: p for p in ['park', 'get', 'both']}

# Fault classes and their default reactions.  See zbolt_toolchanger.md for
# the full matrix and the recovery procedure for each class.
FAULT_DEFAULTS = {
    'both_undocked': 'pause',
    'head_lost': 'pause',
    'slider_unexpected': 'pause',
    'park_failed': 'pause',
    'get_failed': 'pause',
    'unexpected_edge': 'pause',
    'sensor_unstable': 'error',
    'startup': 'error',
    'change_while_unhomed': 'ignore',
    'wrong_tool_active': 'warn',
}

FAULT_MESSAGES = {
    'both_undocked': "обе головы вне парковок",
    'head_lost': "голова не в парковке и не на слайдере",
    'slider_unexpected': "обе головы в парковках, но слайдер занята",
    'park_failed': "голова не встала в парковку",
    'get_failed': "голова не снялась с парковки",
    'unexpected_edge': "датчик сработал вне смены инструмента",
    'sensor_unstable': "датчик не выдаёт стабильное состояние",
    'startup': "состояние при старте не распознано",
    'change_while_unhomed': "смена инструмента без хоминга",
    'wrong_tool_active': "активный экструдер не совпадает с датчиками",
}

# Motion phases.  Sensor edges are only legal while a change is running.
PHASE_IDLE = 'idle'
PHASE_PARK = 'park'
PHASE_PICKUP = 'pickup'
PHASE_HOMING = 'homing'

# Distinct from None (slider empty) and from any tool number.
INVALID = object()


class Sensor:
    """A dock or slider switch with host side edge tracking.

    State is refreshed from an MCU callback running in the reactor, i.e.
    within milliseconds of the physical edge and independent of the g-code
    queue.  value() returns the normalised meaning (docked / occupied), not
    the raw electrical level.
    """

    def __init__(self, toolchanger, config, pin, name):
        self.toolchanger = toolchanger
        self.printer = config.get_printer()
        self.name = name
        # The MCU only reports a button when its state CHANGES from the
        # initial zero, so a switch that reads low at boot never sends
        # anything.  Stock gcode_button resolves this the same way: assume
        # not-triggered until told otherwise.  Waiting for a report instead
        # would hang forever on whichever head happens to sit in its dock.
        self.raw = False
        self.reported = False
        self.last_change = 0.
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons([pin], self._handle_edge)

    def _handle_edge(self, eventtime, state):
        # The initial level report (if any) lands before startup detection
        # finishes, and handle_sensor_edge() ignores everything until then.
        self.reported = True
        if state == self.raw:
            return
        self.raw = state
        self.last_change = eventtime
        self.toolchanger.handle_sensor_edge(self, eventtime)

    def value(self):
        return bool(self.raw)

    def raw_text(self):
        return "сработал" if self.raw else "не сработал"


class ToolchangerTool:
    """One tool: its extruder, dock sensor, dock geometry and offsets."""

    def __init__(self, toolchanger, config):
        self.toolchanger = toolchanger
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.number = config.getint('tool_number', minval=0)
        self.extruder_name = config.get('extruder')
        # Dock geometry.  Names map onto the old save_variables:
        #   park_xy   -> t<n>x_park / t<n>y_park
        #   unpark_xy -> t<n>x_unpark / t<n>y_unpark (defaults to park_xy)
        #   safe_xy   -> t<n>x_safe / t<n>y_safe
        self.park_xy = config.getfloatlist('park_xy', count=2)
        self.unpark_xy = config.getfloatlist('unpark_xy', self.park_xy,
                                             count=2)
        self.safe_xy = config.getfloatlist('safe_xy', count=2)
        self.stage_xy = config.getfloatlist('stage_xy', None, count=2)
        self.exit_xy = config.getfloatlist('exit_xy', None, count=2)
        self.approach_offset = config.getfloat('approach_offset', 0.)
        self.retreat_speed = config.getchoice('retreat_speed', SPEED_CHOICES,
                                              'slow')
        self.wipe_moves = self._parse_wipe(config)
        # Coupling
        self.coupling = config.getchoice('coupling', COUPLINGS, 'latch')
        self.solenoid_pin = config.get('solenoid_pin', None)
        self.release_delay = config.getfloat('release_delay', 0., minval=0.)
        self.release_jerk = config.getfloat('release_jerk', 0.)
        self.release_jerk_speed = config.getfloat('release_jerk_speed', 5.,
                                                  above=0.)
        self.seat_offset_xy = config.getfloatlist('seat_offset_xy', None,
                                                  count=2)
        self.seat_jerk = config.getfloat('seat_jerk', 0.)
        self.seat_return_speed = config.getfloat('seat_return_speed', 50.,
                                                 above=0.)
        # Driver current boost during the slow dock moves
        self.current_boost = config.getfloat('current_boost', 0., minval=0.)
        self.current_boost_phase = config.getchoice('current_boost_phase',
                                                    BOOST_PHASES, 'get')
        # Offsets live in save_variables so they stay editable without a
        # restart; an empty option means "this axis has no offset".
        self.offset_vars = {
            'x': config.get('offset_x_variable', None),
            'y': config.get('offset_y_variable', None),
            'z': config.get('offset_z_variable', None),
        }
        # Optional full-trajectory overrides
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.park_template = None
        self.get_template = None
        if config.get('park_gcode', None) is not None:
            self.park_template = gcode_macro.load_template(config,
                                                           'park_gcode')
        if config.get('get_gcode', None) is not None:
            self.get_template = gcode_macro.load_template(config, 'get_gcode')
        # Dock sensor
        # Triggered = the head sits in this dock; add "!" to the pin to
        # match the wiring, exactly like any other Klipper endstop.
        self.sensor = Sensor(toolchanger, config,
                             config.get('dock_sensor_pin'),
                             "датчик парковки T%d" % (self.number,))

    def _parse_wipe(self, config):
        raw = config.get('wipe_moves', '')
        moves = []
        for line in raw.split('\n'):
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 2:
                raise config.error(
                    "wipe_moves in %s must list 'dx, dy' pairs"
                    % (config.get_name(),))
            try:
                moves.append((float(parts[0]), float(parts[1])))
            except ValueError:
                raise config.error("Invalid wipe_moves entry '%s' in %s"
                                   % (line, config.get_name()))
        return moves

    def is_docked(self):
        return self.sensor.value()

    def get_offsets(self):
        """Tool offsets, read live from save_variables on every change."""
        variables = self.toolchanger.get_saved_variables()
        offsets = {}
        for axis, var in self.offset_vars.items():
            value = 0.
            if var:
                try:
                    value = float(variables.get(var, 0.))
                except (TypeError, ValueError):
                    value = 0.
            offsets[axis] = value
        return offsets

    def get_status(self, eventtime=None):
        return {
            'name': self.name,
            'tool_number': self.number,
            'extruder': self.extruder_name,
            'docked': self.is_docked(),
            'sensor_raw': self.sensor.raw_text(),
            'park_x': self.park_xy[0], 'park_y': self.park_xy[1],
            'unpark_x': self.unpark_xy[0], 'unpark_y': self.unpark_xy[1],
            'safe_x': self.safe_xy[0], 'safe_y': self.safe_xy[1],
        }


class ZBoltToolchanger:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.tools = {}
        self.tool_list = []
        self.boost_base = {}
        # sensor_settle_time replaces the old blind "G4 P2000": after
        # wait_moves() the host matches the machine, so a short stability
        # window is all a mechanical switch needs.
        self.settle_time = config.getfloat('sensor_settle_time', 0.05,
                                           minval=0., maxval=2.)
        self.sensor_timeout = config.getfloat('sensor_timeout', 3., above=0.)
        self.verify_retries = config.getint('verify_retries', 1, minval=0)
        self.startup_grace = config.getfloat('startup_grace', 3., minval=0.)
        # How long an anomalous sensor reading must persist before it counts
        # as a fault (filters switch bounce from machine vibration).
        self.guard_confirm_time = config.getfloat('guard_confirm_time', 0.25,
                                                  minval=0.)
        # Speeds are mm/s, Klipper style.  The old macros used F values in
        # mm/min: divide by 60 when migrating (F18000 -> 300).
        self.travel_speed = config.getfloat('travel_speed', 300., above=0.)
        self.dock_speed = config.getfloat('dock_speed', 60., above=0.)
        self.force_speed_factor = config.getfloat('force_speed_factor', 0.,
                                                  minval=0.)
        # Homing
        # Lift before homing is relative (old macros used G91 + G1 Z5);
        # the height after homing Z is absolute (old macros used G90 + G1 Z5).
        self.z_lift_before_home = config.getfloat('z_lift_before_home', 5.)
        self.z_after_home = config.getfloat('z_after_home', 5.)
        self.z_lift_speed = config.getfloat('z_lift_speed', 15., above=0.)
        self.home_axes_order = [a.lower() for a in
                                config.getlist('home_axes_order', ('y', 'x'))]
        for axis in self.home_axes_order:
            if axis not in ('x', 'y'):
                raise config.error("home_axes_order must list only x and y")
        self.home_z_position = config.getfloatlist('home_z_position', None,
                                                   count=2)
        self.after_home_position = config.getfloatlist('after_home_position',
                                                       None, count=2)
        self.initial_tool = config.getint('initial_tool', 0, minval=0)
        # Calibration defaults for TOOLCHANGER_CALIBRATE_OFFSET
        self.calibrate_temp = config.getfloat('calibrate_temp', 160.,
                                              minval=0.)
        self.calibrate_stabilize = config.getfloat('calibrate_stabilize_time',
                                                   60., minval=0.)
        self.calibrate_samples = config.getint('calibrate_samples', 3,
                                               minval=1)
        self.calibrate_lift = config.getfloat('calibrate_lift', 2., above=0.)
        # Optional slider occupancy sensor (planned hardware).  Without it
        # occupancy is inferred from the dock sensors.
        self.slider_sensor = None
        slider_pin = config.get('slider_sensor_pin', None)
        if slider_pin:
            # Triggered = the slider carries a head.
            self.slider_sensor = Sensor(self, config, slider_pin,
                                        "датчик слайдера")
        # Offsets are applied through a template so the SET_GCODE_OFFSET
        # wrapper in bed.cfg (which persists babysteps into offset_z) is not
        # triggered by our internal calls.
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.apply_offset_template = self.gcode_macro.load_template(
            config, 'apply_offset_gcode',
            '_SET_GCODE_OFFSET_INTERNAL X={x} Y={y} Z={z}')
        self.global_offset_variable = config.get('global_offset_variable',
                                                 'z_adjust')
        # Service mode is for setting a machine up on the bench: skew
        # compensation stays off, faults only warn and homing never grabs a
        # tool, so an assembler can pull heads off the docks by hand.  It is
        # persisted so a FIRMWARE_RESTART mid-setup does not silently arm the
        # machine again.
        self.service_mode_variable = config.get('service_mode_variable',
                                                'service_mode')
        # Hooks that wrapped the old T0/T1 macros on some machines
        # (for example UNSKEW ... SKEW around a tool change).
        self.before_change_template = None
        self.after_change_template = None
        if config.get('before_change_gcode', None) is not None:
            self.before_change_template = self.gcode_macro.load_template(
                config, 'before_change_gcode')
        if config.get('after_change_gcode', None) is not None:
            self.after_change_template = self.gcode_macro.load_template(
                config, 'after_change_gcode')
        # Fault reactions
        self.reactions = {}
        for code, default in FAULT_DEFAULTS.items():
            self.reactions[code] = config.getchoice('on_fault_%s' % (code,),
                                                    REACTIONS, default)
        # Runtime state
        self.phase = PHASE_IDLE
        self.active_tool = None
        self.state = 'unknown'
        self.fault = None
        self.abort_requested = None
        self.startup_reported = False
        self.anomaly_pending = False

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self._register_commands()

    # ------------------------------------------------------------------
    # Configuration wiring
    # ------------------------------------------------------------------

    def add_tool(self, config):
        tool = ToolchangerTool(self, config)
        if tool.number in self.tools:
            raise config.error("Duplicate tool_number %d in %s"
                               % (tool.number, config.get_name()))
        self.tools[tool.number] = tool
        self.tool_list = [self.tools[n] for n in sorted(self.tools)]
        # T<n> is registered here, so no [gcode_macro T0] may remain.
        self.gcode.register_command(
            'T%d' % (tool.number,), self._make_tool_command(tool.number),
            desc="Выбрать инструмент T%d" % (tool.number,))
        return tool

    def _make_tool_command(self, number):
        def cmd(gcmd):
            self.select_tool(number, gcmd)
        return cmd

    def _register_commands(self):
        for name in ('SELECT', 'PARK', 'PICKUP', 'HOME', 'STATUS',
                     'QUERY_SENSORS', 'SET_STATE', 'RELEASE', 'COUPLE',
                     'CALIBRATE_OFFSET', 'SERVICE_MODE', 'GOTO', 'SET_DOCK',
                     'SAVE_ADJUST'):
            command = 'TOOLCHANGER_%s' % (name,)
            handler = getattr(self, 'cmd_%s' % (command,))
            helptext = getattr(self, 'cmd_%s_help' % (command,))
            self.gcode.register_command(command, handler, desc=helptext)

    def _handle_ready(self):
        if not self.tools:
            raise self.printer.config_error(
                "zbolt_toolchanger has no zbolt_toolchanger_tool sections")
        # Derive the real state from the sensors instead of trusting a
        # default or whatever was saved before the restart.
        self.reactor.register_callback(self._startup_detect,
                                       self.reactor.monotonic() + 0.5)

    def _startup_detect(self, eventtime):
        # Runs in the reactor, so nothing here may raise.
        try:
            self._await_initial_reports()
            self._sample_sensors(wait_moves=False, async_fault=True)
            tool = self._derive_state()
            self.startup_reported = True
            if self.state == 'fault':
                self._react_async(self.fault, self._describe_sensors())
                self._respond(
                    "Хоминг заблокирован. Приведите головы в исходное "
                    "положение, либо задайте состояние вручную командой "
                    "TOOLCHANGER_SET_STATE")
            else:
                if tool is not None:
                    self._activate_extruder(tool)
                    # Restore the full offset of whatever is on the slider,
                    # not just the shared z_adjust: after a restart the tool
                    # offset would otherwise be missing until the next
                    # change.
                    self._apply_tool_offsets(tool)
                else:
                    self._apply_offset(0., 0., self._get_global_offset())
                self._respond(self._describe_state())
        except Exception:
            logging.exception("zbolt_toolchanger: startup detection failed")
        return self.reactor.NEVER

    # ------------------------------------------------------------------
    # Sensors and state
    # ------------------------------------------------------------------

    def _await_initial_reports(self):
        """Give the MCU time to report the switches that read high.

        A pin that reads low never generates a report, so this cannot wait
        for everything - it waits until nothing new has arrived for a while,
        bounded by startup_grace.
        """
        sensors = [t.sensor for t in self.tool_list]
        if self.slider_sensor is not None:
            sensors.append(self.slider_sensor)
        deadline = self.reactor.monotonic() + self.startup_grace
        while self.reactor.monotonic() < deadline:
            if all(s.reported for s in sensors):
                return
            self.reactor.pause(self.reactor.monotonic() + 0.05)

    def _observed_tool(self):
        """What the sensors say, without touching state.

        Returns the tool number on the slider, None if the slider is empty,
        or INVALID if the reading is not a valid resting state.
        """
        undocked = [t for t in self.tool_list if t.is_docked() is False]
        occupied = self._slider_occupied()
        if len(undocked) > 1:
            return INVALID
        if not undocked:
            return INVALID if occupied is True else None
        if occupied is False:
            return INVALID
        return undocked[0].number

    def handle_sensor_edge(self, sensor, eventtime):
        """Reactor callback: runs within ms of the physical edge."""
        if self.phase != PHASE_IDLE or not self.startup_reported:
            # Edges are expected while a change runs; the motion code
            # verifies them at its own checkpoints.
            return
        if self._resolve_edge():
            return
        # Anomalous.  A switch can bounce from machine vibration, and a
        # spurious pause mid-print is its own kind of failure, so require
        # the anomaly to persist before crying wolf.  A head that actually
        # came off never comes back on its own.
        if self.anomaly_pending:
            return
        self.anomaly_pending = True
        self.reactor.register_callback(
            self._confirm_anomaly,
            self.reactor.monotonic() + self.guard_confirm_time)

    def _resolve_edge(self):
        """True when the current reading needs no action."""
        observed = self._observed_tool()
        if observed is not INVALID and observed == self.active_tool:
            # Consistent with what we already believe - a late initial level
            # report from the MCU, not a head that moved.
            return True
        if self.state == 'fault' and observed is not INVALID:
            # We were faulted and the sensors have settled into a valid
            # resting state: adopt it instead of stacking another fault.
            self.active_tool = observed
            self._clear_fault()
            self._respond(self._describe_state())
            return True
        return False

    def _confirm_anomaly(self, eventtime):
        self.anomaly_pending = False
        if self.phase != PHASE_IDLE or not self.startup_reported:
            return self.reactor.NEVER
        if self._resolve_edge():
            return self.reactor.NEVER
        self.abort_requested = 'unexpected_edge'
        self._react_async('unexpected_edge', self._describe_sensors())
        return self.reactor.NEVER

    def _sample_sensors(self, wait_moves=True, async_fault=False):
        """Sample every sensor where the host matches physical position.

        wait_moves() drains the queue, so the reading reflects the machine
        and not a position it will only reach seconds later.  This is what
        the old "G4 P2000" was blindly approximating.

        async_fault must be set when called from a reactor callback, where
        raising a command error would escape into the reactor loop.
        """
        if wait_moves:
            self.printer.lookup_object('toolhead').wait_moves()
        sensors = [t.sensor for t in self.tool_list]
        if self.slider_sensor is not None:
            sensors.append(self.slider_sensor)
        deadline = self.reactor.monotonic() + self.sensor_timeout
        while True:
            now = self.reactor.monotonic()
            pending = [s for s in sensors
                       if s.raw is None
                       or (now - s.last_change) < self.settle_time]
            if not pending:
                return
            if now >= deadline:
                names = ", ".join(s.name for s in pending)
                if async_fault:
                    self._react_async('sensor_unstable', names)
                else:
                    self._react('sensor_unstable', names)
                return
            self.reactor.pause(now + 0.01)

    def _slider_occupied(self):
        if self.slider_sensor is not None:
            return self.slider_sensor.value()
        return None

    def _derive_state(self):
        """Map sensor readings onto a resting state.

        R1 both docked / slider empty, R2 T0 on slider, R3 T1 on
        slider.  Anything else is a fault.
        """
        unknown = [t for t in self.tool_list if t.is_docked() is None]
        undocked = [t for t in self.tool_list if t.is_docked() is False]
        occupied = self._slider_occupied()
        if unknown:
            self._set_fault('startup')
            return None
        if len(undocked) > 1:
            self._set_fault('both_undocked')
            return None
        if not undocked:
            if occupied is True:
                self._set_fault('slider_unexpected')
                return None
            self._clear_fault()
            self.active_tool = None
            return None
        tool = undocked[0]
        if occupied is False:
            self._set_fault('head_lost')
            return None
        self._clear_fault()
        self.active_tool = tool.number
        return tool

    def _clear_fault(self):
        """The machine is back in a valid resting state.

        Also drops a pending abort: it can only have been latched by an
        idle-time edge (the guard stays quiet during a change), and the
        operator has since put things right.  Without this, touching a head
        by hand would sabotage the next tool change.
        """
        self.state = 'ok'
        self.fault = None
        self.abort_requested = None

    def _set_fault(self, code):
        self.state = 'fault'
        self.fault = code
        self.active_tool = None

    def _verify_state(self, expect_tool):
        """Check the sensors against what the state machine expects."""
        self._sample_sensors()
        for tool in self.tool_list:
            docked = tool.is_docked()
            if docked is None:
                self._react('sensor_unstable', tool.sensor.name)
                return False
            should_be_docked = (expect_tool is None
                                or tool.number != expect_tool)
            if docked != should_be_docked:
                return False
        occupied = self._slider_occupied()
        if occupied is not None and occupied != (expect_tool is not None):
            self._react('slider_unexpected' if expect_tool is None
                        else 'head_lost', self._describe_sensors())
            return False
        return True

    def _describe_sensors(self):
        parts = []
        for tool in self.tool_list:
            docked = tool.is_docked()
            where = ("парковка" if docked else
                     "слайдер" if docked is False else "нет данных")
            parts.append("T%d — %s" % (tool.number, where))
        if self.slider_sensor is not None:
            parts.append("слайдер — %s"
                         % ("занята" if self.slider_sensor.value()
                            else "пуста"))
        return ", ".join(parts)

    def _describe_state(self):
        if self.active_tool is None:
            head = "обе головы в парковках"
        else:
            head = "активная голова T%d" % (self.active_tool,)
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        name = extruder.get_name() if extruder is not None else "нет"
        return ("Тулченджер: %s (%s). Активный экструдер: %s"
                % (head, self._describe_sensors(), name))

    # ------------------------------------------------------------------
    # Fault handling
    # ------------------------------------------------------------------

    def _respond(self, message):
        self.gcode.respond_info("[+] %s" % (message,))

    def _respond_error(self, message):
        self.gcode.respond_raw("!! %s" % (message,))

    def _format_fault(self, code, detail):
        message = FAULT_MESSAGES.get(code, code)
        if detail:
            message = "%s (%s)" % (message, detail)
        return "Тулченджер — %s" % (message,)

    def _effective_reaction(self, code):
        reaction = self.reactions.get(code, 'error')
        blocking = reaction in ('pause', 'error', 'shutdown')
        if blocking and self.is_service_mode():
            # Nothing may block an assembler who is deliberately moving
            # heads around by hand.
            return 'warn'
        return reaction

    def _react(self, code, detail=""):
        """Apply the configured reaction to a fault.  May raise."""
        reaction = self._effective_reaction(code)
        message = self._format_fault(code, detail)
        logging.warning("zbolt_toolchanger fault %s: %s", code, message)
        if reaction == 'ignore':
            return
        if reaction == 'warn':
            self._respond(message)
            return
        self._set_fault(code)
        if reaction == 'shutdown':
            self.printer.invoke_shutdown(message)
            raise self.printer.command_error(message)
        self._respond_error(message)
        if reaction == 'pause' and self._is_printing():
            self.gcode.run_script_from_command("PAUSE")
        raise self.printer.command_error(message)

    def _react_async(self, code, detail=""):
        """Fault reaction from a reactor callback: never raises."""
        reaction = self._effective_reaction(code)
        message = self._format_fault(code, detail)
        logging.warning("zbolt_toolchanger fault %s: %s", code, message)
        if reaction == 'ignore':
            return
        if reaction == 'warn':
            self._respond(message)
            return
        self._set_fault(code)
        if reaction == 'shutdown':
            self.printer.invoke_shutdown(message)
            return
        self._respond_error(message)
        if reaction == 'pause' and self._is_printing():
            self.reactor.register_callback(
                lambda e: self.gcode.run_script("PAUSE"))

    def _is_printing(self):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            state = print_stats.get_status(
                self.reactor.monotonic()).get('state')
            return state == 'printing'
        idle_timeout = self.printer.lookup_object('idle_timeout', None)
        if idle_timeout is not None:
            return idle_timeout.get_status(
                self.reactor.monotonic()).get('state') == 'Printing'
        return False

    def _check_abort(self):
        if self.abort_requested is None:
            return
        code = self.abort_requested
        self.abort_requested = None
        self._react(code, "последовательность прервана")

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def _move(self, x=None, y=None, speed=None):
        speed = self.travel_speed if speed is None else speed
        self.printer.lookup_object('toolhead').manual_move([x, y, None], speed)

    def _move_rel(self, dx=0., dy=0., speed=None):
        pos = self.printer.lookup_object('toolhead').get_position()
        self._move(pos[0] + dx, pos[1] + dy, speed)

    def _lift_z(self, distance):
        if not distance:
            return
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        toolhead.manual_move([None, None, pos[2] + distance],
                             self.z_lift_speed)

    def _move_z_to(self, height):
        self.printer.lookup_object('toolhead').manual_move(
            [None, None, height], self.z_lift_speed)

    def _bed_centre(self):
        status = self.printer.lookup_object('toolhead').get_status(
            self.reactor.monotonic())
        return (status['axis_maximum'][0] / 2., status['axis_maximum'][1] / 2.)

    def _apply_speed_factor(self):
        if self.force_speed_factor:
            self.gcode.run_script_from_command("M220 S%.0f"
                                               % (self.force_speed_factor,))

    def _apply_offset(self, x, y, z):
        script = self.apply_offset_template.render({'x': x, 'y': y, 'z': z})
        self.gcode.run_script_from_command(script)

    def _clear_xy_offset(self):
        # Only X and Y are zeroed while docking; Z stays as it is.
        gcode_move = self.printer.lookup_object('gcode_move', None)
        z = 0.
        if gcode_move is not None:
            z = gcode_move.get_status(self.reactor.monotonic())[
                'homing_origin'].z
        self._apply_offset(0., 0., z)

    def _find_tmc(self, stepper):
        """Locate the driver object for a stepper across driver families."""
        for prefix in ('tmc2209', 'tmc5160', 'tmc2130', 'tmc2208', 'tmc2660',
                       'tmc2240', 'tmc5150'):
            driver = self.printer.lookup_object('%s %s' % (prefix, stepper),
                                                None)
            if driver is not None:
                return driver
        return None

    def _boost_applies(self, tool, phase):
        return bool(tool.current_boost) and \
            tool.current_boost_phase in (phase, 'both')

    def _set_current_boost(self, tool, enable):
        for stepper in ('stepper_x', 'stepper_y'):
            driver = self._find_tmc(stepper)
            if driver is None:
                continue
            if enable:
                status = driver.get_status(self.reactor.monotonic())
                base = status.get('run_current')
                if base is None:
                    continue
                self.boost_base[stepper] = base
                value = base * tool.current_boost
            else:
                if stepper not in self.boost_base:
                    continue
                value = self.boost_base.pop(stepper)
            self.gcode.run_script_from_command(
                "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f" % (stepper, value))

    def _release_coupling(self, tool):
        """Let go of the head once it is seated in its dock."""
        if tool.coupling == 'solenoid':
            if tool.solenoid_pin:
                self.gcode.run_script_from_command(
                    "SET_PIN PIN=%s VALUE=0" % (tool.solenoid_pin,))
            if tool.release_delay:
                self.printer.lookup_object('toolhead').dwell(
                    tool.release_delay)
        elif tool.release_jerk:
            self._move_rel(dy=-tool.release_jerk,
                           speed=tool.release_jerk_speed)
            self._move_rel(dy=tool.release_jerk,
                           speed=tool.release_jerk_speed)

    def _arm_coupling(self, tool):
        """Prepare the coupling before approaching the dock."""
        if tool.coupling == 'solenoid' and tool.solenoid_pin:
            self.gcode.run_script_from_command(
                "SET_PIN PIN=%s VALUE=1" % (tool.solenoid_pin,))

    def _engage_coupling(self, tool):
        """Latch the head onto the slider once the dock is reached."""
        if tool.coupling == 'latch' and tool.release_jerk:
            self._move_rel(dy=tool.release_jerk, speed=self.travel_speed)
            self._move_rel(dy=-tool.release_jerk, speed=self.travel_speed)

    def _template_context(self, tool):
        context = self.gcode_macro.create_template_context()
        context.update({
            'park_x': tool.park_xy[0], 'park_y': tool.park_xy[1],
            'unpark_x': tool.unpark_xy[0], 'unpark_y': tool.unpark_xy[1],
            'safe_x': tool.safe_xy[0], 'safe_y': tool.safe_xy[1],
            'tool_number': tool.number,
            'fast': self.travel_speed * 60.,
            'slow': self.dock_speed * 60.,
        })
        return context

    # ------------------------------------------------------------------
    # Park / pickup
    # ------------------------------------------------------------------

    def park_tool(self, number):
        tool = self._get_tool(number)
        self.phase = PHASE_PARK
        try:
            for attempt in range(self.verify_retries + 1):
                self._run_park(tool)
                if self._verify_state(None):
                    break
                if attempt < self.verify_retries:
                    self._respond("T%d не подтвердила парковку, повтор"
                                  % (tool.number,))
                    continue
                self._react('park_failed', "T%d, %s"
                            % (tool.number, self._describe_sensors()))
        finally:
            self.phase = PHASE_IDLE
        self.active_tool = None
        self._clear_fault()

    def _run_park(self, tool):
        if tool.park_template is not None:
            self.gcode.run_script_from_command(
                tool.park_template.render(self._template_context(tool)))
            return
        fast, slow = self.travel_speed, self.dock_speed
        self._apply_speed_factor()
        self._clear_xy_offset()
        if tool.stage_xy is not None:
            self._move(tool.stage_xy[0], tool.stage_xy[1], fast)
        self._move(tool.safe_xy[0], tool.safe_xy[1], fast)
        self._check_abort()
        # Wipe the nozzle on the purge block: offsets are relative to the
        # safe point, which is how the S400/S600 macros expressed it.
        for dx, dy in tool.wipe_moves:
            self._move(tool.safe_xy[0] + dx, tool.safe_xy[1] + dy, fast)
        # Align onto the dock row, then come in along X.
        self._move(tool.safe_xy[0], tool.unpark_xy[1], fast)
        if tool.approach_offset:
            self._move(tool.park_xy[0] + tool.approach_offset,
                       tool.park_xy[1], fast)
        self._check_abort()
        boost = self._boost_applies(tool, 'park')
        if boost:
            self._set_current_boost(tool, True)
        try:
            self._move(tool.park_xy[0], tool.park_xy[1], slow)
            self._release_coupling(tool)
            self._move(tool.unpark_xy[0], tool.safe_xy[1], slow)
        finally:
            if boost:
                self._set_current_boost(tool, False)

    def pickup_tool(self, number):
        tool = self._get_tool(number)
        self.phase = PHASE_PICKUP
        try:
            for attempt in range(self.verify_retries + 1):
                self._run_pickup(tool)
                if self._verify_state(tool.number):
                    break
                if attempt < self.verify_retries:
                    self._respond("T%d не подтвердила захват, повтор"
                                  % (tool.number,))
                    continue
                self._react('get_failed', "T%d, %s"
                            % (tool.number, self._describe_sensors()))
        finally:
            self.phase = PHASE_IDLE
        self.active_tool = tool.number
        self._clear_fault()
        self._activate_extruder(tool)
        self._apply_tool_offsets(tool)

    def _run_pickup(self, tool):
        if tool.get_template is not None:
            self.gcode.run_script_from_command(
                tool.get_template.render(self._template_context(tool)))
            return
        fast, slow = self.travel_speed, self.dock_speed
        retreat = slow if tool.retreat_speed == 'slow' else fast
        self._apply_speed_factor()
        self._clear_xy_offset()
        self._arm_coupling(tool)
        self._move(tool.unpark_xy[0], tool.safe_xy[1], fast)
        self._check_abort()
        boost = self._boost_applies(tool, 'get')
        self._move(tool.park_xy[0], tool.park_xy[1], slow)
        self._engage_coupling(tool)
        if boost:
            self._set_current_boost(tool, True)
        try:
            self._move(tool.safe_xy[0], tool.unpark_xy[1], retreat)
            if tool.exit_xy is not None:
                self._move(tool.exit_xy[0], tool.exit_xy[1], fast)
        finally:
            if boost:
                self._set_current_boost(tool, False)
        self._check_abort()

    def _seat_tool(self, tool):
        """Press a head already on the slider back into its coupling.

        Replaces the old _PUSH_EXTRUDER0/1 recovery used after homing.
        """
        if tool.seat_offset_xy is None:
            return
        fast = self.travel_speed
        self._move(tool.park_xy[0] + tool.seat_offset_xy[0],
                   tool.park_xy[1] + tool.seat_offset_xy[1], fast)
        self._arm_coupling(tool)
        if tool.seat_jerk:
            self._move_rel(dy=-tool.seat_jerk, speed=tool.release_jerk_speed)
            if tool.release_delay:
                self.printer.lookup_object('toolhead').dwell(
                    tool.release_delay)
            self._move_rel(dy=tool.seat_jerk, speed=tool.seat_return_speed)
        self._move(tool.safe_xy[0], tool.safe_xy[1], fast)

    # ------------------------------------------------------------------
    # Tool selection and offsets
    # ------------------------------------------------------------------

    def _get_tool(self, number):
        tool = self.tools.get(number)
        if tool is None:
            raise self.printer.command_error("Неизвестный инструмент T%s"
                                             % (number,))
        return tool

    def get_saved_variables(self):
        save_variables = self.printer.lookup_object('save_variables', None)
        if save_variables is None:
            return {}
        return save_variables.allVariables

    def _get_global_offset(self):
        try:
            return float(self.get_saved_variables().get(
                self.global_offset_variable, 0.))
        except (TypeError, ValueError):
            return 0.

    def is_service_mode(self):
        value = self.get_saved_variables().get(self.service_mode_variable,
                                               False)
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _activate_extruder(self, tool):
        toolhead = self.printer.lookup_object('toolhead')
        current = toolhead.get_extruder()
        if current is not None and current.get_name() == tool.extruder_name:
            return
        self.gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=%s"
                                           % (tool.extruder_name,))

    def _apply_tool_offsets(self, tool):
        offsets = tool.get_offsets()
        self._apply_offset(offsets['x'], offsets['y'],
                           offsets['z'] + self._get_global_offset())

    def _capture_babystep(self):
        """Fold a babystep made with the current tool into the global offset.

        Mirrors the preamble the old T0/T1 macros carried: whatever the
        operator dialled in with SET_GCODE_OFFSET while printing has to
        survive the tool change.
        """
        if self.active_tool is None:
            return
        tool = self.tools.get(self.active_tool)
        gcode_move = self.printer.lookup_object('gcode_move', None)
        if tool is None or gcode_move is None:
            return
        current_z = gcode_move.get_status(
            self.reactor.monotonic())['homing_origin'].z
        global_offset = self._get_global_offset()
        stored_z = tool.get_offsets()['z'] + global_offset
        delta = round(current_z - stored_z, 5)
        if abs(delta) <= 0.0001:
            return
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=%s VALUE=%.5f"
            % (self.global_offset_variable,
               round(global_offset + delta, 5)))

    def _run_hook(self, template):
        if template is not None:
            self.gcode.run_script_from_command(
                template.render(self.gcode_macro.create_template_context()))

    def select_tool(self, number, gcmd=None, force=False):
        tool = self._get_tool(number)
        self._run_hook(self.before_change_template)
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(self.reactor.monotonic())['homed_axes']
        if 'x' not in homed or 'y' not in homed or 'z' not in homed:
            self._react('change_while_unhomed', "выполняется G28")
            self.gcode.run_script_from_command("G28")
        self._capture_babystep()
        self._sample_sensors()
        self._derive_state()
        if self.state == 'fault':
            self._react(self.fault, self._describe_sensors())
            return
        if self.active_tool == number and not force:
            # Already selected: re-apply extruder and offsets, no motion.
            self._activate_extruder(tool)
            self._apply_tool_offsets(tool)
        else:
            if self.active_tool is not None:
                self.park_tool(self.active_tool)
            self.pickup_tool(number)
        if not self.is_service_mode():
            self._run_hook(self.after_change_template)

    # ------------------------------------------------------------------
    # Homing
    # ------------------------------------------------------------------

    def _run_homing(self):
        # Sample before moving: nothing has been commanded yet, so there is
        # no queue to drain and the sensors already reflect reality.
        self._sample_sensors(wait_moves=False)
        unknown = [t for t in self.tool_list if t.is_docked() is None]
        if unknown:
            self._react('sensor_unstable',
                        ", ".join(t.sensor.name for t in unknown))
            return
        undocked = [t for t in self.tool_list if t.is_docked() is False]
        if len(undocked) > 1:
            self._react('both_undocked', self._describe_sensors())
            return
        self._clear_fault()
        carried = undocked[0] if undocked else None
        if carried is None:
            self._respond("Обе головы в парковках")
        else:
            self._respond("На слайдере T%d" % (carried.number,))
        self._lift_z(self.z_lift_before_home)
        for axis in self.home_axes_order:
            self.gcode.run_script_from_command("G28 %s" % (axis.upper(),))
        if self.is_service_mode():
            self._respond("Сервисный режим: инструмент не забирается")
            if carried is None:
                self._respond("На слайдере нет головы, хоминг Z пропущен")
                self._clear_fault()
                return
            self.active_tool = carried.number
            self._activate_extruder(carried)
            centre = self.home_z_position or self._bed_centre()
            self._move(centre[0], centre[1], self.travel_speed)
            self.gcode.run_script_from_command("G28 Z")
            self._move_z_to(self.z_after_home)
            self._clear_fault()
            self._respond(self._describe_state())
            return
        # Recovery plan, replacing the old nested if/else in homing.cfg.
        target = self.initial_tool
        if carried is None:
            self.active_tool = None
            self.pickup_tool(target)
        elif carried.number == target:
            self.active_tool = carried.number
            self._seat_tool(carried)
            self._activate_extruder(carried)
            self._apply_tool_offsets(carried)
        else:
            self.active_tool = carried.number
            self._seat_tool(carried)
            self.park_tool(carried.number)
            self.pickup_tool(target)
        # Only now is a head guaranteed to be on the slider, so the
        # nozzle-contact probe can home Z.
        if not self._verify_state(self.active_tool):
            self._react('get_failed', self._describe_sensors())
            return
        centre = self.home_z_position
        if centre is None:
            centre = self._bed_centre()
        self._move(centre[0], centre[1], self.travel_speed)
        self.gcode.run_script_from_command("G28 Z")
        self._move_z_to(self.z_after_home)
        if self.after_home_position is not None:
            self._move(self.after_home_position[0],
                       self.after_home_position[1], self.travel_speed)
        self._clear_fault()
        self._respond(self._describe_state())

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    cmd_TOOLCHANGER_SELECT_help = "Выбрать инструмент (TOOL=0|1)"
    def cmd_TOOLCHANGER_SELECT(self, gcmd):
        self.select_tool(gcmd.get_int('TOOL'), gcmd,
                         bool(gcmd.get_int('FORCE', 0)))

    cmd_TOOLCHANGER_PARK_help = "Поставить инструмент в парковку"
    def cmd_TOOLCHANGER_PARK(self, gcmd):
        default = self.active_tool if self.active_tool is not None else -1
        number = gcmd.get_int('TOOL', default)
        if number < 0:
            self._respond("Нет активного инструмента, парковка не требуется")
            return
        if self._get_tool(number).is_docked():
            self._respond("T%d уже в парковке" % (number,))
            return
        self.park_tool(number)

    cmd_TOOLCHANGER_PICKUP_help = "Забрать инструмент из парковки"
    def cmd_TOOLCHANGER_PICKUP(self, gcmd):
        self.pickup_tool(gcmd.get_int('TOOL'))

    cmd_TOOLCHANGER_HOME_help = (
        "Хоминг с восстановлением состояния тулченджера. "
        "Вызывается из [homing_override]")
    def cmd_TOOLCHANGER_HOME(self, gcmd):
        self.phase = PHASE_HOMING
        try:
            self._run_homing()
        finally:
            self.phase = PHASE_IDLE

    cmd_TOOLCHANGER_STATUS_help = "Показать состояние тулченджера"
    def cmd_TOOLCHANGER_STATUS(self, gcmd):
        self._sample_sensors()
        self._derive_state()
        if self.state == 'fault':
            self._respond_error("Тулченджер — %s. %s"
                                % (FAULT_MESSAGES.get(self.fault, self.fault),
                                   self._describe_sensors()))
            return
        self._respond(self._describe_state())

    cmd_TOOLCHANGER_QUERY_SENSORS_help = (
        "Мгновенный опрос датчиков парковок и слайдера")
    def cmd_TOOLCHANGER_QUERY_SENSORS(self, gcmd):
        self._sample_sensors()
        for tool in self.tool_list:
            gcmd.respond_info("%s: %s — голова %s"
                              % (tool.sensor.name, tool.sensor.raw_text(),
                                 "в парковке" if tool.is_docked()
                                 else "на слайдере"))
        if self.slider_sensor is not None:
            gcmd.respond_info("%s: %s — слайдер %s"
                              % (self.slider_sensor.name,
                                 self.slider_sensor.raw_text(),
                                 "занята" if self.slider_sensor.value()
                                 else "пуста"))

    cmd_TOOLCHANGER_SET_STATE_help = (
        "Задать состояние вручную при разборе аварии (TOOL=0|1|NONE)")
    def cmd_TOOLCHANGER_SET_STATE(self, gcmd):
        raw = gcmd.get('TOOL').strip().lower()
        if raw in ('none', 'нет', '-'):
            self.active_tool = None
        else:
            try:
                tool = self._get_tool(int(raw))
            except ValueError:
                raise gcmd.error("TOOL должен быть номером инструмента "
                                 "или NONE")
            self.active_tool = tool.number
            self._activate_extruder(tool)
        self._clear_fault()
        self._respond(self._describe_state())

    cmd_TOOLCHANGER_RELEASE_help = "Расцепить сцепку инструмента"
    def cmd_TOOLCHANGER_RELEASE(self, gcmd):
        default = self.active_tool if self.active_tool is not None else 0
        self._release_coupling(self._get_tool(gcmd.get_int('TOOL', default)))

    cmd_TOOLCHANGER_COUPLE_help = "Сцепить голову со слайдером"
    def cmd_TOOLCHANGER_COUPLE(self, gcmd):
        default = self.active_tool if self.active_tool is not None else 0
        tool = self._get_tool(gcmd.get_int('TOOL', default))
        self._arm_coupling(tool)
        self._engage_coupling(tool)

    cmd_TOOLCHANGER_SAVE_ADJUST_help = (
        "Сохранить подстройку Z в переменную общего офсета с учётом офсета "
        "активной головы. Вызывается из обёртки SET_GCODE_OFFSET")
    def cmd_TOOLCHANGER_SAVE_ADJUST(self, gcmd):
        self._capture_babystep()

    cmd_TOOLCHANGER_SERVICE_MODE_help = (
        "Сервисный режим настройки: ENABLE=1|0, без параметра — показать")
    def cmd_TOOLCHANGER_SERVICE_MODE(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is None:
            self._respond("Сервисный режим %s"
                          % ("включен" if self.is_service_mode()
                             else "выключен"))
            return
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=%s VALUE=%s"
            % (self.service_mode_variable, "True" if enable else "False"))
        if enable:
            self._respond(
                "Сервисный режим включен: компенсация перекоса не "
                "включается после смены инструмента, аварии только "
                "предупреждают, хоминг не забирает голову")
        else:
            self._respond("Сервисный режим выключен")

    POINTS = ('park', 'unpark', 'safe', 'stage', 'exit')

    def _point_of(self, tool, name):
        value = {'park': tool.park_xy, 'unpark': tool.unpark_xy,
                 'safe': tool.safe_xy, 'stage': tool.stage_xy,
                 'exit': tool.exit_xy}[name]
        if value is None:
            raise self.printer.command_error(
                "У T%d не задана точка %s" % (tool.number, name))
        return value

    cmd_TOOLCHANGER_GOTO_help = (
        "Подъехать к точке парковки для настройки: TOOL=n POINT=park|unpark|"
        "safe|stage|exit")
    def cmd_TOOLCHANGER_GOTO(self, gcmd):
        tool = self._get_tool(gcmd.get_int('TOOL'))
        name = gcmd.get('POINT', 'safe').strip().lower()
        if name not in self.POINTS:
            raise gcmd.error("POINT должен быть одним из: %s"
                             % (", ".join(self.POINTS),))
        point = self._point_of(tool, name)
        speed = gcmd.get_float('SPEED', self.dock_speed, above=0.)
        self._respond("T%d -> %s (X%.3f Y%.3f)"
                      % (tool.number, name, point[0], point[1]))
        self._move(point[0], point[1], speed)

    cmd_TOOLCHANGER_SET_DOCK_help = (
        "Записать координату парковки: TOOL=n POINT=park|unpark|safe|stage|"
        "exit [X= Y=]. Без X/Y берётся текущая позиция; затем SAVE_CONFIG")
    def cmd_TOOLCHANGER_SET_DOCK(self, gcmd):
        tool = self._get_tool(gcmd.get_int('TOOL'))
        name = gcmd.get('POINT').strip().lower()
        if name not in self.POINTS:
            raise gcmd.error("POINT должен быть одним из: %s"
                             % (", ".join(self.POINTS),))
        position = self.printer.lookup_object('toolhead').get_position()
        x = gcmd.get_float('X', position[0])
        y = gcmd.get_float('Y', position[1])
        option = '%s_xy' % (name,)
        configfile = self.printer.lookup_object('configfile')
        configfile.set('zbolt_toolchanger_tool %s' % (tool.name,), option,
                       '%.3f, %.3f' % (x, y))
        setattr(tool, option, [x, y])
        self._respond("T%d %s = %.3f, %.3f — выполните SAVE_CONFIG, чтобы "
                      "сохранить" % (tool.number, option, x, y))

    cmd_TOOLCHANGER_CALIBRATE_OFFSET_help = (
        "Калибровка Z-офсета второй головы по касанию сопла")
    def cmd_TOOLCHANGER_CALIBRATE_OFFSET(self, gcmd):
        number = gcmd.get_int('TOOL', 1)
        temp = gcmd.get_float('TEMP', self.calibrate_temp, minval=0.)
        samples = gcmd.get_int('SAMPLES', self.calibrate_samples, minval=1)
        stabilize = gcmd.get_float('STABILIZE', self.calibrate_stabilize,
                                   minval=0.)
        tool = self._get_tool(number)
        variable = tool.offset_vars.get('z')
        if not variable:
            raise gcmd.error("У T%d не задан offset_z_variable" % (number,))
        probe = self.printer.lookup_object('probe', None)
        if probe is None:
            raise gcmd.error("Секция [probe] не настроена")
        reference = self.tools.get(self.initial_tool)
        if reference is None or reference.number == number:
            raise gcmd.error("Нужен опорный инструмент, отличный от T%d"
                             % (number,))
        self._respond("Запуск калибровки Z-офсета T%d по опорной T%d"
                      % (number, reference.number))
        # One homing for both measurements: the result is their difference,
        # so re-homing in between would throw the common reference away.
        self.gcode.run_script_from_command("G28")
        for tool_obj in self.tool_list:
            self.gcode.run_script_from_command(
                "SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.0f"
                % (tool_obj.extruder_name, temp))
        for tool_obj in self.tool_list:
            self.gcode.run_script_from_command(
                "TEMPERATURE_WAIT SENSOR=%s MINIMUM=%.0f"
                % (tool_obj.extruder_name, temp))
        if stabilize:
            self._respond("Термостабилизация — %.0f с" % (stabilize,))
            self.printer.lookup_object('toolhead').dwell(stabilize)
        point = self.home_z_position or self._bed_centre()
        z_ref = self._probe_with_tool(reference, point, samples, probe)
        z_tool = self._probe_with_tool(tool, point, samples, probe)
        # Difference of two measurements at the same physical spot: the
        # probe's own z_offset, any homing drift and the bed shape under
        # that point all cancel out, so nothing but the head-to-head
        # difference survives.
        value = round(z_tool - z_ref, 3)
        self._respond("T%d: z=%.3f, T%d: z=%.3f -> офсет T%d = %.3f мм"
                      % (reference.number, z_ref, number, z_tool, number,
                         value))
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=%s VALUE=%.3f" % (variable, value))
        self.gcode.run_script_from_command("TURN_OFF_HEATERS")

    def _probe_with_tool(self, tool, point, samples, probe):
        """Probe one physical spot with one tool.

        The move is issued as g-code on purpose: the tool's own X/Y offset
        is applied as a gcode offset, so every tool puts its NOZZLE on the
        same physical spot.  Moving the carriage in kinematic coordinates
        instead would probe a different place for a tool with XY offsets,
        and both bed shape and axis_twist_compensation would leak into the
        result.
        """
        self.select_tool(tool.number)
        self.gcode.run_script_from_command("G90")
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F%.0f"
            % (point[0], point[1], self.travel_speed * 60.))
        # Let the probe helper do the averaging and tolerance checking.
        self.gcode.run_script_from_command("PROBE SAMPLES=%d" % (samples,))
        z = float(probe.get_status(
            self.reactor.monotonic())['last_z_result'])
        self._lift_z(self.calibrate_lift)
        return z

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def get_status(self, eventtime):
        return {
            'state': self.state,
            'fault': self.fault or '',
            'phase': self.phase,
            'service_mode': self.is_service_mode(),
            'active_tool': self.active_tool,
            'slider_occupied': self._slider_occupied(),
            # String keys keep the status JSON-safe for Moonraker and give
            # macros a readable path: printer['zbolt_toolchanger'].tools.t0
            'tools': {'t%d' % (t.number,): t.get_status(eventtime)
                      for t in self.tool_list},
        }


def load_config(config):
    return ZBoltToolchanger(config)
