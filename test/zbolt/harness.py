"""Offline harness for the Z-Bolt Klipper extras.

Stubs just enough of the Klipper host API to exercise config parsing, state
derivation, park/pickup trajectories, homing recovery and the fault matrix
without a printer.  Not a Klipper emulator - only the calls the module makes.
"""
import sys, os, types, ast

# Модули лежат рядом, в klippy/extras этого же репозитория.
EXTRAS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy', 'extras')
EXTRAS = os.environ.get('TC_EXTRAS', EXTRAS)
sys.path.insert(0, os.path.abspath(EXTRAS))

import zbolt_toolchanger as tc

sentinel = object()


class ConfigError(Exception):
    pass


class CommandError(Exception):
    pass


class Config:
    def __init__(self, printer, name, options):
        self.printer = printer
        self.name = name
        self.options = options

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def error(self, msg):
        return ConfigError(msg)

    def _raw(self, option, default=sentinel):
        if option in self.options:
            return self.options[option]
        if default is sentinel:
            raise ConfigError("Option '%s' missing in [%s]"
                              % (option, self.name))
        return default

    def get(self, option, default=sentinel, note_valid=True):
        value = self._raw(option, default)
        return value if value is None else str(value)

    def getint(self, option, default=sentinel, minval=None, maxval=None,
               note_valid=True):
        value = self._raw(option, default)
        return int(value)

    def getfloat(self, option, default=sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        value = self._raw(option, default)
        return float(value)

    def getboolean(self, option, default=sentinel, note_valid=True):
        return bool(self._raw(option, default))

    def getchoice(self, option, choices, default=sentinel, note_valid=True):
        value = self._raw(option, default)
        if value not in choices:
            raise ConfigError("Invalid '%s' in [%s]" % (option, self.name))
        return choices[value]

    def getlist(self, option, default=sentinel, sep=',', count=None,
                note_valid=True):
        value = self._raw(option, default)
        if value is None or isinstance(value, (list, tuple)):
            return value
        return [p.strip() for p in str(value).split(sep) if p.strip()]

    def getfloatlist(self, option, default=sentinel, sep=',', count=None,
                     note_valid=True):
        value = self._raw(option, default)
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            return [float(v) for v in value]
        parts = [float(p) for p in str(value).split(sep) if p.strip()]
        if count is not None and len(parts) != count:
            raise ConfigError("Option '%s' in [%s] needs %d values"
                              % (option, self.name, count))
        return parts


class Reactor:
    NEVER = 9e99

    def __init__(self):
        self.time = 100.
        self.callbacks = []

    def monotonic(self):
        return self.time

    def pause(self, waketime):
        self.time = max(self.time, waketime)
        return self.time

    def register_callback(self, callback, waketime=None):
        self.callbacks.append((callback, waketime))

    def run_pending(self):
        pending, self.callbacks = self.callbacks, []
        for callback, waketime in pending:
            if waketime:
                self.time = max(self.time, waketime)
            callback(self.time)


class Coord:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class GCode:
    def __init__(self, printer):
        self.printer = printer
        self.commands = {}
        self.script = []
        self.output = []
        self.error = CommandError

    def register_command(self, name, handler, desc=None):
        if name in self.commands:
            raise ConfigError("Command %s already registered" % (name,))
        self.commands[name] = handler

    def respond_info(self, msg, log=True):
        self.output.append(msg)

    def respond_raw(self, msg):
        self.output.append(msg)

    def run_script_from_command(self, script):
        for line in script.split('\n'):
            line = line.strip()
            if line:
                self.script.append(line)
                self.printer.dispatch(line)

    def run_script(self, script):
        self.run_script_from_command(script)


class Toolhead:
    def __init__(self, printer):
        self.printer = printer
        self.position = [0., 0., 0., 0.]
        self.moves = []
        self.homed = ''
        self.extruder = None
        self.axis_maximum = Coord(600., 700., 800.)

    def wait_moves(self):
        self.printer.log.append('wait_moves')

    def manual_move(self, coord, speed):
        for i, value in enumerate(coord):
            if value is not None:
                self.position[i] = value
        self.moves.append((round(self.position[0], 3),
                           round(self.position[1], 3),
                           round(self.position[2], 3), round(speed, 3)))

    def get_position(self):
        return list(self.position)

    def get_extruder(self):
        return self.extruder

    def dwell(self, delay):
        self.printer.log.append('dwell %.3f' % (delay,))

    def get_status(self, eventtime):
        return {'homed_axes': self.homed,
                'axis_maximum': (self.axis_maximum.x, self.axis_maximum.y,
                                 self.axis_maximum.z)}


class Extruder:
    def __init__(self, name):
        self.name = name

    def get_name(self):
        return self.name


class GCodeMove:
    def __init__(self):
        self.homing_origin = Coord(0., 0., 0.)

    def get_status(self, eventtime):
        return {'homing_origin': self.homing_origin}


class SaveVariables:
    def __init__(self, variables):
        self.allVariables = dict(variables)


class ConfigFile:
    def __init__(self):
        self.sets = []

    def set(self, section, option, value):
        self.sets.append((section, option, value))


class Buttons:
    def __init__(self):
        self.registered = []

    def register_buttons(self, pins, callback):
        self.registered.append((tuple(pins), callback))


class Template:
    def __init__(self, text):
        self.text = text

    def render(self, context=None):
        text = self.text
        for key, value in (context or {}).items():
            text = text.replace('{%s}' % (key,), str(value))
        return text


class GCodeMacro:
    def load_template(self, config, option, default=None):
        text = config.get(option, default)
        return Template(text)

    def create_template_context(self, eventtime=None):
        return {}


class TMC:
    def __init__(self, current):
        self.current = current

    def get_status(self, eventtime):
        return {'run_current': self.current}


class Probe:
    def __init__(self):
        self.last_z_result = 0.
        self.z_offset = -1.
        # физическая высота сопла каждой головы относительно нуля Z
        self.nozzle_z = {'extruder': -1.0, 'extruder1': -0.85}

    def get_status(self, eventtime):
        return {'last_z_result': self.last_z_result}

    def get_offsets(self):
        return (0., 0., self.z_offset)


class PrintStats:
    def __init__(self):
        self.state = 'standby'

    def get_status(self, eventtime):
        return {'state': self.state}


class Printer:
    def __init__(self, variables):
        self.objects = {}
        self.event_handlers = {}
        self.reactor = Reactor()
        self.log = []
        self.shutdowns = []
        self.gcode = GCode(self)
        self.toolhead = Toolhead(self)
        self.objects['gcode'] = self.gcode
        self.objects['toolhead'] = self.toolhead
        self.objects['gcode_move'] = GCodeMove()
        self.objects['buttons'] = Buttons()
        self.objects['configfile'] = ConfigFile()
        self.objects['gcode_macro'] = GCodeMacro()
        self.objects['save_variables'] = SaveVariables(variables)
        self.objects['print_stats'] = PrintStats()
        self.objects['probe'] = Probe()
        self.objects['tmc2209 stepper_x'] = TMC(1.2)
        self.objects['tmc2209 stepper_y'] = TMC(1.2)
        self.extruders = {'extruder': Extruder('extruder'),
                          'extruder1': Extruder('extruder1')}

    # --- Klipper printer API -------------------------------------------
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise ConfigError("Unknown object %s" % (name,))
        return default

    def load_object(self, config, name):
        if name in self.objects:
            return self.objects[name]
        raise ConfigError("Cannot load %s" % (name,))

    def add_object(self, name, obj):
        self.objects[name] = obj

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, handler):
        self.event_handlers.setdefault(event, []).append(handler)

    def send_event(self, event):
        for handler in self.event_handlers.get(event, []):
            handler()

    def command_error(self, msg):
        return CommandError(msg)

    def config_error(self, msg):
        return ConfigError(msg)

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)

    # --- fake g-code execution -----------------------------------------
    def dispatch(self, line):
        parts = line.split()
        name = parts[0].upper()
        if name == 'G28':
            axes = ''.join(p[0].lower() for p in parts[1:]
                           if p[0].upper() in 'XYZ') or 'xyz'
            self.toolhead.homed = ''.join(
                sorted(set(self.toolhead.homed) | set(axes)))
            if 'x' in axes:
                self.toolhead.position[0] = 0.
            if 'y' in axes:
                self.toolhead.position[1] = 0.
            if 'z' in axes:
                self.toolhead.position[2] = 0.
        elif name == 'ACTIVATE_EXTRUDER':
            for part in parts[1:]:
                if part.upper().startswith('EXTRUDER='):
                    self.toolhead.extruder = self.extruders[part.split('=')[1]]
        elif name == 'PROBE':
            probe = self.objects['probe']
            ext = self.toolhead.extruder
            probe.last_z_result = probe.nozzle_z[ext.get_name()]
        elif name == 'SAVE_VARIABLE':
            variable = value = None
            for part in parts[1:]:
                key, _, val = part.partition('=')
                if key.upper() == 'VARIABLE':
                    variable = val
                elif key.upper() == 'VALUE':
                    value = val
            try:
                parsed = ast.literal_eval(value)
            except (ValueError, SyntaxError):
                parsed = value
            self.objects['save_variables'].allVariables[variable] = parsed


class GCmd:
    def __init__(self, printer, params=None):
        self.printer = printer
        self.params = {k.upper(): v for k, v in (params or {}).items()}
        self.error = CommandError

    def get(self, name, default=sentinel):
        if name.upper() in self.params:
            return str(self.params[name.upper()])
        if default is sentinel:
            raise CommandError("Missing %s" % (name,))
        return default

    def get_int(self, name, default=sentinel, minval=None, maxval=None):
        value = self.get(name, default)
        return None if value is None else int(value)

    def get_float(self, name, default=sentinel, minval=None, maxval=None,
                  above=None, below=None):
        value = self.get(name, default)
        return float(value)

    def respond_info(self, msg, log=True):
        self.printer.gcode.output.append(msg)


# ----------------------------------------------------------------------
# Fixtures
# ----------------------------------------------------------------------

S600_MAIN = {
    'sensor_settle_time': 0.05,
    'travel_speed': 300.,
    'dock_speed': 60.,
    'home_axes_order': 'y, x',
    'home_z_position': '300, 350',
    'after_home_position': '190, 642',
    'z_lift_before_home': 5.,
    'z_after_home': 10.,
    'initial_tool': 0,
    'apply_offset_gcode': '_SET_GCODE_OFFSET_INTERNAL X={x} Y={y} Z={z}',
}

S600_T0 = {
    'tool_number': 0, 'extruder': 'extruder', 'dock_sensor_pin': '^PB14',
    'park_xy': '-10, 655.6', 'safe_xy': '190, 627',
    'approach_offset': 50., 'retreat_speed': 'fast',
    'exit_xy': '190, 642', 'coupling': 'latch',
    'wipe_moves': '\n0, 28.6\n0, 0\n',
    'offset_z_variable': 't0offset',
}

S600_T1 = {
    'tool_number': 1, 'extruder': 'extruder1', 'dock_sensor_pin': '^PA2',
    'park_xy': '600, 654.3', 'safe_xy': '404, 635',
    'approach_offset': -50., 'retreat_speed': 'fast',
    'exit_xy': '404, 650', 'coupling': 'latch',
    'wipe_moves': '\n0, 19.3\n0, 0\n',
    'offset_x_variable': 't1_x_offset',
    'offset_y_variable': 't1_y_offset',
    'offset_z_variable': 't1_z_offset',
}

S300_MAIN = dict(S600_MAIN, home_z_position='150, 150',
                 after_home_position='10, 290', force_speed_factor=100.)
S300_T0 = {
    'tool_number': 0, 'extruder': 'extruder', 'dock_sensor_pin': '^PA1',
    'park_xy': '8.8, -57.8', 'unpark_xy': '8.3, -57.8',
    'safe_xy': '63.8, -37.8', 'stage_xy': '63.8, 0',
    'coupling': 'solenoid', 'solenoid_pin': 'sol', 'release_delay': 0.3,
    'current_boost': 1.5, 'current_boost_phase': 'get',
    'exit_xy': '63.8, 0', 'retreat_speed': 'slow',
    'seat_offset_xy': '5, 15', 'seat_jerk': 5.,
    'offset_z_variable': 't0offset',
}
S300_T1 = dict(S300_T0, tool_number=1, extruder='extruder1',
               dock_sensor_pin='^PB13')
S300_T1.update({'park_xy': '292.5, -61.0', 'unpark_xy': '292.5, -61.0',
                'safe_xy': '237.5, -41.0', 'stage_xy': '237.5, 0',
                'exit_xy': '237.5, 0', 'seat_offset_xy': '-5, 15',
                'seat_jerk': 5.5,
                'offset_x_variable': 't1_x_offset',
                'offset_y_variable': 't1_y_offset',
                'offset_z_variable': 't1_z_offset'})

VARIABLES = {'z_adjust': 0.0, 't0offset': 0.0, 't1_x_offset': 0.1,
             't1_y_offset': -0.2, 't1_z_offset': 0.029}


def build(main_opts, tool_opts, variables=None):
    printer = Printer(variables if variables is not None else VARIABLES)
    config = Config(printer, 'zbolt_toolchanger', main_opts)
    changer = tc.ZBoltToolchanger(config)
    printer.objects['zbolt_toolchanger'] = changer
    for opts in tool_opts:
        name = 'zbolt_toolchanger_tool t%s' % (opts['tool_number'],)
        changer.add_tool(Config(printer, name, opts))
    return printer, changer


def set_sensors(printer, changer, docked, via_callback=False):
    """docked: {tool_number: True/False}; True = head sits in its dock.

    Polarity now lives in the pin ("!"), so a triggered dock sensor means
    the head is in its dock: raw == docked.
    via_callback drives the real edge handler, exercising the realtime guard.
    """
    for tool in changer.tool_list:
        raw = docked[tool.number]
        if via_callback:
            tool.sensor._handle_edge(printer.reactor.monotonic(), raw)
        else:
            tool.sensor.raw = raw
            tool.sensor.last_change = 0.


def ready(printer, changer, docked):
    printer.send_event('klippy:ready')
    set_sensors(printer, changer, docked)
    printer.reactor.run_pending()
