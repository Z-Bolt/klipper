# Thermal Z compensation for enclosed printers
#
# Compensates nozzle-to-bed gap drift from nozzle, bed and chamber
# temperatures. Reference is captured on Z homing. Chamber uses a
# first-order lag so frame expansion after the air sensor is at target
# is still applied.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

def _slope(xs, zs):
    n = len(xs)
    if n < 2:
        return 0.
    mx = sum(xs) / n
    mz = sum(zs) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den < 1e-12:
        return 0.
    return sum((x - mx) * (z - mz) for x, z in zip(xs, zs)) / den


def _parse_temps(gcmd, name, default):
    raw = gcmd.get(name, default)
    vals = []
    for part in str(raw).replace(';', ',').split(','):
        part = part.strip()
        if not part:
            continue
        vals.append(float(part))
    if not vals:
        raise gcmd.error("%s must contain at least one temperature" % (name,))
    return vals


class ThermalZComp:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()

        self.nozzle_sensor = config.get('nozzle_sensor', 'extruder')
        self.bed_sensor = config.get('bed_sensor', 'heater_bed')
        self.chamber_sensor = config.get('chamber_sensor', None)

        self.nozzle_coeff = config.getfloat('nozzle_coeff', 0.,
                                            minval=-1., maxval=1.)
        self.bed_coeff = config.getfloat('bed_coeff', 0.,
                                         minval=-1., maxval=1.)
        self.chamber_coeff = config.getfloat('chamber_coeff', 0.,
                                             minval=-1., maxval=1.)
        self.chamber_tau = config.getfloat('chamber_tau', 180., minval=1.)
        self.max_z_adjust = config.getfloat('max_z_adjustment', 1., minval=0.)
        self.off_above_z = config.getfloat('z_adjust_off_above', 99999999.)
        self.park_z = config.getfloat('park_z', 10., above=1.)
        self.probe_speed = config.getfloat('probe_move_speed', 300., above=10.)

        self.calibrate_x = config.getfloat('calibrate_x', None)
        self.calibrate_y = config.getfloat('calibrate_y', None)
        self.calibrate_nozzle_temp = config.getfloat(
            'calibrate_nozzle_temp', 160., minval=0.)
        self.calibrate_bed_temp = config.getfloat(
            'calibrate_bed_temp', 80., minval=0.)
        self.soak_time = config.getfloat('soak_time', 90., minval=0.)
        self.stable_z = config.getfloat('stable_z', 0.01, above=0.)
        self.stable_interval = config.getfloat('stable_interval', 30.,
                                               above=5.)
        self.stable_max_time = config.getfloat('stable_max_time', 900.,
                                               minval=30.)

        self.adjust_enable = config.getboolean('enable', True)
        self.reset_on_z_home = config.getboolean('reset_on_z_home', True)

        self.ref_nozzle = 0.
        self.ref_bed = 0.
        self.ref_chamber = 0.
        self.chamber_eff = 0.
        self.last_temp_time = 0.
        self.z_adjust_mm = 0.
        self.last_z_adjust_mm = 0.
        self.last_position = [0., 0., 0., 0.]
        self.next_transform = None
        self.z_step_dist = 0.0025
        self.samples = []
        self.calibrating = False

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.temp_timer = self.reactor.register_timer(self._temp_event,
                                                      self.reactor.NEVER)

        self.gcode.register_command(
            'SET_THERMAL_Z', self.cmd_SET_THERMAL_Z,
            desc=self.cmd_SET_THERMAL_Z_help)
        self.gcode.register_command(
            'THERMAL_Z_STATUS', self.cmd_THERMAL_Z_STATUS,
            desc=self.cmd_THERMAL_Z_STATUS_help)
        self.gcode.register_command(
            'THERMAL_Z_CALIBRATE', self.cmd_THERMAL_Z_CALIBRATE,
            desc=self.cmd_THERMAL_Z_CALIBRATE_help)
        self.gcode.register_command(
            'THERMAL_Z_FIT', self.cmd_THERMAL_Z_FIT,
            desc=self.cmd_THERMAL_Z_FIT_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        gcode_move = self.printer.lookup_object('gcode_move')
        self.next_transform = gcode_move.set_move_transform(self, force=True)
        kin = self.toolhead.get_kinematics()
        steppers = [s.get_name() for s in kin.get_steppers()]
        z_stepper = kin.get_steppers()[steppers.index("stepper_z")]
        self.z_step_dist = z_stepper.get_step_dist()
        self._capture_ref(reset_lag=True)
        self.reactor.update_timer(self.temp_timer, self.reactor.monotonic())

    def _handle_home_rails_end(self, homing_state, rails):
        if not self.reset_on_z_home or self.calibrating:
            return
        if 2 in homing_state.get_axes():
            self._capture_ref(reset_lag=False)
            self.z_adjust_mm = 0.
            self.last_z_adjust_mm = 0.

    def _lookup_sensor(self, name):
        if not name:
            return None
        try:
            return self.printer.lookup_object(name)
        except self.printer.config_error:
            return None

    def _read_temp(self, name):
        obj = self._lookup_sensor(name)
        if obj is None:
            return 0.
        eventtime = self.reactor.monotonic()
        status = obj.get_status(eventtime)
        temp = status.get('temperature')
        if temp is None:
            return 0.
        return float(temp)

    def _temps(self):
        return (
            self._read_temp(self.nozzle_sensor),
            self._read_temp(self.bed_sensor),
            self._read_temp(self.chamber_sensor),
        )

    def _capture_ref(self, reset_lag=False):
        tn, tb, tc = self._temps()
        self.ref_nozzle = tn
        self.ref_bed = tb
        if reset_lag:
            self.chamber_eff = tc
        self.ref_chamber = self.chamber_eff
        self.last_temp_time = self.reactor.monotonic()

    def _temp_event(self, eventtime):
        tn, tb, tc = self._temps()
        dt = eventtime - self.last_temp_time
        if dt < 0.:
            dt = 0.
        self.last_temp_time = eventtime
        if self.chamber_tau > 0.:
            k = min(dt / self.chamber_tau, 1.)
            self.chamber_eff += (tc - self.chamber_eff) * k
        else:
            self.chamber_eff = tc
        self._calc_adjust_from_temps(tn, tb, self.chamber_eff)
        return eventtime + 1.

    def _calc_adjust_from_temps(self, tn, tb, tc_eff):
        if not self.adjust_enable or self.calibrating:
            return
        adjust = -(
            self.nozzle_coeff * (tn - self.ref_nozzle)
            + self.bed_coeff * (tb - self.ref_bed)
            + self.chamber_coeff * (tc_eff - self.ref_chamber)
        )
        sign = 1. if adjust >= 0. else -1.
        if abs(adjust) > self.max_z_adjust:
            adjust = self.max_z_adjust * sign
        if abs(adjust - self.z_adjust_mm) > self.z_step_dist:
            self.z_adjust_mm = adjust

    def _is_calibrated(self):
        return (abs(self.nozzle_coeff) > 1e-7
                or abs(self.bed_coeff) > 1e-7
                or abs(self.chamber_coeff) > 1e-7)

    def get_status(self, eventtime):
        tn, tb, tc = self._temps()
        return {
            'enabled': self.adjust_enable,
            'calibrated': self._is_calibrated(),
            'nozzle_coeff': self.nozzle_coeff,
            'bed_coeff': self.bed_coeff,
            'chamber_coeff': self.chamber_coeff,
            'chamber_tau': self.chamber_tau,
            'current_z_adjust': round(self.z_adjust_mm, 5),
            'ref_nozzle_temp': round(self.ref_nozzle, 2),
            'ref_bed_temp': round(self.ref_bed, 2),
            'ref_chamber_temp': round(self.ref_chamber, 2),
            'nozzle_temp': round(tn, 2),
            'bed_temp': round(tb, 2),
            'chamber_temp': round(tc, 2),
            'chamber_eff_temp': round(self.chamber_eff, 2),
            'samples': len(self.samples),
        }

    def calc_adjust(self, pos):
        if pos[2] < self.off_above_z:
            tn, tb, tc = self._temps()
            self._calc_adjust_from_temps(tn, tb, self.chamber_eff)
        new_z = pos[2] + self.z_adjust_mm
        self.last_z_adjust_mm = self.z_adjust_mm
        return [pos[0], pos[1], new_z] + pos[3:]

    def calc_unadjust(self, pos):
        return [pos[0], pos[1], pos[2] - self.z_adjust_mm] + pos[3:]

    def get_position(self):
        position = self.calc_unadjust(self.next_transform.get_position())
        self.last_position = self.calc_adjust(position)
        return position

    def move(self, newpos, speed):
        if (newpos[0:2] == self.last_position[0:2]) or not self.adjust_enable \
                or self.calibrating:
            z = newpos[2] + self.last_z_adjust_mm
            adjusted_pos = [newpos[0], newpos[1], z, newpos[3]]
            self.next_transform.move(adjusted_pos, speed)
        else:
            self.next_transform.move(self.calc_adjust(newpos), speed)
        self.last_position[:] = newpos

    def _status_msg(self):
        st = self.get_status(self.reactor.monotonic())
        state = '1 (enabled)' if st['enabled'] else '0 (disabled)'
        cal = 'yes' if st['calibrated'] else 'no'
        return (
            "enable: %s\n"
            "calibrated: %s\n"
            "nozzle_coeff: %.8f mm/degC\n"
            "bed_coeff: %.8f mm/degC\n"
            "chamber_coeff: %.8f mm/degC\n"
            "chamber_tau: %.1f s\n"
            "ref: nozzle=%.1f bed=%.1f chamber_eff=%.1f\n"
            "now: nozzle=%.1f bed=%.1f chamber=%.1f chamber_eff=%.1f\n"
            "applied Z adjustment: %.4f mm"
            % (state, cal,
               st['nozzle_coeff'], st['bed_coeff'], st['chamber_coeff'],
               st['chamber_tau'],
               st['ref_nozzle_temp'], st['ref_bed_temp'],
               st['ref_chamber_temp'],
               st['nozzle_temp'], st['bed_temp'], st['chamber_temp'],
               st['chamber_eff_temp'], st['current_z_adjust'])
        )

    cmd_SET_THERMAL_Z_help = "Set or query thermal Z compensation"
    def cmd_SET_THERMAL_Z(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        kn = gcmd.get_float('NOZZLE_COEFF', None, minval=-1., maxval=1.)
        kb = gcmd.get_float('BED_COEFF', None, minval=-1., maxval=1.)
        kc = gcmd.get_float('CHAMBER_COEFF', None, minval=-1., maxval=1.)
        tau = gcmd.get_float('CHAMBER_TAU', None, minval=1.)
        if kn is not None:
            self.nozzle_coeff = kn
        if kb is not None:
            self.bed_coeff = kb
        if kc is not None:
            self.chamber_coeff = kc
        if tau is not None:
            self.chamber_tau = tau
        if enable is not None and bool(enable) != self.adjust_enable:
            self.adjust_enable = bool(enable)
            gcode_move = self.printer.lookup_object('gcode_move')
            gcode_move.reset_last_position()
        capture = gcmd.get_int('CAPTURE_REF', 0, minval=0, maxval=1)
        if capture:
            self._capture_ref(reset_lag=False)
            self.z_adjust_mm = 0.
            self.last_z_adjust_mm = 0.
            gcode_move = self.printer.lookup_object('gcode_move')
            gcode_move.reset_last_position()
        gcmd.respond_info(self._status_msg())

    cmd_THERMAL_Z_STATUS_help = "Report thermal Z compensation state"
    def cmd_THERMAL_Z_STATUS(self, gcmd):
        gcmd.respond_info(self._status_msg())

    def _heater_name(self, sensor_name):
        if sensor_name.startswith('heater_generic '):
            return sensor_name.split(' ', 1)[1]
        return sensor_name

    def _set_heater(self, sensor_name, target):
        if not sensor_name:
            return
        heater = self._heater_name(sensor_name)
        self.gcode.run_script_from_command(
            "SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.1f" % (heater, target))

    def _wait_heater(self, sensor_name, target, band=1.):
        if not sensor_name:
            return
        lo = max(0., target - band)
        self.gcode.run_script_from_command(
            "TEMPERATURE_WAIT SENSOR=\"%s\" MINIMUM=%.1f" % (sensor_name, lo))

    def _move_xy(self, x, y):
        self.gcode.run_script_from_command(
            "G90\nG1 Z%.2f F600\nG1 X%.2f Y%.2f F%.0f"
            % (self.park_z, x, y, self.probe_speed * 60.))

    def _ensure_homed(self):
        th = self.printer.lookup_object('toolhead')
        homed = th.get_status(self.reactor.monotonic()).get('homed_axes', '')
        if not set(homed) >= set('xyz'):
            self.gcode.run_script_from_command("G28")

    def _probe_xy(self):
        kin = self.toolhead.get_status(self.reactor.monotonic())
        axis_min = kin['axis_minimum']
        axis_max = kin['axis_maximum']
        x = self.calibrate_x
        y = self.calibrate_y
        if x is None:
            x = 0.5 * (axis_min[0] + axis_max[0])
        if y is None:
            y = 0.5 * (axis_min[1] + axis_max[1])
        return x, y

    def _probe_once(self):
        probe = self.printer.lookup_object('probe')
        self.gcode.run_script_from_command("PROBE")
        eventtime = self.reactor.monotonic()
        z = probe.get_status(eventtime)['last_z_result']
        self.gcode.run_script_from_command("G1 Z%.2f F600" % (self.park_z,))
        return float(z)

    def _record(self, phase, z, extra=None):
        tn, tb, tc = self._temps()
        sample = {
            'phase': phase,
            'z': z,
            'nozzle': tn,
            'bed': tb,
            'chamber': tc,
            'time': self.reactor.monotonic(),
        }
        if extra:
            sample.update(extra)
        self.samples.append(sample)
        self.gcode.respond_info(
            "thermal_z sample %s: Z=%.5f Tn=%.1f Tb=%.1f Tc=%.1f"
            % (phase, z, tn, tb, tc))
        return sample

    def _probe_stable(self, phase):
        deadline = self.reactor.monotonic() + self.stable_max_time
        prev = None
        last_change = self.reactor.monotonic()
        z = None
        while True:
            z = self._probe_once()
            self._record(phase, z)
            now = self.reactor.monotonic()
            if prev is not None and abs(z - prev) <= self.stable_z:
                if now - last_change >= self.stable_interval:
                    return z
            else:
                last_change = now
            prev = z
            if now >= deadline:
                self.gcode.respond_info(
                    "thermal_z: Z did not fully stabilize, using last sample")
                return z
            remain = min(self.stable_interval, deadline - now)
            self.gcode.run_script_from_command("G4 P%.0f" % (remain * 1000.,))

    cmd_THERMAL_Z_CALIBRATE_help = (
        "Heat nozzle/bed/chamber and probe Z drift of this printer")
    def cmd_THERMAL_Z_CALIBRATE(self, gcmd):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            state = print_stats.get_status(self.reactor.monotonic())['state']
            if state == 'printing':
                raise gcmd.error("Cannot calibrate thermal Z while printing")
        if self.printer.lookup_object('probe', None) is None:
            raise gcmd.error("thermal_z_comp requires [probe]")

        phase = gcmd.get('PHASE', 'all').strip().lower()
        if phase not in ('all', 'nozzle', 'bed', 'chamber'):
            raise gcmd.error("PHASE must be all, nozzle, bed or chamber")

        nozzle_temps = _parse_temps(
            gcmd, 'NOZZLE_TEMPS',
            "%.0f,230" % (self.calibrate_nozzle_temp,))
        bed_temps = _parse_temps(gcmd, 'BED_TEMPS', '60,100')
        chamber_temps = _parse_temps(gcmd, 'CHAMBER_TEMPS', '40,70')
        bed_hold = gcmd.get_float('BED_HOLD', self.calibrate_bed_temp)
        nozzle_hold = gcmd.get_float('NOZZLE_HOLD',
                                     self.calibrate_nozzle_temp)
        soak = gcmd.get_float('SOAK', self.soak_time, minval=0.)
        clear = gcmd.get_int('CLEAR', 1, minval=0, maxval=1)
        if clear:
            self.samples = []

        was_enabled = self.adjust_enable
        self.calibrating = True
        self.adjust_enable = False
        self.z_adjust_mm = 0.
        self.last_z_adjust_mm = 0.
        gcode_move = self.printer.lookup_object('gcode_move')
        gcode_move.reset_last_position()
        try:
            self._ensure_homed()
            x, y = self._probe_xy()
            self._move_xy(x, y)
            gcmd.respond_info(
                "thermal_z_comp: calibrate at X=%.1f Y=%.1f, keep the nozzle"
                " clean. Unload filament if it oozes." % (x, y))
            if phase in ('all', 'nozzle'):
                self._run_nozzle_phase(gcmd, nozzle_temps, bed_hold, soak)
            if phase in ('all', 'bed'):
                self._run_bed_phase(gcmd, bed_temps, nozzle_hold, soak)
            if phase in ('all', 'chamber'):
                if not self.chamber_sensor:
                    raise gcmd.error("chamber_sensor is not configured")
                self._run_chamber_phase(gcmd, chamber_temps, nozzle_hold,
                                        bed_hold, soak)
            self._fit_and_store(gcmd)
        finally:
            self.calibrating = False
            self.adjust_enable = was_enabled
            self._capture_ref(reset_lag=False)
            self.z_adjust_mm = 0.
            self.last_z_adjust_mm = 0.
            gcode_move.reset_last_position()

    def _run_nozzle_phase(self, gcmd, temps, bed_hold, soak):
        gcmd.respond_info("thermal_z: nozzle sweep %s" % (temps,))
        self._set_heater(self.bed_sensor, bed_hold)
        self._wait_heater(self.bed_sensor, bed_hold, band=2.)
        if self.chamber_sensor:
            self._set_heater(self.chamber_sensor, 0.)
        for t in temps:
            self._set_heater(self.nozzle_sensor, t)
            self._wait_heater(self.nozzle_sensor, t, band=3.)
            if soak > 0.:
                self.gcode.run_script_from_command(
                    "G4 P%.0f" % (soak * 1000.,))
            z1 = self._probe_once()
            z2 = self._probe_once()
            z = 0.5 * (z1 + z2)
            if abs(z1 - z2) > 0.04:
                gcmd.respond_info(
                    "thermal_z: nozzle samples differ by %.3f mm at %.0fC"
                    " (ooze?). Using average." % (abs(z1 - z2), t))
            self._record('nozzle', z)

    def _run_bed_phase(self, gcmd, temps, nozzle_hold, soak):
        gcmd.respond_info("thermal_z: bed sweep %s" % (temps,))
        self._set_heater(self.nozzle_sensor, nozzle_hold)
        self._wait_heater(self.nozzle_sensor, nozzle_hold, band=3.)
        for t in temps:
            self._set_heater(self.bed_sensor, t)
            self._wait_heater(self.bed_sensor, t, band=2.)
            if soak > 0.:
                self.gcode.run_script_from_command(
                    "G4 P%.0f" % (soak * 1000.,))
            self._probe_stable('bed')

    def _run_chamber_phase(self, gcmd, temps, nozzle_hold, bed_hold, soak):
        gcmd.respond_info("thermal_z: chamber sweep %s" % (temps,))
        self._set_heater(self.nozzle_sensor, nozzle_hold)
        self._set_heater(self.bed_sensor, bed_hold)
        self._wait_heater(self.nozzle_sensor, nozzle_hold, band=3.)
        self._wait_heater(self.bed_sensor, bed_hold, band=2.)
        for t in temps:
            self._set_heater(self.chamber_sensor, t)
            self._wait_heater(self.chamber_sensor, t, band=2.)
            if soak > 0.:
                self.gcode.run_script_from_command(
                    "G4 P%.0f" % (min(soak, 60.) * 1000.,))
            self._probe_stable('chamber')

    def _fit_and_store(self, gcmd):
        kn = self._fit_phase('nozzle', 'nozzle')
        kb = self._fit_phase('bed', 'bed')
        kc, tau = self._fit_chamber()
        if kn is not None:
            self.nozzle_coeff = kn
        if kb is not None:
            self.bed_coeff = kb
        if kc is not None:
            self.chamber_coeff = kc
        if tau is not None:
            self.chamber_tau = tau
        cfg = self.printer.lookup_object('configfile')
        cfg.set(self.name, 'nozzle_coeff', "%.8f" % (self.nozzle_coeff,))
        cfg.set(self.name, 'bed_coeff', "%.8f" % (self.bed_coeff,))
        cfg.set(self.name, 'chamber_coeff', "%.8f" % (self.chamber_coeff,))
        cfg.set(self.name, 'chamber_tau', "%.1f" % (self.chamber_tau,))
        gcmd.respond_info(
            "thermal_z_comp fitted:\n"
            "  nozzle_coeff=%.8f mm/degC\n"
            "  bed_coeff=%.8f mm/degC\n"
            "  chamber_coeff=%.8f mm/degC\n"
            "  chamber_tau=%.1f s\n"
            "SAVE_CONFIG to store. Convention matches [z_thermal_adjust]:"
            " a negative coeff lifts the nozzle as that temperature rises."
            % (self.nozzle_coeff, self.bed_coeff, self.chamber_coeff,
               self.chamber_tau))

    def _fit_phase(self, phase, key):
        rows = [s for s in self.samples if s['phase'] == phase]
        if len(rows) < 2:
            return None
        xs = [s[key] for s in rows]
        zs = [s['z'] for s in rows]
        if max(xs) - min(xs) < 3.:
            return None
        return _slope(xs, zs)

    def _fit_chamber(self):
        rows = [s for s in self.samples if s['phase'] == 'chamber']
        if len(rows) < 2:
            return None, None
        xs = [s['chamber'] for s in rows]
        zs = [s['z'] for s in rows]
        kc = None
        if max(xs) - min(xs) >= 3.:
            kc = _slope(xs, zs)
        tau = self._estimate_tau(rows)
        return kc, tau

    def _estimate_tau(self, rows):
        if len(rows) < 3:
            return None
        t0 = rows[0]['time']
        z0 = rows[0]['z']
        z1 = rows[-1]['z']
        dz = z1 - z0
        if abs(dz) < 0.01:
            return None
        target = z0 + 0.63 * dz
        best_t = None
        best_err = 1e9
        for s in rows[1:]:
            err = abs(s['z'] - target)
            if err < best_err:
                best_err = err
                best_t = s['time'] - t0
        if best_t is None or best_t < 15.:
            return None
        return max(30., min(best_t, 1800.))

    cmd_THERMAL_Z_FIT_help = "Refit coefficients from the last calibration run"
    def cmd_THERMAL_Z_FIT(self, gcmd):
        if len(self.samples) < 2:
            raise gcmd.error("No calibration samples")
        self._fit_and_store(gcmd)


def load_config(config):
    return ThermalZComp(config)
