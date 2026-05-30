# Calibration of heater PID settings
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import heaters

TUNING_METHODS = {
    'ziegler_nichols': (0.6, 0.5, 0.125),
    'tyreus_luyben': (0.45, 2.2, 1. / 6.3),
    'some_overshoot': (0.33, 0.5, 1. / 3.),
    'no_overshoot': (0.2, 0.5, 1. / 3.),
}

DEFAULT_TUNE = {
    'min_peaks': 12,
    'skip_peaks': 4,
    'sample_cycles': 3,
    'tuning': 'ziegler_nichols',
    'tune_delta': 5.0,
    'amplitude_tolerance': 0.25,
}


class PIDCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.defaults = self._load_tune_defaults(config)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('PID_CALIBRATE', self.cmd_PID_CALIBRATE,
                               desc=self.cmd_PID_CALIBRATE_help)
    def _load_tune_defaults(self, config):
        defaults = dict(DEFAULT_TUNE)
        if not config.has_section('pid_calibrate'):
            return defaults
        pc = config.getsection('pid_calibrate')
        defaults['min_peaks'] = pc.getint(
            'min_peaks', defaults['min_peaks'], minval=8)
        defaults['skip_peaks'] = pc.getint(
            'skip_peaks', defaults['skip_peaks'], minval=0)
        defaults['sample_cycles'] = pc.getint(
            'sample_cycles', defaults['sample_cycles'], minval=1)
        defaults['tuning'] = pc.getchoice(
            'tuning', TUNING_METHODS, defaults['tuning'])
        defaults['tune_delta'] = pc.getfloat(
            'tune_delta', defaults['tune_delta'], above=0.)
        defaults['amplitude_tolerance'] = pc.getfloat(
            'amplitude_tolerance', defaults['amplitude_tolerance'],
            minval=0., maxval=1.)
        return defaults
    def _get_tune_options(self, gcmd):
        defaults = self.defaults
        tuning = gcmd.get('TUNING', defaults['tuning']).lower()
        if tuning not in TUNING_METHODS:
            raise gcmd.error("Unknown TUNING '%s'" % (tuning,))
        return {
            'min_peaks': gcmd.get_int('MIN_PEAKS', defaults['min_peaks'],
                                      minval=8),
            'skip_peaks': gcmd.get_int('SKIP_PEAKS', defaults['skip_peaks'],
                                       minval=0),
            'sample_cycles': gcmd.get_int(
                'SAMPLE_CYCLES', defaults['sample_cycles'], minval=1),
            'tuning': tuning,
            'tune_delta': gcmd.get_float('TUNE_DELTA', defaults['tune_delta'],
                                         above=0.),
            'amplitude_tolerance': gcmd.get_float(
                'AMPLITUDE_TOLERANCE', defaults['amplitude_tolerance'],
                minval=0., maxval=1.),
        }
    cmd_PID_CALIBRATE_help = "Run PID calibration test"
    def cmd_PID_CALIBRATE(self, gcmd):
        heater_name = gcmd.get('HEATER')
        target = gcmd.get_float('TARGET')
        write_file = gcmd.get_int('WRITE_FILE', 0)
        tune_opts = self._get_tune_options(gcmd)
        adaptive_name = 'adaptive_pid %s' % (heater_name,)
        adaptive = self.printer.lookup_object(adaptive_name, None)
        save_adaptive = gcmd.get_int('ADAPTIVE', adaptive is not None)
        pheaters = self.printer.lookup_object('heaters')
        try:
            heater = pheaters.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))
        self.printer.lookup_object('toolhead').get_last_move_time()
        calibrate = ControlAutoTune(heater, target, **tune_opts)
        old_control = heater.set_control(calibrate)
        try:
            pheaters.set_temperature(heater, target, True)
        except self.printer.command_error as e:
            heater.set_control(old_control)
            raise
        heater.set_control(old_control)
        if write_file:
            calibrate.write_file('/tmp/heattest.txt')
        if calibrate.check_busy(0., 0., 0.):
            raise gcmd.error("pid_calibrate interrupted")
        # Log and report results
        Kp, Ki, Kd = calibrate.calc_final_pid()
        logging.info("Autotune: final: Kp=%f Ki=%f Kd=%f", Kp, Ki, Kd)
        tune_info = ("tuning=%s cycles=%d/%d"
                     % (tune_opts['tuning'], calibrate.sample_count,
                        calibrate.stable_cycle_count))
        if save_adaptive:
            if adaptive is None:
                raise gcmd.error(
                    "ADAPTIVE=1 requires [adaptive_pid %s] in config"
                    % (heater_name,))
            adaptive.save_profile(target, Kp, Ki, Kd)
            gcmd.respond_info(
                "PID parameters for %.0fC: pid_Kp=%.3f pid_Ki=%.3f "
                "pid_Kd=%.3f (%s)\n"
                "Saved to adaptive_pid profile. The SAVE_CONFIG command "
                "will update the printer config file and restart the "
                "printer." % (target, Kp, Ki, Kd, tune_info))
            return
        gcmd.respond_info(
            "PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f (%s)\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer."
            % (Kp, Ki, Kd, tune_info))
        # Store results for SAVE_CONFIG
        cfgname = heater.get_name()
        configfile = self.printer.lookup_object('configfile')
        configfile.set(cfgname, 'control', 'pid')
        configfile.set(cfgname, 'pid_Kp', "%.3f" % (Kp,))
        configfile.set(cfgname, 'pid_Ki', "%.3f" % (Ki,))
        configfile.set(cfgname, 'pid_Kd', "%.3f" % (Kd,))


class ControlAutoTune:
    def __init__(self, heater, target, min_peaks=12, skip_peaks=4,
                 sample_cycles=3, tuning='ziegler_nichols', tune_delta=5.,
                 amplitude_tolerance=0.25):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.calibrate_temp = target
        self.min_peaks = min_peaks
        self.skip_peaks = skip_peaks
        self.sample_cycles = sample_cycles
        self.tuning = tuning
        self.tune_delta = tune_delta
        self.amplitude_tolerance = amplitude_tolerance
        self.sample_count = 0
        self.stable_cycle_count = 0
        # Heating control
        self.heating = False
        self.peak = 0.
        self.peak_time = 0.
        # Peak recording
        self.peaks = []
        # Sample recording
        self.last_pwm = 0.
        self.pwm_samples = []
        self.temp_samples = []
    # Heater control
    def set_pwm(self, read_time, value):
        if value != self.last_pwm:
            self.pwm_samples.append(
                (read_time + self.heater.get_pwm_delay(), value))
            self.last_pwm = value
        self.heater.set_pwm(read_time, value)
    def temperature_update(self, read_time, temp, target_temp):
        self.temp_samples.append((read_time, temp))
        # Check if the temperature has crossed the target and
        # enable/disable the heater if so.
        if self.heating and temp >= target_temp:
            self.heating = False
            self.check_peaks()
            self.heater.alter_target(self.calibrate_temp - self.tune_delta)
        elif not self.heating and temp <= target_temp:
            self.heating = True
            self.check_peaks()
            self.heater.alter_target(self.calibrate_temp)
        # Check if this temperature is a peak and record it if so
        if self.heating:
            self.set_pwm(read_time, self.heater_max_power)
            if temp < self.peak:
                self.peak = temp
                self.peak_time = read_time
        else:
            self.set_pwm(read_time, 0.)
            if temp > self.peak:
                self.peak = temp
                self.peak_time = read_time
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        if self.heating or len(self.peaks) < self.min_peaks:
            return True
        return False
    # Analysis
    def check_peaks(self):
        self.peaks.append((self.peak, self.peak_time))
        if self.heating:
            self.peak = 9999999.
        else:
            self.peak = -9999999.
        if len(self.peaks) < 4:
            return
        self.calc_pid(len(self.peaks)-1)
    def _cycle_period(self, pos):
        return self.peaks[pos][1] - self.peaks[pos-2][1]
    def _cycle_amplitude(self, pos):
        return .5 * abs(self.peaks[pos][0] - self.peaks[pos-1][0])
    def _stable_cycle_positions(self):
        min_pos = self.skip_peaks + 2
        if len(self.peaks) <= min_pos:
            return []
        positions = list(range(min_pos, len(self.peaks)))
        amplitudes = [self._cycle_amplitude(pos) for pos in positions]
        median_amp = sorted(amplitudes)[len(amplitudes) // 2]
        if median_amp <= 0.:
            stable = positions
        else:
            tol = self.amplitude_tolerance
            stable = [pos for pos in positions
                      if abs(self._cycle_amplitude(pos) - median_amp)
                      / median_amp <= tol]
            if not stable:
                stable = positions
        periods = [(self._cycle_period(pos), pos) for pos in stable]
        median_period = sorted(p for p, _ in periods)[len(periods) // 2]
        stable.sort(key=lambda pos: abs(
            self._cycle_period(pos) - median_period))
        return stable
    def calc_pid(self, pos):
        temp_diff = self.peaks[pos][0] - self.peaks[pos-1][0]
        time_diff = self._cycle_period(pos)
        # Use Astrom-Hagglund method to estimate Ku and Tu
        amplitude = .5 * abs(temp_diff)
        Ku = 4. * self.heater_max_power / (math.pi * amplitude)
        Tu = time_diff
        Kp, Ki, Kd = self._pid_from_ku_tu(Ku, Tu)
        logging.info("Autotune: raw=%f/%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f",
                     temp_diff, self.heater_max_power, Ku, Tu, Kp, Ki, Kd)
        return Kp, Ki, Kd
    def _pid_from_ku_tu(self, Ku, Tu):
        kp_mult, ti_mult, td_mult = TUNING_METHODS[self.tuning]
        Kp = kp_mult * Ku * heaters.PID_PARAM_BASE
        Ti = ti_mult * Tu
        Td = td_mult * Tu
        Ki = Kp / Ti if Ti else 0.
        Kd = Kp * Td
        return Kp, Ki, Kd
    def calc_final_pid(self):
        stable = self._stable_cycle_positions()
        if not stable:
            return self.calc_pid(len(self.peaks) - 1)
        self.stable_cycle_count = len(stable)
        self.sample_count = min(self.sample_cycles, len(stable))
        selected = stable[:self.sample_count]
        results = [self.calc_pid(pos) for pos in selected]
        n = float(len(results))
        Kp = sum(r[0] for r in results) / n
        Ki = sum(r[1] for r in results) / n
        Kd = sum(r[2] for r in results) / n
        logging.info(
            "Autotune: tuning=%s averaged %d/%d stable cycles: "
            "Kp=%f Ki=%f Kd=%f",
            self.tuning, self.sample_count, self.stable_cycle_count,
            Kp, Ki, Kd)
        return Kp, Ki, Kd
    # Offline analysis helper
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
        f = open(filename, "w")
        f.write('\n'.join(pwm + out))
        f.close()

def load_config(config):
    return PIDCalibrate(config)
