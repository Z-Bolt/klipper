# Temperature-dependent PID profiles for heaters
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import heaters

PID_OPTION_PREFIX = 'pid_'


class AdaptivePID:
    def __init__(self, config):
        self.name = config.get_name()
        self.heater_name = self.name.split(' ', 1)[1]
        self.printer = config.get_printer()
        self.profiles = {}
        self.active_temp = None
        self._load_profiles_from_options(config)
        self._load_legacy_profile_sections(config)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("ADAPTIVE_PID_STATUS", "HEATER",
                                   self.heater_name,
                                   self.cmd_ADAPTIVE_PID_STATUS,
                                   desc=self.cmd_ADAPTIVE_PID_STATUS_help)
    def _load_profiles_from_options(self, config):
        for option in config.get_prefix_options(PID_OPTION_PREFIX):
            try:
                temp = float(option[len(PID_OPTION_PREFIX):])
            except ValueError:
                logging.info("adaptive_pid: ignoring invalid option '%s'",
                             option)
                continue
            Kp, Ki, Kd = config.getfloatlist(option, count=3)
            self.profiles[temp] = {'pid_Kp': Kp, 'pid_Ki': Ki, 'pid_Kd': Kd}
    def _load_legacy_profile_sections(self, config):
        # Support old autosave format: [adaptive_pid extruder 200]
        prefix = self.name + ' '
        for profile in config.get_prefix_sections(self.name):
            pname = profile.get_name()
            if pname == self.name or not pname.startswith(prefix):
                continue
            temp = profile.getfloat('temperature', None)
            if temp is None:
                continue
            self.profiles[temp] = {
                'pid_Kp': profile.getfloat('pid_Kp'),
                'pid_Ki': profile.getfloat('pid_Ki'),
                'pid_Kd': profile.getfloat('pid_Kd'),
            }
    def _pid_option_name(self, temp):
        return "%s%.0f" % (PID_OPTION_PREFIX, temp)
    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        heater = pheaters.lookup_heater(self.heater_name)
        if not isinstance(heater.control, heaters.ControlPID):
            raise self.printer.config_error(
                "adaptive_pid: heater '%s' must use PID control"
                % (self.heater_name,))
        pheaters.register_adaptive_pid(self.heater_name, self)
        logging.info("adaptive_pid: loaded %d profile(s) for %s",
                     len(self.profiles), self.heater_name)
    def _find_nearest_temp(self, target_temp):
        return min(self.profiles.keys(),
                   key=lambda t: abs(t - target_temp))
    def apply_for_temp(self, target_temp):
        if not target_temp or not self.profiles:
            return None
        cal_temp = self._find_nearest_temp(target_temp)
        if self.active_temp == cal_temp:
            return cal_temp
        prof = self.profiles[cal_temp]
        pheaters = self.printer.lookup_object('heaters')
        heater = pheaters.lookup_heater(self.heater_name)
        heater.control.set_params(prof['pid_Kp'], prof['pid_Ki'],
                                  prof['pid_Kd'])
        self.active_temp = cal_temp
        logging.info("adaptive_pid: %s target=%.1f using profile %.1f "
                     "(Kp=%.3f Ki=%.3f Kd=%.3f)",
                     self.heater_name, target_temp, cal_temp,
                     prof['pid_Kp'], prof['pid_Ki'], prof['pid_Kd'])
        return cal_temp
    def save_profile(self, temperature, Kp, Ki, Kd):
        temp = float(temperature)
        self.profiles[temp] = {
            'pid_Kp': Kp, 'pid_Ki': Ki, 'pid_Kd': Kd,
        }
        self.active_temp = None
        configfile = self.printer.lookup_object('configfile')
        option = self._pid_option_name(temp)
        configfile.set(self.name, option,
                       "%.3f, %.3f, %.3f" % (Kp, Ki, Kd))
        # Remove legacy per-temperature section if present
        legacy_section = "%s %.0f" % (self.name, temp)
        configfile.remove_section(legacy_section)
    def get_status(self, eventtime):
        profile_temps = sorted(self.profiles.keys())
        return {
            'heater': self.heater_name,
            'profiles': ["%.0f" % (t,) for t in profile_temps],
            'active_profile_temp': self.active_temp,
        }
    cmd_ADAPTIVE_PID_STATUS_help = "Report adaptive PID profiles for a heater"
    def cmd_ADAPTIVE_PID_STATUS(self, gcmd):
        profile_temps = sorted(self.profiles.keys())
        if not profile_temps:
            gcmd.respond_info("adaptive_pid %s: no profiles" % (self.heater_name,))
            return
        lines = ["adaptive_pid %s profiles:" % (self.heater_name,)]
        for temp in profile_temps:
            prof = self.profiles[temp]
            active = " (active)" if temp == self.active_temp else ""
            lines.append("  %.0fC: Kp=%.3f Ki=%.3f Kd=%.3f%s"
                         % (temp, prof['pid_Kp'], prof['pid_Ki'],
                            prof['pid_Kd'], active))
        gcmd.respond_info('\n'.join(lines))


def load_config_prefix(config):
    return AdaptivePID(config)
