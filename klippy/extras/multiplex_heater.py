# Multiplex heater: one public heater backed by multiple segments
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class SectionNameWrapper:
    def __init__(self, config, section_name):
        self._config = config
        self._section_name = section_name

    def get_name(self):
        return self._section_name

    def __getattr__(self, name):
        return getattr(self._config, name)


class MultiplexHeater:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.name = config.get_name().split()[-1]
        self.min_temp = config.getfloat('min_temp', minval=-273.15)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.gcode_id = config.get('gcode_id', None)
        self.segment_heaters = []
        self.active_indices = []
        self.target_temp = 0.
        self._registered = False
        # Segments call add_segment() as their sections load.
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def add_segment(self, segment_config):
        parent = segment_config.get('multiplex_heater')
        if parent != self.name:
            raise segment_config.error(
                "Segment %s belongs to multiplex_heater '%s', not '%s'"
                % (segment_config.get_name(), parent, self.name))
        pheaters = self.printer.load_object(segment_config, 'heaters')
        seg_short = segment_config.get_name().split()[-1]
        private_name = "_" + seg_short
        wrapped = SectionNameWrapper(
            segment_config, "multiplex_heater_segment %s" % (private_name,))
        heater = pheaters.setup_heater(wrapped)
        # Keep segments out of the public heater/sensor lists (UI).
        full_name = wrapped.get_name()
        if full_name in pheaters.available_heaters:
            pheaters.available_heaters.remove(full_name)
        if full_name in pheaters.available_sensors:
            pheaters.available_sensors.remove(full_name)
        self.segment_heaters.append(heater)
        self.active_indices = list(range(len(self.segment_heaters)))
        if not self._registered:
            self._register_facade(pheaters)
        return heater

    def _register_facade(self, pheaters):
        if self.name in pheaters.heaters:
            raise self.config.error(
                "Heater %s already registered" % (self.name,))
        pheaters.heaters[self.name] = self
        pheaters.available_heaters.append(self.name)
        # Z-Bolt / older Klipper: TEMPERATURE_WAIT is a single global command
        # that resolves sensors via available_sensors + heaters dict.
        # Upstream mux registration must not be used here.
        pheaters.available_sensors.append(self.name)
        if self.gcode_id is not None:
            if self.gcode_id in pheaters.gcode_id_to_sensor:
                raise self.config.error(
                    "G-Code sensor id %s already registered" % (self.gcode_id,))
            pheaters.gcode_id_to_sensor[self.gcode_id] = self
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE", "HEATER", self.name,
            self.cmd_SET_HEATER_TEMPERATURE,
            desc=self.cmd_SET_HEATER_TEMPERATURE_help)
        gcode.register_mux_command(
            "SET_MULTIPLEX_HEATER", "HEATER", self.name,
            self.cmd_SET_MULTIPLEX_HEATER,
            desc=self.cmd_SET_MULTIPLEX_HEATER_help)
        # Alias for macros: printer.heater_bed / printer.chamber
        if self.name not in self.printer.objects:
            self.printer.add_object(self.name, self)
        if self.name == 'heater_bed':
            gcode.register_command("M140", self.cmd_M140)
            gcode.register_command("M190", self.cmd_M190)
        self._registered = True

    def _handle_ready(self):
        if not self.segment_heaters:
            raise self.printer.config_error(
                "multiplex_heater %s has no multiplex_heater_segment sections"
                % (self.name,))
        if not self._registered:
            pheaters = self.printer.lookup_object('heaters')
            self._register_facade(pheaters)

    def _parse_segments(self, gcmd):
        raw = gcmd.get('SEGMENTS', None)
        if raw is None:
            return list(range(len(self.segment_heaters)))
        indices = []
        for part in raw.split(','):
            part = part.strip()
            if not part:
                continue
            try:
                idx = int(part)
            except ValueError:
                raise gcmd.error("Invalid SEGMENTS entry '%s'" % (part,))
            if idx < 0 or idx >= len(self.segment_heaters):
                raise gcmd.error(
                    "SEGMENTS index %d out of range (0..%d)"
                    % (idx, len(self.segment_heaters) - 1))
            if idx not in indices:
                indices.append(idx)
        if not indices:
            raise gcmd.error("SEGMENTS must list at least one segment")
        return indices

    def _apply_targets(self):
        for i, heater in enumerate(self.segment_heaters):
            if i in self.active_indices:
                heater.set_temp(self.target_temp)
            else:
                heater.set_temp(0.)

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        self.target_temp = degrees
        if degrees:
            self._apply_targets()
        else:
            # Turn off every segment; keep active mask for the next heat.
            for heater in self.segment_heaters:
                heater.set_temp(0.)

    def get_temp(self, eventtime):
        if not self.active_indices:
            return 0., self.target_temp
        total = 0.
        for i in self.active_indices:
            temp, _target = self.segment_heaters[i].get_temp(eventtime)
            total += temp
        return total / len(self.active_indices), self.target_temp

    def check_busy(self, eventtime):
        for i in self.active_indices:
            if self.segment_heaters[i].check_busy(eventtime):
                return True
        return False

    def get_name(self):
        return self.name

    def get_status(self, eventtime):
        temp, target = self.get_temp(eventtime)
        power = 0.
        if self.active_indices:
            for i in self.active_indices:
                power += self.segment_heaters[i].get_status(eventtime)['power']
            power /= len(self.active_indices)
        return {
            'temperature': round(temp, 2),
            'target': target,
            'power': power,
            'active_segments': list(self.active_indices),
        }

    def stats(self, eventtime):
        temp, target = self.get_temp(eventtime)
        status = self.get_status(eventtime)
        is_active = target or temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.name, target, temp, status['power'])

    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp)

    cmd_SET_MULTIPLEX_HEATER_help = (
        "Set multiplex heater target and active segments")
    def cmd_SET_MULTIPLEX_HEATER(self, gcmd):
        self.active_indices = self._parse_segments(gcmd)
        temp = gcmd.get_float('TARGET', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp)

    def cmd_M140(self, gcmd, wait=False):
        temp = gcmd.get_float('S', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp, wait)

    def cmd_M190(self, gcmd):
        self.cmd_M140(gcmd, wait=True)


def load_config_prefix(config):
    return MultiplexHeater(config)
