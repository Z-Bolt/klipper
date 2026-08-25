# Named secondary Z-probes for multi-toolhead printers
#
# Compatible with Klipper probe.py that uses ProbeEndstopWrapper(config)
# and list positions [x, y, z] (pre-ProbeResult API).
#
# Install: copy or symlink into ~/klipper/klippy/extras/named_probe.py
# then FIRMWARE_RESTART. Stock [probe] stays primary for homing / bed mesh.
#
# Example:
#   [named_probe t1]
#   pin: PA2
#   z_offset: 0.0
#
#   PROBE_NAMED PROBE=t1
#   QUERY_PROBE_NAMED PROBE=t1
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import probe


class NamedProbeSession:
    """HomingViaProbeHelper-style probing without probe:z_virtual_endstop."""

    def __init__(self, config, mcu_probe, param_helper):
        self.printer = config.get_printer()
        self.mcu_probe = mcu_probe
        self.param_helper = param_helper
        self.multi_probe_pending = False
        self.z_min_position = probe.lookup_minimum_z(config)
        self.results = []
        probe.LookupZSteppers(config, self.mcu_probe.add_stepper)
        self.printer.register_event_handler(
            "homing:homing_move_begin", self._handle_homing_move_begin)
        self.printer.register_event_handler(
            "homing:homing_move_end", self._handle_homing_move_end)
        self.printer.register_event_handler(
            "homing:home_rails_begin", self._handle_home_rails_begin)
        self.printer.register_event_handler(
            "homing:home_rails_end", self._handle_home_rails_end)
        self.printer.register_event_handler(
            "gcode:command_error", self._handle_command_error)

    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)

    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)

    def _handle_home_rails_begin(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.mcu_probe.multi_probe_begin()
            self.multi_probe_pending = True

    def _handle_home_rails_end(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.multi_probe_pending and self.mcu_probe in endstops:
            self.multi_probe_pending = False
            self.mcu_probe.multi_probe_end()

    def _handle_command_error(self):
        if self.multi_probe_pending:
            self.multi_probe_pending = False
            try:
                self.mcu_probe.multi_probe_end()
            except Exception:
                logging.exception("NamedProbe multi-probe end")

    def start_probe_session(self, gcmd):
        self.mcu_probe.multi_probe_begin()
        self.results = []
        return self

    def run_probe(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        pos[2] = self.z_min_position
        speed = self.param_helper.get_probe_params(gcmd)['probe_speed']
        phoming = self.printer.lookup_object('homing')
        self.results.append(phoming.probing_move(self.mcu_probe, pos, speed))

    def pull_probed_results(self):
        res = self.results
        self.results = []
        return res

    def end_probe_session(self):
        self.results = []
        self.mcu_probe.multi_probe_end()


class NamedProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object('gcode')
        self.mcu_probe = probe.ProbeEndstopWrapper(config)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.param_helper = probe.ProbeParameterHelper(config)
        self.homing_helper = NamedProbeSession(
            config, self.mcu_probe, self.param_helper)
        self.probe_session = probe.ProbeSessionHelper(
            config, self.param_helper, self.homing_helper.start_probe_session)
        self.query_endstop = self.mcu_probe.query_endstop
        self.last_state = False
        self.last_z_result = 0.
        gcode.register_mux_command(
            'PROBE_NAMED', 'PROBE', self.name,
            self.cmd_PROBE_NAMED, desc=self.cmd_PROBE_NAMED_help)
        gcode.register_mux_command(
            'QUERY_PROBE_NAMED', 'PROBE', self.name,
            self.cmd_QUERY_PROBE_NAMED, desc=self.cmd_QUERY_PROBE_NAMED_help)

    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self):
        return self.probe_offsets.get_offsets()

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'last_query': self.last_state,
            'last_z_result': self.last_z_result,
        }

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    cmd_PROBE_NAMED_help = "Probe Z-height with a named secondary probe"
    def cmd_PROBE_NAMED(self, gcmd):
        pos = probe.run_single_probe(self, gcmd)
        gcmd.respond_info(
            "named_probe %s: Result is z=%.6f" % (self.name, pos[2]))
        self.last_z_result = pos[2]

    cmd_QUERY_PROBE_NAMED_help = "Return the status of a named secondary probe"
    def cmd_QUERY_PROBE_NAMED(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info(
            "named_probe %s: %s"
            % (self.name, ["open", "TRIGGERED"][not not res]))


def load_config_prefix(config):
    return NamedProbe(config)
