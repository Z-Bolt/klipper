# Named secondary Z-probes for multi-toolhead printers
#
# Stock [probe] stays primary for homing / bed mesh.  This extra does
# not register probe:z_virtual_endstop.
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

from . import probe


class NamedProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object('gcode')
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.param_helper = probe.ProbeParameterHelper(config)
        self.mcu_probe = probe.ProbeEndstopWrapper(
            config, self.probe_offsets, self.param_helper)
        self.probe_session = probe.SampleAveragingHelper(
            config, self.param_helper, self.mcu_probe.start_probe_session)
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

    def get_offsets(self, gcmd=None):
        return self.probe_offsets.get_offsets(gcmd)

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
            "named_probe %s: Result is z=%.6f" % (self.name, pos.bed_z))
        self.last_z_result = pos.bed_z

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
