# Tool definition for zbolt_toolchanger
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

def load_config_prefix(config):
    printer = config.get_printer()
    parent = printer.load_object(config, 'zbolt_toolchanger')
    return parent.add_tool(config)
