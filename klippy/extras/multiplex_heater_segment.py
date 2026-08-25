# Segment definition for multiplex_heater
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

def load_config_prefix(config):
    printer = config.get_printer()
    parent_name = config.get('multiplex_heater')
    section = 'multiplex_heater %s' % (parent_name,)
    parent = printer.load_object(config, section)
    return parent.add_segment(config)
