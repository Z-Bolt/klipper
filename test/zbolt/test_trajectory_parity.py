# -*- coding: utf-8 -*-
"""Trajectory parity: the module must reproduce the old macros exactly.

Expected waypoints are hand-expanded from the pre-migration tools.cfg of each
model, substituting that model's toolchanger-settings.cfg values.  If a
number here disagrees with the real config, the migration changed machine
behaviour - which is exactly what must not happen.
"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8',
                              errors='replace')
import harness as H

F, S = 300.0, 60.0          # speed_fast 18000 / 60, speed_slow 3600 / 60

MAIN = {
    'sensor_settle_time': 0.05, 'travel_speed': F, 'dock_speed': S,
    'home_axes_order': 'y, x', 'initial_tool': 0,
    'apply_offset_gcode': '_SET_GCODE_OFFSET_INTERNAL X={x} Y={y} Z={z}',
}

MODELS = {}

# ---------------------------------------------------------------- S300 ----
MODELS['S300'] = {
    'main': dict(MAIN, force_speed_factor=100.),
    'tools': [
        {'tool_number': 0, 'extruder': 'extruder', 'dock_sensor_pin': '^!PA1',
         'park_xy': '8.8, -57.8', 'unpark_xy': '8.3, -57.8',
         'safe_xy': '63.8, -37.8', 'stage_xy': '63.8, 0',
         'exit_xy': '63.8, 0', 'retreat_speed': 'slow',
         'coupling': 'solenoid', 'solenoid_pin': 'sol', 'release_delay': 0.3,
         'current_boost': 1.5, 'current_boost_phase': 'get',
         'seat_offset_xy': '5, 15', 'seat_jerk': 5.,
         'offset_z_variable': 't0offset'},
        {'tool_number': 1, 'extruder': 'extruder1', 'dock_sensor_pin': '^!PB13',
         'park_xy': '292.5, -61.0', 'unpark_xy': '292.5, -61.0',
         'safe_xy': '237.5, -41.0', 'stage_xy': '237.5, 0',
         'exit_xy': '237.5, 0', 'retreat_speed': 'slow',
         'coupling': 'solenoid', 'solenoid_pin': 'sol', 'release_delay': 0.3,
         'current_boost': 1.5, 'current_boost_phase': 'get',
         'seat_offset_xy': '-5, 15', 'seat_jerk': 5.5,
         'offset_x_variable': 't1_x_offset',
         'offset_y_variable': 't1_y_offset',
         'offset_z_variable': 't1_z_offset'},
    ],
    'park': {
        0: [(63.8, 0, F), (63.8, -37.8, F), (63.8, -57.8, F),
            (8.8, -57.8, S), (8.3, -37.8, S)],
        1: [(237.5, 0, F), (237.5, -41.0, F), (237.5, -61.0, F),
            (292.5, -61.0, S), (292.5, -41.0, S)],
    },
    'pickup': {
        0: [(8.3, -37.8, F), (8.8, -57.8, S), (63.8, -57.8, S), (63.8, 0, F)],
        1: [(292.5, -41.0, F), (292.5, -61.0, S), (237.5, -61.0, S),
            (237.5, 0, F)],
    },
}

# ---------------------------------------------------------------- S310 ----
MODELS['S310'] = {
    'main': dict(MAIN, force_speed_factor=100.),
    'tools': [
        {'tool_number': 0, 'extruder': 'extruder', 'dock_sensor_pin': '^!PA1',
         'park_xy': '9.0, -61.0', 'unpark_xy': '9.0, -61.0',
         'safe_xy': '64.0, -41.0', 'stage_xy': '64.0, 0',
         'exit_xy': '64.0, 0', 'retreat_speed': 'slow',
         'coupling': 'solenoid', 'solenoid_pin': 'sol', 'release_delay': 0.3,
         'current_boost': 1.5, 'current_boost_phase': 'get',
         'seat_offset_xy': '5, 15', 'seat_jerk': 5.5,
         'offset_z_variable': 't0offset'},
        {'tool_number': 1, 'extruder': 'extruder1', 'dock_sensor_pin': '^!PB13',
         'park_xy': '292.5, -61.0', 'unpark_xy': '292.5, -61.0',
         'safe_xy': '237.5, -41.0', 'stage_xy': '237.5, 0',
         'exit_xy': '237.5, 0', 'retreat_speed': 'slow',
         'coupling': 'solenoid', 'solenoid_pin': 'sol', 'release_delay': 0.3,
         'current_boost': 1.5, 'current_boost_phase': 'get',
         'seat_offset_xy': '-5, 15', 'seat_jerk': 5.5,
         'offset_x_variable': 't1_x_offset',
         'offset_y_variable': 't1_y_offset',
         'offset_z_variable': 't1_z_offset'},
    ],
    'park': {
        0: [(64.0, 0, F), (64.0, -41.0, F), (64.0, -61.0, F),
            (9.0, -61.0, S), (9.0, -41.0, S)],
        1: [(237.5, 0, F), (237.5, -41.0, F), (237.5, -61.0, F),
            (292.5, -61.0, S), (292.5, -41.0, S)],
    },
    'pickup': {
        0: [(9.0, -41.0, F), (9.0, -61.0, S), (64.0, -61.0, S), (64.0, 0, F)],
        1: [(292.5, -41.0, F), (292.5, -61.0, S), (237.5, -61.0, S),
            (237.5, 0, F)],
    },
}

# ---------------------------------------------------------------- S400 ----
MODELS['S400'] = {
    'main': dict(MAIN, force_speed_factor=100.),
    'tools': [
        {'tool_number': 0, 'extruder': 'extruder', 'dock_sensor_pin': '^!PA1',
         'park_xy': '4.0, 455.1', 'unpark_xy': '4.0, 455.1',
         'safe_xy': '59.0, 430.1', 'retreat_speed': 'slow',
         'coupling': 'latch', 'release_jerk': 0.,
         'current_boost': 1.6, 'current_boost_phase': 'park',
         'wipe_moves': '\n0, 25\n-10, 25\n-10, 0\n0, 0\n',
         'offset_z_variable': 't0offset'},
        {'tool_number': 1, 'extruder': 'extruder1', 'dock_sensor_pin': '^!PA2',
         'park_xy': '416.4, 455.4', 'unpark_xy': '416.4, 455.4',
         'safe_xy': '361.4, 430.4', 'retreat_speed': 'slow',
         'coupling': 'latch', 'release_jerk': 0.,
         'current_boost': 1.6, 'current_boost_phase': 'park',
         'wipe_moves': '\n0, 25\n10, 25\n10, 0\n0, 0\n',
         'offset_x_variable': 't1_x_offset',
         'offset_y_variable': 't1_y_offset',
         'offset_z_variable': 't1_z_offset'},
    ],
    'park': {
        0: [(59.0, 430.1, F), (59.0, 455.1, F), (49.0, 455.1, F),
            (49.0, 430.1, F), (59.0, 430.1, F), (59.0, 455.1, F),
            (4.0, 455.1, S), (4.0, 430.1, S)],
        1: [(361.4, 430.4, F), (361.4, 455.4, F), (371.4, 455.4, F),
            (371.4, 430.4, F), (361.4, 430.4, F), (361.4, 455.4, F),
            (416.4, 455.4, S), (416.4, 430.4, S)],
    },
    'pickup': {
        0: [(4.0, 430.1, F), (4.0, 455.1, S), (59.0, 455.1, S)],
        1: [(416.4, 430.4, F), (416.4, 455.4, S), (361.4, 455.4, S)],
    },
}

# ---------------------------------------------------------------- S600 ----
MODELS['S600'] = {
    'main': dict(MAIN),
    'tools': [
        {'tool_number': 0, 'extruder': 'extruder', 'dock_sensor_pin': '^!PB14',
         'park_xy': '-10, 655.6', 'safe_xy': '190, 627',
         'approach_offset': 50., 'retreat_speed': 'fast',
         'exit_xy': '190, 642', 'coupling': 'latch', 'release_jerk': 0.,
         'wipe_moves': '\n0, 38.6\n0, 0\n',
         'offset_z_variable': 't0offset'},
        {'tool_number': 1, 'extruder': 'extruder1', 'dock_sensor_pin': '^!PA2',
         'park_xy': '600, 654.3', 'safe_xy': '404, 635',
         'approach_offset': -50., 'retreat_speed': 'fast',
         'exit_xy': '404, 650', 'coupling': 'latch', 'release_jerk': 0.,
         'wipe_moves': '\n0, 29.3\n0, 0\n',
         'offset_x_variable': 't1_x_offset',
         'offset_y_variable': 't1_y_offset',
         'offset_z_variable': 't1_z_offset'},
    ],
    'park': {
        0: [(190, 627, F), (190, 665.6, F), (190, 627, F), (190, 655.6, F),
            (40, 655.6, F), (-10, 655.6, S), (-10, 627, S)],
        1: [(404, 635, F), (404, 664.3, F), (404, 635, F), (404, 654.3, F),
            (550, 654.3, F), (600, 654.3, S), (600, 635, S)],
    },
    'pickup': {
        0: [(-10, 627, F), (-10, 655.6, S), (190, 655.6, F), (190, 642, F)],
        1: [(600, 635, F), (600, 654.3, S), (404, 654.3, F), (404, 650, F)],
    },
}

if __name__ == '__main__':
    results = []
    
    
    def run(model, action, number):
        spec = MODELS[model]
        printer, changer = H.build(spec['main'], spec['tools'])
        docked = {0: True, 1: True}
        if action == 'park':
            docked[number] = False
        H.ready(printer, changer, docked)
        printer.toolhead.homed = 'xyz'
        printer.toolhead.moves = []
        tool = changer.tools[number]
        if action == 'park':
            changer._run_park(tool)
        else:
            changer._run_pickup(tool)
        return [(round(x, 3), round(y, 3), s)
                for x, y, z, s in printer.toolhead.moves]
    
    
    for model in ('S300', 'S310', 'S400', 'S600'):
        for action in ('park', 'pickup'):
            for number in (0, 1):
                want = [(float(x), float(y), float(s))
                        for x, y, s in MODELS[model][action][number]]
                got = [(float(x), float(y), float(s)) for x, y, s in
                       run(model, action, number)]
                ok = got == want
                results.append(('%s %s T%d' % (model, action, number), ok))
                if not ok:
                    print('FAIL %s %s T%d' % (model, action, number))
                    print('  ожидалось: %s' % (want,))
                    print('  получено : %s' % (got,))
    
    bad = [r for r in results if not r[1]]
    print('\n%d/%d траекторий совпадают с исходными макросами'
          % (len(results) - len(bad), len(results)))
    sys.exit(1 if bad else 0)
