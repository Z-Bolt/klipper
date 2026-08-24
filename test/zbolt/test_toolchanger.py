# -*- coding: utf-8 -*-
"""Behaviour tests for zbolt_toolchanger against the offline harness."""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8',
                              errors='replace')

import harness as H

RESULTS = []


def check(name, condition, detail=''):
    RESULTS.append((name, bool(condition), detail))


def moves_of(printer):
    return list(printer.toolhead.moves)


def simulate_mechanics(printer, changer):
    """Make the fake dock sensors follow the motion, like real hardware."""
    original_park, original_pickup = changer._run_park, changer._run_pickup

    def park(tool):
        original_park(tool)
        H.set_sensors(printer, changer, {n: True for n in changer.tools})

    def pickup(tool):
        original_pickup(tool)
        H.set_sensors(printer, changer,
                      {n: n != tool.number for n in changer.tools})

    changer._run_park, changer._run_pickup = park, pickup


# ----------------------------------------------------------------------
def test_startup_states():
    for docked, expect_tool, expect_state in (
            ({0: True, 1: True}, None, 'ok'),
            ({0: False, 1: True}, 0, 'ok'),
            ({0: True, 1: False}, 1, 'ok'),
            ({0: False, 1: False}, None, 'fault')):
        printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
        H.ready(printer, changer, docked)
        check('startup %s -> tool' % (docked,),
              changer.active_tool == expect_tool,
              'got %s want %s' % (changer.active_tool, expect_tool))
        check('startup %s -> state' % (docked,),
              changer.state == expect_state,
              'got %s want %s' % (changer.state, expect_state))
        if expect_state == 'fault':
            check('startup fault code', changer.fault == 'both_undocked',
                  changer.fault)
            check('startup fault reported',
                  any('!!' in o for o in printer.gcode.output),
                  str(printer.gcode.output))
        else:
            check('startup prints status',
                  any('Тулченджер' in o for o in printer.gcode.output),
                  str(printer.gcode.output))
        if expect_tool is not None:
            want = 'extruder' if expect_tool == 0 else 'extruder1'
            check('startup activates %s' % want,
                  printer.toolhead.extruder is not None
                  and printer.toolhead.extruder.get_name() == want)


# ----------------------------------------------------------------------
def test_no_blind_dwell():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.log = []
    printer.gcode.script = []
    simulate_mechanics(printer, changer)
    changer.park_tool(0)
    dwells = [e for e in printer.log if e.startswith('dwell')]
    check('park: no dwell on latch coupling', not dwells, str(dwells))
    check('park: syncs host to machine before sampling',
          'wait_moves' in printer.log, str(printer.log))
    check('park: no G4 in emitted script',
          not any(s.upper().startswith('G4') for s in printer.gcode.script),
          str(printer.gcode.script))


# ----------------------------------------------------------------------
def test_s600_park_trajectory():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.toolhead.moves = []
    simulate_mechanics(printer, changer)
    changer.park_tool(0)
    got = [(x, y, s) for x, y, z, s in moves_of(printer)]
    # Reproduces the old _PARK_EXTRUDER of S600 one for one.
    want = [(190.0, 627.0, 300.0),      # safe
            (190.0, 655.6, 300.0),      # wipe out
            (190.0, 627.0, 300.0),      # wipe back
            (190.0, 655.6, 300.0),      # align on dock row
            (40.0, 655.6, 300.0),       # approach offset +50
            (-10.0, 655.6, 60.0),       # dock, slow
            (-10.0, 627.0, 60.0)]       # retreat, slow
    check('S600 park trajectory', got == want, 'got %s' % (got,))


def test_s600_pickup_trajectory():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: True, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.toolhead.moves = []
    simulate_mechanics(printer, changer)
    changer.pickup_tool(0)
    got = [(x, y, s) for x, y, z, s in moves_of(printer)]
    want = [(-10.0, 627.0, 300.0),      # approach column
            (-10.0, 655.6, 60.0),       # dock, slow
            (190.0, 655.6, 300.0),      # retreat fast (retreat_speed: fast)
            (190.0, 642.0, 300.0)]      # exit
    check('S600 pickup trajectory', got == want, 'got %s' % (got,))


def test_s300_park_trajectory():
    printer, changer = H.build(H.S300_MAIN, [H.S300_T0, H.S300_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.toolhead.moves = []
    printer.gcode.script = []
    simulate_mechanics(printer, changer)
    changer.park_tool(0)
    got = [(x, y, s) for x, y, z, s in moves_of(printer)]
    want = [(63.8, 0.0, 300.0),         # stage at Y0
            (63.8, -37.8, 300.0),       # safe
            (63.8, -57.8, 300.0),       # align on dock row
            (8.8, -57.8, 60.0),         # dock, slow
            (8.3, -37.8, 60.0)]         # retreat, slow
    check('S300 park trajectory', got == want, 'got %s' % (got,))
    check('S300 park releases solenoid',
          'SET_PIN PIN=sol VALUE=0' in printer.gcode.script,
          str(printer.gcode.script))
    check('S300 park forces M220 S100',
          'M220 S100' in printer.gcode.script, str(printer.gcode.script))


def test_s300_pickup_boost():
    printer, changer = H.build(H.S300_MAIN, [H.S300_T0, H.S300_T1])
    H.ready(printer, changer, {0: True, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.gcode.script = []
    simulate_mechanics(printer, changer)
    changer.pickup_tool(0)
    script = printer.gcode.script
    check('S300 pickup arms solenoid before approach',
          'SET_PIN PIN=sol VALUE=1' in script, str(script))
    boosts = [s for s in script if s.startswith('SET_TMC_CURRENT')]
    check('S300 pickup boosts and restores X/Y current',
          boosts == ['SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1.800',
                     'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=1.800',
                     'SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1.200',
                     'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=1.200'],
          str(boosts))


# ----------------------------------------------------------------------
def test_tool_change_offsets():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.gcode.script = []

    simulate_mechanics(printer, changer)
    changer.select_tool(1)
    check('T1 activates extruder1',
          printer.toolhead.extruder.get_name() == 'extruder1')
    offsets = [s for s in printer.gcode.script
               if s.startswith('_SET_GCODE_OFFSET_INTERNAL')]
    check('T1 applies stored offsets last',
          offsets[-1] == '_SET_GCODE_OFFSET_INTERNAL X=0.1 Y=-0.2 Z=0.029',
          str(offsets))
    check('offsets go through the INTERNAL bypass, never SET_GCODE_OFFSET',
          not any(s.startswith('SET_GCODE_OFFSET ')
                  for s in printer.gcode.script), str(printer.gcode.script))


def test_babystep_capture():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    # Operator babystepped -0.05 while printing with T0.
    printer.objects['gcode_move'].homing_origin = H.Coord(0., 0., -0.05)
    changer._capture_babystep()
    saved = printer.objects['save_variables'].allVariables['z_adjust']
    check('babystep folded into offset_z', abs(saved + 0.05) < 1e-6,
          str(saved))


# ----------------------------------------------------------------------
def test_realtime_guard():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.gcode.output = []
    # A head drops off the carriage while the printer is idle.
    H.set_sensors(printer, changer, {0: True, 1: True}, via_callback=True)
    printer.reactor.run_pending()
    check('idle edge raises unexpected_edge',
          changer.fault == 'unexpected_edge', str(changer.fault))
    check('idle edge reported as error',
          any('!!' in o for o in printer.gcode.output),
          str(printer.gcode.output))
    check('idle edge requests abort',
          changer.abort_requested == 'unexpected_edge',
          str(changer.abort_requested))


def test_edges_ignored_during_change():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    changer.phase = H.tc.PHASE_PARK
    printer.gcode.output = []
    H.set_sensors(printer, changer, {0: True, 1: True}, via_callback=True)
    check('edges during a change are not faults', changer.fault is None,
          str(changer.fault))


def test_park_failure():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.gcode.output = []
    raised = None
    try:
        changer.park_tool(0)          # sensors never confirm the dock
    except H.CommandError as exc:
        raised = str(exc)
    check('park failure raises', raised is not None, str(raised))
    check('park failure code', changer.fault == 'park_failed',
          str(changer.fault))
    check('park failure retried once',
          sum('повтор' in o for o in printer.gcode.output) == 1,
          str(printer.gcode.output))


def test_pause_on_fault_while_printing():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.objects['print_stats'].state = 'printing'
    printer.gcode.script = []
    try:
        changer.park_tool(0)
    except H.CommandError:
        pass
    check('fault pauses an active print', 'PAUSE' in printer.gcode.script,
          str(printer.gcode.script))


def test_shutdown_reaction():
    main = dict(H.S600_MAIN, on_fault_both_undocked='shutdown')
    printer, changer = H.build(main, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: False})
    printer.gcode.output = []
    try:
        changer.cmd_TOOLCHANGER_STATUS(H.GCmd(printer))
    except H.CommandError:
        pass
    check('shutdown reaction is configurable',
          printer.shutdowns and 'вне парковок' in printer.shutdowns[0],
          str(printer.shutdowns))


# ----------------------------------------------------------------------
def test_homing_from_each_state():
    # Both docked -> pick up T0.
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: True, 1: True})
    simulate_mechanics(printer, changer)
    changer.cmd_TOOLCHANGER_HOME(H.GCmd(printer))
    check('homing from both-docked picks T0', changer.active_tool == 0,
          str(changer.active_tool))
    check('homing homes Z after a head is on the carriage',
          'G28 Z' in printer.gcode.script, str(printer.gcode.script))
    order = [s for s in printer.gcode.script if s.startswith('G28')]
    check('homing order Y, X, then Z', order == ['G28 Y', 'G28 X', 'G28 Z'],
          str(order))

    # Wrong tool on the carriage -> park it, take T0.
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: True, 1: False})
    simulate_mechanics(printer, changer)
    changer.cmd_TOOLCHANGER_HOME(H.GCmd(printer))
    check('homing parks the wrong tool and takes T0',
          changer.active_tool == 0, str(changer.active_tool))

    # Both undocked -> refuse, no motion.
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: False})
    printer.toolhead.moves = []
    raised = False
    try:
        changer.cmd_TOOLCHANGER_HOME(H.GCmd(printer))
    except H.CommandError:
        raised = True
    check('homing refuses when both heads are out', raised)
    check('homing does not move when both heads are out',
          not printer.toolhead.moves, str(printer.toolhead.moves))


def test_seat_recovery_s300():
    printer, changer = H.build(H.S300_MAIN, [H.S300_T0, H.S300_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.moves = []
    printer.gcode.script = []
    changer.cmd_TOOLCHANGER_HOME(H.GCmd(printer))
    check('S300 homing re-seats the carried head',
          (13.8, -42.8, 300.0) in [(x, y, s) for x, y, z, s in
                                   moves_of(printer)],
          str(moves_of(printer)))
    check('S300 seat energises the solenoid',
          'SET_PIN PIN=sol VALUE=1' in printer.gcode.script,
          str(printer.gcode.script))


# ----------------------------------------------------------------------
def test_status_exposes_geometry():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    status = changer.get_status(printer.reactor.monotonic())
    check('status exposes active tool', status['active_tool'] == 0)
    check('status exposes dock geometry for load-unload macros',
          status['tools']['t1']['park_x'] == 600.0
          and status['tools']['t0']['safe_y'] == 627.0, str(status['tools']))
    check('status exposes docked flags',
          status['tools']['t0']['docked'] is False
          and status['tools']['t1']['docked'] is True, str(status['tools']))


def test_commands_registered():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    for name in ('T0', 'T1', 'TOOLCHANGER_SELECT', 'TOOLCHANGER_PARK',
                 'TOOLCHANGER_PICKUP', 'TOOLCHANGER_HOME',
                 'TOOLCHANGER_STATUS', 'TOOLCHANGER_QUERY_SENSORS',
                 'TOOLCHANGER_SET_STATE', 'TOOLCHANGER_RELEASE',
                 'TOOLCHANGER_COUPLE', 'TOOLCHANGER_CALIBRATE_OFFSET'):
        check('command %s registered' % name, name in printer.gcode.commands)


def test_set_state_clears_fault():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: False})
    check('fault before manual override', changer.state == 'fault')
    changer.cmd_TOOLCHANGER_SET_STATE(H.GCmd(printer, {'TOOL': '1'}))
    check('SET_STATE clears the fault', changer.state == 'ok'
          and changer.active_tool == 1 and changer.fault is None)
    changer.cmd_TOOLCHANGER_SET_STATE(H.GCmd(printer, {'TOOL': 'NONE'}))
    check('SET_STATE NONE clears the active tool',
          changer.active_tool is None)


def test_slider_sensor_faults():
    main = dict(H.S600_MAIN, slider_sensor_pin='^PC0',
                slider_sensor_occupied_state='triggered')
    printer, changer = H.build(main, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    changer.slider_sensor.raw = False        # carriage reports empty
    changer.slider_sensor.last_change = 0.
    changer._derive_state()
    check('carriage sensor catches a lost head',
          changer.fault == 'head_lost', str(changer.fault))
    H.set_sensors(printer, changer, {0: True, 1: True})
    changer.slider_sensor.raw = True         # carriage reports occupied
    changer._derive_state()
    check('carriage sensor catches an unexpected load',
          changer.fault == 'slider_unexpected', str(changer.fault))





# --- сервисный режим и помощники настройки парковок ---------------------
def test_service_mode():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    check('по умолчанию сервисный режим выключен', not changer.is_service_mode())
    changer.cmd_TOOLCHANGER_SERVICE_MODE(H.GCmd(printer, {'ENABLE': '1'}))
    check('SERVICE_MODE=1 включает режим', changer.is_service_mode())
    check('режим виден в статусе',
          changer.get_status(printer.reactor.monotonic())['service_mode'])
    # аварии перестают блокировать
    printer.toolhead.homed = 'xyz'
    raised = None
    try:
        changer.park_tool(0)          # датчики не подтвердят парковку
    except H.CommandError as exc:
        raised = str(exc)
    check('в сервисном режиме авария не бросает исключение', raised is None,
          str(raised))
    changer.cmd_TOOLCHANGER_SERVICE_MODE(H.GCmd(printer, {'ENABLE': '0'}))
    check('SERVICE_MODE=0 выключает режим', not changer.is_service_mode())


def test_service_mode_skips_skew_hook():
    main = dict(H.S600_MAIN, before_change_gcode='UNSKEW',
                after_change_gcode='SKEW')
    printer, changer = H.build(main, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    simulate_mechanics(printer, changer)
    changer.select_tool(0)
    check('вне сервисного режима SKEW возвращается',
          'SKEW' in printer.gcode.script, str(printer.gcode.script))
    changer.cmd_TOOLCHANGER_SERVICE_MODE(H.GCmd(printer, {'ENABLE': '1'}))
    printer.gcode.script = []
    changer.select_tool(0)
    check('в сервисном режиме UNSKEW есть', 'UNSKEW' in printer.gcode.script)
    check('в сервисном режиме SKEW не включается обратно',
          'SKEW' not in printer.gcode.script, str(printer.gcode.script))


def test_service_mode_homing_does_not_grab():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: True, 1: True})
    changer.cmd_TOOLCHANGER_SERVICE_MODE(H.GCmd(printer, {'ENABLE': '1'}))
    printer.toolhead.moves = []
    printer.gcode.script = []
    changer.cmd_TOOLCHANGER_HOME(H.GCmd(printer))
    check('в сервисном режиме хоминг не забирает голову',
          changer.active_tool is None, str(changer.active_tool))
    check('без головы на каретке Z не хомится',
          'G28 Z' not in printer.gcode.script, str(printer.gcode.script))
    check('XY всё равно хомятся',
          [s for s in printer.gcode.script if s.startswith('G28')] ==
          ['G28 Y', 'G28 X'], str(printer.gcode.script))


def test_goto_and_set_dock():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    printer.toolhead.moves = []
    changer.cmd_TOOLCHANGER_GOTO(H.GCmd(printer, {'TOOL': '1',
                                                  'POINT': 'park'}))
    check('GOTO едет в точку парковки',
          moves_of(printer)[-1][:2] == (600.0, 654.3), str(moves_of(printer)))
    changer.cmd_TOOLCHANGER_SET_DOCK(H.GCmd(printer, {'TOOL': '1',
                                                      'POINT': 'park',
                                                      'X': '601.5',
                                                      'Y': '653.0'}))
    check('SET_DOCK правит координату в памяти',
          changer.tools[1].park_xy == [601.5, 653.0],
          str(changer.tools[1].park_xy))
    check('SET_DOCK пишет в configfile для SAVE_CONFIG',
          printer.objects['configfile'].sets ==
          [('zbolt_toolchanger_tool t1', 'park_xy', '601.500, 653.000')],
          str(printer.objects['configfile'].sets))


def test_late_initial_report_is_not_a_fault():
    """Регресс: доклад MCU по датчику, пришедший позже стартовой детекции,
    принимался за настоящий фронт и портил старт."""
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    printer.send_event('klippy:ready')
    # T0 на слайдере: его датчик не сработал и молчит. T1 в парковке —
    # сработал, но доклад придёт с опозданием.
    changer.tools[0].sensor.raw = False
    changer.tools[0].sensor.reported = True
    printer.reactor.run_pending()
    check('стартовая детекция дождалась докладов',
          changer.tools[1].sensor.reported is False)
    printer.gcode.output = []
    # запоздавший доклад по T1
    changer.tools[1].sensor._handle_edge(printer.reactor.monotonic(), True)
    check('поздний доклад не считается аварией',
          changer.fault != 'unexpected_edge', str(changer.fault))
    check('состояние сходится с датчиками', changer._observed_tool() == 0,
          str(changer._observed_tool()))


def test_idle_fault_does_not_break_next_change():
    """Регресс: ручное касание головы в простое залипало во флаге прерывания
    и срывало следующую законную смену инструмента."""
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'
    simulate_mechanics(printer, changer)
    # оператор потрогал голову руками в простое
    H.set_sensors(printer, changer, {0: True, 1: True}, via_callback=True)
    printer.reactor.run_pending()
    check('касание в простое даёт аварию', changer.fault == 'unexpected_edge')
    check('флаг прерывания взведён',
          changer.abort_requested == 'unexpected_edge')
    # голова возвращена на место, оператор командует смену
    H.set_sensors(printer, changer, {0: False, 1: True})
    raised = None
    try:
        changer.select_tool(1)
    except H.CommandError as exc:
        raised = str(exc)
    check('следующая смена не срывается', raised is None, str(raised))
    check('смена выполнилась', changer.active_tool == 1,
          str(changer.active_tool))
    check('авария снята', changer.fault is None and changer.state == 'ok',
          '%s / %s' % (changer.fault, changer.state))
    check('флаг прерывания сброшен', changer.abort_requested is None)


def test_status_has_no_stale_fault_after_recovery():
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    H.set_sensors(printer, changer, {0: True, 1: True}, via_callback=True)
    printer.reactor.run_pending()
    H.set_sensors(printer, changer, {0: False, 1: True})
    changer.cmd_TOOLCHANGER_STATUS(H.GCmd(printer))
    st = changer.get_status(printer.reactor.monotonic())
    check('после восстановления state и fault согласованы',
          st['state'] == 'ok' and st['fault'] == '',
          '%s / %r' % (st['state'], st['fault']))


def test_failed_release_never_reaches_the_other_dock():
    """Исторический кейс: голова не расцепилась при парковке, а принтер
    ехал за второй головой и бил в неё надетой."""
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.toolhead.homed = 'xyz'

    original_park = changer._run_park

    def park_that_does_not_release(tool):
        original_park(tool)          # движение отрабатывает штатно...
        # ...но голова осталась на слайдере: датчик парковки не сработал

    changer._run_park = park_that_does_not_release
    printer.toolhead.moves = []
    raised = None
    try:
        changer.select_tool(1)
    except H.CommandError as exc:
        raised = str(exc)

    check('несостоявшееся расцепление обрывает команду', raised is not None)
    check('класс аварии — park_failed', changer.fault == 'park_failed',
          str(changer.fault))
    t1 = changer.tools[1]
    near_t1 = [m for m in moves_of(printer)
               if abs(m[0] - t1.park_xy[0]) < 60 and abs(m[1] - t1.park_xy[1]) < 60]
    check('к парковке второй головы принтер не поехал', not near_t1,
          str(near_t1))
    check('активным остался прежний инструмент, а не запрошенный',
          changer.active_tool != 1, str(changer.active_tool))


def test_failed_grab_does_not_pretend_success():
    """Обратный случай: голова не снялась с парковки."""
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: True, 1: True})
    printer.toolhead.homed = 'xyz'
    raised = None
    try:
        changer.pickup_tool(0)       # датчики так и покажут «в парковке»
    except H.CommandError as exc:
        raised = str(exc)
    check('несостоявшийся захват обрывает команду', raised is not None)
    check('класс аварии — get_failed', changer.fault == 'get_failed',
          str(changer.fault))


def test_sensor_bounce_does_not_pause_a_print():
    """Кратковременный дребезг концевика от вибрации не должен ронять
    печать в паузу: аномалия обязана продержаться."""
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: False, 1: True})
    printer.objects['print_stats'].state = 'printing'
    printer.gcode.script = []
    printer.gcode.output = []
    # концевик T1 дёрнулся и тут же вернулся
    changer.tools[1].sensor._handle_edge(printer.reactor.monotonic(), False)
    changer.tools[1].sensor._handle_edge(printer.reactor.monotonic(), True)
    printer.reactor.run_pending()
    check('дребезг не поднимает аварию', changer.fault is None,
          str(changer.fault))
    check('дребезг не ставит печать в паузу',
          'PAUSE' not in printer.gcode.script, str(printer.gcode.script))
    # а вот устойчивый отрыв — поднимает
    changer.tools[1].sensor._handle_edge(printer.reactor.monotonic(), False)
    printer.reactor.run_pending()
    check('устойчивая аномалия даёт аварию',
          changer.fault == 'unexpected_edge', str(changer.fault))


def test_babystep_on_second_tool_is_not_inflated():
    """Подстройка Z на T1 не должна утащить в общий офсет собственный
    офсет второй головы."""
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    H.ready(printer, changer, {0: True, 1: False})   # на слайдере T1
    variables = printer.objects['save_variables'].allVariables
    variables['z_adjust'] = -0.13
    t1_z = variables['t1_z_offset']                  # 0.029
    # оператор подкрутил первый слой на -0.05, gcode_move отражает сумму
    printer.objects['gcode_move'].homing_origin = H.Coord(0., 0.,
                                                          t1_z - 0.13 - 0.05)
    changer.cmd_TOOLCHANGER_SAVE_ADJUST(H.GCmd(printer))
    saved = variables['z_adjust']
    check('в общий офсет попала только подстройка',
          abs(saved - (-0.18)) < 1e-6,
          'записано %s, ожидалось -0.18' % saved)
    check('офсет второй головы не утёк в общий',
          abs(saved - (-0.18 - t1_z)) > 1e-6, str(saved))


def test_calibration_is_immune_to_other_offsets():
    """Калибровка Z-офсета второй головы не должна зависеть ни от z_adjust,
    ни от z_offset пробы, ни от уже записанного t1_z_offset."""
    results = {}
    for z_adjust, probe_z, old_t1 in ((0.0, -1.0, 0.0),
                                      (-0.25, -1.0, 0.094),
                                      (0.4, -0.3, -0.5)):
        printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
        H.ready(printer, changer, {0: False, 1: True})
        printer.toolhead.homed = 'xyz'
        simulate_mechanics(printer, changer)
        v = printer.objects['save_variables'].allVariables
        v['z_adjust'] = z_adjust
        v['t1_z_offset'] = old_t1
        printer.objects['probe'].z_offset = probe_z
        printer.gcode.script = []
        changer.cmd_TOOLCHANGER_CALIBRATE_OFFSET(H.GCmd(printer, {'STABILIZE': '0'}))
        results[(z_adjust, probe_z, old_t1)] = v['t1_z_offset']
        moves = [l for l in printer.gcode.script if l.startswith('G1 X')]
        check('обе головы пробуют одну точку (%s)' % (z_adjust,),
              len(moves) == 2 and moves[0] == moves[1],
              str(moves))
    values = set(round(x, 6) for x in results.values())
    check('результат не зависит от посторонних офсетов',
          len(values) == 1, str(results))
    check('измерена именно разница высот голов',
          abs(list(values)[0] - 0.15) < 1e-6, str(values))


def test_startup_restores_full_offset():
    """После рестарта должен восстанавливаться не только общий z_adjust,
    но и собственный офсет головы, стоящей на слайдере."""
    for docked, expect in (({0: True, 1: False}, 0.029 - 0.25),   # T1 на слайдере
                           ({0: False, 1: True}, 0.0 - 0.25)):    # T0 на слайдере
        printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
        printer.objects['save_variables'].allVariables['z_adjust'] = -0.25
        H.ready(printer, changer, docked)
        applied = [l for l in printer.gcode.script
                   if l.startswith('_SET_GCODE_OFFSET_INTERNAL')]
        check('при старте офсет применяется (%s)' % (docked,), bool(applied),
              str(printer.gcode.script))
        z = float(applied[-1].split('Z=')[1])
        check('восстановлен полный офсет (%s)' % (docked,),
              abs(z - expect) < 1e-6, 'применено Z=%s, ожидалось %s' % (z, expect))

    # слайдер пуст — применяется только общий офсет
    printer, changer = H.build(H.S600_MAIN, [H.S600_T0, H.S600_T1])
    printer.objects['save_variables'].allVariables['z_adjust'] = -0.25
    H.ready(printer, changer, {0: True, 1: True})
    applied = [l for l in printer.gcode.script
               if l.startswith('_SET_GCODE_OFFSET_INTERNAL')]
    z = float(applied[-1].split('Z=')[1])
    check('пустой слайдер — только общий офсет', abs(z + 0.25) < 1e-6, str(z))


for test in [test_startup_restores_full_offset,
             test_calibration_is_immune_to_other_offsets,
             test_babystep_on_second_tool_is_not_inflated,
             test_sensor_bounce_does_not_pause_a_print,
             test_failed_release_never_reaches_the_other_dock,
             test_failed_grab_does_not_pretend_success,
             test_late_initial_report_is_not_a_fault,
             test_idle_fault_does_not_break_next_change,
             test_status_has_no_stale_fault_after_recovery,
             test_service_mode, test_service_mode_skips_skew_hook,
             test_service_mode_homing_does_not_grab, test_goto_and_set_dock,
             test_startup_states, test_no_blind_dwell,
             test_s600_park_trajectory, test_s600_pickup_trajectory,
             test_s300_park_trajectory, test_s300_pickup_boost,
             test_tool_change_offsets, test_babystep_capture,
             test_realtime_guard, test_edges_ignored_during_change,
             test_park_failure, test_pause_on_fault_while_printing,
             test_shutdown_reaction, test_homing_from_each_state,
             test_seat_recovery_s300, test_status_exposes_geometry,
             test_commands_registered, test_set_state_clears_fault,
             test_slider_sensor_faults]:
    try:
        test()
    except Exception as exc:
        import traceback
        RESULTS.append((test.__name__, False,
                        'EXCEPTION ' + traceback.format_exc().strip()
                        .splitlines()[-1]))

failed = [r for r in RESULTS if not r[1]]
for name, ok, detail in RESULTS:
    if not ok:
        print('FAIL  %s\n      %s' % (name, detail))
print('\n%d/%d проверок пройдено' % (len(RESULTS) - len(failed), len(RESULTS)))
sys.exit(1 if failed else 0)
