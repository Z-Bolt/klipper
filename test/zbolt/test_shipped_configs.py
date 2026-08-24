# -*- coding: utf-8 -*-
"""Parity check driven by the SHIPPED zbolt_toolchanger.cfg of each profile.

tc_parity.py proves the module reproduces the old macros for a given set of
parameters.  This proves the parameters actually written into each profile
are that set - i.e. the configs going onto the machines are correct.
"""
import sys, io, os, configparser
import harness as H
import test_trajectory_parity as P   # wraps sys.stdout for utf-8 output

# Конфиги принтеров лежат в соседнем репозитории Config_printer.
# Путь переопределяется переменной окружения TC_REPO.
REPO = os.environ.get(
    'TC_REPO', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', '..', '..', 'Config_printer'))
BASE = os.path.join(REPO, 'Fysetc Spider', 'RPi CM4')
PROFILES = {
    'S300': 'S300 HT Dual v2.2',
    'S310': 'S310 HT Dual v2.2',
    'S400': 'S400 HT Dual v3.2',
    'S600': 'S600 HT Dual v3.2',
}


def load_profile(model):
    path = os.path.join(BASE, PROFILES[model], 'klipper-config',
                        'zbolt_toolchanger.cfg')
    parser = configparser.RawConfigParser(inline_comment_prefixes=('#', ';'),
                                          strict=False)
    parser.read_file(io.open(path, encoding='utf-8'))
    main, tools = None, []
    for section in parser.sections():
        options = {k: v for k, v in parser.items(section)}
        if section == 'zbolt_toolchanger':
            main = options
        elif section.startswith('zbolt_toolchanger_tool'):
            tools.append(options)
    assert main is not None, 'нет [zbolt_toolchanger] в %s' % path
    assert len(tools) == 2, 'ожидалось 2 инструмента в %s' % path
    tools.sort(key=lambda o: int(o['tool_number']))
    return main, tools


results = []
for model in ('S300', 'S310', 'S400', 'S600'):
    main, tools = load_profile(model)
    for action in ('park', 'pickup'):
        for number in (0, 1):
            printer, changer = H.build(main, tools)
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
            got = [(float(round(x, 3)), float(round(y, 3)), float(s))
                   for x, y, z, s in printer.toolhead.moves]
            want = [(float(x), float(y), float(s))
                    for x, y, s in P.MODELS[model][action][number]]
            ok = got == want
            results.append(ok)
            if not ok:
                print('FAIL %s %s T%d' % (model, action, number))
                print('  ожидалось: %s' % (want,))
                print('  из конфига: %s' % (got,))

print('\n%d/%d траекторий из реальных конфигов совпали со старыми макросами'
      % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
