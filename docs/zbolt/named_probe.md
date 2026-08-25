# named_probe — дополнительные Z-датчики для мультиголовых принтеров

Расширение Klipper для машин с **несколькими независимыми** tip-датчиками
(контакт сопла со столом / пьезо / endstop на каждой голове).

**Зачем:** в stock Klipper допускается только одна секция `[probe]`. Команда
`PROBE`, homing через `probe:z_virtual_endstop`, bed mesh и QGL всегда
используют этот единственный датчик. На dual/IDEX с отдельным пином на T0 и T1
нужно явно пробовать **другим** датчиком — без подмены глобального `PROBE`.

**Что делает модуль:**

- регистрирует именованные секции `[named_probe NAME]`;
- даёт команды `PROBE_NAMED` / `QUERY_PROBE_NAMED` с mux-параметром `PROBE=`;
- **не** перехватывает stock `PROBE` и **не** регистрирует
  `probe:z_virtual_endstop`.

**Где лежат файлы:**

| Файл | Назначение |
|------|------------|
| `klippy/extras/named_probe.py` | Модуль |
| `docs/zbolt/named_probe.md` | Эта документация |

Исходники: `klippy/extras/named_probe.py` в форке Z-Bolt.

Рабочий пример на S800 HT Dual:

- конфиг датчика: `Config_printer/…/S800 HT Dual v2.0/klipper-config/bed.cfg`
- макросы `Zoffset` / `_GT1`: `…/klipper-config/tools.cfg`

---

## Идея

```
[probe]  pin=PA0          ← T0, homing / QGL / BED_MESH / обычный PROBE
     │
     └── stock Klipper

[named_probe t1] pin=PA2  ← T1, только явным вызовом
     │
     ├── PROBE_NAMED PROBE=t1
     └── QUERY_PROBE_NAMED PROBE=t1
```

Типичный сценарий калибровки Z-офсета между головами:

1. Homing / касание стола соплом T0 через обычный `PROBE`.
2. Смена инструмента на T1.
3. Касание тем же местом стола соплом T1 через `PROBE_NAMED PROBE=t1`.
4. Макрос читает `printer["named_probe t1"].last_z_result` и сохраняет офсет.

```
Zoffset
  │
  ├─ T0: PROBE / PROBE          ← stock [probe]
  ├─ T1 (toolchange)
  ├─ T1: PROBE_NAMED × 3        ← [named_probe t1]
  └─ _GT1 → SAVE_VARIABLE t1_z_offset
```

---

## Требования и совместимость

- Нужен установленный stock `[probe]` (для Z-homing / mesh на этой машине).
- Модуль рассчитан на API `probe.py`, где:
  - `ProbeEndstopWrapper(config)` принимает только `config`;
  - результат пробы — список `[x, y, z]`;
  - есть `ProbeSessionHelper`, `ProbeParameterHelper`, `ProbeOffsetsHelper`,
    `run_single_probe`.
- Проверено на Klipper `v0.13.0` (в т.ч. форк Z-Bolt / Spider).
- На совсем свежем upstream с другим конструктором `ProbeEndstopWrapper`
  модуль может потребовать правки — смотрите traceback при загрузке.

---

## Установка

Модули входят в состав форка Z-Bolt (`klippy/extras/`), поэтому отдельно
ставить ничего не нужно — они приезжают вместе с обновлением прошивки:

```bash
cd ~/klipper && git pull
sudo systemctl restart klipper
# в консоли Klipper / Fluidd:
FIRMWARE_RESTART
```

На принтерах Klipper обновляется через `update_manager` в Moonraker.

После правок только `.py` достаточно `restart klipper` + `FIRMWARE_RESTART`.
После правок `.cfg` — достаточно `RESTART` / `FIRMWARE_RESTART`.

---

## Конфигурация

### Секция `[named_probe NAME]`

Имя после `named_probe` — идентификатор датчика для команд и статуса.
Пример: `[named_probe t1]` → команды с `PROBE=t1`, статус
`printer["named_probe t1"]`.

Можно объявить несколько датчиков:

```cfg
[named_probe t1]
pin: PA2
z_offset: 0.0
...

[named_probe t2]
pin: PB0
z_offset: 0.0
...
```

### Параметры (как у stock `[probe]`)

| Параметр | Обязательный | Описание |
|----------|--------------|----------|
| `pin` | да | GPIO endstop/датчика (например `PA2`, `^PA2`, `!PB0`, `expander:PA3`) |
| `z_offset` | да | Z-смещение датчика относительно сопла (мм), как у `[probe]` |
| `x_offset` | нет, `0` | XY-смещение датчика относительно сопла |
| `y_offset` | нет, `0` | |
| `speed` | нет, `5.0` | Скорость спуска при пробе (мм/с) |
| `lift_speed` | нет, = `speed` | Скорость подъёма между сэмплами |
| `samples` | нет, `1` | Число касаний за одну команду |
| `sample_retract_dist` | нет, `2.0` | Подъём между сэмплами (мм) |
| `samples_result` | нет, `average` | `average` или `median` |
| `samples_tolerance` | нет, `0.1` | Допуск разброса сэмплов (мм) |
| `samples_tolerance_retries` | нет, `0` | Сколько раз повторить серию при выходе за допуск |
| `deactivate_on_each_sample` | нет, `True` | Вызывать deactivate между сэмплами |
| `activate_gcode` | нет | G-code перед касанием (опускание щупа и т.п.) |
| `deactivate_gcode` | нет | G-code после касания |

### Пример для S800 (T0 stock + T1 named)

```cfg
[probe]
pin: PA0
x_offset: 0.0
y_offset: 0.0
z_offset: -0.0
speed: 0.8
lift_speed: 25
samples: 1
sample_retract_dist: 2.0
samples_result: average
samples_tolerance: 0.04
samples_tolerance_retries: 0

[named_probe t1]
pin: PA2
x_offset: 0.0
y_offset: 0.0
z_offset: 0.0
speed: 0.8
lift_speed: 25
samples: 1
sample_retract_dist: 2.0
samples_result: average
samples_tolerance: 0.04
samples_tolerance_retries: 0
```

`stepper_z` по-прежнему использует `endstop_pin: probe:z_virtual_endstop` —
это endstop **только** от stock `[probe]`, не от `named_probe`.

---

## Команды

Команды — **mux**: обязательно указать `PROBE=<имя>` из секции
`[named_probe <имя>]`.

### `PROBE_NAMED`

Проба Z текущей XY-позиции выбранным датчиком.

```text
PROBE_NAMED PROBE=t1
```

Опциональные переопределения параметров (как у stock `PROBE`):

| Параметр | Описание |
|----------|----------|
| `PROBE` | **Обязательный.** Имя датчика (`t1`, `t2`, …) |
| `PROBE_SPEED` | Скорость спуска (мм/с) |
| `LIFT_SPEED` | Скорость подъёма между сэмплами |
| `SAMPLES` | Число касаний |
| `SAMPLE_RETRACT_DIST` | Подъём между касаниями |
| `SAMPLES_TOLERANCE` | Допуск разброса |
| `SAMPLES_TOLERANCE_RETRIES` | Повторы серии |
| `SAMPLES_RESULT` | `average` / `median` |

Пример:

```text
PROBE_NAMED PROBE=t1 SAMPLES=3 SAMPLE_RETRACT_DIST=2 PROBE_SPEED=0.8
```

Поведение:

1. Требует, чтобы ось Z уже была захомлена (`Must home before probe`).
2. Опускает Z до срабатывания пина датчика.
3. При нескольких сэмплах усредняет / берёт медиану.
4. Пишет в консоль: `named_probe t1: Result is z=…`.
5. Сохраняет результат в `printer["named_probe t1"].last_z_result`
   (значение Z в мм, координата касания).

### `QUERY_PROBE_NAMED`

Опрос состояния пина без движения.

```text
QUERY_PROBE_NAMED PROBE=t1
```

Ответ: `named_probe t1: open` или `named_probe t1: TRIGGERED`.

Полезно для проверки проводки до первой реальной пробы: вручную
надавите на датчик / коснитесь соплом и смотрите смену состояния.

### Чего модуль намеренно не делает

| Действие | Как делать |
|----------|------------|
| Homing Z | Stock `[probe]` + `G28` / `probe:z_virtual_endstop` |
| Bed mesh / QGL | Обычные команды Klipper → всегда stock `probe` |
| `PROBE` без имени | Stock датчик T0 |
| Автосмена датчика при `T0`/`T1` | Не встроена; вызывайте `PROBE_NAMED` явно в макросах |

---

## Статус для макросов и Moonraker

Объект: `printer["named_probe t1"]` (имя с пробелом — в кавычках).

| Поле | Тип | Описание |
|------|-----|----------|
| `name` | string | Короткое имя (`t1`) |
| `last_z_result` | float | Z последней успешной `PROBE_NAMED` |
| `last_query` | bool | Результат последнего `QUERY_PROBE_NAMED` (`true` = TRIGGERED) |

Пример в Jinja:

```jinja
{% set z = printer["named_probe t1"].last_z_result|float %}
{% set cfg = printer["configfile"].config["named_probe t1"] %}
{% set z_off = cfg.z_offset|default(0)|float %}
{% set tool_z = (z - z_off)|round(3) %}
```

Запрос через Moonraker:

```text
GET /printer/objects/query?named_probe%20t1
```

---

## Пример макроса Z-офсета (S800)

Логика:

1. Прогрев обеих голов, термостабилизация.
2. Две пробы stock `PROBE` на T0 (опорная плоскость / прогрев механизма).
3. `T1`, три пробы `PROBE_NAMED PROBE=t1`.
4. `_GT1` пишет `t1_z_offset` в `variables.cfg`.

Фрагмент:

```gcode
[gcode_macro _GT1]
gcode:
    {% set T1ProbeValue = printer["named_probe t1"].last_z_result|default(0)|float %}
    {% set probeConfig = printer['configfile'].config["named_probe t1"] %}
    {% set ProbeOffset = 0 %}
    {% if probeConfig %}
        {% set ProbeOffset = probeConfig.z_offset|default(0)|float %}
    {% endif %}
    {% set T1OffsetZ = (T1ProbeValue - ProbeOffset)|round(3) %}
    RESPOND PREFIX="[+]" MSG="Офсет T1 - {T1OffsetZ}мм"
    SAVE_VARIABLE VARIABLE=t1_z_offset VALUE={T1OffsetZ}

[gcode_macro Zoffset]
gcode:
    ...
    ; T0
    PROBE
    G1 Z2 F1200
    PROBE
    ...
    T1
    PROBE_NAMED PROBE=t1
    G1 Z2 F1200
    PROBE_NAMED PROBE=t1
    G1 Z2 F1200
    PROBE_NAMED PROBE=t1
    G1 Z2 F1200
    _GT1
```

Полные макросы — в `tools.cfg` профиля S800.

---

## Как устроено внутри (кратко)

1. `load_config_prefix` создаёт экземпляр на каждую секцию
   `[named_probe NAME]`.
2. Используется stock `ProbeEndstopWrapper` на своём `pin`.
3. Сессия пробы (`NamedProbeSession`) повторяет логику
   `HomingViaProbeHelper` (спуск через `homing.probing_move`), но
   **без** `pins.register_chip('probe', …)` — чтобы не конфликтовать
   с основным `z_virtual_endstop`.
4. Усреднение сэмплов — через stock `ProbeSessionHelper`.
5. Команды регистрируются как mux (`register_mux_command`), ключ `PROBE`.

---

## Проверка после установки

1. `FIRMWARE_RESTART` → Printer is ready.
2. В Moonraker / Fluidd объект `named_probe t1` присутствует в статусе.
3. `QUERY_PROBE_NAMED PROBE=t1` → `open`; при ручном срабатывании датчика →
   `TRIGGERED`.
4. После `G28` и позиционирования над столом на активной T1:
   `PROBE_NAMED PROBE=t1` → корректный Z в ответе и в `last_z_result`.
5. Полный прогон `Zoffset` → обновление `t1_z_offset` в `variables.cfg`.

---

## Типичные проблемы

1. **Unknown config object `named_probe …`**
   Файл не лежит в `~/klipper/klippy/extras/named_probe.py`, опечатка в имени
   или не сделан `FIRMWARE_RESTART`.

2. **`ProbeEndstopWrapper.__init__() takes 2 positional arguments but 4 were given`**
   На хосте другая (более новая) версия `probe.py`. Нужна адаптация модуля
   под актуальный API.

3. **`Pin '…' is not a valid pin name`**
   В конфиге остался placeholder или неверный GPIO. Укажите реальный пин MCU
   (`PA2`, `expander:PBx`, с `^` / `!` при необходимости).

4. **`Must home before probe`**
   Сначала `G28` (Z через stock probe).

5. **Проба T1 срабатывает «не тем» датчиком**
   Убедитесь, что вызываете именно `PROBE_NAMED PROBE=t1`, а не `PROBE`.
   Проверьте пин секции `[named_probe t1]` мультиметром /
   `QUERY_PROBE_NAMED`.

6. **Конфликт пина**
   Пин `named_probe` не должен совпадать с уже занятым (`[probe]`,
   endstop осей, fan, servo на том же MCU-пине). `expander:PA2` и `PA2`
   на основном MCU — **разные** пины.

7. **Bed mesh / QGL «не видят» второй датчик**
   Так и задумано. Mesh и QGL идут через stock `[probe]`. Для второй головы
   используйте офсет инструмента (`SET_GCODE_OFFSET` / переменные), а не
   второй mesh.

---

## Лицензия

GNU GPLv3 (как у Klipper).
