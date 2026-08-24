# multiplex_heater — сегментные нагреватели как один объект

Расширение Klipper для принтеров с несколькими независимыми нагревателями
стола и/или камеры (у каждого свой пин и свой термистор).

**Зачем:** в Fluidd / Mainsail / KlipperScreen видны один стол и одна камера,
а не 4+2 отдельных `heater_generic`. Сегменты настраиваются в конфиге
расширения и скрыты от UI.

**Где лежат файлы модуля** (в `~/klipper/klippy/extras/` их много чужих —
ищите по имени):

| Файл | Назначение |
|------|------------|
| `klippy/extras/multiplex_heater.py` | Логика «фасада» (общий нагреватель) |
| `klippy/extras/multiplex_heater_segment.py` | Подключение секций сегментов |
| `docs/zbolt/multiplex_heater.md` | Эта документация |

Исходники: `klippy/extras/multiplex_heater*.py` в форке Z-Bolt.

Пример использования на S800 HT Dual: `Config_printer/…/klipper-config/heaters.cfg`.

---

## Идея

```
Fluidd / M140 / SET_HEATER_TEMPERATURE
            │
            ▼
   [multiplex_heater heater_bed]     ← один публичный объект
            │
    ┌───────┼───────┬───────┐
    ▼       ▼       ▼       ▼
  _bed0   _bed1   _bed2   _bed3      ← реальные нагреватели (скрыты, префикс _)
```

- Отображаемая температура и power — **среднее по активным** сегментам.
- Каждый сегмент греется своим PID по своему термистору.
- `verify_heater` работает на каждом сегменте отдельно.

Почему два `.py`: в Klipper имя секции = имя модуля.
`[multiplex_heater …]` → `multiplex_heater.py`,
`[multiplex_heater_segment …]` → `multiplex_heater_segment.py`.

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

Не объявляйте параллельно обычный `[heater_bed]` — его роль берёт
`[multiplex_heater heater_bed]` (включая M140 / M190).

---

## Конфигурация

### Публичный нагреватель

```cfg
[multiplex_heater heater_bed]
gcode_id: B
min_temp: 0
max_temp: 145

[multiplex_heater chamber]
gcode_id: C
min_temp: 0
max_temp: 95
```

| Параметр | Описание |
|----------|----------|
| имя после `multiplex_heater` | Публичное имя: `heater_bed`, `chamber`, … |
| `gcode_id` | Буква в ответе M105 (для стола обычно `B`) |
| `min_temp` / `max_temp` | Лимиты цели для фасада |

Для Jinja-макросов доступны `printer.heater_bed` и `printer.chamber`.

### Сегмент

```cfg
[multiplex_heater_segment heater_bed0]  # Передняя левая плита
multiplex_heater: heater_bed
heater_pin: expander: PB4
sensor_type: PT1000
sensor_pin: expander: PC0
max_power: 0.7
control: pid
pid_Kp: 40
pid_Ki: 1.2
pid_Kd: 335
min_temp: 0
max_temp: 145

[verify_heater _heater_bed0]
max_error: 300
check_gain_time: 600
hysteresis: 5
heating_gain: 2
```

| Параметр | Описание |
|----------|----------|
| `multiplex_heater` | Имя родителя (`heater_bed` / `chamber`) |
| остальное | Как у обычного `heater_generic` (пин, сенсор, PID, …) |

Индекс сегмента для `SEGMENTS=` — **порядок объявления** секций
`[multiplex_heater_segment]` у данного родителя, с нуля: 0, 1, 2, …

Внутри Klipper сегмент регистрируется как `_heater_bed0`, `_chamber0` и т.д.
(префикс `_` — скрытие в UI). Секция `verify_heater` должна совпадать
с этим именем: `[verify_heater _heater_bed0]`.

`heater_fan` можно вешать на фасад, например: `heater: chamber`.

---

## Команды

### Обычный нагрев (вся текущая маска)

```text
SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=60
SET_HEATER_TEMPERATURE HEATER=chamber TARGET=50
M140 S60
M190 S60
TEMPERATURE_WAIT SENSOR=heater_bed MINIMUM=60
TEMPERATURE_WAIT SENSOR=chamber MINIMUM=50
```

По умолчанию маска = все сегменты. После частичного нагрева
`SET_HEATER_TEMPERATURE` / `M140` используют **уже выбранную** маску.

### Частичный нагрев

```text
SET_MULTIPLEX_HEATER HEATER=heater_bed TARGET=60 SEGMENTS=0,1
```

| Параметр | Описание |
|----------|----------|
| `HEATER` | Публичное имя фасада |
| `TARGET` | Цель для **активных** сегментов |
| `SEGMENTS` | Индексы через запятую; если не указан — все |

Неактивные сегменты получают target `0`.

Выключение (`TARGET=0` или `TURN_OFF_HEATERS`) гасит **все** сегменты,
но **маску не сбрасывает** — следующий нагрев снова пойдёт в ту же зону.
Чтобы снова греть весь стол: `SET_MULTIPLEX_HEATER HEATER=heater_bed TARGET=…`
без `SEGMENTS` (или со всеми индексами).

### Статус (Moonraker / макросы)

```text
printer.heater_bed.temperature
printer.heater_bed.target
printer.heater_bed.power
printer.heater_bed.active_segments   # например [0, 1, 2, 3]
```

То же для `printer.chamber`.

---

## Замечания по Z-Bolt / форкам

На хостах Z-Bolt команда `TEMPERATURE_WAIT` — одна глобальная (не mux, как
в свежем upstream). Модуль это учитывает: достаточно попасть в
`available_sensors` / словарь нагревателей.

PID-калибровку отдельного сегмента при необходимости вызывайте по
скрытому имени, например: `PID_CALIBRATE HEATER=_heater_bed0`.

---

## Типичные проблемы

1. **Unknown config section `[multiplex_heater …]`** — модули не лежат в
   `~/klipper/klippy/extras/` или опечатка в имени файла.
2. **В UI снова куча нагревателей** — сегменты объявлены как обычный
   `heater_generic` без multiplex, либо старый конфиг не залит.
3. **Нет M140** — публичное имя должно быть ровно `heater_bed`.
4. **TEMPERATURE_WAIT Unknown sensor** — используйте короткое имя
   (`heater_bed`, `chamber`), не `heater_generic …`.
