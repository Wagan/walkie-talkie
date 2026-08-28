# REPORT: LAB09 шаг 4 — драйвер DW3000 в ThirdParty и платформенный слой

Автор: Vagan Sarukhanov · Дата: 2026-08-28 · Ветка `main` · Коммит `<второй коммит>`

Драйвер Qorvo `dwt_uwb_driver` перенесён в `ThirdParty/`, отобран минимум для чтения DEV_ID,
написан платформенный слой под F411/SPI4, команда `devid` читает идентификатор **через драйвер**.
Радиообмен не начинали; `.ioc`/HAL/BSP/раскладку выводов/конфигурацию SPI4 не трогали.

---

## 1. Источник, версия, состав поставки

- **Источник:** `github.com/br101/dw3000-decadriver-source`, коммит `effcf6e6` (2026-07-22), склонирован
  по сети (CC). Это зеркало с исходной поставкой Qorvo `dwt_uwb_driver` **версии 08.02.02** из пакета
  `DW3_QM33_SDK_1.0.2.zip`; авторская обвязка br101 — под ISC (отдельно от лицензии Qorvo).
- **Состав поставки (верхний уровень):** `dwt_uwb_driver/` (драйвер Qorvo + `LICENSES/`),
  `platform/` (общий слой + порты zephyr/esp-idf/nrf-sdk), `LICENSE.txt` (ISC), `README.md`.
- Проверено: обе платы отвечают `0xDECA0302` (шаг devid), поэтому взят драйвер под DW3000 (не DW3720).

## 2. Что перенесено в `ThirdParty/dw3000_driver` и что оставлено

Критерий — **компилируемость и связность** (взято, потому что без него не собирается/не линкуется
чтение DEV_ID). Границу определил автономным линк-тестом: скомпилировал кандидатов под Cortex-M4 и
слинковал вызов `dwt_probe()`+`dwt_readdevid()` с заглушкой платформы — единственные внешние символы
оказались 4 платформенные функции (см. §4), значит набор ниже самодостаточен.

**Перенесено (исходники Qorvo, без единой правки — шапки/копирайты сохранены):**
- `platform/deca_compat.c` — слой обратной совместимости, тут `dwt_probe()`/`dwt_readdevid()`
  (это АКТИВНАЯ версия; `dwt_readdevid` = `dwt_read32bitoffsetreg`, без ioctl);
- `platform/deca_ull.h`;
- `dwt_uwb_driver/deca_interface.c` (ops-обёртки `interface_*`, на них ссылается таблица драйвера);
- `dwt_uwb_driver/deca_rsl.c` (+ `lib/qmath/src/qmath.c`) — на них ссылается `dw3000_device.c`;
- `dwt_uwb_driver/dw3000/dw3000_device.c` — реализация DW3000 (ops-таблица `dw3000_driver`);
- заголовки: `deca_device_api.h, deca_interface.h, deca_private.h, deca_rsl.h, deca_types.h,
  deca_version.h, dw3000/dw3000_deca_regs.h, dw3000/dw3000_deca_vals.h, lib/qmath/include/qmath.h`;
- `LICENSES/LicenseRef-QORVO-2.txt` (лицензия Qorvo), `LICENSE-ISC-platform.txt` (ISC br101),
  `UPSTREAM_README.md` (происхождение).
- Предсобранная **`Lib/libdw3000_cm4.a`** — из 5 `.c` выше (arm-none-eabi-gcc 12.3.1, `-mcpu=cortex-m4
  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -ffunction/-fdata-sections -DCONFIG_DW3000_CHIP_DW3000`).

**Оставлено за бортом (и почему):**
- `dwt_uwb_driver/dw3720/` — драйвер другого чипа (DW3720); наш `dwt_probe` берёт `driver_list` от
  вызывающего (только `dw3000_driver`), список `tmp_ptr` со ссылкой на `dw3720_driver` — под `#ifdef
  WIN32`, на цели не компилируется → 442 КБ исходника не нужны;
- `dwt_uwb_driver/deca_compat.c` — **устаревшая копия**: вызывает удалённый из ops член `ioctl`, не
  компилируется; активная версия — `platform/deca_compat.c` (её и берёт zephyr-CMake);
- `platform/dw3000_spi_trace.c` — опциональная SPI-трассировка, требует платформенный `log.h`; наш
  SPI-слой её не зовёт;
- `platform/{esp-idf,zephyr,nrf-sdk}`, `platform/{dw3000_hw.h,dw3000_spi.h,dw3000.h,deca_probe_interface.h}`
  — платформенная обвязка под чужие ОС/платы; наш слой её заменяет (см. §4);
- `dwt_uwb_driver/{cmake,utest}`, `lib/qmath/{utest,Recipes,CMakeLists}` — тесты/сборочные файлы.

**Правку чужого кода не делали** (STOP-условие не сработало): разделение — отбором ЦЕЛЫХ файлов, ни один
исходник Qorvo не редактировался. Драйвер собран как `.a` (как Codec2/PDM2PCM) — так vendor-warnings не
попадают в сборку IDE (остаётся 0 warnings), а исходники лежат рядом для соблюдения лицензии.

## 3. Лицензия Qorvo (дословно ограничения) и README

Полный текст — `ThirdParty/dw3000_driver/LICENSES/LicenseRef-QORVO-2.txt`. Ключевые условия дословно:

> 1. Redistributions of source code must retain the above copyright notice, this list of conditions,
>    and the following disclaimer.
> 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
>    and the following disclaimer in the documentation and/or other materials provided with the distribution.
> 3. You may only use this software, with or without any modification, with an integrated circuit
>    developed by Qorvo US, Inc. or any of its affiliates (collectively, "Qorvo"), or any module that
>    contains such integrated circuit.
> 4. You may not reverse engineer, disassemble, decompile, decode, adapt, or otherwise attempt to derive
>    or gain access to the source code to any software distributed under this license in binary or object
>    code form, in whole or in part.
> 5. You may not use any Qorvo name, trademarks, ... to endorse or promote products derived from this
>    software without specific prior written permission from Qorvo US, Inc. You must not call products
>    derived from this software "Qorvo" ...

То есть (факты, без правовой оценки): использование — только с микросхемой Qorvo (наш случай — DW3110);
сохранять копирайт/условия в исходной и бинарной форме; запрет реверс-инжиниринга; запрет использования
имени Qorvo. Копирайт-шапки в перенесённых файлах сохранены полностью, ничего не вычищали.

**README.md:** в раздел `ThirdParty/` добавлена строка о драйвере — версия 08.02.02, источник (зеркало
br101, пакет Qorvo `DW3_QM33_SDK_1.0.2`), лицензия Qorvo (`LicenseRef-QORVO-2`, ограничение «только с
чипами Qorvo», запрет реверса), предсобранная `libdw3000_cm4.a`, платформенный слой в `App/Common`.

## 4. Платформенный слой под F411 (наш код)

`App/Common/{Inc/dw3000_port.h, Src/dw3000_port.c}` — НАШ код, реализует интерфейс, который ждёт
драйвер (не производный от Qorvo). Компилируется только при `LAB_ID==9 && UWB_CHIP_DW3000`. Использует
уже проверенные железом SPI4 и выводы из `uwb_core.c`: `CSn=PE11`, обмен по `hspi4`.

Драйвер ожидает (`struct dwt_spi_s` + глобальные функции):
- `readfromspi(hlen,hbuf,rlen,rbuf)` → CSn низкий, `HAL_SPI_Transmit(header)` + `HAL_SPI_Receive(data)`, CSn высокий;
- `writetospi(hlen,hbuf,blen,bbuf)` → CSn низкий, Transmit(header)+Transmit(body), CSn высокий;
- `writetospiwithcrc(...)` → как write + байт CRC (SPI-CRC режим не используем, реализовано на случай вызова);
- `setslowrate()/setfastrate()` → смена делителя `hspi4` (см. §5);
- `deca_sleep(ms)` → `HAL_Delay`; `deca_usleep(us)` → задержка по счётчику циклов DWT;
- `decamutexon()/decamutexoff()` → критическая секция через `PRIMASK` (без RTOS);
- `wakeup_device_with_io()` → пусто (сон не используем; сброс делает `uwb_core` в `Lab_Init`).

Обёртки: `Dw3000Port_Probe()` один раз собирает `dwt_probe_s` (наш `dwt_spi_s`, `driver_list={dw3000_driver}`,
`dw_driver_num=1`) и зовёт `dwt_probe()`; `Dw3000Port_ReadDevId()` → `dwt_readdevid()`.

Проводка: адаптер `uwb_dw3000.c::UwbChip_ReadDevIdViaDriver()` → `Dw3000Port_ReadDevId()`. Общий движок
`uwb_core.c::cmd_devid` сначала пробует драйвер, при неудаче/отсутствии — прямое чтение. Для DW1000
`UwbChip_ReadDevIdViaDriver` возвращает 0 (драйвера нет) → прямое чтение.

## 5. Скорость SPI: инициализация и после

- **Инициализация — 1.5 МГц** (`setslowrate`, делитель `/64` от PCLK2=APB2=96 МГц). Годится для
  состояния после сброса: в DW3000 INIT_RC потолок SPICLK **7 МГц** (DW3000 UM §2.4 Table 4).
- **После выхода в рабочее состояние — 12 МГц** (`setfastrate`, делитель `/8`). DW3000 в IDLE_PLL
  допускает до **38 МГц** (UM Table 4); взяли консервативно 12 МГц с запасом на жгут переходной платы.
- **Момент переключения задаёт драйвер:** `setslowrate` он зовёт до/во время инициализации, `setfastrate`
  — после захвата PLL (в `dwt_initialise`/`dwt_configure`). На шаге devid полная инициализация в IDLE_PLL
  **не выполняется**, поэтому обмен остаётся на 1.5 МГц; `setfastrate` реализован, но не достигается —
  проверять на железе при добавлении TX/RX.

## 6. Размеры секций и результат сборки

Headless CubeIDE, обе конфигурации — **0 errors, 0 warnings**.

| Конфигурация | text | data | bss | Комментарий |
|---|---|---|---|---|
| LAB09 (DW3000) — **до** | 49088 | 360 | 13000 | без драйвера (шаг 3) |
| LAB09 (DW3000) — **после** | **59556** | 360 | **13264** | + драйвер + `devidraw` |
| LAB09_DW1000 — до | 49032 | 360 | 13000 | шаг 3 |
| LAB09_DW1000 — после | 49504 | 360 | 13000 | + `devidraw`/баннер, **драйвера нет** |

**Стоимость драйвера (изолированно)** = LAB09 − LAB09_DW1000 (после) = **+10052 байт flash, +264 байт RAM**
после отбрасывания неиспользуемого компоновщиком (`--gc-sections`). Из ~512 КБ flash и 128 КБ RAM — с
большим запасом. Проверено логом: `dw3000_port.c` компилируется в обеих (в DW1000 — пустой по guard),
`libdw3000_cm4.a` линкуется **только** в LAB09; ветвь DW1000 драйвер не тянет.

## 7. Коммит

`<заполняется вторым коммитом>`

## 8. Что разошлось с ожиданиями / осталось неясным

1. **Две `deca_compat.c` в поставке.** Активна `platform/deca_compat.c` (её берёт сборка,
   `dwt_readdevid` без ioctl); `dwt_uwb_driver/deca_compat.c` — устаревшая копия, ссылается на удалённый
   член `ioctl`, не компилируется. Взяли активную, устаревшую не переносили.
2. **br101 уже минимально модифицировал файлы Qorvo** (один чип на плату, убрана большая IOCTL-функция) —
   это правки АВТОРА зеркала, не наши; мы их берём как есть под лицензией Qorvo.
3. **Fast-rate (12 МГц) не проверен на железе** — на шаге devid чип не доходит до IDLE_PLL, обмен
   остаётся 1.5 МГц. Значение и момент переключения проверить при добавлении полной инициализации/TX-RX.
4. **Проверка «devid через драйвер» на железе — за владельцем:** ожидается `0xDECA0302`, полученный
   `dwt_readdevid()` через наш платформенный слой; сверить с `devidraw` (прямое чтение).

## Проверка на железе (владелец)

Прошить LAB09 (умолчание — DW3000), выполнить `devid` — ожидается
`family=DW3000 via=driver devid=0xDECA0302` и `verdict: OK - DW3000 alive (non-PDoA)`. Сравнить с
`devidraw` (прямое чтение нашей транзакцией). `spistat` — SCK 1.5 МГц, mode 0.
