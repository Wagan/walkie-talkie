# ОТЧЁТ: интеграция BSP аудиотракта (TASK_bsp_audio_integration2)

Дата: 2026-08-10. Опора по фактам: `docs/REPORT_st_audio_example.md` (коммит ad30dc1) и
файлы пакета `STM32Cube_FW_F4_V1.28.0`. Проект не собирался, `main.c` и `walkie-talkie.ioc`
не изменялись. Перед правкой `.cproject` сделана резервная копия `.cproject.bak.before-bsp`
(добавлена в `.gitignore`, в репозиторий не входит).

Предварительная сверка состояния (подтверждает контекст задания):
- `main.c` содержит только `MX_GPIO_Init()`; вызовов `MX_I2S2/I2S3/I2C1/SPI1/USB_HOST_Init` нет,
  дескрипторов периферии нет.
- Каталогов `USB_HOST/` и `Middlewares/` в проекте не было; `Drivers/` = CMSIS + HAL.

---

## Задача A. Копирование файлов BSP и PDM2PCM

Скопированы без правок, с сохранением структуры пакета. Добавленное дерево и размеры (байт):

```
      3682  Drivers/BSP/Components/Common/audio.h
     14111  Drivers/BSP/Components/cs43l22/cs43l22.c
      6841  Drivers/BSP/Components/cs43l22/cs43l22.h
     22618  Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery.c
     11828  Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery.h
     40139  Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_audio.c
     10178  Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_audio.h
      3328  Middlewares/ST/STM32_Audio/Addons/PDM/Inc/pdm2pcm_glo.h
     14066  Middlewares/ST/STM32_Audio/Addons/PDM/Lib/libPDMFilter_CM4_GCC_wc32.a
```

**Проверка включений (includes) копируемых файлов:** все `#include` разрешаются внутри проекта:
- `stm32f411e_discovery.h` → `stm32f4xx_hal.h` (есть в проекте);
- `stm32f411e_discovery_audio.h` → `../Components/cs43l22/cs43l22.h`, `stm32f411e_discovery.h`,
  `../../../Middlewares/ST/STM32_Audio/Addons/PDM/Inc/pdm2pcm_glo.h` — все скопированы, относительные
  пути корректны в нашей раскладке (Middlewares лежит в корне проекта);
- `cs43l22.h` → `../Common/audio.h` (скопирован); `audio.h` → `<stdint.h>` (системный).

**Недостающих включений среди скопированных файлов нет.** Отдельно: для успешной *компиляции* этих
файлов нужны HAL-модули I2S/I2C/SPI, сейчас выключенные — это не «недостающий include», а настройка
сборки, вынесена в раздел «Осталось перед первой сборкой».

---

## Задача B. Настройки сборки (.cproject)

Способ ST прочитан из `…/Audio_playback_and_record/STM32CubeIDE/.cproject` и повторён в нашем
`.cproject` **для обеих конфигураций (Debug и Release)**. XML после правок валиден (проверено),
переводы строк сохранены CRLF. Изменения (чистый diff относительно `.cproject.bak.before-bsp`):

### 1. Пути включения (-I) — добавлено в оба блока `option.includepaths`
(Debug id `…includepaths.1868459859`; Release id `…includepaths.908820608`), после `../Drivers/CMSIS/Include`:
```
<listOptionValue builtIn="false" value="../Drivers/BSP/STM32F411E-Discovery"/>
<listOptionValue builtIn="false" value="../Middlewares/ST/STM32_Audio/Addons/PDM"/>
```
(Такой же набор, как у ST, адаптированный к нашей локальной раскладке `../…`. Подкаталоги
`Components/cs43l22` и `Components/Common` ST в -I не добавляет — они подключаются относительными
include, поэтому и у нас не добавлены.)

### 2. Подключение библиотеки PDM2PCM — тем же способом, что у ST
ST задаёт библиотеку через **путь поиска (-L) + имя файла с префиксом `-l:`**, не абсолютным путём.
Повторено: в оба линкер-тула (Debug id `…c.linker.985580443`; Release id `…c.linker.757650138`)
после опции Linker Script добавлены две опции:
```
<option ... id="…c.linker.option.directories.9000000NN" name="Library search path (-L)"
        superClass="…c.linker.option.directories" valueType="libPaths">
  <listOptionValue builtIn="false" value="../Middlewares/ST/STM32_Audio/Addons/PDM/Lib"/>
</option>
<option ... id="…c.linker.option.libraries.9000000NN" name="Libraries (-l)"
        superClass="…c.linker.option.libraries" valueType="libs">
  <listOptionValue builtIn="false" value=":libPDMFilter_CM4_GCC_wc32.a"/>
</option>
```
(id: Debug — directories `900000001`, libraries `900000002`; Release — `900000003`, `900000004`.)
Значение `:libPDMFilter_CM4_GCC_wc32.a` с ведущим двоеточием — это точное указание файла (`-l:<file>`),
как и в эталоне (там `:libPDMFilter_CM4_GCC_wc32.a`).

### 3. Символ препроцессора
В эталоне у C-компилятора определён специфичный символ **`USE_STM32F411E_DISCO`** (наряду с
DEBUG/STM32F411xE/USE_HAL_DRIVER). Добавлен в оба блока `option.definedsymbols`
(Debug id `…definedsymbols.1591269546`; Release id `…definedsymbols.2136902829`):
```
<listOptionValue builtIn="false" value="USE_STM32F411E_DISCO"/>
```
Замечание: `stm32f411e_discovery.h:75-76` само-определяет этот символ, если он не задан
(`#if !defined(USE_STM32F411E_DISCO) #define USE_STM32F411E_DISCO`). То есть формально сборка
не сломается и без него, но ST задаёт его явно — повторено ради точного соответствия.

### 4. Тактирование CRC — на уровне настроек проекта НЕ требуется
PDM-библиотека разблокируется включением тактирования CRC, и это делается **только кодом**:
`stm32f411e_discovery_audio.c:1053` вызывает `__HAL_RCC_CRC_CLK_ENABLE()` внутри `PDMDecoder_Init()`.
Это RCC-макрос (из `stm32f4xx_hal_rcc.h`), он не требует включения HAL-модуля CRC. Вызовов
`HAL_CRC_*` в скопированных файлах нет. Вывод: **никаких настроек CRC в `.cproject` добавлять не
нужно; HAL_CRC_MODULE_ENABLED тоже не нужен.** Кодовую часть (она уже внутри BSP) в этом задании
не трогаем.

### Область сборки: попадают ли добавленные каталоги автоматически
Наш `.cproject` содержит явные `<sourceEntries>` с двумя папками-источниками: **`Core` и `Drivers`**
(в обеих конфигурациях, строки backup 77-78 и 153-154). Отсюда:
- `Drivers/BSP/**` (наши `.c`) попадают в сборку **автоматически** — `Drivers` целиком объявлена
  папкой-источником. Явно добавлять пути к исходникам BSP **не требуется**, ничего не менял.
- `Middlewares/**` в источники НЕ входит, но там нет ни одного `.c` (только заголовок `pdm2pcm_glo.h`
  и бинарная `.a`): заголовок берётся по `-I`, библиотека — по `-L/-l`. Компилировать в Middlewares
  нечего, поэтому в `sourceEntries` его добавлять не нужно.

(Отличие от эталона: у ST `<sourceEntries>` отсутствует вовсе — там в сборку идёт весь корень.
У нас корень ограничен Core+Drivers, и этого достаточно для добавленного BSP.)

---

## Задача C. Обработчики прерываний

Прочитаны `stm32f4xx_it.c/.h` эталона. Факты:
- Обработчики DMA заданы в эталоне через **макро-алиасы**: `void I2S2_IRQHandler(void)` и
  `void I2S3_IRQHandler(void)`, где `I2S2_IRQHandler`≡`DMA1_Stream3_IRQHandler` и
  `I2S3_IRQHandler`≡`DMA1_Stream7_IRQHandler` (макросы из `stm32f411e_discovery_audio.h`, стр. 118 и 87).
  Тела: `HAL_DMA_IRQHandler(hAudioInI2s.hdmarx)` и `HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx)`.
- **Отдельных обработчиков I2S-периферии (SPI2/SPI3 global) в эталоне НЕТ** — под «I2S*_IRQHandler»
  там скрываются именно потоки DMA. Поэтому переносить нечего, кроме этих двух DMA-обработчиков.

Решение по именованию: в наш `it.c` перенесены обработчики под **каноническими именами CMSIS**
`DMA1_Stream3_IRQHandler` / `DMA1_Stream7_IRQHandler` (это в точности то, во что раскрываются
макросы эталона). Так не требуется тянуть BSP-заголовок в `it.c`; в стартап-файле
`Core/Startup/startup_stm32f411vetx.s` оба вектора присутствуют как `.weak` (строки 159/192 и
311-312/383-384) — наши определения их переопределяют.

Весь код размещён **строго внутри блоков USER CODE** (переживёт регенерацию CubeMX).

### Core/Src/stm32f4xx_it.c — в блок `USER CODE BEGIN PV` (внешние дескрипторы):
```c
/* Audio BSP I2S handles (defined in Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_audio.c) */
extern I2S_HandleTypeDef hAudioInI2s;  /* microphone input  (I2S2, DMA1_Stream3 Rx) */
extern I2S_HandleTypeDef hAudioOutI2s; /* codec output      (I2S3, DMA1_Stream7 Tx) */
```

### Core/Src/stm32f4xx_it.c — в блок `USER CODE BEGIN 0` (сами обработчики):
```c
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}

void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}
```
(с поясняющими комментариями-шапками, включая упоминание макро-алиасов эталона.)

### Core/Inc/stm32f4xx_it.h — в блок `USER CODE BEGIN EFP` (прототипы):
```c
void DMA1_Stream3_IRQHandler(void); /* audio IN  (microphone, I2S2 Rx) */
void DMA1_Stream7_IRQHandler(void); /* audio OUT (codec, I2S3 Tx) */
```

Подходящие блоки USER CODE в нужных местах присутствовали (PV, «0», EFP) — вставка за их пределы
не потребовалась.

Замечание про NVIC: включение самих IRQ (`HAL_NVIC_SetPriority`/`HAL_NVIC_EnableIRQ` для
DMA1_Stream3 и DMA1_Stream7) выполняется кодом BSP в `BSP_AUDIO_IN_MspInit`/`BSP_AUDIO_OUT_MspInit` —
отдельной настройки NVIC в CubeMX/проекте не требуется.

---

## Расхождения с эталоном

1. **Именование DMA-обработчиков:** эталон — через макросы `I2S2_IRQHandler`/`I2S3_IRQHandler`;
   у нас — канонические `DMA1_Stream3/7_IRQHandler` (эквивалент, задокументировано в коде и выше).
   Ничего в эталоне не «исправлялось».
2. **`<sourceEntries>`:** у нас есть (Core+Drivers), у ST — нет (весь корень). На результат для BSP
   не влияет (Drivers покрывает BSP).
3. **I2S2 half-duplex vs наш прежний .ioc (full-duplex):** в этом задании I2S2/I2S3/I2C1 в CubeMX
   выключены, инициализацию ведёт BSP (half-duplex master RX для микрофона). Конфликта нет —
   прежняя full-duplex-конфигурация неактивна. `.ioc` не трогали.

Иных расхождений, требующих STOP, не обнаружено.

---

## Осталось сделать перед первой сборкой (по факту кода BSP, не по предположению)

Эти пункты — следствие копирования полных файлов BSP; они **вне scope A/B/C** этого задания и
здесь не выполнялись, только зафиксированы:

1. **Включить HAL-модули в `Core/Inc/stm32f4xx_hal_conf.h`** — сейчас закомментированы:
   - `HAL_I2S_MODULE_ENABLED` (строка 57) — BSP использует HAL_I2S (≈24 вызова, тракты I2S2/I2S3).
   - `HAL_I2C_MODULE_ENABLED` (строка 56) — управление кодеком по I2C1 (≈6 вызовов).
   - `HAL_SPI_MODULE_ENABLED` (строка 65) — `stm32f411e_discovery.c` **безусловно** компилирует
     функции `SPIx_Init/SPIx_WriteRead` (гироскоп L3GD20), вызывающие `HAL_SPI_*` (стр. 394/417/433/447).
     Даже если аудио их не вызывает, файл не скомпилируется без HAL_SPI.
   - `HAL_CRC_MODULE_ENABLED` — **НЕ требуется** (см. B4).
2. **Добавить HAL-исходники** в `Drivers/STM32F4xx_HAL_Driver/Src/` — сейчас отсутствуют:
   `stm32f4xx_hal_i2s.c`, `stm32f4xx_hal_i2c.c`, `stm32f4xx_hal_spi.c`.
   (Присутствуют: hal, cortex, dma(+ex), exti, flash(+ex/ramfunc), gpio, pwr(+ex), rcc(+ex), tim(+ex).)
   `stm32f4xx_hal_rcc_ex.c` уже есть — нужен для `HAL_RCCEx_PeriphCLKConfig` (пересчёт PLLI2S).
   Штатный способ получить эти три файла и правки hal_conf — включить соответствующие модули так,
   как это делает CubeMX/пакет; конкретный способ — решение владельца (в этом задании не выполнялось,
   чтобы не выходить за рамки A/B/C и не гадать).
3. **Прикладная часть (следующее задание, не сейчас):** вызовы `BSP_AUDIO_OUT_Init` /
   `BSP_AUDIO_IN_Init` / `BSP_AUDIO_IN_Record` в `main.c`, реализация callback'ов
   (`BSP_AUDIO_IN_HalfTransfer_CallBack`/`…TransferComplete…`), буферы. Тактирование CRC при этом
   отдельно включать не надо — его включает `PDMDecoder_Init()` внутри BSP.

После пунктов 1–2 добавленный BSP и обработчики из Задачи C компилируются и линкуются; аудио
«оживёт» уже прикладным кодом из следующего задания.

---

## Публикация

Замечание о составе коммита (прозрачность): на момент задания рабочее дерево содержало
**непрокоммиченную перегенерацию проекта, сделанную владельцем** в IDE (отключение
USB_HOST/USB_OTG_FS/SPI1/I2S2/I2S3/I2C1 и перегенерация): изменены `main.c`, `main.h`,
`stm32f4xx_hal_conf.h`, `stm32f4xx_hal_msp.c`, `.ioc`, `.mxproject`; удалены каталог
`USB_HOST/`, USB Host Library и HAL-исходники i2c/i2s/spi/hcd/usb. Эти изменения —
**не моя правка** (я `main.c` и `.ioc` не трогал), но они являются согласованным базисом
задания. Коммит фиксирует цельное состояние проекта: базис владельца + моя интеграция BSP
(Задачи A/B/C). Мои правки перечислены выше пофайлово; резервная копия `.cproject.bak.before-bsp`
в репозиторий не входит (игнорируется).

- Отчёт: `docs/REPORT_bsp_audio_integration.md`.
- Хеш коммита: **`df71bd1aa553ff59fdb95d350debf0bb20c9bf23`** · Push: OK (`bd92490..df71bd1 main -> main`, exit 0).
- (Хеш вписан вторым коммитом.)
