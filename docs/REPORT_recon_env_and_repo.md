# ОТЧЁТ ПО РАЗВЕДКЕ: REPORT_recon_env_and_repo

Проект: walkie-talkie
Исполнитель: Wagan's Team (файлы и git). Владелец: Vagan Sarukhanov.
Дата: 2026-08-10
Машина: рабочий ноутбук, Windows 11, PowerShell 5.1. Единственный диск — `C:`.

Метод: Часть A выполнена только на чтение. Пакет прошивок физически лежит **zip-архивом**
(см. A2) и **не был распакован** — его содержимое читалось из архива в памяти
(`System.IO.Compression.ZipArchive.OpenRead`), без извлечения на диск. Каждый факт ниже
сопровождается точным путём, по которому он получен.

---

## A1. STM32CubeIDE: версия и пути

- **Что искал:** версию и путь установки IDE, физическое расположение workspace.
- **Где искал:** `C:\ST`, `C:\Program Files\STMicroelectronics`, маркерный файл версии.
- **Что нашёл:**
  - Установка IDE: `C:\ST\STM32CubeIDE_1.16.0\STM32CubeIDE\stm32cubeide.exe`
  - Версия **1.16.0**, взята из файла `C:\ST\STM32CubeIDE_1.16.0\STM32CubeIDE\.eclipseproduct`,
    строка `version=1.16.0` (там же `id=com.st.stm32cube.ide.mcu.rcp.product`).
  - Workspace физически лежит: `C:\Users\user\STM32CubeIDE\workspace_1.16.0`
    (в нём и находится папка проекта `walkie-talkie`).
- **Расхождения:** нет. Версия каталога, workspace и `.eclipseproduct` согласованы.

---

## A2. Пакет STM32Cube_FW_F4: путь и версия

- **Что искал:** путь и точную версию FW-пакета F4, ожидание V1.28.0.
- **Где искал:** стандартный репозиторий `C:\Users\user\STM32Cube\Repository`, `C:\ST`,
  `C:\Program Files*`, затем полный обход `C:\`.
- **Что нашёл:**
  - Стандартный репозиторий CubeMX/CubeIDE `C:\Users\user\STM32Cube\Repository` **пуст**
    (единственный файл — `en.stm32h5cubemxbanner.png`, к F4 отношения не имеет).
  - Пакет присутствует как **zip-архив, не распакованный**:
    `C:\Software\packages\f4\stm32cube_fw_f4_v1280.zip` (641 199 780 байт).
  - Внутри архива единственный корневой каталог: `STM32Cube_FW_F4_V1.28.0/` (всего 56 355 записей).
  - Версия подтверждена файлом `STM32Cube_FW_F4_V1.28.0/package.xml`:
    `<PackDescription Release="FW.F4.1.28.0">`.
  - Других установленных версий F4 на диске не найдено. Рядом лежат такие же архивы
    других серий: `C:\Software\packages\{f1,f7,h7}` (к заданию не относятся).
- **Расхождения (ВАЖНО):** версия совпала (V1.28.0), **но пакет не установлен/не
  распакован** — он существует только в виде zip. Это значит, что перед работой с BSP,
  примерами и middleware пакет придётся распаковать/импортировать (действие Части B/
  следующих заданий, не выполнялось). Все факты A3–A5 получены чтением внутри архива.

---

## A3. BSP для платы + Components (ЦАП cs43l22, MEMS-микрофон)

- **Что искал:** папку BSP под кит; драйверы cs43l22 и MEMS-микрофона в Components.
- **Где искал:** внутри архива, `.../Drivers/BSP/`.
- **Что нашёл:**
  - Фактическое имя папки BSP платы: **`Drivers/BSP/STM32F411E-Discovery`**. Файлы:
    - `stm32f411e_discovery.c` / `.h`
    - `stm32f411e_discovery_audio.c` / `.h`
    - `stm32f411e_discovery_accelerometer.c` / `.h`
    - `stm32f411e_discovery_gyroscope.c` / `.h`
    - `LICENSE.txt`, `Release_Notes.html`, `STM32F411E-Discovery_BSP_User_Manual.chm`,
      подпапка `_htmresc/` (css, png для html).
  - **ЦАП-усилитель cs43l22:** папка `Drivers/BSP/Components/cs43l22` присутствует.
    Файлы: `cs43l22.c`, `cs43l22.h`, `LICENSE.txt`, `Release_Notes.html`, ресурсы `_htmresc`.
    Версия: **V2.0.5 / 19-June-2023** — взята из `cs43l22/Release_Notes.html`
    (актуальная секция). В шапке `cs43l22.h` поля `@version` **нет** (ST убрала его после
    перехода на внешний LICENSE-файл; в шапке только `@file/@author/@brief` и copyright 2015).
  - **MEMS-микрофон:** отдельного драйвера-компонента под MEMS-микрофон (MP45DT02) в
    `Drivers/BSP/Components` **НЕТ**. Полный список папок Components:
    ampire480272, ampire640480, Common, cs43l22, dp83848, exc7200, ft3x67, ft6x06,
    i3g4250d, ili9325, ili9341, l3gd20, lan8742, lis302dl, lis3dsh, ls016b8uy, lsm303agr,
    lsm303dlhc, mfxstm32l152, n25q128a, n25q256a, n25q512a, nt35510, otm8009a, ov2640,
    ov5640, s25fl512s, s5k5cag, st7735, st7789h2, stmpe1600, stmpe811, ts3510, wm8994.
    Работа с микрофоном реализована **прямо в BSP платы** (`stm32f411e_discovery_audio.c/.h`):
    PDM-поток снимается по I2S2, конвертируется через PDM2PCM. В `stm32f411e_discovery_audio.h`
    объявлены `BSP_AUDIO_IN_Init`, `BSP_AUDIO_IN_Record`, `BSP_AUDIO_IN_PDMToPCM`,
    константы `DEFAULT_AUDIO_IN_FREQ = I2S_AUDIOFREQ_16K`, `DEFAULT_AUDIO_IN_CHANNEL_NBR = 1` и др.
- **Расхождения:** (1) версия компонента взята не из шапки, а из Release_Notes — в шапках
  `@version` больше нет; (2) отдельного BSP-драйвера MEMS-микрофона в Components не
  существует — микрофон обслуживается кодом BSP платы + PDM2PCM. (Даташит `mp45dt02-955096.pdf`
  лежит в `docs/reference`, см. A7, но это документ, а не драйвер.)

---

## A4. Middleware PDM2PCM

- **Что искал:** наличие PDM2PCM, путь, форму поставки (исходники или библиотека),
  варианты файлов по ядрам, заголовок API и имена функций init/обработки.
- **Где искал:** внутри архива, `.../Middlewares/ST/STM32_Audio/`.
- **Что нашёл:**
  - Путь: **`Middlewares/ST/STM32_Audio/Addons/PDM/`**.
  - Форма поставки: **прекомпилированная библиотека**, не исходники. Единственный
    исходный текст — заголовок API `Inc/pdm2pcm_glo.h`. Бинарники в `Lib/`.
  - Версия PDM-библиотеки: **V3.5.1** — из `Addons/PDM/Release_Notes.html`.
  - Варианты `.a`/`.lib` по ядрам и тулчейнам (`wc16`/`wc32` = разрядность коэффициентов;
    `soft`/`softfp` = FP-ABI):
    - CM0: GCC wc16, GCC wc32, IAR wc32, Keil wc16
    - CM3: GCC wc16, GCC wc32, IAR wc16, IAR wc32, Keil wc16
    - **CM4 (наше ядро, F411 = Cortex-M4F):** GCC wc16, GCC wc16_soft, GCC wc16_softfp,
      GCC wc32, GCC wc32_soft, GCC wc32_softfp, IAR wc16, IAR wc32, Keil wc16
    - CM7: GCC wc16, GCC wc16_softfp, GCC wc32, GCC wc32_softfp, IAR wc16, IAR wc32, Keil wc16
    - CM33: GCC wc16, GCC wc16_softfp, GCC wc32, GCC wc32_softfp, IAR wc32, Keil wc16
  - Заголовок API `pdm2pcm_glo.h`. Основные функции:
    - `PDM_Filter_Init(PDM_Filter_Handler_t *pHandler)` — инициализация;
    - `PDM_Filter_setConfig(...)` / `PDM_Filter_getConfig(...)` — конфигурирование;
    - `PDM_Filter(void *pDataIn, void *pDataOut, PDM_Filter_Handler_t *pHandler)` — **обработка**
      (PDM→PCM);
    - `PDM_Filter_deInterleave(...)` — деинтерливинг.
    Также определены константы децимации (`PDM_FILTER_DEC_FACTOR_*`), порядка бит и
    endianness, коды ошибок.
- **Расхождения:** нет по существу. Отмечено, что для GCC под CM4 есть три FP-варианта
  (soft/softfp/hard-без-суффикса) × две разрядности — выбор конкретного `.a` останется
  задачей сборки прошивки.

---

## A5. Примеры под STM32F411E-Discovery

- **Что искал:** содержимое папки примеров под кит; отдельно всё, что связано со звуком
  (запись, воспроизведение, USB audio); для аудио-примеров — под какие среды есть проекты
  и есть ли среди них STM32CubeIDE.
- **Где искал:** внутри архива, `.../Projects/STM32F411E-Discovery/`.
- **Что нашёл:**
  - Фактическое имя папки: **`Projects/STM32F411E-Discovery`**. Категории:
    `Applications`, `Demonstrations`, `Examples`, `Templates`, `Templates_LL`.
  - `Applications`: **Audio**, EEPROM, FatFs.
  - `Examples`: ADC, BSP, DMA, FLASH, GPIO, HAL, I2C, PWR, RCC, SPI, TIM, UART.
  - **Аудио-пример:** `Applications/Audio/Audio_playback_and_record`.
    - Назначение (по составу): запись с MEMS-микрофона и воспроизведение WAV. Исходники
      `Src/`: `main.c`, `waveplayer.c`, `waverecorder.c`, `usbh_conf.c`,
      `usbh_diskio_dma.c`, `stm32f4xx_it.c`, `system_stm32f4xx.c`. Заголовки `Inc/`:
      `main.h`, `waveplayer.h`, `waverecorder.h`, `usbh_conf.h`, `usbh_diskio_dma.h`,
      `ffconf.h`, `stm32f4xx_hal_conf.h`, `stm32f4xx_it.h`.
    - Среды в примере: **EWARM, MDK-ARM и STM32CubeIDE** (папки `EWARM/`, `MDK-ARM/`,
      `STM32CubeIDE/`). В `STM32CubeIDE/` есть готовый проект: `.project`, `.cproject`,
      `startup_stm32f411vehx.s`, `syscalls.c`, `sysmem.c`, линкер `STM32F411VEHX_FLASH.ld`.
  - **USB audio:** отдельного примера USB Audio class под этот кит **НЕТ**. В
    `Audio_playback_and_record` USB используется как **USB Host (mass storage)** для чтения/
    записи WAV на флешку (`usbh_conf.c`, `usbh_diskio_dma.c`), а не как аудио-класс.
- **Расхождения:** позитивное — проект **STM32CubeIDE присутствует** в аудио-примере,
  то есть отдельный перенос из EWARM/MDK не требуется (риск из задания снят). Уточнение:
  «USB audio» в чистом виде среди примеров этого кита отсутствует; аудио-пример завязан
  на USB Host + FatFs.

---

## A6. Тулчейн arm-none-eabi-gcc

- **Что искал:** версию и путь gcc, поставляемого с CubeIDE.
- **Где искал:** плагины CubeIDE `.../plugins`, вызов `--version`.
- **Что нашёл:**
  - Путь: `C:\ST\STM32CubeIDE_1.16.0\STM32CubeIDE\plugins\`
    `com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623\tools\bin\arm-none-eabi-gcc.exe`
  - Версия (вывод `--version`): **arm-none-eabi-gcc 12.3.1 20230626**
    (баннер: `GNU Tools for STM32 12.3.rel1.20240612-1315`).
- **Расхождения:** нет (в задании конкретная версия не задавалась).

---

## A7. Инвентаризация docs/reference

- **Что искал:** список файлов с размерами (файлы не открывать).
- **Где искал:** `C:\Users\user\STM32CubeIDE\workspace_1.16.0\walkie-talkie\docs\reference`.
- **Что нашёл:** 10 PDF-файлов (размер в байтах):

  | Размер (байт) | Имя файла |
  |---:|---|
  | 90 775 | an3998-pdm-audio-software-decoding-on-stm32-microcontrollers-stmicroelectronics.pdf |
  | 1 548 538 | an4031-using-the-stm32f2-stm32f4-and-stm32f7-series-dma-controller-stmicroelectronics.pdf |
  | 699 444 | CS43L22_F2.pdf |
  | 334 282 | mp45dt02-955096.pdf |
  | 2 732 268 | pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf |
  | 14 188 321 | rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf |
  | 77 338 422 | um1718-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf |
  | 34 176 589 | um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf |
  | 1 695 359 | um1842-discovery-kit-with-stm32f411ve-mcu-stmicroelectronics.pdf |
  | 25 834 326 | um2609-stm32cubeide-user-guide-stmicroelectronics.pdf |

- **Расхождения:** нет. Содержимое файлов не открывалось; в репозиторий папка не
  добавляется (исключена `.gitignore`, проверка — B6.4).

---

## A8. Состояние git и окружения

- **Что искал:** наличие `.git`; настройки user.name/user.email; работу SSH к GitHub.
- **Где искал:** папка `walkie-talkie`; `git config --global`; `ssh -T git@github.com`.
- **Что нашёл:**
  - `.git` в `walkie-talkie` на момент разведки **отсутствовал** (Test-Path → False).
  - `git config --global user.name` = **`Wagan Sarukhanov`**
  - `git config --global user.email` = **`94711+Wagan@users.noreply.github.com`**
  - `git --version` = `git version 2.55.0.windows.3`
  - SSH: `ssh -T git@github.com` →
    `Hi Wagan! You've successfully authenticated, but GitHub does not provide shell access.`
    (код возврата 1 — штатное поведение GitHub при успешной аутентификации).
- **Расхождения:** нет. Оба значения совпали с ожидаемыми
  (`Wagan Sarukhanov` и `94711+Wagan@users.noreply.github.com`). Значения глобальные
  (локального репо на тот момент не было). Ничего не менялось.

---

## Сводка расхождений (кратко)

1. **FW_F4 не установлен, а лежит zip-архивом** (`C:\Software\packages\f4\stm32cube_fw_f4_v1280.zip`);
   стандартный `STM32Cube\Repository` пуст. Перед работой пакет нужно распаковать/импортировать.
2. **Аудио-пример имеет проект STM32CubeIDE** — переноса из EWARM/MDK не требуется (риск снят).
3. **Отдельного USB-Audio примера под кит нет**; аудио-пример использует USB Host + FatFs.
4. **Отдельного BSP-драйвера MEMS-микрофона нет**; микрофон обслуживается BSP платы + PDM2PCM.
5. **`@version` в шапках Components отсутствует** — версии брались из Release_Notes.html.

Остальные ожидания (версия FW V1.28.0, git user.name/email, SSH-доступ) подтвердились фактами.

---

## Часть B. Публикация (заполняется по факту)

- Ветка: `main`. Remote `origin` = `git@github.com:Wagan/walkie-talkie.git`.
- Закоммичены ровно 4 файла: `.gitignore`, `README.md`,
  `docs/TASK_recon_env_and_repo.md`, `docs/REPORT_recon_env_and_repo.md`.
- Хеш первого коммита (push OK, `main -> main`, exit 0):
  **`d58ecb0c9f881a0ef1948daa46dc1d964356a101`**
- Проверка B6.4 (`git ls-files docs/`): в индексе только
  `docs/REPORT_recon_env_and_repo.md` и `docs/TASK_recon_env_and_repo.md`.
  Ни одного файла из `docs/reference/` в выводе нет — **справочная документация не утекла (OK)**.

> Примечание: этот отчёт с заполненной секцией B зафиксирован отдельным вторым коммитом
> (хеш первого коммита выше указывает на состояние на момент публикации по B6).

---

# УТОЧНЕНИЕ A2–A5 (запрос владельца, 2026-08-10)

Повод: владелец указал, что пакет должен быть распакован где-то на диске (проекты
собирались, в Project Manager фигурировала версия STM32Cube_FW_F4_V1.28.0), а
`C:\Software` — это склад скачанных дистрибутивов, а не место установки. Ниже —
проверка только на чтение, ничего не распаковывалось и не устанавливалось.

## 1. Источник каждого факта из A3/A4/A5

**ВАЖНО: все перечисленные ниже факты получены чтением содержимого ZIP-архива**
`C:\Software\packages\f4\stm32cube_fw_f4_v1280.zip` (через `ZipArchive.OpenRead`, без
извлечения на диск). Распакованных копий этих файлов на диске нет (см. раздел 2).
Пути указаны как они лежат *внутри* архива, под корнем `STM32Cube_FW_F4_V1.28.0/`.

A3 (BSP и Components):
- Имя и состав папки BSP платы → из архива: `Drivers/BSP/STM32F411E-Discovery/` (листинг
  записей архива).
- cs43l22, состав папки → из архива: `Drivers/BSP/Components/cs43l22/`.
- cs43l22 версия **V2.0.5 / 19-June-2023** → из архива, файл
  `Drivers/BSP/Components/cs43l22/Release_Notes.html` (в шапке `cs43l22.h` `@version` нет).
- Отсутствие драйвера MEMS-микрофона и список папок Components → из архива, листинг
  `Drivers/BSP/Components/`.
- Функции работы с микрофоном (BSP_AUDIO_IN_*) → из архива, чтение
  `Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_audio.h`.

A4 (PDM2PCM):
- Путь, форма поставки (библиотека), список `.a/.lib` по ядрам → из архива, листинг
  `Middlewares/ST/STM32_Audio/Addons/PDM/` и `.../Lib/`.
- Версия **V3.5.1** → из архива, файл `.../Addons/PDM/Release_Notes.html`.
- Имена функций API (PDM_Filter_Init, PDM_Filter, setConfig/getConfig, deInterleave) →
  из архива, чтение `.../Addons/PDM/Inc/pdm2pcm_glo.h`.

A5 (примеры):
- Структура `Projects/STM32F411E-Discovery`, состав аудио-примера, наличие проекта
  `STM32CubeIDE/` в `Applications/Audio/Audio_playback_and_record/` → из архива, листинг
  соответствующих записей.

## 2. Поиск распакованного дерева пакета — результаты по каждому пункту

- **Репозиторий прошивок в профиле** `C:\Users\user\STM32Cube\Repository`:
  папка существует, но пакета F4 в ней НЕТ. Единственный файл —
  `en.stm32h5cubemxbanner.png` (81 284 байт). Это же расположение подтверждено как
  активный Repository самой CubeIDE: в `...\.metadata\.ide.log` (запись 2026-08-10
  10:33:07) строка `Set advertising image to C:/Users/user/STM32Cube/Repository/\en.stm32h5cubemxbanner.png`.
- **Настройки репозитория в CubeIDE** (`workspace_1.16.0\.metadata`): отдельного ключа
  `RepositoryFolder`/переопределения пути не найдено. В `.ide.log` фигурирует список
  «CubeFinder database FW Pack versions», где для F4 указана `STM32Cube_FW_F4_V1.28.3` —
  это КАТАЛОГ известных паков (не признак установки), и версия там 1.28.**3**, а не 1.28.0.
- **Настройки STM32CubeMX в профиле**: каталог `C:\Users\user\AppData\Roaming\STM32CubeMX`
  существует, но конфигурационного файла с `Repository Folder` в нём нет (пусто, кроме
  подпапок `plugins`/`thirdparties`). Отдельного `~/.stm32cubemx` с конфигом нет. Вывод:
  использовался встроенный в CubeIDE CubeMX с дефолтным Repository (см. выше).
- **Поиск по всему диску C: папок `STM32Cube_FW_F4_V1.28*`**: не найдено ни одной.
- **Поиск по всему диску C: файла `stm32f4xx_hal.h`**: 5 копий, все — ВНУТРИ конкретных
  проектов (не репозиторий пакета):
  - `...\workspace_1.16.0\Mini-Stand-3.0.2\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal.h`
  - `...\workspace_1.16.0\mks-firmware\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal.h`
  - `...\OneDrive\...\МКС\firmware\deep1\Mini-Stand-RTOS\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal.h`
  - `...\OneDrive\...\МКС\firmware\deep1\Mini-stend.1\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal.h`
  - `...\OneDrive\...\РАДИОМОДЕМ\...\ethernet-drivers\driver-st-stm32f4\stm32f4xx_hal.h`
- **Поиск по всему диску C: файлов `stm32f411e_discovery_audio.h`, `pdm2pcm_glo.h`,
  `cs43l22.h`**: НЕ найдено ни одного (0 совпадений). То есть BSP платы, PDM2PCM и
  cs43l22 в распакованном виде на диске отсутствуют — они есть только внутри zip.
- **Соседний проект `mks-firmware`** (только чтение, ничего не менялось):
  - `.ioc` (`mks-firmware-project.ioc`): `ProjectManager.FirmwarePackage=STM32Cube FW_F4 V1.28.0`
    — это МЕТКА версии пакета, использованного при генерации, а не путь к каталогу.
  - `.mxproject`, секция `[PreviousLibFiles]`: все пути к HAL/CMSIS/USB —
    **проектно-относительные** (`Drivers\STM32F4xx_HAL_Driver\...`,
    `Drivers\CMSIS\...`, `Middlewares\ST\STM32_USB_Device_Library\...`). Абсолютного пути
    к внешнему распакованному пакету в проекте нет.
  - `Drivers/` проекта содержит `CMSIS`, `STM32F4xx_HAL_Driver`, `decadriver`. Папки `BSP`
    нет (проект под Nucleo F411RE, board-BSP не используется).

## 3. Исходники в проектах: скопированы или ссылка на внешний каталог?

**Скопированы генератором внутрь проекта.** Доказательства: (а) в `.mxproject` пути
библиотек проектно-относительные; (б) файлы `stm32f4xx_hal.h`/`.c` физически лежат в
`<проект>\Drivers\STM32F4xx_HAL_Driver\` каждого проекта; (в) внешнего распакованного
пакета, на который можно было бы ссылаться, на диске нет вовсе.

**Следствие:** для сборки уже сгенерированного проекта распакованный пакет НЕ нужен —
проект самодостаточен. Распакованный пакет требуется CubeMX только в момент генерации/
доген­рации (добавить новый модуль HAL, BSP платы, middleware вроде PDM2PCM). Именно
поэтому проекты собираются, хотя каталога пакета сейчас на диске нет.

## 4. Вывод и STOP

Распакованного дерева `STM32Cube_FW_F4_V1.28.0` (равно как 1.28.3) на диске C: **не
найдено нигде**. Пакет присутствует только как zip `C:\Software\packages\f4\stm32cube_fw_f4_v1280.zip`.
Утверждение «пакет распакован где-то» фактами не подтверждается; проекты собираются
за счёт локально скопированных исходников, а не за счёт наличия установленного пакета.

Расхождения (зафиксированы, не «подгонялись»):
- Repository CubeIDE (`C:\Users\user\STM32Cube\Repository`) пуст — пакет туда не установлен.
- Каталог CubeFinder в CubeIDE знает F4 версии **1.28.3**, тогда как на диске лежит zip
  **1.28.0**, и проекты сгенерированы под меткой **1.28.0**.

Согласно правилам задания, ничего не распаковывал и не устанавливал. **STOP** — решение
о распаковке/установке пакета за владельцем. Если для следующих заданий понадобится
генерировать новый проект под STM32F411E-Discovery с BSP/PDM2PCM, пакет надо будет
предварительно распаковать/импортировать в Repository (например, через CubeIDE
Embedded Software Packages или распаковкой zip в каталог Repository) — это отдельное
действие, вне текущего задания.

---

# RECON: Repository после установки (запрос владельца, 2026-08-10)

Пакет установлен штатным механизмом CubeIDE; проверка только на чтение.

## 1. Содержимое Repository

Фактический путь Repository: `C:\Users\user\STM32Cube\Repository` (тот же, что и раньше).
Каталоги верхнего уровня с размерами:

| Размер | Каталог |
|---:|---|
| 1679,5 МБ | `STM32Cube_FW_F4_V1.28.0` |

Файлы верхнего уровня: `en.stm32h5cubemxbanner.png` (81 284 байт) — присутствовал и до
установки (LastWriteTime 2026-08-10 10:33:07, раньше самого пакета).

## 2. Пакет F4: имя каталога и проверка путей

- Точное имя каталога: **`STM32Cube_FW_F4_V1.28.0`**.
- Проверка требуемых путей внутри (наличие подтверждено файлами):

  | Есть/Нет | Путь (относительно каталога пакета) |
  |---|---|
  | **ЕСТЬ** | `Drivers/BSP/STM32F411E-Discovery` |
  | **ЕСТЬ** | `Drivers/BSP/Components/cs43l22` |
  | **ЕСТЬ** | `Middlewares/ST/STM32_Audio/Addons/PDM/Lib` (библиотека PDM2PCM) |
  | **ЕСТЬ** | `Projects/STM32F411E-Discovery/Applications/Audio/Audio_playback_and_record` |
  | **ЕСТЬ** | …/Audio_playback_and_record/`STM32CubeIDE` (проект CubeIDE) |

  Подтверждающие файлы:
  - BSP платы: `stm32f411e_discovery.c/.h`, `stm32f411e_discovery_audio.c/.h`,
    `..._accelerometer.c/.h`, `..._gyroscope.c/.h`, `Release_Notes.html`, `LICENSE.txt`,
    `STM32F411E-Discovery_BSP_User_Manual.chm`.
  - cs43l22: `cs43l22.c`, `cs43l22.h`, `Release_Notes.html`, `LICENSE.txt`.
  - PDM2PCM: библиотека лежит по полному пути
    `C:\Users\user\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.0\Middlewares\ST\STM32_Audio\Addons\PDM\Lib`
    (заголовок API — `.../Addons/PDM/Inc/pdm2pcm_glo.h`, тоже на месте).
    Варианты под наше ядро CM4 (GCC): `libPDMFilter_CM4_GCC_wc16.a`, `wc16_soft`,
    `wc16_softfp`, `wc32`, `wc32_soft`, `wc32_softfp` — все присутствуют.
  - Проект CubeIDE аудио-примера: `.project`, `.cproject`, `STM32F411VEHX_FLASH.ld`, плюс
    `Application\Startup\startup_stm32f411vehx.s`, `Application\User\syscalls.c`,
    `Application\User\sysmem.c` (стартап и syscalls лежат в подпапках `Application\`).

## 3. Установленная версия и источник

Установлена версия **1.28.0** (не 1.28.3). Видно из файла
`C:\Users\user\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.0\package.xml`:
`<PackDescription Release="FW.F4.1.28.0">`. Имя каталога согласуется с этим.

Примечание: каталог CubeFinder в CubeIDE знает и более свежую F4 **1.28.3** (см. раздел
«УТОЧНЕНИЕ A2–A5»), но установлена и распакована именно **1.28.0**.

## 4. Что ещё появилось в Repository помимо F4

В самом каталоге Repository (`C:\Users\user\STM32Cube\Repository`) на верхнем уровне
кроме `STM32Cube_FW_F4_V1.28.0` **ничего нового не появилось**: единственный прочий
элемент — файл `en.stm32h5cubemxbanner.png`, который был там ещё до установки (его
LastWriteTime 10:33:07 предшествует времени пакета 11:46:07). Других каталогов пакетов
(CMSIS-паков, X-CUBE, патчей и т.п.) в Repository нет.

Расхождение с ожиданием: владелец отметил, что после Refresh «доустановилось что-то
ещё» — но **в каталоге Repository** это не отражено, там только пакет F4. Если Refresh
что-то доустановил, оно легло не в этот Repository (возможно, во внутренний каталог
паков самой CubeIDE в её установке) — это вне указанного в задании пути и здесь не
проверялось.
