# ОТЧЁТ: инвентаризация сгенерированного проекта walkie-talkie

Проект: walkie-talkie (CubeIDE, Board Selector → STM32F411E-DISCO, инициализация
периферии платы по умолчанию). Дата: 2026-08-10.
Метод: только чтение. Проект не собирался, код не правился.
Источник фактов раздела 2 — файл `walkie-talkie.ioc` (MxCube 6.12.0, DB.6.0.120).

---

## 1. Структура проекта

Верхний уровень (`C:\Users\user\STM32CubeIDE\workspace_1.16.0\walkie-talkie`):

- Каталоги: `.git/`, `.settings/`, `Core/`, `Drivers/`, `Middlewares/`, `USB_HOST/`, `docs/`
- Файлы проекта: `walkie-talkie.ioc`, `.cproject`, `.project`, `.mxproject`,
  `STM32F411VETX_FLASH.ld`, `STM32F411VETX_RAM.ld`, `.gitignore`, `README.md`

`Core/Inc`: `main.h`, `stm32f4xx_hal_conf.h`, `stm32f4xx_it.h`
`Core/Src`: `main.c`, `stm32f4xx_hal_msp.c`, `stm32f4xx_it.c`, `syscalls.c`, `sysmem.c`,
`system_stm32f4xx.c`

`Drivers/`: только **`CMSIS`** и **`STM32F4xx_HAL_Driver`** (каждый с `Inc`/`Src`/…).
`Middlewares/`: только `ST/STM32_USB_Host_Library` (`Core`, `Class`).
`USB_HOST/`: `App/usb_host.c/.h`, `Target/usbh_conf.c/.h`, `Target/usbh_platform.c/.h`.

---

## 2. Конфигурация из walkie-talkie.ioc

MCU: **STM32F411VETx** (CPN STM32F411VET6), корпус LQFP100, плата `STM32F411E-DISCO`.
FirmwarePackage: `STM32Cube FW_F4 V1.28.0`. LibraryCopy=1 (исходники копируются в проект).

### 2.1. Включённые периферийные блоки (Mcu.IP0…IP8)

`I2C1`, `I2S2`, `I2S3`, `NVIC`, `RCC`, `SPI1`, `SYS`, `USB_HOST`, `USB_OTG_FS` (всего 9).

- **I2S2** (instance SPI2): AudioFreq=`I2S_AUDIOFREQ_96K`, RealAudioFreq=**96.153 KHz**,
  ErrorAudioFreq=0.15 %, **FullDuplexMode=ENABLE**, VirtualMode=`I2S_MODE_MASTER`.
  → тракт **MEMS-микрофона MP45DT02** (PDM), полный дуплекс (задействован I2S2_ext).
- **I2S3** (instance SPI3): AudioFreq=`I2S_AUDIOFREQ_96K`, RealAudioFreq=**89.285 KHz**,
  ErrorAudioFreq=−6.99 %, FullDuplexMode=DISABLE, VirtualMode=`I2S_MODE_MASTER`.
  → тракт **аудиокодека CS43L22** (выход на наушники/динамик).
- **I2C1**: включён; отдельных параметров скорости в .ioc не сохранено (значения по
  умолчанию). → управление **CS43L22** (control interface).
- **SPI1**: Mode=`SPI_MODE_MASTER`, Direction=`2LINES`, BaudRatePrescaler=2,
  CalculateBaudRate=**48.0 MBits/s**. → гироскоп **L3GD20**.
- **USB_OTG_FS**: VirtualMode=`Host_Only`, phy_itface=`HCD_PHY_EMBEDDED`.
- **USB_HOST**: класс `Cdc` (VirtualModeFS=Cdc), handle `hUsbHostFS`; функция включения
  VBUS через GPIO PC0 (`Drive_VBUS_FS`).
- **NVIC**: включён `OTG_FS_IRQn`; `NVIC.ForceEnableDMAVector=true`; PriorityGroup=0.

**DMA:** отдельного блока DMA в списке периферии (Mcu.IP*) **НЕТ**. В .ioc присутствует
только флаг `NVIC.ForceEnableDMAVector=true`; каналов/потоков DMA не сконфигурировано.
(Расхождение с ожиданием: в задании DMA перечислен, но в дефолтной генерации он не задан.)

### 2.2. Назначение выводов (пин → функция)

Аудиокодек CS43L22 (I2S3 + I2C1 + сброс):
| Пин | Функция | Метка |
|---|---|---|
| PA4 | I2S3_WS | CS43L22_LRCK |
| PC7 | I2S3_MCK | CS43L22_MCLK |
| PC10 | I2S3_CK | CS43L22_SCLK |
| PC12 | I2S3_SD | CS43L22_SDIN |
| PB6 | I2C1_SCL (AF_OD) | Audio_SCL / CS43L22_SCL |
| PB9 | I2C1_SDA (AF_OD) | Audio_SDA / CS43L22_SDA |
| PD4 | GPIO_Output | Audio_RST / CS43L22_RESET |

MEMS-микрофон MP45DT02 (I2S2 full-duplex, PDM):
| Пин | Функция | Метка |
|---|---|---|
| PB10 | I2S2_CK | CLK_IN / MP45DT02_CLK |
| PB12 | I2S2_WS | — |
| PC3 | I2S2_SD | PDM_OUT / MP45DT02_DOUT |
| PC2 | I2S2_ext_SD | — |

Гироскоп L3GD20 (SPI1) и его EXTI:
| Пин | Функция | Метка |
|---|---|---|
| PA5 | SPI1_SCK | L3GD20_SC/SPC |
| PA6 | SPI1_MISO | L3GD20_AS0/SDO |
| PA7 | SPI1_MOSI | L3GD20_SDA/SDI/SDO |
| PE3 | GPIO_Output | L3GD20_CS_I2C/SPI |
| PE1 | EXTI1 (EVT rising) | L3GD20_INT2 |

USB OTG FS (Host):
| Пин | Функция | Метка |
|---|---|---|
| PA9 | USB_OTG_FS_VBUS | VBUS_FS |
| PA10 | USB_OTG_FS_ID | OTG_FS_ID |
| PA11 | USB_OTG_FS_DM | OTG_FS_DM |
| PA12 | USB_OTG_FS_DP | OTG_FS_DP |
| PC0 | GPIO_Output (set) | OTG_FS_PowerSwitchOn |
| PD5 | GPIO_Input | OTG_FS_OverCurrent |

Прочее (акселерометр/магнитометр LSM303DLHC, светодиоды, отладка, тактирование):
| Пин | Функция | Метка |
|---|---|---|
| PE2 | GPIO_Input | LSM303DLHC_DRDY |
| PE4 | EXTI4 (EVT rising) | LSM303DLHC_INT1 |
| PE5 | EXTI5 (EVT rising) | LSM303DLHC_INT2 |
| PA0-WKUP | EXTI0 (EVT rising) | — |
| PD12 | GPIO_Output | LD4 Green |
| PD13 | GPIO_Output | LD3 Orange |
| PD14 | GPIO_Output | LD5 Red |
| PD15 | GPIO_Output | LD6 Blue |
| PA13 | SYS_JTMS-SWDIO | SWDIO |
| PA14 | SYS_JTCK-SWCLK | SWCLK |
| PB3 | SYS_JTDO-SWO | SWO |
| PH0 | RCC_OSC_IN | HSE-External-Clock-Source |
| PH1 | RCC_OSC_OUT | HSE-External-Clock-Source |
| PC14 | RCC_OSC32_IN | LSE-External-Oscillator |
| PC15 | RCC_OSC32_OUT | LSE-External-Oscillator |

### 2.3. Тактирование (RCC)

- Источник SYSCLK: **PLLCLK** (`RCC.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK`).
- HSE: **8 МГц** (`HSE_VALUE=8000000`), режим **HSE-External-Clock-Source** (тактовый
  вход от ST-LINK MCO, не кварц). Дополнительно LSE (внешний осциллятор 32.768 кГц) на PC14/PC15.
- **SYSCLK = 96 МГц** (`SYSCLKFreq_VALUE=96000000`).
- Главный PLL: **PLLM=4**, **PLLP=RCC_PLLP_DIV4**, **PLLQ=8**.
  Производные (как записаны в .ioc): VCOInput=2 МГц, VCOOutput=384 МГц → SYSCLK 96 МГц;
  PLLQ-выход = **48 МГц** (тактирование USB FS). PLLN отдельным ключом в .ioc не хранится
  (из VCOOutput 384 МГц / VCOInput 2 МГц следует N=192 — приведено как расчёт, не как факт из файла).
- Шины: **AHB/HCLK = 96 МГц**, **APB1 = 24 МГц** (`APB1CLKDivider=RCC_HCLK_DIV4`),
  **APB2 = 96 МГц**. Cortex/FCLK = 96 МГц.
- **Тактирование I2S (отдельный PLLI2S):** **PLLI2SM=5**, **PLLI2SN=200**.
  VCOInputM=1.6 МГц, VCO(I2S)=320 МГц, **I2SClocksFreq_Value = 160 МГц**
  (`VcooutputI2S=160000000`). Именно от этих 160 МГц берётся частота обоих I2S (96K профиль).

---

## 3. BSP платы

В `Drivers/` присутствуют **только `CMSIS` и `STM32F4xx_HAL_Driver`**. Папки
**`Drivers/BSP` НЕТ**. Следовательно, драйверов уровня BSP платы в проекте нет.

**Явно:** аудиокодек CS43L22 и MEMS-микрофон MP45DT02 в этом проекте **BSP не
обслуживаются**. CubeMX лишь сконфигурировал сырую периферию (I2S3+I2C1 под кодек,
I2S2 под микрофон) и назначил выводы; готового кода драйвера кодека/микрофона
(тип `stm32f411e_discovery_audio.*`, `cs43l22.*`) в проекте нет. Их пришлось бы
добавить отдельно (из пакета) или написать поверх HAL.

---

## 4. Библиотека PDM2PCM

В проекте **отсутствует**. В `Middlewares/` есть только `STM32_USB_Host_Library`;
каталога/файлов PDM2PCM (`Addons/PDM`, `pdm2pcm_glo.h`, `libPDMFilter_*`) нет.
Единственное совпадение по «pdm» во всём дереве — это PDF-даташит в `docs/reference`
(в репозиторий не входит). То есть преобразование PDM→PCM в текущей генерации не подключено.

---

## 5. Конфигурации сборки (из .cproject)

Существуют две конфигурации, обе тип exe (артефакт `${ProjName}.elf`):
- **Debug** — id `com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug.408357737`,
  buildType=…buildType.debug.
- **Release** — id `com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.release.1901236340`,
  buildType=…buildType.release.

---

## 6. Достаточность .gitignore

Текущий `.gitignore` **достаточен** для сгенерированного проекта — ничего лишнего в
коммит не попадёт. Проверка:

- Артефакты сборки: каталоги `Debug/` и `Release/`, а также `*.o *.d *.su *.map *.list
  *.elf *.bin *.hex` уже исключены. (Сейчас проект не собран, `Debug/` ещё нет, но
  правило его перехватит.)
- Служебные файлы среды: каталог `.settings/` содержит ровно два файла —
  `language.settings.xml` и `stm32cubeide.project.prefs` — и оба перечислены в
  `.gitignore`. Подтверждение: `git status` НЕ показывает `.settings/` среди
  неотслеживаемого (оба файла отфильтрованы).
- `git status --porcelain` в неотслеживаемом показал только легитимные исходники/конфиги
  проекта: `.cproject`, `.project`, `.mxproject`, `walkie-talkie.ioc`, `*.ld`, `Core/`,
  `Drivers/`, `Middlewares/`, `USB_HOST/` — всё это должно быть в репозитории.

**Изменений в `.gitignore` не потребовалось** (добавлять нечего — всё, что не должно
коммититься, уже покрыто). На будущее замечание (не добавлялось): при первом запуске
отладчика CubeIDE может создать файл `*.launch`; если решите не хранить его в репо,
тогда стоит добавить строку `*.launch` — но сейчас такого файла нет, и это на усмотрение владельца.

---

## 7. Публикация

- Коммит проекта (все файлы верхнего уровня + Core/Drivers/Middlewares/USB_HOST + отчёт),
  всего 129 отслеживаемых файлов добавлено.
- Хеш коммита проекта: **`71c9f04c1942ee6f93bdfa34e1e9df542ceeec38`**
- Push в `origin/main`: OK (`7126956..71c9f04  main -> main`, exit 0).
- Проверки после коммита: `git ls-files docs/` не содержит `reference` (не утекло);
  среди отслеживаемых файлов нет `*.o/*.elf/*.map/*.bin/*.hex` и каталогов `Debug/`/`Release/`.

> Раздел 7 с хешем зафиксирован отдельным вторым коммитом (хеш выше — состояние проекта
> на момент публикации).
