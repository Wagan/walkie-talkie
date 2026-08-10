# ОТЧЁТ: разбор эталонного примера ST (Audio_playback_and_record)

Только чтение. Источники (Repository, STM32Cube_FW_F4_V1.28.0):
- `Projects\STM32F411E-Discovery\Applications\Audio\Audio_playback_and_record\` (Src/Inc, проекты сред)
- `Drivers\BSP\STM32F411E-Discovery\stm32f411e_discovery_audio.c/.h`, `stm32f411e_discovery.c/.h`
- `Drivers\BSP\Components\cs43l22\cs43l22.c/.h`
Дата: 2026-08-10. Все номера строк ниже — из перечисленных файлов.

---

## 1. Захват с микрофона: режим I2S2

**Half-duplex master receive.** В `stm32f411e_discovery_audio.c`, функция `I2S2_Init()`
(строки 1080–1106):
- `hAudioInI2s.Init.Mode = I2S_MODE_MASTER_RX;`
- `Standard = I2S_STANDARD_LSB;`, `CPOL = I2S_CPOL_HIGH;`, `DataFormat = I2S_DATAFORMAT_16B;`
- `MCLKOutput = I2S_MCLKOUTPUT_DISABLE;`
- `AudioFreq = 2 * AudioFreq;` (передискретизация под PDM — I2S тактирует микрофон на 2×Fs).
- Instance = I2S2 (=SPI2).

Пины (MspInit, строки 918–936; определения в .h строки 96–107): задействованы **только два**:
| Пин | Сигнал | AF | Роль |
|---|---|---|---|
| PB10 | I2S2_CK (SCK) | GPIO_AF5_SPI2 | тактирование микрофона (CLK_IN / MP45DT02_CLK) |
| PC3 | I2S2_SD (в BSP назван «MOSI») | GPIO_AF5_SPI2 | приём PDM-данных (PDM_OUT / MP45DT02_DOUT) |

WS и ext_SD **не используются** (это следствие half-duplex master RX).

### Сравнение с нашим CubeMX (I2S2 FullDuplex, PB10/PB12/PC3/PC2)

**НЕ совпадает.** Отличия:
- Режим: эталон ST — **half-duplex `I2S_MODE_MASTER_RX`**; наш CubeMX — **FullDuplex**.
- Пины: эталон — **2 пина** (PB10 CK, PC3 SD); CubeMX — **4 пина** (добавлены PB12 WS и
  PC2 I2S2_ext_SD).
- Совпадают только PB10 (CK) и PC3 (SD). Лишние в нашей генерации PB12 и PC2 в BSP-тракте
  микрофона не нужны.
- Также `Standard`: BSP использует `I2S_STANDARD_LSB` для микрофона.

**Вывод:** чтобы переиспользовать штатный BSP-рекордер, I2S2 в нашем проекте нужно
переконфигурировать в half-duplex master receive (убрать WS/ext_SD) либо адаптировать код.

---

## 2. DMA

Определения — в `stm32f411e_discovery_audio.h`; параметры — в MspInit (.c).

### Приём с микрофона (I2S2 Rx) — BSP_AUDIO_IN_MspInit, строки 941–971
- Поток/канал: **DMA1_Stream3, DMA_CHANNEL_0** (.h строки 112–113).
- Direction = `DMA_PERIPH_TO_MEMORY`; PeriphInc = DISABLE; MemInc = ENABLE.
- Data align: периферия и память — **HALFWORD** (16 бит).
- **Mode = `DMA_CIRCULAR`** (непрерывный приём).
- **Priority = `DMA_PRIORITY_HIGH`**; FIFO = DISABLE; MBurst/PBurst = SINGLE.
- IRQ = `DMA1_Stream3_IRQn`, преэмпшн-приоритет `AUDIO_IN_IRQ_PREPRIO = 0x0F`.

### Передача в кодек (I2S3 Tx) — BSP_AUDIO_OUT_MspInit, строки 569–599
- Поток/канал: **DMA1_Stream7, DMA_CHANNEL_0** (.h строки 80–81).
- Direction = `DMA_MEMORY_TO_PERIPH`; MemInc = ENABLE.
- Data align: **HALFWORD/HALFWORD**.
- **Mode = `DMA_NORMAL`** (не circular; следующий буфер подаётся из callback'ов приложения).
- **Priority = `DMA_PRIORITY_HIGH`**; FIFO = ENABLE (threshold FULL); MBurst/PBurst = SINGLE.
- IRQ = `DMA1_Stream7_IRQn`, преэмпшн-приоритет `AUDIO_OUT_IRQ_PREPRIO = 0x0E`.

### Размеры буферов (waverecorder.c / audio.h)
- DMA-приём: `BSP_AUDIO_IN_Record(&InternalBuffer[0], INTERNAL_BUFF_SIZE)` — circular на
  `INTERNAL_BUFF_SIZE` полуслов. `INTERNAL_BUFF_SIZE = 128*Fs/16000*Ch` → для 16 кГц моно = **128** (uint16).
- `RecBuf[PCM_OUT_SIZE*2]` = **32** (стерео PCM, где `PCM_OUT_SIZE = Fs/1000 = 16`).
- `WrBuffer[WR_BUFFER_SIZE]`, `WR_BUFFER_SIZE = 4096` полуслов (накопление перед записью на USB).

### Обработка половины и конца передачи (приём, circular DMA — «пинг-понг»)
- HAL вызывает `HAL_I2S_RxHalfCpltCallback` → `BSP_AUDIO_IN_HalfTransfer_CallBack()`
  (waverecorder.c 261): конвертирует **первую** половину — `PDMToPCM(&InternalBuffer[0], &RecBuf[0])`.
- `HAL_I2S_RxCpltCallback` → `BSP_AUDIO_IN_TransferComplete_CallBack()` (228):
  конвертирует **вторую** половину — `PDMToPCM(&InternalBuffer[INTERNAL_BUFF_SIZE/2], &RecBuf[0])`.
- Оба копируют `PCM_OUT_SIZE*4` байт в `WrBuffer`; по заполнении половины/всего WrBuffer
  выставляется `AUDIODataReady=1`, и главный цикл пишет блок на USB-флешку.
- Передача (кодек): `HAL_I2S_TxHalfCpltCallback`/`TxCpltCallback` (audio.c 462–483) →
  `BSP_AUDIO_OUT_HalfTransfer_CallBack` / `..._TransferComplete_CallBack` (в waveplayer их
  реализует приложение, подавая следующий блок).

---

## 3. Тактирование PLLI2S и функция пересчёта

Функции пересчёта **есть** — это `BSP_AUDIO_OUT_ClockConfig()` и
`BSP_AUDIO_IN_ClockConfig()` (обе `__weak`), плюс таблица соответствия в начале
`stm32f411e_discovery_audio.c` (строки 141–144), приведена **как в исходнике**:

```c
/* These PLL parameters are valid when the f(VCO clock) = 1Mhz */
const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256,  429,   213,   429,   426,   271,   258,   344};
const uint32_t I2SPLLR[8] = {5,    4,     4,     4,     4,     6,     3,     1};
```

Условия: `f(VCO вход) = HSE/PLLM = 8 МГц/8 = 1 МГц` (везде `PLLI2SM = 8`). Отсюда
`I2SCLK = PLLI2SN/PLLI2SR` (МГц). Расчёт по таблице (для кодека/выхода):

| Fs, Гц | PLLI2SN | PLLI2SR | I2SCLK, МГц |
|---:|---:|---:|---:|
| 8000 | 256 | 5 | 51.2 |
| 11025 | 429 | 4 | 107.25 |
| 16000 | 213 | 4 | 53.25 |
| 22050 | 429 | 4 | 107.25 |
| 32000 | 426 | 4 | 106.5 |
| 44100 | 271 | 6 | 45.167 |
| 48000 | 258 | 3 | 86.0 |
| 96000 | 344 | 1 | 344.0 |

Важные нюансы фактических функций:
- **`BSP_AUDIO_IN_ClockConfig`** (тракт микрофона, строки 877–906) таблицу **не**
  использует: для Fs, кратной 8 (8/16/32/48/96) ставит `PLLI2SM=8, PLLI2SN=192,
  PLLI2SR=6 → I2SCLK = 32 МГц`; для 11.025/22.05/44.1 — `N=290, R=2 → 145 МГц`.
- **`BSP_AUDIO_OUT_ClockConfig`** (тракт кодека, 493–530) берёт из таблицы
  `I2SPLLN[idx]/I2SPLLR[idx]` только при idx==0 (8000), а для остальных ставит
  фиксированные `N=258, R=3` (проверка `(freqindex & 0x7)==0`). Приведено как факт из кода.
- `BSP_AUDIO_OUT_SetFrequency` (426–456) переключает PLLI2S по тому же принципу, что и IN
  (192/6 для кратных 8, иначе 290/2).

Расхождение с нашим CubeMX: наш проект задаёт PLLI2S фиксированно `M=5, N=200 → 160 МГц`,
что не совпадает ни с одной из схем эталона (таблица M=8; либо 32/145 МГц в ClockConfig).

---

## 4. Поддержанные частоты дискретизации

Фактически поддержанный набор — таблица `I2SFreq[8]`:
**8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 Гц.**

- **16 кГц — есть** (индекс 2). Это и есть значение по умолчанию:
  `DEFAULT_AUDIO_IN_FREQ = I2S_AUDIOFREQ_16K` (.h строка 135).
- **8 кГц — есть** (индекс 0).

Про минимальную ошибку: сам исходник ST **не приводит таблицу процентов ошибки**. По
логике кода деление на два семейства: кратные 8 кГц (8/16/32/48/96) и «дробные»
(11.025/22.05/44.1). Для тракта **микрофона** все кратные 8 кГц используют один
`I2SCLK = 32 МГц`, что даёт целочисленно-удобные коэффициенты и наименьшую ошибку именно
на 8/16/32/48/96 кГц; для тракта **кодека** таблица `I2SPLLN/I2SPLLR` подобрана ST так,
чтобы минимизировать ошибку на каждой из 8 стандартных частот. (Для сравнения: наш CubeMX
на своей схеме 160 МГц показал в .ioc ошибку I2S2 0.15 % на 96K и I2S3 −6.99 % — но это
артефакт нашей генерации, не этого примера.)

---

## 5. PDM2PCM: инициализация, обработка, параметры, вариант библиотеки

Инициализация — `PDMDecoder_Init()` (audio.c 1048–1071), вызывается из
`BSP_AUDIO_IN_Init` как `PDMDecoder_Init(AudioFreq, ChnlNbr, 2)`:
- Включается тактирование CRC (`__HAL_RCC_CRC_CLK_ENABLE()`) — обязательно для разблокировки PDM-библиотеки.
- На каждый входной канал: `bit_order = PDM_FILTER_BIT_ORDER_LSB`,
  `endianness = PDM_FILTER_ENDIANNESS_LE`, `high_pass_tap = 2122358088`,
  `out_ptr_channels = 2`, `in_ptr_channels = ChnlNbr`, затем `PDM_Filter_Init(&handler)`.
- Конфиг: `output_samples_number = AudioFreq/1000` (=16 при 16 кГц), `mic_gain = 24`,
  **`decimation_factor = PDM_FILTER_DEC_FACTOR_64`**, затем `PDM_Filter_setConfig(...)`.

Каналы: **вход 1** (`DEFAULT_AUDIO_IN_CHANNEL_NBR = 1`, один микрофон), **выход 2**
(сэмпл дублируется в стерео).

Обработка — `BSP_AUDIO_IN_PDMToPCM()` (audio.c 821–846):
- Байт-своп `HTONS` над `INTERNAL_BUFF_SIZE/2` словами во временный `AppPDM[]`.
- `PDM_Filter((uint8_t*)&AppPDM[i], (uint16_t*)&PCMBuf[i], &PDM_FilterHandler[i])` для каждого
  входного канала (здесь 1).
- Дублирование моно→стерео по `PCM_OUT_SIZE` отсчётам.

Размеры блоков (при Fs=16 кГц, моно):
- Вход PDM на один вызов: `INTERNAL_BUFF_SIZE/2 = 64` uint16 (половина circular-буфера).
- Полный circular-буфер приёма: `INTERNAL_BUFF_SIZE = 128` uint16.
- Выход PCM: `output_samples_number = PCM_OUT_SIZE = Fs/1000 = 16` отсчётов/канал (→ 32 стерео).
- Децимация: **64**; усиление микрофона: 24.

Вариант библиотеки — зависит от среды (F411 = Cortex-M4F). Фактически подключено:
| Среда | Файл библиотеки | Где прописано |
|---|---|---|
| **STM32CubeIDE (GCC)** | **`libPDMFilter_CM4_GCC_wc32.a`** | `STM32CubeIDE/.cproject` (строки 26/69/113/152) |
| EWARM (IAR) | `libPDMFilter_CM4_IAR_wc32.a` | `EWARM/Project.ewp:1148` |
| MDK-ARM (Keil) | `libPDMFilter_CM4_Keil_wc16.lib` | `MDK-ARM/Project.uvprojx:408/410` |

(Комментарий в `waverecorder.c:92` упоминает обобщённо `libPDMFilter_CM4_IAR.a`.)
**Для нашего пути (CubeIDE/GCC) актуален `libPDMFilter_CM4_GCC_wc32.a`.**

---

## 6. Минимально необходимые файлы BSP/Components (кодек на вывод + микрофон на вход)

Пути от корня `STM32Cube_FW_F4_V1.28.0`:

- `Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_audio.c`
- `Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_audio.h`
- `Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery.c` — реализует I2C1-шину и
  IO-функции для кодека, а также сброс кодека (см. ниже).
- `Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery.h` — определения:
  `DISCOVERY_I2Cx = I2C1` (PB6/PB9, AF4), `AUDIO_I2C_ADDRESS = 0x94`,
  `AUDIO_RESET_PIN = GPIO_PIN_4`/`AUDIO_RESET_GPIO = GPIOD` (PD4).
- `Drivers/BSP/Components/cs43l22/cs43l22.c`
- `Drivers/BSP/Components/cs43l22/cs43l22.h`
- `Drivers/BSP/Components/Common/audio.h` — подключается из `cs43l22.h` (`#include "../Common/audio.h"`).
- `Middlewares/ST/STM32_Audio/Addons/PDM/Inc/pdm2pcm_glo.h`
- `Middlewares/ST/STM32_Audio/Addons/PDM/Lib/libPDMFilter_CM4_GCC_wc32.a` (для GCC/CubeIDE).

Подразумеваются базовые слои HAL (I2S, I2C, DMA, RCC, GPIO, CORTEX) и CMSIS — они уже есть
в нашем сгенерированном проекте. Микрофон отдельного компонент-драйвера не требует
(обслуживается кодом `stm32f411e_discovery_audio.*` + PDM-библиотека).

---

## 7. Публикация

- Отчёт: `docs/REPORT_st_audio_example.md`.
- Хеш коммита: `<PLACEHOLDER>` · Push: `<PLACEHOLDER>`
