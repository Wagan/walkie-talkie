# ОТЧЁТ: демо loopback «микрофон → кодек» (TASK_audio_loopback_demo)

Дата: 2026-08-10. Нулевая лаба курса — проверка стенда: звук с MP45DT02 в реальном
времени на джек 3.5 мм через CS43L22, Fs = 16 кГц. Правила соблюдены: `.ioc`, HAL, CMSIS
и содержимое `Drivers/BSP` не менялись; код в `main.c` — только в блоках USER CODE; проект
не собирался. API BSP взят из фактических заголовков/исходников в проекте.

Опора: `docs/REPORT_st_audio_example.md`, `docs/REPORT_i2s_clock_analysis.md`. Единый PLLI2S
(решение владельца): **I2SCLK = 86 МГц, PLLI2SM = 8, PLLI2SN = 258, PLLI2SR = 3**.

---

## Созданные и изменённые файлы

Созданы:
- `App/Inc/audio_loopback.h` — интерфейс демо (Init / Process / HasError).
- `App/Src/audio_loopback.c` — реализация loopback.
- `App/Src/bsp_audio_clock.c` — перекрытие тактирования (единый PLLI2S).

Изменены:
- `.cproject` — `App` добавлен в область сборки, `App/Inc` в пути включения (обе конфигурации).
- `.gitignore` — добавлены `.cproject.bak.before-app` и `walkie-talkie.launch` (см. §Отладка).
- `Core/Src/main.c` — только в USER CODE: подключение заголовка, вызов Init и Process.

Резервная копия перед правкой: `.cproject.bak.before-app` (игнорируется, в репозиторий не идёт).

---

## Задача A. Каталог App и сборка

Создан `App/{Inc,Src}`. В `.cproject` (Debug и Release):
- в `<sourceEntries>` добавлена запись `<entry kind="sourcePath" name="App"/>` — теперь `App/Src/*.c`
  компилируются автоматически (проверено: `grep -c 'sourcePath name="App"' .cproject` → 2, тем же
  способом, что для Drivers);
- в include-пути добавлен `../App/Inc` (проверено: 2 вхождения). XML валиден, CRLF сохранён.

---

## Задача B. Перекрытие тактирования (`App/Src/bsp_audio_clock.c`)

Определены **сильные** одноимённые функции, перекрывающие `__weak` из BSP, с сигнатурами
**точно** как в `stm32f411e_discovery_audio.c`:
```c
void BSP_AUDIO_IN_ClockConfig (I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
void BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
```
Обе программируют один PLLI2S (M=8, N=258, R=3 → 86 МГц) тем же способом, что оригинал ST:
`RCC_PeriphCLKInitTypeDef` + `HAL_RCCEx_GetPeriphCLKConfig` / `HAL_RCCEx_PeriphCLKConfig`
(без второго стиля работы с RCC). В шапке файла — пояснение (один PLLI2S на кристалл, штатные
функции ставят разные значения, вторая инициализация ломает первую).

**Расчёт фактических частот при 86 МГц** (формула RM0383 §20.4.4, 16-битный кадр; записан
комментарием в самом файле):

| Fs | Тракт | MCLK | D=2·I2SDIV+ODD | I2SDIV, ODD | Реальн. Fs | Погрешность |
|---|---|---|---:|---|---:|---:|
| 16000 | выход | вкл | 20.996→21 | 10, 1 | 15997.02 | −0.0186 % |
| 16000 | вход (прогр.32000) | выкл | 83.98→84 | 42, 0 | 31994.05 (ауд. 15997.02) | −0.0186 % |
| 8000 | выход | вкл | 41.99→42 | 21, 0 | 7998.51 | −0.0186 % |
| 8000 | вход (прогр.16000) | выкл | 167.97→168 | 84, 0 | 15997.02 (ауд. 7998.51) | −0.0186 % |

**Вывод по 8 кГц (на будущее, этап радиоканала):** тот же PLLI2S 86 МГц даёт ≈ −0.019 % на обоих
трактах и на 8 кГц — **приемлемо**, менять решение не требуется.

---

## Задача C. Демо loopback (`App/Src/audio_loopback.c`, `App/Inc/audio_loopback.h`)

Интерфейс: `AudioLoopback_Init()`, `AudioLoopback_Process()`, `AudioLoopback_HasError()`.

**Использованные функции BSP (фактические сигнатуры из проекта):**
| Функция | Сигнатура | Единицы размера |
|---|---|---|
| Init выхода | `uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)` | Fs в Гц |
| Старт выхода | `uint8_t BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size)` | **байты** |
| Смена буфера | `void BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size)` | **слова (u16)** |
| Init входа | `uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)` | — |
| Старт входа | `uint8_t BSP_AUDIO_IN_Record(uint16_t* pbuf, uint32_t size)` | **слова (u16)** |
| PDM→PCM | `uint8_t BSP_AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)` | — |
| Светодиоды | `void BSP_LED_Init/On/Off/Toggle(Led_TypeDef)` | LED4/5/6 |
| Перекрыты (__weak) | `BSP_AUDIO_IN_ClockConfig`, `BSP_AUDIO_OUT_ClockConfig` | см. Задачу B |
| Реализованы колбэки (__weak) | `BSP_AUDIO_IN_HalfTransfer_CallBack`, `BSP_AUDIO_IN_TransferComplete_CallBack`, `BSP_AUDIO_OUT_TransferComplete_CallBack`, `BSP_AUDIO_IN_Error_Callback`, `BSP_AUDIO_OUT_Error_CallBack` | точные имена ST |

(Имена колбэков ошибок у ST РАЗНЫЕ: у входа `Error_Callback`, у выхода `Error_CallBack` — взяты
буквально из заголовка, чтобы перекрытие связалось.)

**Схема прохождения данных и размеры на каждом шаге:**
```
MP45DT02 --PDM--> I2S2 (master RX, MCLK off, прогр. 2·Fs=32000)
   --DMA1_Stream3 (кольцевой)--> pdmBuf[INTERNAL_BUFF_SIZE] = 128 слов u16
   (полбуфера = 64 слова PDM на один вызов; половина=pdmBuf[0..63], конец=pdmBuf[64..127])
   --BSP_AUDIO_IN_PDMToPCM (децимация 64; HTONS внутри; 1 вход→2 выхода)-->
   pcmBuf[slot] = PCM_OUT_SIZE·2 = 32 слова (16 стерео-кадров = 1 мс звука)
   --BSP_AUDIO_OUT_ChangeBuffer(32 слова)--> I2S3 (master TX, MCLK on)
   --DMA1_Stream7 (NORMAL)--> CS43L22 --> джек 3.5 мм
```
Откуда числа: `INTERNAL_BUFF_SIZE = 128·DEFAULT_AUDIO_IN_FREQ/16000·DEFAULT_AUDIO_IN_CHANNEL_NBR
= 128·(16000/16000)·1 = 128`; `PCM_OUT_SIZE = DEFAULT_AUDIO_IN_FREQ/1000 = 16`; PCM-слот стерео
`= PCM_OUT_SIZE·2 = 32`. Все — макросы BSP, раскрытие под Fs=16000. Стартовый размер Play в байтах
`= PCM_OUT_SIZE·2·AUDIODATA_SIZE = 32·2 = 64`; Record в словах `= INTERNAL_BUFF_SIZE = 128`.

**Реализация по пунктам задания:**
1. Порядок Init: **сначала выход** (кодек: I2C-конфиг CS43L22 + I2S3), **затем вход** (I2S2 + PDM).
   Обоснование в коде: оба ClockConfig перекрыты одним PLLI2S, поэтому вторая инициализация не
   сбивает частоту первой; кодек поднимаем первым, чтобы к старту приёма выход был готов.
2. Приём — **кольцевой DMA** на `pdmBuf[128]`, как в эталоне.
3. В колбэках половины/конца — `BSP_AUDIO_IN_PDMToPCM` с исходными параметрами (децимация 64,
   HTONS-подготовка внутри функции), half → `pdmBuf[0]`, complete → `pdmBuf[64]` (как у ST).
4. Выход — **DMA_NORMAL** (проверено по `BSP_AUDIO_OUT_MspInit`), поэтому следующий блок подаётся в
   `BSP_AUDIO_OUT_TransferComplete_CallBack` через `BSP_AUDIO_OUT_ChangeBuffer` — способ ST (waveplayer).
5. **Развязка:** двойной буфер `pcmBuf[2][32]`. Микрофонный колбэк пишет слот `pcmFillIdx`, публикует
   его в `pPcmReady`, переключает индекс; кодек играет `pPcmReady`. Слот в работе кодека не
   переписывается (микрофон пишет другой). Оба тракта на одном PLLI2S → строго 16 кГц с двух сторон,
   без дрейфа; двух слотов достаточно (задержка ≈1–2 мс). Публикация указателя атомарна на M4.
6. **CRC:** проверено по коду — `PDMDecoder_Init()` (из `BSP_AUDIO_IN_Init`) делает
   `__HAL_RCC_CRC_CLK_ENABLE()`; отдельно в нашем коде включать не нужно (и не включаем).
7. **Светодиоды:** LED4 (зелёный) — успешная инициализация (горит); LED6 (синий) — мигает на каждый
   обработанный блок; LED5 (красный) — ошибка.
8. **Ошибки явные:** любой ненулевой возврат BSP → `loopbackError=1`, LED5 On, `AudioLoopback_Init`
   возвращает `AUDIO_ERROR`; флаг виден снаружи через `AudioLoopback_HasError()`. Колбэки ошибок
   трактов тоже поднимают флаг и зажигают LED5.

---

## Задача D. Подключение в main.c и обработчики

В `Core/Src/main.c`, только в USER CODE:
- `USER CODE BEGIN Includes`: `#include "audio_loopback.h"`
- `USER CODE BEGIN 2` (после `MX_GPIO_Init`): `AudioLoopback_Init();`
- `USER CODE BEGIN 3` (в `while(1)`): `AudioLoopback_Process();`
Других изменений в main.c нет (diff: +4/−2, всё внутри блоков).

**Проверка DMA-обработчиков:** BSP использует DMA1_Stream3 (приём, I2S2) и DMA1_Stream7 (передача,
I2S3) — по `stm32f411e_discovery_audio.h` (`I2S2_DMAx_STREAM=DMA1_Stream3`, `I2S3_DMAx_STREAM=DMA1_Stream7`).
В `Core/Src/stm32f4xx_it.c` **оба** обработчика уже есть (`DMA1_Stream3_IRQHandler` →
`hAudioInI2s.hdmarx`, `DMA1_Stream7_IRQHandler` → `hAudioOutI2s.hdmatx`). Обработчик передачи на
кодек присутствует — **добавлять ничего не потребовалось**.

---

## Задача E. Конфигурация отладки walkie-talkie.launch

Проверено содержимое (82 строки). Найдена **машинно-специфичная привязка**:
- строка 35: `com.st.stm32cube.ide.mcu.debug.stlink.log_file` =
  `C:\Users\user\STM32CubeIDE\workspace_1.16.0\walkie-talkie\Debug\st-link_gdbserver_log.txt`
  — абсолютный путь к домашнему каталогу владельца.

Серийного номера отладчика нет: строка 39 `stlink_check_serial_number=false`, строка 40
`stlink_txt_serial_number=""` (пусто). Остальное — относительные пути (`Debug/walkie-talkie.elf`),
`localhost`, порты, имя проекта — не машинно-специфично.

**Решение:** из-за абсолютного пути в строке 35 файл **не коммитим**, внесён в `.gitignore`.

---

## Что проверить владельцу при первом запуске (и как выглядит успех)

Подготовка: активные колонки в джек 3.5 мм (CN4), кит по USB (ST-LINK), собрать и прошить в IDE.

Ожидаемое поведение:
1. Сразу после старта — **зелёный LED4 горит** ровно: инициализация BSP прошла (кодек по I2C
   ответил, микрофон и DMA запущены).
2. **Синий LED6 часто мигает** (переключается каждый обработанный блок ≈ каждую 1 мс, глазом —
   ровное свечение/быстрое мерцание): идёт поток PDM→PCM.
3. В колонках — **звук с микрофона в реальном времени** (постучать/поговорить рядом с MEMS-микрофоном
   у края платы). Небольшая задержка ≈1–2 мс незаметна.
4. **Красный LED5 не горит.** Если LED5 загорелся или LED4 не зажёгся — ошибка BSP
   (`AudioLoopback_HasError()` вернёт 1): проверить питание/пайку кодека, подключение колонок,
   и что HAL-модули I2S/I2C/SPI включены, а сборка чистая.

Диагностика без звука: LED4 горит, LED6 мигает, но тихо → проверить громкость/устройство вывода
(джек), целостность колонок. LED6 не мигает → не идёт приём с микрофона (DMA/тактирование).

---

## Публикация

- Отчёт: `docs/REPORT_audio_loopback_demo.md`.
- `walkie-talkie.launch` в коммит не входит (см. Задачу E).
- Хеш коммита: **`50721d7f51f65634a34db11f2882f3502f404636`** · Push: OK (`abe5775..50721d7 main -> main`, exit 0).
- (Хеш вписан вторым коммитом.)
