/**
  ******************************************************************************
  * @file    App/Common/Src/voice.c
  * @author  Wagan Sarukhanov
  * @brief   Общий голосовой движок (сжатие речи + радио): захват→кодек→кадр→UART→
  *          джиттер→декод→воспроизведение, лесенка кодеков (raw/µ-law/ADPCM/Codec2),
  *          децимация 16→8 с антиалиасом, headroom, PTT, offload, probe C. Реализует
  *          Voice_Init/Voice_Process; работы (LAB07 сжатие, LAB08 радио) — тонкие адаптеры.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Вынесено из App/Labs/speech.c (этап 1 разделения LAB07/LAB08, TASK_lab08_split_recon) БЕЗ
  * изменения поведения. Компилируется в конфигурациях LAB07 и LAB08 (guard ниже).
  *
  * База — LAB04 (кадрирование SLIP, джиттер-буфер, PTT, полудуплекс). Отличие: перед
  * упаковкой блок отсчётов кодируется выбранным кодеком (App/Common/codec.c), после
  * распаковки — декодируется. В кадре передаётся признак кодека и частоты, чтобы приёмник
  * знал, чем и как раскодировать. Частота 8 кГц реализована ПРОГРАММНЫМ децимированием
  * 16→8 (BSP остаётся на 16 кГц — его буферы/PDM жёстко под 16 кГц, см. отчёт, задача A).
  * Децимация — линейно-фазовым FIR-НЧ (антиалиас ~3.6 кГц, decim fir) с возможностью вернуть
  * старое усреднение пар (decim avg) для сравнения. Это подготовка к Codec2: вокодер
  * анализирует спектр, и алиасинг слабой децимации портил бы разборчивость.
  * Воспроизведение всегда 16 кГц (аппаратный тракт неизменен): принятый 8-кГц звук
  * интерполируется обратно в 16 кГц.
  *
  * Печать из ISR запрещена. Приёмный ISR лишь ставит готовый кадр в очередь (frame_enqueue);
  * тяжёлый разбор (Codec_Decode, апсемплинг, jb_push = voice_sink) вынесен в Voice_Process
  * (rx_drain), чтобы приёмный ISR не задерживал 1-мс пере-взвод аудио-выхода — см.
  * docs/REPORT_rx_isr_offload.md и docs/REPORT_isr_deadline_probe.md.
  ******************************************************************************
  */

#include "lab.h"
#include "uwb_config.h"   /* UWB_CHIP_DW3000: речь по UWB собирается только для пути DW3000 */

#if (LAB_ID == 7) || (LAB_ID == 8) || ((LAB_ID == 9) && defined(UWB_CHIP_DW3000))

/* Транспорт кадров: LAB07/08 — провод (uart_port + кадрирование SLIP/CRC), LAB09 — радио UWB
 * (готовые пакеты со своей CRC, SLIP не нужен). Разводится макросом; поведение LAB07/08 не меняется. */
#define VOICE_XPORT_UART  ((LAB_ID == 7) || (LAB_ID == 8))
#define VOICE_XPORT_UWB   (LAB_ID == 9)

#include "voice.h"
#include "console.h"
#if VOICE_XPORT_UART
#include "uart_port.h"
#include "frame.h"
#endif
#if VOICE_XPORT_UWB
#include "dw3000_port.h"
#endif
#include "audio.h"
#include "codec.h"
#include "preempt.h"
#include "codec2.h"          /* вокодер Codec2 (ThirdParty) — только для команды c2load (замер) */
#include "trace_log.h"
#include "stm32f4xx_hal.h"
#include "stm32f411e_discovery.h"
#include <stdlib.h>
#include <string.h>

/* ================= ПАРАМЕТРЫ ================= */
#define BLOCK_MS        5u                       /* мс звука в блоке/кадре */
#define SAMP16          (AUDIO_BLOCK_SAMPLES * BLOCK_MS)   /* 80 отсчётов 16 кГц в блоке */
#define SAMP8           (SAMP16 / 2u)            /* 40 отсчётов 8 кГц */
#define VOICE_HDR       4u                       /* заголовок кадра: seq(u16) codec(u8) rate(u8) */
#define PAYLOAD_MAX     (VOICE_HDR + SAMP16 * 2u)/* худший случай: raw 16 кГц = 160 байт + hdr */
#define ENC_MAX         (2u + 2u * (PAYLOAD_MAX + 2u))     /* худший SLIP */

#define JB_SIZE         4096u                    /* ёмкость джиттер-буфера: 256 мс @16 кГц (было 128) */
#define JB_MASK         (JB_SIZE - 1u)
/* Prefill — порог старта воспроизведения (наполнение). Настраивается командой `prefill [ms]`.
 * Умолчание поднято до 60 мс (field): у HC-12 задержка передачи гуляет 4–80 мс, буфер, набирающий
 * меньше джиттера канала, работать не может. Ёмкость 256 мс поднята, т.к. HC-12 отдаёт кадры пачками:
 * пачка ~80 мс поверх наполнения 60 мс при прежних 128 мс давала переполнение. ms→отсчёты = ×16
 * (AUDIO_BLOCK_SAMPLES = отсчётов на 1 мс при 16 кГц). Лимит команды — не выше половины ёмкости. */
#define JB_PREFILL_MS_DEFAULT  60u               /* умолчание prefill, мс */
#define JB_PREFILL_MAX         (JB_SIZE / 2u)    /* максимум prefill, отсчёты (= половина ёмкости = 128 мс) */
#define LOSS_FILL_CAP   8u

#define RATE_8K         0u
#define RATE_16K        1u

#define WIRE_CODEC2     3u                       /* id вокодера Codec2 в заголовке кадра (вне codec.c) */

/* Рабочая скорость линии по умолчанию для LAB07 (field default): под HC-12 FU3 9600 (Codec2 3200).
 * Ставится в Voice_Init через UartPort_SetBaud, чтобы плата стартовала готовой к эфиру без команд —
 * иначе после прошивки baud берётся из .ioc (см. грабли хендоффа). Консоль (USB CDC) от этого не
 * зависит: она на USB, линия — на USART2. Команда baud по-прежнему переопределяет на живой плате. */
#define LINE_BAUD_DEFAULT   9600u

/* Антиалиас-фильтр децимации 16→8 (см. docs/REPORT_codec2_recon.md и REPORT_aa_decim.md).
 * Старый способ (усреднение пар) — очень слабый НЧ: тон 5 кГц проходил и отражался в 3 кГц.
 * Новый — линейно-фазовый FIR-НЧ перед прореживанием 2:1. Оба доступны (команда decim). */
#define AA_TAPS         31u                      /* тапов FIR (нечётное → линейная фаза, тип I) */
#define DECIM_AVG       0u                       /* старый способ: усреднение пар */
#define DECIM_FIR       1u                       /* новый: FIR-НЧ ~3.6 кГц + прореживание */

/* ================= НАСТРОЙКИ (по умолчанию) ================= */
static volatile uint8_t  g_codec = WIRE_CODEC2;          /* field default: вокодер Codec2 (готов к HC-12) */
static volatile uint8_t  g_rate  = RATE_8K;              /* field default: 8 кГц (Codec2 только 8 кГц) */
static volatile uint8_t  g_decim = DECIM_FIR;            /* метод децимации 16→8 (по умолч. FIR) */
static volatile uint16_t g_jbPrefill = (uint16_t)(JB_PREFILL_MS_DEFAULT * AUDIO_BLOCK_SAMPLES); /* порог старта, отсчёты (60 мс) */
static volatile uint8_t  g_ptt   = 0u;
static volatile uint8_t  g_tone  = 0u;
static volatile uint16_t g_toneHz = 1000u;              /* частота тона (Гц), по умолчанию 1 кГц */
static uint8_t           g_pttCmd = 0u;
static volatile uint8_t  g_c2mode = 0u;                 /* индекс режима Codec2 в c2ModeTab (0 = 3200) */

/* Запас по уровню (headroom) ПЕРЕД кодеком (TASK_input_headroom). Микрофонный PDM-фильтр BSP отдаёт
 * PCM с высоким фиксированным усилением (mic_gain=24, BSP не трогаем); на громкой речи отсчёты
 * подходят к полной шкале → насыщение (FIR, предсказатель ADPCM) и перегрузка ADPCM по крутизне =
 * хрип (docs/REPORT_adpcm_overload_recon.md). Лечим аттенюацией мик-PCM в НАШЕМ коде (LAB07-локально,
 * audio.c/LAB00/LAB04 не трогаем). Усиление — Q8 (256 = ×1 = 0 дБ), целочисленно, без деления в
 * горячем пути. Тон (test-сигнал) НЕ аттенюируется — он подменяет сигнал уже в tx_flush. */
#define HR_NEAR_FS   32000                             /* |отсчёт| >= порога = «подход к полной шкале» (~0.2 дБ от FS) */
static volatile uint16_t g_hrGain = 128u;              /* Q8-усиление: 128 = ×0.5 = −6 дБ (ПОДТВЕРЖДЁН на железе) */
static volatile uint8_t  g_hrDb   = 6u;                /* аттенюация в дБ (для печати) */

/* Измеритель уровня: пик |PCM| и число отсчётов у полной шкалы — ДО и ПОСЛЕ аттенюации (мик-путь,
 * только при передаче). Пишет аудио-ISR (единственный писатель), читает voice; сброс — reset.
 * ВАЖНО (проверено на железе): истинный индикатор перегрузки на входе — hrPeakPre (=32768 → срез в
 * PDM-фильтре ВЫШЕ нашей точки). hrClipPost/«post nearFS» — ложный зелёный: обнуляется просто от
 * масштабирования и не видит апстрим-срез. Аттенюация лечит перегрузку ADPCM ПО КРУТИЗНЕ (даёт
 * headroom кодеку), а не срез PDM-фильтра (он остаётся, но на слух безвреден — см. REPORT_input_headroom.md). */
static volatile int32_t  hrPeakPre = 0, hrPeakPost = 0;
static volatile uint32_t hrClipPre = 0u, hrClipPost = 0u;

/* ================= ПЕРЕДАЧА (аудио-ISR) ================= */
static int16_t  acc16[SAMP16];
static uint16_t accN = 0u;
static uint16_t txSeq = 0u;
static uint8_t  txPayload[PAYLOAD_MAX];
#if VOICE_XPORT_UART
static uint8_t  txEnc[ENC_MAX];          /* выход SLIP-кодера — только провод */
#endif

static const int16_t toneLUT[16] =
{ 0, 3062, 5657, 7391, 8000, 7391, 5657, 3062, 0, -3062, -5657, -7391, -8000, -7391, -5657, -3062 };
static uint32_t tonePhaseAcc = 0u;   /* фаза NCO (Q32); индекс = биты 31..28, дробь — 27..12 */

/* FIR-НЧ антиалиас для децимации 16→8 (Q15, окно Хэмминга, fc≈3600 Гц @16 кГц).
 * АЧХ (проверено численно): 0..3 кГц ≈ 0 дБ, 3.4 кГц −3.2 дБ, 4.0 кГц −16.6 дБ,
 * 4.6 кГц −54 дБ, 5.0 кГц −60 дБ, 6..8 кГц −64..−78 дБ. Симметричен (линейная фаза,
 * групповая задержка 15 отсчётов 16 кГц). Сумма = 32768 → единичное усиление по DC. */
static const int16_t aaCoef[AA_TAPS] =
{
      39,     54,    -44,   -138,     34,    323,     72,   -609,
    -397,    957,   1133,  -1296,  -2819,   1544,  10175,  14712,
   10175,   1544,  -2819,  -1296,   1133,    957,   -397,   -609,
      72,    323,     34,   -138,    -44,     54,     39,
};
static int16_t aaHist[AA_TAPS - 1u];   /* хвост предыдущего блока 16 кГц (непрерывность FIR) */

/* ================= ПРИЁМ / ДЖИТТЕР-БУФЕР (16 кГц) ================= */
#if VOICE_XPORT_UART
static Frame_Decoder     rxDec;          /* SLIP-декодер — только проводной транспорт */
#endif
static uint16_t          rxExpSeq = 0u;
static uint8_t           rxHaveSeq = 0u;
static int16_t           jbuf[JB_SIZE];
static volatile uint16_t jbHead = 0u, jbTail = 0u;
static volatile uint8_t  jbPlaying = 0u;
static int16_t           lastOut[AUDIO_BLOCK_SAMPLES];

/* ================= ЗАМЕР ЗАГРУЗКИ ЯДРА (DWT) ================= */
static volatile uint32_t encCyc = 0u, encCnt = 0u;
static volatile uint32_t decCyc = 0u, decCnt = 0u;

/* ================= СЧЁТЧИКИ ================= */
static volatile uint32_t cRec = 0u, cSent = 0u, cTxDrop = 0u, cRecv = 0u, cLost = 0u;
static volatile uint32_t cPlayed = 0u, cUnder = 0u, cOver = 0u;
static volatile uint32_t cQover = 0u;                 /* offload: кадр принят, но очередь полна */
static volatile uint32_t loopMaxCyc = 0u;             /* макс. интервал между вызовами Voice_Process (тактов) */
static uint32_t          loopPrevCyc = 0u;
static uint8_t           loopHavePrev = 0u;

/* ================= ОЧЕРЕДЬ КАДРОВ: вынос разбора из ISR (TASK_rx_isr_offload) =================
 * Приёмный ISR (uart_port RxEventCallback → voice_rx_byte → Frame_DecodeByte) при готовом
 * КАДРЕ лишь КОПИРУЕТ его в очередь (frame_enqueue) и двигает head. Тяжёлый разбор
 * (заголовок, Codec_Decode, апсемплинг, jb_push = voice_sink) выполняется в Voice_Process
 * (rx_drain). Так приёмный ISR перестаёт задерживать 1-мс пере-взвод аудио-выхода.
 *
 * Один производитель (ISR пишет ТОЛЬКО fqHead и слоты) и один потребитель (Voice_Process
 * пишет ТОЛЬКО fqTail) → индексы атомарны по построению, критических секций в ISR нет.
 *
 * Глубина: кадр приходит каждые 5 мс; период Voice_Process — доли мс (консоль неблокирующая:
 * raw_put отбрасывает при полном кольце, tx_drain — один CDC_Transmit_FS без ожидания),
 * т.е. между разгрузками накапливается <1 кадра. FQ_DEPTH=16 = 80 мс буфера — >16× запас
 * даже против маловероятной 5-мс задержки цикла; фактический период показывает loop_max в
 * команде voice. Переполнение считается cQover (молчаливой потери нет). Слот = FRAME_MAX_CONTENT,
 * чтобы никогда не усекать CRC-корректный кадр. */
#define FQ_DEPTH   16u
#define FQ_MASK    (FQ_DEPTH - 1u)
/* Размер слота: у провода — макс. содержимое SLIP-кадра; у радио — свой предел (frame.h не тянем). */
#if VOICE_XPORT_UART
#define VOICE_SLOT_MAX  FRAME_MAX_CONTENT
#else
#define VOICE_SLOT_MAX  128u
#endif
typedef struct { uint16_t len; uint8_t data[VOICE_SLOT_MAX]; } fq_item_t;
static fq_item_t         fq[FQ_DEPTH];
static volatile uint16_t fqHead = 0u;   /* пишет только ISR-производитель */
static volatile uint16_t fqTail = 0u;   /* пишет только потребитель (Voice_Process) */

static uint16_t jb_fill(void) { return (uint16_t)((jbHead - jbTail) & JB_MASK); }

static void jb_push(const int16_t *s, uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++)
  {
    uint16_t nx = (uint16_t)((jbHead + 1u) & JB_MASK);
    if (nx == jbTail) { cOver++; return; }
    jbuf[jbHead] = s[i]; jbHead = nx;
  }
}
static void jb_push_silence(uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++)
  {
    uint16_t nx = (uint16_t)((jbHead + 1u) & JB_MASK);
    if (nx == jbTail) { cOver++; return; }
    jbuf[jbHead] = 0; jbHead = nx;
  }
}

/* Антиалиас-децимация 16→8: FIR-НЧ (aaCoef) по блоку 16 кГц с переносом истории между
 * блоками, затем прореживание 2:1. Именно это убирает алиасинг (тон 5 кГц свернулся бы в
 * 3 кГц при простом усреднении). Вычисляем выход только в нужных (чётных) позициях. */
static void aa_decimate(const int16_t *src16, int16_t *out8)
{
  int16_t  ext[(AA_TAPS - 1u) + SAMP16];   /* история (N−1) + текущий блок */
  uint16_t m, k;

  for (k = 0u; k < (AA_TAPS - 1u); k++) { ext[k] = aaHist[k]; }
  for (k = 0u; k < SAMP16; k++)          { ext[(AA_TAPS - 1u) + k] = src16[k]; }

  for (m = 0u; m < SAMP8; m++)
  {
    int32_t acc = 0;
    const int16_t *w = &ext[2u * m];
    for (k = 0u; k < AA_TAPS; k++) { acc += (int32_t)aaCoef[k] * (int32_t)w[k]; }
    acc >>= 15;                            /* Q15 → целое */
    if (acc > 32767) { acc = 32767; } else if (acc < -32768) { acc = -32768; }  /* насыщение */
    out8[m] = (int16_t)acc;
  }
  /* сохранить последние (N−1) отсчётов блока как историю для следующего вызова */
  for (k = 0u; k < (AA_TAPS - 1u); k++) { aaHist[k] = src16[SAMP16 - (AA_TAPS - 1u) + k]; }
}

/* ================= ИНТЕГРАЦИЯ CODEC2 (TASK_codec2_integration) =================
 * Codec2 — четвёртый кодек лесенки, ВСЕГДА 8 кГц (аппаратный тракт 16 кГц → децимация 16→8
 * тем же FIR-антиалиасом, что и legacy 8к). Отличия от legacy кодеков:
 *  1) кадр вокодера = 160 (3200/2400) или 320 (1600/1300/700C) отсчётов @8кГц (20/40 мс), а не
 *     5-мс блок. Один вокодерный кадр укладывается в ОДИН проводной SLIP-кадр; заголовок несёт
 *     индекс режима (5-й байт) — приёмник создаёт декодер под тот же режим;
 *  2) кодирование/декодирование ДОРОГИЕ (десятки мс) → выполняются в ГЛАВНОМ ЦИКЛЕ, не в ISR:
 *     TX — аудио-ISR лишь децимирует блок и кладёт отсчёты 8кГц в SPSC-кольцо c2Tx; кодирование
 *     и отправку делает c2_tx_process() из Voice_Process. RX — codec2_decode в voice_sink, который
 *     и так уносится в Voice_Process через rx_drain. Длинных __disable_irq НЕ вводим (иначе вернём
 *     дефект пере-взвода аудио-выхода).
 * Аллокатор Codec2 (codec2_malloc, ниже) — bump-пул со сбросом при возврате счётчика выделений к
 * нулю: держим ОДИН экземпляр (c2inst) — его хватает и кодеру, и декодеру (полудуплекс, направления
 * не пересекаются во времени). Смена режима → destroy старого (счётчик → 0, пул сброшен) + create
 * нового, поэтому многократная смена не исчерпывает пул. Все обращения к пулу — только из главного
 * цикла (TX-encode, RX-decode, c2load), гонок нет. */
#define VOICE_HDR_C2    (VOICE_HDR + 1u)          /* заголовок кадра Codec2: base(4) + индекс режима(1) */
#define C2_MODE_COUNT   5u
#define C2_SPF_MAX      320                       /* макс. отсчётов на кадр вокодера (@8кГц) */

/* Таблица режимов: константа Codec2, имя, битрейт (для budget), длительность кадра (для печати).
 * Битрейт/длительность — документированные параметры режимов Codec2 (samples_per_frame/bits берём
 * из библиотеки в рантайме, тут — только для human-readable вывода). */
static const struct { int mode; const char *name; uint32_t bps; uint16_t frameMs; } c2ModeTab[C2_MODE_COUNT] =
{
  { CODEC2_MODE_3200, "3200", 3200u, 20u },
  { CODEC2_MODE_2400, "2400", 2400u, 20u },
  { CODEC2_MODE_1600, "1600", 1600u, 40u },
  { CODEC2_MODE_1300, "1300", 1300u, 40u },
  { CODEC2_MODE_700C, "700C",  700u, 40u },
};

static struct CODEC2 *c2inst    = NULL;          /* текущий экземпляр (enc+dec) */
static int            c2instMode = -1;           /* его режим (константа CODEC2_MODE_*), -1 — нет */

/* SPSC-кольцо отсчётов 8кГц: производитель — аудио-ISR (c2tx_push), потребитель — Voice_Process. */
#define C2TX_SIZE       1024u
#define C2TX_MASK       (C2TX_SIZE - 1u)
static int16_t          c2Tx[C2TX_SIZE];
static volatile uint16_t c2TxHead = 0u;          /* пишет только ISR */
static volatile uint16_t c2TxTail = 0u;          /* пишет только Voice_Process */

/* Рабочие буферы вокодера (только главный цикл; TX и RX не пересекаются — полудуплекс). */
static short   c2SpIn[C2_SPF_MAX];               /* вход кодера (8кГц) */
static short   c2SpOut[C2_SPF_MAX];              /* выход декодера (8кГц) */
static int16_t c2Up[2 * C2_SPF_MAX];             /* после апсемплинга 8→16 */
static unsigned char c2Bits[16];                 /* закодированный кадр (макс 8 Б: 3200/1600) */

/* Живой замер длительности enc/dec В РАБОЧЕМ ТРАКТЕ (DWT) — печатается в voice (TASK п.5).
 * Суммы тактов — 64-бит: кадр ~1.2 млн тактов, за тысячи кадров 32-бит переполнился бы (> 4.29e9)
 * и среднее становилось мусором. cnt/max — по ФАКТИЧЕСКИМ операциям (инкремент только вокруг
 * реального codec2_encode/decode). Обнуляются при смене режима (c2_get) и по reset, иначе max/avg
 * смешали бы кадры разных режимов. */
static volatile uint32_t c2EncMaxCyc = 0u, c2EncCnt = 0u;
static volatile uint64_t c2EncSumCyc = 0u;
static volatile uint32_t c2DecMaxCyc = 0u, c2DecCnt = 0u;
static volatile uint64_t c2DecSumCyc = 0u;
/* Переполнение SPSC-кольца c2Tx (аудио-ISR не смог положить отсчёт — главный цикл не успел
 * закодировать). ОТДЕЛЬНЫЙ печатаемый счётчик, чтобы этот путь потери не был молчаливым
 * (как приёмный qover). Инкремент в ISR, чтение/сброс — в главном цикле. */
static volatile uint32_t c2TxRingOver = 0u;

static void c2_stats_reset(void)
{
  c2EncMaxCyc = 0u; c2EncCnt = 0u; c2EncSumCyc = 0u;
  c2DecMaxCyc = 0u; c2DecCnt = 0u; c2DecSumCyc = 0u;
}

/* forward: пул и аллокатор Codec2 определены ниже (общие с командой c2load). Предварительное
 * объявление (tentative) — чтобы c2_get видел флаг «пул исчерпан» до определения аллокатора. */
static volatile uint8_t c2PoolFail;

/* Получить экземпляр под нужный режим (создаёт/пересоздаёт по необходимости). NULL при нехватке пула.
 * Пересоздание: destroy старого возвращает счётчик выделений к 0 → bump-пул сбрасывается, утечки нет. */
static struct CODEC2 *c2_get(int mode)
{
  if ((c2inst != NULL) && (c2instMode == mode)) { return c2inst; }
  if (c2inst != NULL) { codec2_destroy(c2inst); c2inst = NULL; c2instMode = -1; }
  c2PoolFail = 0u;
  c2inst = codec2_create(mode);
  if ((c2inst == NULL) || (c2PoolFail != 0u))
  {
    if (c2inst != NULL) { codec2_destroy(c2inst); c2inst = NULL; }
    c2instMode = -1;
    return NULL;
  }
  c2instMode = mode;
  c2_stats_reset();   /* новый режим/экземпляр — живой замер enc/dec с чистого листа */
  return c2inst;
}
#if VOICE_XPORT_UART
static void c2_release(void)   /* используется только замером c2load (провод LAB07) */
{
  if (c2inst != NULL) { codec2_destroy(c2inst); c2inst = NULL; c2instMode = -1; }
}
#endif

/* Кольцо c2Tx: производитель (ISR). Переполнение — свой счётчик c2TxRingOver (печатается в voice),
 * НЕ сваливаем в txdrop: это разные причины (кольцо = главный цикл не успел кодировать; txdrop =
 * отказ отправки готового кадра). При здоровом тракте 0. */
static void c2tx_push(const int16_t *s, uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++)
  {
    uint16_t nx = (uint16_t)((c2TxHead + 1u) & C2TX_MASK);
    if (nx == c2TxTail) { c2TxRingOver++; return; }
    c2Tx[c2TxHead] = s[i]; c2TxHead = nx;
  }
}
static uint16_t c2tx_avail(void) { return (uint16_t)((c2TxHead - c2TxTail) & C2TX_MASK); }

/* Отображаемое имя кодека: legacy — из codec.c, WIRE_CODEC2 — "codec2". */
static const char *codec_disp_name(uint8_t c)
{
  if (c == (uint8_t)WIRE_CODEC2) { return "codec2"; }
  return (c < (uint8_t)CODEC_COUNT) ? Codec_Name((codec_id_t)c) : "?";
}

/* ================= ПЕРЕДАЧА: кодирование блока и отправка (аудио-ISR) ================= */
static void tx_flush(void)
{
  int16_t  src16[SAMP16];              /* блок на ВХОДНОЙ частоте 16 кГц (микрофон или тон) */
  int16_t  work[SAMP16];               /* блок на РАБОЧЕЙ частоте (80 @16кГц или 40 @8кГц) */
  uint16_t nsamp;
  uint16_t enc, frameLen, i;
  uint32_t t0;

  /* 1) Сформировать блок 16 кГц. Тон генерируется на ВХОДНОЙ частоте (16 кГц) фазовым
   *    аккумулятором (inc = g_toneHz/16000), поэтому он проходит ЧЕРЕЗ антиалиас-фильтр —
   *    только так тест «тон 5 кГц при rate 8000» реально проверяет фильтр (иначе NCO на
   *    8 кГц свернул бы 5→3 кГц ещё до децимации). Слышимая частота = g_toneHz НЕЗАВИСИМО
   *    от rate (для полосных тонов); фаза непрерывна и при смене rate (inc постоянен). */
  if (g_tone != 0u)
  {
    uint32_t inc = (uint32_t)(((uint64_t)g_toneHz << 32) / 16000u);
    for (i = 0u; i < SAMP16; i++)
    {
      uint32_t idx  = tonePhaseAcc >> 28;
      int32_t  frac = (int32_t)((tonePhaseAcc >> 12) & 0xFFFFu);
      int16_t  a = toneLUT[idx];
      int16_t  b = toneLUT[(idx + 1u) & 0x0Fu];
      src16[i] = (int16_t)(a + (int16_t)((((int32_t)(b - a)) * frac) >> 16));
      tonePhaseAcc += inc;
    }
  }
  else { for (i = 0u; i < SAMP16; i++) { src16[i] = acc16[i]; } }

  /* 2) Рабочая частота. Codec2 ВСЕГДА 8 кГц; legacy — по g_rate. Децимация 16→8: FIR-антиалиас
   *    (или старое усреднение для сравнения). 16 кГц — блок без изменений. */
  if ((g_codec == WIRE_CODEC2) || (g_rate == RATE_8K))
  {
    if (g_decim == DECIM_FIR) { aa_decimate(src16, work); }
    else { for (i = 0u; i < SAMP8; i++) { work[i] = (int16_t)(((int32_t)src16[2u * i] + src16[2u * i + 1u]) / 2); } }
    nsamp = SAMP8;
  }
  else { for (i = 0u; i < SAMP16; i++) { work[i] = src16[i]; } nsamp = SAMP16; }

  /* 3a) Codec2: кодирование дорогое — здесь лишь кладём отсчёты 8кГц в кольцо, кодирует и шлёт
   *     главный цикл (c2_tx_process). Блок 5 мс потреблён, seq продвинет отправитель кадра. */
  if (g_codec == WIRE_CODEC2) { c2tx_push(work, nsamp); accN = 0u; return; }

  /* 3b) Legacy (raw/ulaw/adpcm): кодируем и отправляем прямо здесь (дёшево, единицы мкс). */
  t0 = DWT->CYCCNT;
  enc = Codec_Encode((codec_id_t)g_codec, work, nsamp, txPayload + VOICE_HDR,
                     (uint16_t)(sizeof(txPayload) - VOICE_HDR));
  encCyc += (uint32_t)(DWT->CYCCNT - t0); encCnt++;

  if (enc == 0u) { cTxDrop++; txSeq++; accN = 0u; return; }

  txPayload[0] = (uint8_t)(txSeq >> 8);
  txPayload[1] = (uint8_t)(txSeq & 0xFFu);
  txPayload[2] = g_codec;
  txPayload[3] = g_rate;
  frameLen = (uint16_t)(VOICE_HDR + enc);

#if VOICE_XPORT_UART
  {
    uint16_t e = Frame_Encode(txPayload, frameLen, txEnc, (uint16_t)sizeof(txEnc));
    if ((e != 0u) && (UartPort_SendRaw(txEnc, e) == 0u)) { cSent++; }
    else { cTxDrop++; }
  }
#else  /* UWB: payload напрямую в радиокадр (SLIP/CRC не нужны) */
  if (Dw3000Port_VoiceTx(txPayload, frameLen) == 0u) { cSent++; } else { cTxDrop++; }
#endif
  txSeq++; accN = 0u;
}

void Audio_OnCapture(const int16_t *mono, uint16_t n)
{
  uint16_t i;
  cRec++;
  if (g_ptt == 0u) { return; }
  for (i = 0u; i < n; i++)
  {
    int16_t s = mono[i];
    int32_t m = (s < 0) ? -(int32_t)s : (int32_t)s;      /* |отсчёт| ДО аттенюации — что даёт микрофон */
    int32_t a;
    int16_t sa;
    if (m > hrPeakPre) { hrPeakPre = m; }
    if (m >= HR_NEAR_FS) { hrClipPre++; }

    /* Аттенюация Q8 (256 = ×1), без деления: даёт кодеку запас по уровню, снимает перегрузку ADPCM
     * по крутизне и клиппинг FIR на громкой речи. Насыщение сохранено (при усилении ≤ ×1 переполнить
     * нельзя, но клампим для строгости). */
    a = ((int32_t)s * (int32_t)g_hrGain) >> 8;
    if (a > 32767) { a = 32767; } else if (a < -32768) { a = -32768; }
    sa = (int16_t)a;

    m = (sa < 0) ? -(int32_t)sa : (int32_t)sa;           /* |отсчёт| ПОСЛЕ аттенюации — что уходит в кодек */
    if (m > hrPeakPost) { hrPeakPost = m; }
    if (m >= HR_NEAR_FS) { hrClipPost++; }

    /* Накапливаем звук с микрофона; подмена на тон — в tx_flush (тон НЕ аттенюируется). */
    acc16[accN++] = sa;
    if (accN >= SAMP16) { tx_flush(); }
  }
}

/* ================= ВОСПРОИЗВЕДЕНИЕ (аудио-ISR, всегда 16 кГц) ================= */
void Audio_FillPlayback(int16_t *mono, uint16_t n)
{
  uint16_t i;
  if (g_ptt != 0u) { for (i = 0u; i < n; i++) { mono[i] = 0; } jbPlaying = 0u; return; }

  if (jbPlaying == 0u)
  {
    if (jb_fill() >= g_jbPrefill) { jbPlaying = 1u; }
    else { for (i = 0u; i < n; i++) { mono[i] = 0; } return; }
  }
  if (jb_fill() >= n)
  {
    for (i = 0u; i < n; i++) { mono[i] = jbuf[jbTail]; jbTail = (uint16_t)((jbTail + 1u) & JB_MASK); }
    for (i = 0u; i < n; i++) { lastOut[i] = mono[i]; }
    cPlayed++;
  }
  else
  {
    cUnder++; jbPlaying = 0u;
    for (i = 0u; i < n; i++)
    {
      mono[i] = (int16_t)(((int32_t)lastOut[i] * (int32_t)(n - 1u - i)) / (int32_t)n);
      lastOut[i] = 0;
    }
  }
}

void Audio_OnError(const char *who) { (void)who; BSP_LED_On(LED5); }

/* Постановка готового кадра в очередь — вызывается в ISR как sink Frame_DecodeByte.
 * Только копирование байтов и публикация head (никакого декодирования). */
static void frame_enqueue(const uint8_t *payload, uint16_t len)
{
  uint16_t nh = (uint16_t)((fqHead + 1u) & FQ_MASK);
  uint16_t i;
  if (nh == fqTail) { cQover++; return; }               /* очередь полна: кадр принят, класть некуда */
  if (len > (uint16_t)VOICE_SLOT_MAX) { len = (uint16_t)VOICE_SLOT_MAX; }
  fq[fqHead].len = len;
  for (i = 0u; i < len; i++) { fq[fqHead].data[i] = payload[i]; }
  __DMB();                                               /* содержимое слота видно до публикации head */
  fqHead = nh;
}

/* ================= ПРИЁМ КАДРОВ (разбор — в Voice_Process через rx_drain) ================= */
static void voice_sink(const uint8_t *payload, uint16_t len)
{
  int16_t  dec[SAMP16];       /* декодировано (в частоте кадра) */
  int16_t  up[SAMP16];        /* после апсемплинга до 16 кГц */
  uint16_t seq, i, dn, outn;
  uint8_t  fcodec, frate;
  uint32_t t0;

  if (len < VOICE_HDR) { return; }
  cRecv++;
  BSP_LED_Toggle(LED4);
  if (g_ptt != 0u) { return; }

  seq    = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
  fcodec = payload[2];
  frate  = payload[3];

  /* Codec2: заголовок несёт индекс режима (5-й байт). Создаём/берём декодер под этот режим,
   * декодируем кадр вокодера (spf отсчётов @8кГц), апсемплим 8→16 и кладём в джиттер-буфер.
   * Потери по seq заполняем тишиной на длину ОДНОГО вокодерного кадра (2*spf отсчётов @16кГц),
   * а не 5-мс блока — иначе timeline поехал бы (см. отчёт: как читать счётчики). */
  if (fcodec == (uint8_t)WIRE_CODEC2)
  {
    struct CODEC2 *c2;
    uint16_t frame16;
    int      spf, bpf;
    uint8_t  midx;
    if (len < (uint16_t)VOICE_HDR_C2) { return; }
    midx = payload[4];
    if (midx >= (uint8_t)C2_MODE_COUNT) { return; }
    c2 = c2_get(c2ModeTab[midx].mode);
    if (c2 == NULL) { return; }
    spf = codec2_samples_per_frame(c2);
    bpf = codec2_bytes_per_frame(c2);
    if (spf > C2_SPF_MAX) { spf = C2_SPF_MAX; }
    if ((uint16_t)(len - VOICE_HDR_C2) < (uint16_t)bpf) { return; }

    t0 = DWT->CYCCNT;
    codec2_decode(c2, c2SpOut, payload + VOICE_HDR_C2);
    { uint32_t d = (uint32_t)(DWT->CYCCNT - t0);
      if (d > c2DecMaxCyc) { c2DecMaxCyc = d; } c2DecSumCyc += d; c2DecCnt++; }

    outn = 0u;                                    /* апсемплинг 8→16 линейной интерполяцией */
    for (i = 0u; i < (uint16_t)spf; i++)
    {
      c2Up[outn++] = c2SpOut[i];
      c2Up[outn++] = ((i + 1u) < (uint16_t)spf) ? (int16_t)(((int32_t)c2SpOut[i] + c2SpOut[i + 1u]) / 2) : c2SpOut[i];
    }
    frame16 = (uint16_t)(2 * spf);

    if (rxHaveSeq != 0u)
    {
      uint16_t gap = (uint16_t)(seq - rxExpSeq);
      if (gap != 0u) { cLost += gap; if (gap <= LOSS_FILL_CAP) { jb_push_silence((uint16_t)(gap * frame16)); } }
    }
    rxExpSeq = (uint16_t)(seq + 1u); rxHaveSeq = 1u;
    jb_push(c2Up, outn);
    return;
  }

  if (fcodec >= (uint8_t)CODEC_COUNT) { return; }

  {
    uint16_t cap = (frate == RATE_8K) ? SAMP8 : SAMP16;
    t0 = DWT->CYCCNT;
    dn = Codec_Decode((codec_id_t)fcodec, payload + VOICE_HDR, (uint16_t)(len - VOICE_HDR), dec, cap);
    decCyc += (uint32_t)(DWT->CYCCNT - t0); decCnt++;
  }

  if (frate == RATE_8K)                            /* апсемплинг 8→16 линейной интерполяцией */
  {
    outn = 0u;
    for (i = 0u; i < dn; i++)
    {
      up[outn++] = dec[i];
      up[outn++] = (i + 1u < dn) ? (int16_t)(((int32_t)dec[i] + dec[i + 1u]) / 2) : dec[i];
    }
  }
  else { for (i = 0u; i < dn; i++) { up[i] = dec[i]; } outn = dn; }

  /* потери по seq: вставляем тишину на длину пропущенных блоков (в отсчётах 16 кГц) */
  if (rxHaveSeq != 0u)
  {
    uint16_t gap = (uint16_t)(seq - rxExpSeq);
    if (gap != 0u) { cLost += gap; if (gap <= LOSS_FILL_CAP) { jb_push_silence((uint16_t)(gap * SAMP16)); } }
  }
  rxExpSeq = (uint16_t)(seq + 1u); rxHaveSeq = 1u;

  jb_push(up, outn);
}

/* Разгрузка очереди кадров в основном цикле: тяжёлый voice_sink здесь, не в ISR. */
static void rx_drain(void)
{
  if (g_ptt != 0u)                        /* передаём — приём не нужен: опустошаем очередь */
  {
    fqTail = fqHead;                      /* без залипших кадров от прошлого сеанса */
    return;
  }
  while (fqTail != fqHead)
  {
    uint16_t t = fqTail;
    voice_sink(fq[t].data, fq[t].len);
    __DMB();                              /* дочитать слот до освобождения (до сдвига tail) */
    fqTail = (uint16_t)((t + 1u) & FQ_MASK);
  }
}

/* Codec2 TX: кодирование накопленных отсчётов и отправка — в ГЛАВНОМ ЦИКЛЕ (кодек стоит десятки
 * мс; в ISR нельзя). Пока в кольце c2Tx есть полный кадр вокодера — кодируем и шлём один SLIP-кадр
 * {seq, WIRE_CODEC2, 8кГц, индекс режима, биты}. Только при активной передаче; режим — текущий
 * g_c2mode (передатчик диктует, приёмник подхватывает по 5-му байту заголовка). */
static void c2_tx_process(void)
{
  struct CODEC2 *c2;
  int spf, bpf, i;

  if ((g_codec != (uint8_t)WIRE_CODEC2) || (g_ptt == 0u)) { return; }
  c2 = c2_get(c2ModeTab[g_c2mode].mode);
  if (c2 == NULL) { return; }
  spf = codec2_samples_per_frame(c2);
  bpf = codec2_bytes_per_frame(c2);
  if (spf > C2_SPF_MAX) { spf = C2_SPF_MAX; }

  while (c2tx_avail() >= (uint16_t)spf)
  {
    uint32_t t0, d;
    for (i = 0; i < spf; i++)
    {
      c2SpIn[i] = c2Tx[c2TxTail];
      c2TxTail = (uint16_t)((c2TxTail + 1u) & C2TX_MASK);
    }
    t0 = DWT->CYCCNT;
    codec2_encode(c2, c2Bits, c2SpIn);
    d = (uint32_t)(DWT->CYCCNT - t0);
    if (d > c2EncMaxCyc) { c2EncMaxCyc = d; } c2EncSumCyc += d; c2EncCnt++;

    txPayload[0] = (uint8_t)(txSeq >> 8);
    txPayload[1] = (uint8_t)(txSeq & 0xFFu);
    txPayload[2] = (uint8_t)WIRE_CODEC2;
    txPayload[3] = RATE_8K;
    txPayload[4] = g_c2mode;
    for (i = 0; i < bpf; i++) { txPayload[VOICE_HDR_C2 + i] = c2Bits[i]; }
    {
      uint16_t frameLen = (uint16_t)(VOICE_HDR_C2 + (uint16_t)bpf);
#if VOICE_XPORT_UART
      uint16_t e = Frame_Encode(txPayload, frameLen, txEnc, (uint16_t)sizeof(txEnc));
      if ((e != 0u) && (UartPort_SendRaw(txEnc, e) == 0u)) { cSent++; } else { cTxDrop++; }
#else  /* UWB */
      if (Dw3000Port_VoiceTx(txPayload, frameLen) == 0u) { cSent++; } else { cTxDrop++; }
#endif
    }
    txSeq++;
  }
}

/* sink в ISR — только постановка кадра в очередь (разбор перенесён в rx_drain/Voice_Process). */
#if VOICE_XPORT_UART
static void voice_rx_byte(uint8_t b) { Frame_DecodeByte(&rxDec, b, frame_enqueue); }
#endif

/* ================= КОМАНДЫ (база как в LAB04) ================= */
#if VOICE_XPORT_UART   /* проводные команды (провод LAB07/08); в радио-наборе LAB09 их нет */
static void cmd_send(int argc, char **argv)
{
  unsigned long m = 1ul, i;
  if (argc > 1) { m = strtoul(argv[1], NULL, 0); }
  if (m == 0ul) { m = 1ul; } if (m > 1000ul) { m = 1000ul; }
  for (i = 0ul; i < m; i++)
  {
    uint32_t t0 = HAL_GetTick();
    while ((UartPort_TxBusy() != 0u) && ((uint32_t)(HAL_GetTick() - t0) < 50u)) { }
    if (UartPort_SendPacket() != 0u) { break; }
  }
  Console_Printf("sent %lu packet(s)\r\n", (unsigned long)i);
}
static void cmd_sendbyte(int argc, char **argv)
{
  unsigned long v;
  if (argc < 2) { Console_Write("usage: sendbyte <value>\r\n"); return; }
  v = strtoul(argv[1], NULL, 0) & 0xFFul;
  if (UartPort_SendByte((uint8_t)v) == 0u) { Console_Printf("sent byte 0x%02lX\r\n", (unsigned long)v); }
  else { Console_Write("sendbyte failed\r\n"); }
}
#endif /* VOICE_XPORT_UART: send/sendbyte */

static void cmd_stat(int argc, char **argv)
{
  (void)argc; (void)argv;
#if VOICE_XPORT_UART
  {
    UartPort_Stats s; UartPort_GetStats(&s);
    Console_Printf("link: baud=%lu bytes tx=%lu rx=%lu | err ore=%lu fe=%lu pe=%lu ne=%lu\r\n",
                   (unsigned long)UartPort_GetBaud(), (unsigned long)s.bytesTx, (unsigned long)s.bytesRx,
                   (unsigned long)s.errOre, (unsigned long)s.errFe, (unsigned long)s.errPe, (unsigned long)s.errNe);
  }
#else  /* UWB: радиолиния */
  {
    uint32_t tx = 0u, rx = 0u, crc = 0u, phe = 0u, to = 0u, sw = 0u;
    Dw3000Port_GetVoiceStats(&tx, &rx, &crc, &phe, &to, &sw);
    Console_Printf("link: UWB inited=%u tx=%lu rx=%lu | crcErr=%lu phrErr=%lu rxTimeout=%lu switch=%lu\r\n",
                   (unsigned)Dw3000Port_IsInited(), (unsigned long)tx, (unsigned long)rx,
                   (unsigned long)crc, (unsigned long)phe, (unsigned long)to, (unsigned long)sw);
  }
#endif
}

#if VOICE_XPORT_UART   /* dump/regs/baud — проводные */
static void cmd_dump(int argc, char **argv)
{
  uint8_t buf[64]; uint16_t n, i; (void)argc; (void)argv;
  n = UartPort_Dump(buf, (uint16_t)sizeof(buf));
  if (n == 0u) { Console_Write("dump: (nothing received)\r\n"); return; }
  Console_Printf("dump: last %u byte(s):\r\n", (unsigned)n);
  for (i = 0u; i < n; i++) { Console_Printf("%02X%s", (unsigned)buf[i], (((i & 0x0Fu) == 0x0Fu) || (i + 1u == n)) ? "\r\n" : " "); }
}
static void cmd_regs(int argc, char **argv)
{
  UartPort_Stats s; (void)argc; (void)argv; UartPort_GetStats(&s);
  Console_Printf("USART2: SR=%08lX CR1=%08lX CR3=%08lX | bytesRx=%lu\r\n",
                 (unsigned long)USART2->SR, (unsigned long)USART2->CR1, (unsigned long)USART2->CR3, (unsigned long)s.bytesRx);
}
static void cmd_baud(int argc, char **argv)
{
  unsigned long b;
  if (argc < 2) { Console_Printf("baud = %lu\r\n", (unsigned long)UartPort_GetBaud()); return; }
  b = strtoul(argv[1], NULL, 0);
  if ((b < 1200ul) || (b > 3000000ul)) { Console_Write("baud out of range\r\n"); return; }
  UartPort_SetBaud((uint32_t)b);
  Preempt_AudioOutHighest();   /* HAL_UART_MspInit при реините вернул USART2 в (0,0) — переустановить */
  Console_Printf("baud set to %lu\r\n", (unsigned long)UartPort_GetBaud());
}
#endif /* VOICE_XPORT_UART: dump/regs/baud */

static void cmd_reset(int argc, char **argv)
{
  (void)argc; (void)argv;
  cRec = cSent = cTxDrop = cRecv = cLost = cPlayed = cUnder = cOver = 0u;
  encCyc = encCnt = decCyc = decCnt = 0u;
  Audio_ResetOutProbe();        /* диагностика A: обнулить и задать базу «секунд от старта» */
  fqHead = fqTail = 0u; cQover = 0u;           /* очередь кадров offload */
  loopMaxCyc = 0u; loopHavePrev = 0u;          /* probeC: период Voice_Process */
  c2_stats_reset();                            /* codec2 живой замер enc/dec */
  c2TxRingOver = 0u;                           /* codec2: переполнение TX-кольца */
  hrPeakPre = hrPeakPost = 0; hrClipPre = hrClipPost = 0u;   /* headroom измеритель уровня */
#if VOICE_XPORT_UART
  UartPort_ResetStats();
  rxDec.stats.framesRx = rxDec.stats.framesCrc = rxDec.stats.resync = rxDec.stats.bytesDropped = 0u;
  UartPort_ResetRxIsrProbe();   /* диагностика B */
  UartPort_ResetRs485Stats();   /* RS-485: время разворота, premature, wd, ошибки */
#else  /* UWB: радиосчётчики */
  Dw3000Port_ResetVoiceStats();
#endif
  Console_Write("counters reset\r\n");
}
static void cmd_voice(int argc, char **argv)
{
  (void)argc; (void)argv;
  Console_Printf("voice: rec=%lu sent=%lu txdrop=%lu recv=%lu lost=%lu played=%lu under=%lu over=%lu\r\n",
                 (unsigned long)cRec, (unsigned long)cSent, (unsigned long)cTxDrop, (unsigned long)cRecv,
                 (unsigned long)cLost, (unsigned long)cPlayed, (unsigned long)cUnder, (unsigned long)cOver);
#if VOICE_XPORT_UART   /* провод: формат как прежде (строка codec с "| rx err ...") — не менять */
  {
    UartPort_Stats s; UartPort_GetStats(&s);
    Console_Printf("voice: codec=%s rate=%u Hz decim=%s jb_fill=%u (%u ms) prefill=%u ms cap=%u ms ptt=%s tone=%s | rx err ore=%lu fe=%lu pe=%lu ne=%lu\r\n",
                   codec_disp_name(g_codec), (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u),
                   (g_decim == DECIM_FIR) ? "fir" : "avg",
                   (unsigned)jb_fill(), (unsigned)(jb_fill() / AUDIO_BLOCK_SAMPLES),
                   (unsigned)(g_jbPrefill / AUDIO_BLOCK_SAMPLES), (unsigned)(JB_SIZE / AUDIO_BLOCK_SAMPLES),
                   (g_ptt ? "on" : "off"), (g_tone ? "on" : "off"),
                   (unsigned long)s.errOre, (unsigned long)s.errFe, (unsigned long)s.errPe, (unsigned long)s.errNe);
  }
#else  /* UWB: строка codec без rx-err + отдельная radio-строка */
  Console_Printf("voice: codec=%s rate=%u Hz decim=%s jb_fill=%u (%u ms) prefill=%u ms cap=%u ms ptt=%s tone=%s\r\n",
                 codec_disp_name(g_codec), (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u),
                 (g_decim == DECIM_FIR) ? "fir" : "avg",
                 (unsigned)jb_fill(), (unsigned)(jb_fill() / AUDIO_BLOCK_SAMPLES),
                 (unsigned)(g_jbPrefill / AUDIO_BLOCK_SAMPLES), (unsigned)(JB_SIZE / AUDIO_BLOCK_SAMPLES),
                 (g_ptt ? "on" : "off"), (g_tone ? "on" : "off"));
  {
    uint32_t tx = 0u, rx = 0u, crc = 0u, phe = 0u, to = 0u, sw = 0u;
    Dw3000Port_GetVoiceStats(&tx, &rx, &crc, &phe, &to, &sw);
    Console_Printf("radio: inited=%u tx=%lu rx=%lu lostSeq=%lu crcErr=%lu phrErr=%lu rxTimeout=%lu switch=%lu\r\n",
                   (unsigned)Dw3000Port_IsInited(), (unsigned long)tx, (unsigned long)rx,
                   (unsigned long)cLost, (unsigned long)crc, (unsigned long)phe,
                   (unsigned long)to, (unsigned long)sw);
  }
#endif
  /* Диагностика срыва дедлайна аудио-выхода (TASK_isr_deadline_probe) — отдельными строками. */
  {
    Audio_OutProbe ap;
    Audio_GetOutProbe(&ap);
    Console_Printf("probeA out-rearm: calls=%lu over(>%luus)=%lu max=%luus first=%lus last=%lus lastmin=%lu%s\r\n",
                   (unsigned long)ap.calls, (unsigned long)ap.threshUs, (unsigned long)ap.over,
                   (unsigned long)ap.maxUs, (unsigned long)ap.firstSec, (unsigned long)ap.lastSec,
                   (unsigned long)ap.lastMinOver, (ap.lastMinCapped != 0u) ? "(capped)" : "");
#if VOICE_XPORT_UART
    {
      uint32_t bcalls = 0u, bmax = 0u, bavg = 0u, blong = 0u;
      UartPort_GetRxIsrProbe(&bcalls, &bmax, &bavg, &blong);
      Console_Printf("probeB rx-isr: calls=%lu max=%luus avg=%luus long(>%uus)=%lu\r\n",
                     (unsigned long)bcalls, (unsigned long)bmax, (unsigned long)bavg,
                     (unsigned)UARTPORT_RXISR_LONG_US, (unsigned long)blong);
    }
#endif
    Console_Printf("probeC offload: qover=%lu qdepth=%u loop_max=%luus\r\n",
                   (unsigned long)cQover, (unsigned)FQ_DEPTH,
                   (unsigned long)(loopMaxCyc / (SystemCoreClock / 1000000u)));
  }
  /* Измеритель уровня и запас (TASK_input_headroom): pre = микрофон как есть, post = что уходит в
   * кодек после аттенюации. nearFS>0 = подход к полной шкале (перегрузка). Меряется при передаче. */
  Console_Printf("headroom: atten=%udB gainQ8=%u | pre peak=%ld nearFS=%lu | post peak=%ld nearFS=%lu (thr=%d)\r\n",
                 (unsigned)g_hrDb, (unsigned)g_hrGain,
                 (long)hrPeakPre, (unsigned long)hrClipPre,
                 (long)hrPeakPost, (unsigned long)hrClipPost, (int)HR_NEAR_FS);
  /* Codec2: живой замер enc/dec в рабочем тракте (TASK п.5) — цена именно живой речи, а не синтетики. */
  if (g_codec == (uint8_t)WIRE_CODEC2)
  {
    uint32_t cyc_us = SystemCoreClock / 1000000u;
    uint32_t eMax = c2EncMaxCyc / cyc_us;
    uint32_t eAvg = (c2EncCnt != 0u) ? (uint32_t)((c2EncSumCyc / c2EncCnt) / cyc_us) : 0u;
    uint32_t dMax = c2DecMaxCyc / cyc_us;
    uint32_t dAvg = (c2DecCnt != 0u) ? (uint32_t)((c2DecSumCyc / c2DecCnt) / cyc_us) : 0u;
    Console_Printf("codec2: mode=%s frame=%ums ring_over=%lu | enc max=%luus avg=%luus (n=%lu) | dec max=%luus avg=%luus (n=%lu)\r\n",
                   c2ModeTab[g_c2mode].name, (unsigned)c2ModeTab[g_c2mode].frameMs,
                   (unsigned long)c2TxRingOver,
                   (unsigned long)eMax, (unsigned long)eAvg, (unsigned long)c2EncCnt,
                   (unsigned long)dMax, (unsigned long)dAvg, (unsigned long)c2DecCnt);
  }
}
#if VOICE_XPORT_UART
static void cmd_proto(int argc, char **argv)
{
  (void)argc; (void)argv;
  Console_Printf("proto: frames=%lu crc_drop=%lu resync=%lu bytes_dropped=%lu\r\n",
                 (unsigned long)rxDec.stats.framesRx, (unsigned long)rxDec.stats.framesCrc,
                 (unsigned long)rxDec.stats.resync, (unsigned long)rxDec.stats.bytesDropped);
}
#endif
static void cmd_ptt(int argc, char **argv)
{
  if (argc < 2) { Console_Printf("ptt(cmd) = %s\r\n", g_pttCmd ? "on" : "off"); return; }
  if (strcmp(argv[1], "on") == 0) { g_pttCmd = 1u; } else if (strcmp(argv[1], "off") == 0) { g_pttCmd = 0u; }
  else { Console_Write("usage: ptt on|off\r\n"); return; }
  Console_Printf("ptt(cmd) = %s\r\n", g_pttCmd ? "on" : "off");
}
/* tone on [freq] | off — тон на ВХОДНОЙ частоте 16 кГц (проходит через децимацию); freq по
 * умолчанию 1000 Гц. Тест фильтра: tone on 5000 + rate 8000 при decim fir не проходит
 * (−60 дБ), а при decim avg отражается в 3 кГц — слышимая разница до/после фильтра. */
#if VOICE_XPORT_UART   /* tone — тест-сигнал, провод LAB07 */
static void cmd_tone(int argc, char **argv)
{
  if (argc < 2)
  {
    Console_Printf("tone = %s (%u Hz)\r\n", g_tone ? "on" : "off", (unsigned)g_toneHz);
    return;
  }
  if (strcmp(argv[1], "on") == 0)
  {
    if (argc > 2)
    {
      unsigned long f = strtoul(argv[2], NULL, 0);
      if ((f < 50ul) || (f > 7900ul)) { Console_Write("freq out of range (50..7900)\r\n"); return; }
      g_toneHz = (uint16_t)f;
    }
    else { g_toneHz = 1000u; }        /* по умолчанию 1 кГц */
    g_tone = 1u;
  }
  else if (strcmp(argv[1], "off") == 0) { g_tone = 0u; }
  else { Console_Write("usage: tone on [freq] | off\r\n"); return; }
  Console_Printf("tone = %s (%u Hz)\r\n", g_tone ? "on" : "off", (unsigned)g_toneHz);
}

/* codec [raw|ulaw|adpcm|codec2] — четвёртый вариант, codec2, вокодер (всегда 8 кГц). */
#endif /* VOICE_XPORT_UART: tone */
static void cmd_codec(int argc, char **argv)
{
  if (argc > 1)
  {
    if      (strcmp(argv[1], "raw")    == 0) { g_codec = (uint8_t)CODEC_RAW; }
    else if (strcmp(argv[1], "ulaw")   == 0) { g_codec = (uint8_t)CODEC_ULAW; }
    else if (strcmp(argv[1], "adpcm")  == 0) { g_codec = (uint8_t)CODEC_ADPCM; }
    else if (strcmp(argv[1], "codec2") == 0) { g_codec = (uint8_t)WIRE_CODEC2; g_rate = RATE_8K; }
    else { Console_Write("usage: codec raw|ulaw|adpcm|codec2\r\n"); return; }
  }
  if (g_codec == (uint8_t)WIRE_CODEC2)
  { Console_Printf("codec = codec2 (mode %s, 8 kHz)\r\n", c2ModeTab[g_c2mode].name); }
  else { Console_Printf("codec = %s\r\n", Codec_Name((codec_id_t)g_codec)); }
}

/* c2mode [3200|2400|1600|1300|700C] — режим вокодера Codec2 (действует при codec=codec2). Экземпляр
 * под новый режим создаст c2_get при следующем кодировании/декодировании (destroy+create, пул не течёт). */
static void cmd_c2mode(int argc, char **argv)
{
  if (argc > 1)
  {
    int mi, found = -1;
    for (mi = 0; mi < (int)C2_MODE_COUNT; mi++) { if (strcmp(argv[1], c2ModeTab[mi].name) == 0) { found = mi; } }
    if (found < 0) { Console_Write("usage: c2mode 3200|2400|1600|1300|700C\r\n"); return; }
    g_c2mode = (uint8_t)found;
  }
  Console_Printf("c2mode = %s (%lu bps, %u ms frame)\r\n", c2ModeTab[g_c2mode].name,
                 (unsigned long)c2ModeTab[g_c2mode].bps, (unsigned)c2ModeTab[g_c2mode].frameMs);
}

/* headroom [0|3|6|9|12] — аттенюация микрофонного PCM ДО кодека, дБ (запас по уровню против
 * перегрузки на громкой речи). 0 = без аттенюации (демонстрация перегрузки для методички). Уровень
 * подбирается по измерителю в voice; дефолт см. docs/REPORT_input_headroom.md. */
#if VOICE_XPORT_UART   /* headroom/decim/rate — провод LAB07 */
static void cmd_headroom(int argc, char **argv)
{
  static const struct { uint8_t db; uint16_t gainQ8; } tab[] = {
    { 0u, 256u }, { 3u, 181u }, { 6u, 128u }, { 9u, 91u }, { 12u, 64u },
  };
  int n = (int)(sizeof(tab) / sizeof(tab[0])), k;
  if (argc > 1)
  {
    unsigned long d = strtoul(argv[1], NULL, 0);
    int found = -1;
    for (k = 0; k < n; k++) { if ((unsigned long)tab[k].db == d) { found = k; } }
    if (found < 0) { Console_Write("usage: headroom 0|3|6|9|12 (dB attenuation before codec)\r\n"); return; }
    g_hrDb = tab[found].db; g_hrGain = tab[found].gainQ8;
  }
  Console_Printf("headroom = %u dB (gainQ8=%u, x%u/256)\r\n",
                 (unsigned)g_hrDb, (unsigned)g_hrGain, (unsigned)g_hrGain);
}

/* decim [fir|avg] — метод децимации 16→8 (действует только при rate 8000):
 * fir = антиалиас FIR-НЧ ~3.6 кГц (по умолчанию, правильный),
 * avg = усреднение пар (старый, слабый антиалиас — для сравнения на слух). */
static void cmd_decim(int argc, char **argv)
{
  if (argc > 1)
  {
    if      (strcmp(argv[1], "fir") == 0) { g_decim = DECIM_FIR; }
    else if (strcmp(argv[1], "avg") == 0) { g_decim = DECIM_AVG; }
    else { Console_Write("usage: decim fir|avg\r\n"); return; }
  }
  Console_Printf("decim = %s%s\r\n", (g_decim == DECIM_FIR) ? "fir (anti-alias)" : "avg (old)",
                 (g_rate == RATE_16K) ? " [active only at rate 8000]" : "");
}

/* rate [8000|16000] */
static void cmd_rate(int argc, char **argv)
{
  if (argc > 1)
  {
    unsigned long r = strtoul(argv[1], NULL, 0);
    if (g_codec == (uint8_t)WIRE_CODEC2) { Console_Write("codec2 is 8 kHz only (rate fixed)\r\n"); return; }
    if      (r == 8000ul)  { g_rate = RATE_8K; }
    else if (r == 16000ul) { g_rate = RATE_16K; }
    else { Console_Write("usage: rate 8000|16000\r\n"); return; }
  }
  Console_Printf("rate = %u Hz\r\n", (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u));
}

/* prefill [ms] — порог наполнения джиттер-буфера перед стартом воспроизведения. Умолч. 60 мс.
 * Больше prefill — устойчивее к джиттеру канала (HC-12: 4–80 мс), но выше задержка «рот→динамик».
 * Лимит — не выше половины ёмкости буфера (JB_PREFILL_MAX = 128 мс при ёмкости 256 мс). */
#endif /* VOICE_XPORT_UART: headroom/decim/rate */
static void cmd_prefill(int argc, char **argv)
{
  if (argc > 1)
  {
    unsigned long ms = strtoul(argv[1], NULL, 0);
    uint32_t s = (uint32_t)ms * AUDIO_BLOCK_SAMPLES;             /* мс → отсчёты (×16 @16 кГц) */
    if (s < AUDIO_BLOCK_SAMPLES) { s = AUDIO_BLOCK_SAMPLES; }    /* минимум 1 мс */
    if (s > JB_PREFILL_MAX)
    {
      Console_Printf("prefill max %u ms (half of %u ms buffer)\r\n",
                     (unsigned)(JB_PREFILL_MAX / AUDIO_BLOCK_SAMPLES),
                     (unsigned)(JB_SIZE / AUDIO_BLOCK_SAMPLES));
      return;
    }
    g_jbPrefill = (uint16_t)s;
  }
  Console_Printf("prefill = %u ms (%u samples; buffer %u ms, max %u ms)\r\n",
                 (unsigned)(g_jbPrefill / AUDIO_BLOCK_SAMPLES), (unsigned)g_jbPrefill,
                 (unsigned)(JB_SIZE / AUDIO_BLOCK_SAMPLES),
                 (unsigned)(JB_PREFILL_MAX / AUDIO_BLOCK_SAMPLES));
}

/* load — загрузка ядра кодеком (DWT). µs на блок и % от реального времени блока (BLOCK_MS). */
#if VOICE_XPORT_UART   /* load/budget/c2load/phy/rs485 — провод LAB07/08 */
static void cmd_load(int argc, char **argv)
{
  uint32_t eus, dus, period_us;
  (void)argc; (void)argv;
  if (g_codec == (uint8_t)WIRE_CODEC2)
  {
    Console_Write("load: codec2 measured live in the real path -- see 'voice' (codec2 enc/dec us)\r\n");
    return;
  }
  period_us = BLOCK_MS * 1000u;
  eus = (encCnt != 0u) ? ((encCyc / encCnt) / (SystemCoreClock / 1000000u)) : 0u;
  dus = (decCnt != 0u) ? ((decCyc / decCnt) / (SystemCoreClock / 1000000u)) : 0u;
  Console_Printf("load: encode %lu us/block (%lu%% ), decode %lu us/block (%lu%% ) of %u ms block\r\n",
                 (unsigned long)eus, (unsigned long)((eus * 100u) / period_us),
                 (unsigned long)dus, (unsigned long)((dus * 100u) / period_us), (unsigned)BLOCK_MS);
}

/* budget — граница канала для HC-12 (FU3): битрейт кодека, сжатие, мин. проводная скорость
 * HC-12, соответствующая эфирная и чувствительность приёмника. Данные HC-12 (FU3) — из
 * документации модуля: сетка ПРОВОДНЫХ скоростей; пары проводных дают одну эфирную:
 *   1200/2400 -> 5000 (-117), 4800/9600 -> 15000 (-112),
 *   19200/38400 -> 58000 (-107), 57600/115200 -> 236000 (-100).
 * Проводной поток = битрейт × 1.30: UART 8N1 даёт 10 бит на байт вместо 8 (×1.25), плюс
 * кадрирование SLIP (маркеры END + CRC + заголовок + стаффинг) ~+5% в среднем. Требуемый
 * поток сравнивается С СЕТКОЙ ПРОВОДНЫХ скоростей — берётся минимальная не меньше потока;
 * если поток > 115200 (максимум HC-12) — не влезает. */
static void cmd_budget(int argc, char **argv)
{
  static const uint32_t wired[8] = { 1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 57600u, 115200u };
  static const uint32_t airR[8]  = { 5000u, 5000u, 15000u, 15000u, 58000u, 58000u, 236000u, 236000u };
  static const int      sensR[8] = { -117, -117, -112, -112, -107, -107, -100, -100 };
  uint32_t fs = (g_rate == RATE_8K) ? 8000u : 16000u;
  int c, t;
  (void)argc; (void)argv;

  Console_Printf("budget @ %lu Hz (HC-12 FU3; wire = bitrate x1.30 = UART 8N1 x1.25 + SLIP ~5%%):\r\n",
                 (unsigned long)fs);
  Console_Write("codec    bitrate  ratio  wire     HC12wire  air      sens\r\n");
  for (c = 0; c < (int)CODEC_COUNT; c++)
  {
    uint32_t bps   = (fs * Codec_BitsX10((codec_id_t)c)) / 10u;
    uint32_t wire  = (bps * 13u) / 10u;
    uint32_t ratio = 160u / Codec_BitsX10((codec_id_t)c);
    int found = 0;
    for (t = 0; t < 8; t++)
    {
      if (wired[t] >= wire)
      {
        Console_Printf("%-6s  %7lu   x%lu   %-7lu  %-7lu   %-6lu   %d dBm\r\n",
                       Codec_Name((codec_id_t)c), (unsigned long)bps, (unsigned long)ratio,
                       (unsigned long)wire, (unsigned long)wired[t], (unsigned long)airR[t], sensR[t]);
        found = 1; break;
      }
    }
    if (found == 0)
    {
      Console_Printf("%-6s  %7lu   x%lu   %-7lu  does not fit HC-12 (> 115200)\r\n",
                     Codec_Name((codec_id_t)c), (unsigned long)bps, (unsigned long)ratio, (unsigned long)wire);
    }
  }
  /* Codec2 (реализован): выбранный режим вокодера открывает дальнобойный эфирный режим,
   * недоступный простым кодекам. Сжатие — относительно raw 8 кГц (128000 бит/с). */
  {
    uint32_t bps   = c2ModeTab[g_c2mode].bps;
    uint32_t wire  = (bps * 13u) / 10u;
    uint32_t ratio = (bps != 0u) ? (128000u / bps) : 0u;
    int done = 0;
    for (t = 0; t < 8; t++)
    {
      if (wired[t] >= wire)
      {
        Console_Printf("codec2 %-4s %6lu  x%-3lu  %-7lu  %-7lu   %-6lu   %d dBm\r\n",
                       c2ModeTab[g_c2mode].name, (unsigned long)bps, (unsigned long)ratio,
                       (unsigned long)wire, (unsigned long)wired[t], (unsigned long)airR[t], sensR[t]);
        done = 1; break;
      }
    }
    if (done == 0)
    {
      Console_Printf("codec2 %-4s %6lu  x%-3lu  %-7lu  does not fit HC-12 (> 115200)\r\n",
                     c2ModeTab[g_c2mode].name, (unsigned long)bps, (unsigned long)ratio, (unsigned long)wire);
    }
  }
}

#endif /* VOICE_XPORT_UART: load/budget */

/* Аллокатор Codec2 (пул) — нужен ВСЕМ работам с Codec2 (в т.ч. UWB LAB09), поэтому вне транспортного
 * guard. Ниже (stk/c2load) — снова только провод.
 * ================= ЗАМЕР ЗАГРУЗКИ CODEC2 (TASK_codec2_port_and_load) =================
 * Голый замер БЕЗ интеграции в тракт: тракт речи (raw/ulaw/adpcm, кадрирование, джиттер)
 * не меняется, Codec2 к нему не подключается. Codec2 собран под __EMBEDDED__ и зовёт внешние
 * codec2_malloc/free — даём аллокатор на СТАТИЧЕСКОМ пуле (без кучи; high-water пула = ОЗУ
 * состояния). bump со сбросом при возврате счётчика аллокаций к нулю (create/destroy
 * сбалансированы); high-water консервативен (включает не переиспользуемые временные буферы
 * времени create). */
#define C2_POOL_BYTES 16384u    /* state ~9.4КБ (замерено); 16К с запасом, освобождает bss под стек */
static uint8_t  c2Pool[C2_POOL_BYTES] __attribute__((aligned(8)));
static uint32_t c2PoolTop = 0u, c2PoolCnt = 0u, c2PoolHigh = 0u;
static volatile uint8_t c2PoolFail = 0u;   /* пул исчерпан — codec2_malloc вернул NULL */

void *codec2_malloc(size_t size)
{
  uint32_t sz = (uint32_t)((size + 7u) & ~7u);
  void *p;
  if ((c2PoolTop + sz) > C2_POOL_BYTES) { c2PoolFail = 1u; return NULL; }   /* НЕ молча: флаг */
  p = &c2Pool[c2PoolTop]; c2PoolTop += sz; c2PoolCnt++;
  if (c2PoolTop > c2PoolHigh) { c2PoolHigh = c2PoolTop; }
  return p;
}
void *codec2_calloc(size_t nmemb, size_t size)
{
  size_t total = nmemb * size;
  void *p = codec2_malloc(total);
  if (p != NULL) { memset(p, 0, total); }
  return p;
}
void codec2_free(void *ptr)
{
  (void)ptr;
  if (c2PoolCnt != 0u) { c2PoolCnt--; }
  if (c2PoolCnt == 0u) { c2PoolTop = 0u; }   /* всё освобождено (destroy) → сброс пула */
}

#if VOICE_XPORT_UART   /* stk-замер + c2load + phy/rs485 — провод LAB07/08 */
/* Замер расхода стека заливкой известным значением. ВАЖНО (регресс прошлой версии): заливать
 * можно ТОЛЬКО зону НИЖЕ живых кадров, и делать это ПОД ЗАПРЕТОМ ПРЕРЫВАНИЙ — иначе кадр
 * прерывания (аудио-выход теперь высшего приоритета, с FPU-контекстом) попадёт в заливаемую
 * область и заливка затрёт его адрес возврата → отказ. Поэтому: окно ограничено (STK_WINDOW),
 * его верх отстоит от SP на STK_GUARD (больше глубины любого ISR-кадра, но замер enc/dec ~десятки
 * КБ ниже — не теряется), а сама запись/скан идут с __disable_irq. Прерывания во время самой
 * операции (не заливки) законно используют залитую зону — это и есть измеряемая высокая вода. */
extern char end[];              /* конец bss / начало кучи (линкер) — для оценки запаса */
extern uint32_t _estack;        /* верх стека (линкер) */
#define STK_PAT     0xA5A5A5A5u
#define STK_GUARD   256u         /* верх окна ниже SP на столько (>= кадр самих paint/used) */
#define STK_WINDOW  0xE000u      /* окно заливки 56 КБ (несатурируемо для ~40 КБ; выше bss/пула) */
static void stk_paint(uint32_t top)
{
  uint32_t prim = __get_PRIMASK();
  uint32_t *p = (uint32_t *)(top - STK_WINDOW);
  __disable_irq();
  while ((uint32_t)p < top) { *p++ = STK_PAT; }
  if (prim == 0u) { __enable_irq(); }
}
static uint32_t stk_used(uint32_t top)   /* байт стека ниже (top+GUARD≈SP), задетых после заливки */
{
  uint32_t prim = __get_PRIMASK();
  uint32_t *p = (uint32_t *)(top - STK_WINDOW);
  __disable_irq();
  while (((uint32_t)p < top) && (*p == STK_PAT)) { p++; }
  if (prim == 0u) { __enable_irq(); }
  return (top - (uint32_t)p) + STK_GUARD;
}

/* c2load [mode] — encode/decode Codec2 (замер тактов DWT, вход тишина/тон/шум) + честный расход
 * стека (create/encode/decode). Без аргумента — все режимы; с аргументом (напр. c2load 3200) —
 * один режим (для локализации). Тракт речи не трогает. */
#define C2_ITERS 8u              /* итераций на вход (максимум стабилен; было 30 — сократил) */
static void cmd_c2load(int argc, char **argv)
{
  static const struct { int mode; const char *name; } ml[] = {
    { CODEC2_MODE_3200, "3200" }, { CODEC2_MODE_2400, "2400" },
    { CODEC2_MODE_1600, "1600" }, { CODEC2_MODE_1300, "1300" },
    { CODEC2_MODE_700C, "700C" },
  };
  static const char *inName[3] = { "silence", "tone", "noise" };
  int      nml = (int)(sizeof(ml) / sizeof(ml[0]));
  uint32_t cyc_us = SystemCoreClock / 1000000u;
  uint32_t sCre[5] = {0}, sEnc[5] = {0}, sDec[5] = {0}, stRam[5] = {0}, msMode[5] = {0}, stkMax = 0u;
  uint8_t  ran[5] = {0};
  uint32_t tAll = HAL_GetTick();
  int only = -1, mi;

  if (argc > 1)
  {
    for (mi = 0; mi < nml; mi++) { if (strcmp(argv[1], ml[mi].name) == 0) { only = mi; } }
    if (only < 0) { Console_Write("usage: c2load [3200|2400|1600|1300|700C]\r\n"); return; }
  }
  /* Освободить рабочий экземпляр тракта, чтобы bump-пул стартовал с нуля (иначе замер стека/пула
   * поплывёт: повторные create/destroy бенча при живом c2inst не сбрасывают top). Тракт пересоздаст
   * экземпляр лениво при следующем кодировании/декодировании. */
  c2_release();

  Console_Printf("c2load Codec2 8kHz, %u iter/input, worst of silence/tone/noise:\r\n", (unsigned)C2_ITERS);
  Console_Write("mode  enc_max enc_avg dec_max dec_avg frame enc%  dec%  stateRAM   ms\r\n");
  Console_Flush();

  for (mi = 0; mi < nml; mi++)
  {
    struct CODEC2 *c2;
    uint32_t top;
    short          sp_in[320], sp_out[320];
    unsigned char  bits[16];
    int nsam, inp, it, i;
    uint32_t frame_us, eMax = 0u, eSum = 0u, dMax = 0u, dSum = 0u, cnt = 0u, seed = 22222u, tMode;
    if ((only >= 0) && (mi != only)) { continue; }

    Console_Printf("  %s: create...\r\n", ml[mi].name); Console_Flush();
    tMode = HAL_GetTick();
    top = (__get_MSP() - STK_GUARD) & ~3u;
    c2PoolFail = 0u; c2PoolHigh = 0u;
    stk_paint(top); c2 = codec2_create(ml[mi].mode); sCre[mi] = stk_used(top);
    if ((c2 == NULL) || (c2PoolFail != 0u))
    {
      Console_Printf("  %s: ПУЛ ИСЧЕРПАН (нужно > %lu B) — режим пропущен\r\n",
                     ml[mi].name, (unsigned long)C2_POOL_BYTES); Console_Flush();
      if (c2 != NULL) { codec2_destroy(c2); }
      continue;
    }
    stRam[mi] = c2PoolHigh;
    nsam = codec2_samples_per_frame(c2);
    if (nsam > 320) { nsam = 320; }
    frame_us = (uint32_t)nsam * 1000000u / 8000u;

    for (i = 0; i < nsam; i++) { seed = seed * 1103515245u + 12345u; sp_in[i] = (short)(seed >> 16); }
    stk_paint(top); codec2_encode(c2, bits, sp_in);  sEnc[mi] = stk_used(top);
    stk_paint(top); codec2_decode(c2, sp_out, bits); sDec[mi] = stk_used(top);
    if (sCre[mi] > stkMax) { stkMax = sCre[mi]; }
    if (sEnc[mi] > stkMax) { stkMax = sEnc[mi]; }
    if (sDec[mi] > stkMax) { stkMax = sDec[mi]; }

    for (inp = 0; inp < 3; inp++)
    {
      Console_Printf("  %s: %s...\r\n", ml[mi].name, inName[inp]); Console_Flush();
      for (i = 0; i < nsam; i++)
      {
        if      (inp == 0) { sp_in[i] = 0; }
        else if (inp == 1) { sp_in[i] = toneLUT[i & 0x0F]; }
        else { seed = seed * 1103515245u + 12345u; sp_in[i] = (short)(seed >> 16); }
      }
      for (it = 0; it < (int)C2_ITERS; it++)
      {
        uint32_t t0, d;
        t0 = DWT->CYCCNT; codec2_encode(c2, bits, sp_in);  d = (uint32_t)(DWT->CYCCNT - t0);
        if (d > eMax) { eMax = d; } eSum += d;
        t0 = DWT->CYCCNT; codec2_decode(c2, sp_out, bits); d = (uint32_t)(DWT->CYCCNT - t0);
        if (d > dMax) { dMax = d; } dSum += d;
        cnt++;
      }
    }
    msMode[mi] = HAL_GetTick() - tMode;
    ran[mi] = 1u;
    {
      uint32_t eMu = eMax / cyc_us, eAu = (eSum / cnt) / cyc_us;
      uint32_t dMu = dMax / cyc_us, dAu = (dSum / cnt) / cyc_us;
      uint32_t fus = (frame_us != 0u) ? frame_us : 1u;
      Console_Printf("%-5s %6lu %6lu %6lu %6lu %3lums %3lu%% %3lu%% %6luB %4lu\r\n",
                     ml[mi].name, (unsigned long)eMu, (unsigned long)eAu,
                     (unsigned long)dMu, (unsigned long)dAu, (unsigned long)(frame_us / 1000u),
                     (unsigned long)((eMu * 100u) / fus), (unsigned long)((dMu * 100u) / fus),
                     (unsigned long)stRam[mi], (unsigned long)msMode[mi]);
      Console_Flush();
    }
    codec2_destroy(c2);
  }

  Console_Write("stack (bytes, honest): mode create encode decode\r\n");
  for (mi = 0; mi < nml; mi++)
  {
    if (ran[mi] == 0u) { continue; }
    Console_Printf("  %-5s %6lu %6lu %6lu%s\r\n", ml[mi].name,
                   (unsigned long)sCre[mi], (unsigned long)sEnc[mi], (unsigned long)sDec[mi],
                   ((sEnc[mi] >= (STK_WINDOW - 256u)) || (sDec[mi] >= (STK_WINDOW - 256u))) ? " (SAT!)" : "");
  }
  {
    uint32_t freeStk = (uint32_t)&_estack - ((uint32_t)end + 0x200u);   /* область под стек, байт */
    Console_Printf("c2load: pool %lu B; stack max=%luB; free stack ~%luB (margin ~%luB); total %lu ms\r\n",
                   (unsigned long)C2_POOL_BYTES, (unsigned long)stkMax, (unsigned long)freeStk,
                   (unsigned long)((freeStk > stkMax) ? (freeStk - stkMax) : 0u),
                   (unsigned long)(HAL_GetTick() - tAll));
    Console_Flush();
  }
}

/* phy [ttl|rs485] — физический слой линии (по образцу decim). rs485 включает секвенс направления
 * (RS485_DE=PE7): подъём до передачи, опускание строго по TC. Смена слоя звук не рвёт (только флаг
 * + DE в приём). См. docs/REPORT_rs485_direction2.md. */
static void cmd_phy(int argc, char **argv)
{
  if (argc > 1)
  {
    if      (strcmp(argv[1], "ttl")   == 0) { UartPort_SetPhy(UARTPORT_PHY_TTL); }
    else if (strcmp(argv[1], "rs485") == 0) { UartPort_SetPhy(UARTPORT_PHY_RS485); }
    else { Console_Write("usage: phy ttl|rs485\r\n"); return; }
  }
  Console_Printf("phy = %s%s\r\n", UartPort_PhyName(),
                 (UartPort_PhyAllowsContention() == 0u) ? " (half-duplex: no simultaneous bidirectional TX)" : "");
}

/* rs485 — статистика направления + защитные интервалы/сторож. Установка:
 *   rs485 guard <preUs> <postUs>  — защитные интервалы до/после передачи (мкс);
 *   rs485 wd <marginUs>           — запас сторожевого таймера сверх времени кадра (мкс). */
static void cmd_rs485(int argc, char **argv)
{
  UartPort_Rs485Stats s;
  if ((argc > 1) && (strcmp(argv[1], "guard") == 0))
  {
    if (argc < 4) { Console_Write("usage: rs485 guard <preUs> <postUs>\r\n"); return; }
    UartPort_SetRs485Guard((uint32_t)strtoul(argv[2], NULL, 0), (uint32_t)strtoul(argv[3], NULL, 0));
  }
  else if ((argc > 1) && (strcmp(argv[1], "wd") == 0))
  {
    if (argc < 3) { Console_Write("usage: rs485 wd <marginUs>\r\n"); return; }
    UartPort_SetRs485WdMarginUs((uint32_t)strtoul(argv[2], NULL, 0));
  }
  else if (argc > 1) { Console_Write("usage: rs485 [guard <pre> <post> | wd <margin>]\r\n"); return; }

  UartPort_GetRs485Stats(&s);
  Console_Printf("rs485: phy=%s guard pre=%luus post=%luus wd_margin=%luus\r\n",
                 UartPort_PhyName(), (unsigned long)s.preUs, (unsigned long)s.postUs,
                 (unsigned long)s.wdMarginUs);
  Console_Printf("rs485: turnaround last=%luus max=%luus | premature=%lu wd_trips=%lu err_drops=%lu\r\n",
                 (unsigned long)s.turnaroundLastUs, (unsigned long)s.turnaroundMaxUs,
                 (unsigned long)s.premature, (unsigned long)s.watchdogTrips, (unsigned long)s.errorDrops);
}
#endif /* VOICE_XPORT_UART: phy/rs485 */

/* ================= НАБОРЫ КОМАНД ПО РАБОТАМ (этап 2 разделения) =================
 * Обработчики общие (движок), а curated-наборы — по работе. Обе таблицы компилируются в обеих
 * конфигурациях (voice.c общий), адаптер выбирает свою геттером Voice_CmdsLabNN. Так все обработчики
 * ссылаемы (нет unused-warning), а на консоли у каждой работы — только её команды. */

#if VOICE_XPORT_UART
/* LAB07 «сжатие на проводе»: полная лесенка кодеков + отладочные инструменты (вкл. RS-485 стенд). */
static const console_cmd_t k_cmds_lab07[] =
{
  { "send",     "send [n] test packets",                    cmd_send     },
  { "sendbyte", "send one raw byte",                        cmd_sendbyte },
  { "stat",     "link byte/error statistics",               cmd_stat     },
  { "reset",    "reset all counters",                       cmd_reset    },
  { "dump",     "hex of last raw received bytes",           cmd_dump     },
  { "regs",     "USART2 registers",                         cmd_regs     },
  { "baud",     "show/set UART baud",                       cmd_baud     },
  { "voice",    "voice path stats + rx line errors",        cmd_voice    },
  { "proto",    "framing protocol statistics (rx)",         cmd_proto    },
  { "ptt",      "ptt on|off (latches tx; blocks rx)",       cmd_ptt      },
  { "tone",     "tone on [freq]|off (default 1 kHz)",       cmd_tone     },
  { "codec",    "codec raw|ulaw|adpcm|codec2",              cmd_codec    },
  { "c2mode",   "codec2 mode 3200|2400|1600|1300|700C",     cmd_c2mode   },
  { "rate",     "rate 8000|16000",                          cmd_rate     },
  { "decim",    "decim fir|avg (16->8 anti-alias)",         cmd_decim    },
  { "headroom", "mic atten before codec (0|3|6|9|12 dB)",   cmd_headroom },
  { "load",     "codec core load (us/block, %)",            cmd_load     },
  { "budget",   "channel budget table (HC-12)",             cmd_budget   },
  { "c2load",   "Codec2 encode/decode load (bench, no path)", cmd_c2load },
  { "phy",      "phy ttl|rs485 (line physical layer)",      cmd_phy      },
  { "rs485",    "rs485 dir stats [guard/wd setters]",       cmd_rs485    },
};

/* LAB08 «речь по радио»: узкий полевой набор (вокодер + канал), без отладочно-проводных команд. */
static const console_cmd_t k_cmds_lab08[] =
{
  { "reset",    "reset all counters",                       cmd_reset    },
  { "codec",    "codec raw|ulaw|adpcm|codec2",              cmd_codec    },
  { "c2mode",   "codec2 mode 3200|2400|1600|1300|700C",     cmd_c2mode   },
  { "baud",     "show/set UART baud (HC-12 wired rate)",    cmd_baud     },
  { "prefill",  "prefill [ms] jitter-buffer start (def 60)", cmd_prefill },
  { "budget",   "channel budget table (HC-12)",             cmd_budget   },
  { "voice",    "voice path stats + rx line errors",        cmd_voice    },
  { "ptt",      "ptt on|off (latches tx; blocks rx)",       cmd_ptt      },
  { "stat",     "link byte/error statistics",               cmd_stat     },
};

const console_cmd_t *Voice_CmdsLab07(uint16_t *count)
{
  if (count != NULL) { *count = (uint16_t)(sizeof(k_cmds_lab07) / sizeof(k_cmds_lab07[0])); }
  return k_cmds_lab07;
}
const console_cmd_t *Voice_CmdsLab08(uint16_t *count)
{
  if (count != NULL) { *count = (uint16_t)(sizeof(k_cmds_lab08) / sizeof(k_cmds_lab08[0])); }
  return k_cmds_lab08;
}
#endif /* VOICE_XPORT_UART: наборы LAB07/08 */

#if VOICE_XPORT_UWB
/* LAB09 «речь по радио UWB»: узкий полевой набор (вокодер), радийные команды добавляет адаптер. */
static const console_cmd_t k_cmds_lab09[] =
{
  { "reset",    "reset all counters",                       cmd_reset    },
  { "codec",    "codec raw|ulaw|adpcm|codec2",              cmd_codec    },
  { "c2mode",   "codec2 mode 3200|2400|1600|1300|700C",     cmd_c2mode   },
  { "prefill",  "prefill [ms] jitter-buffer start",         cmd_prefill  },
  { "voice",    "voice path stats + radio counters",        cmd_voice    },
  { "ptt",      "ptt on|off (latches tx; blocks rx)",       cmd_ptt      },
  { "stat",     "radio link statistics",                    cmd_stat     },
};
const console_cmd_t *Voice_CmdsLab09(uint16_t *count)
{
  if (count != NULL) { *count = (uint16_t)(sizeof(k_cmds_lab09) / sizeof(k_cmds_lab09[0])); }
  return k_cmds_lab09;
}
#endif /* VOICE_XPORT_UWB */

/* ================= ИНТЕРФЕЙС ДВИЖКА =================
 * cfg — стартовые умолчания работы (кодек/частота/режим Codec2/скорость линии); cmds/ncmds — её
 * набор команд. Так провод (LAB07) и эфир (LAB08) расходятся только конфигом и таблицей. */
uint8_t Voice_Init(const VoiceConfig *cfg, const console_cmd_t *cmds, uint16_t ncmds)
{
  BSP_LED_Init(LED3); BSP_LED_Init(LED4); BSP_LED_Init(LED5); BSP_LED_Init(LED6);
  BSP_LED_Off(LED3); BSP_LED_Off(LED4); BSP_LED_Off(LED5); BSP_LED_Off(LED6);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Стартовые умолчания работы. */
  if (cfg != NULL)
  {
    g_codec  = cfg->codec;
    g_rate   = (cfg->rate8k != 0u) ? RATE_8K : RATE_16K;
    g_c2mode = (cfg->c2mode < (uint8_t)C2_MODE_COUNT) ? cfg->c2mode : 0u;
  }

  /* Счётчик тактов ядра для замера загрузки (как в pintest). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  Console_Init();
  Console_Register(cmds, ncmds);

#if VOICE_XPORT_UART
  Frame_DecoderInit(&rxDec);
  UartPort_Init();
  UartPort_SetRxTap(voice_rx_byte);
#endif
  /* Транспорт LAB09 (радио UWB) поднимает адаптер: SPI4 + сброс модуля до Voice_Init; полная
   * инициализация DW3000 — командой uwbinit. Здесь ставить нечего (кроме аудио/вытеснения). */

  if (Audio_Init() != 0u)
  {
    TRACE_ERR("LAB%02u: audio init failed", (unsigned)LAB_ID);
    Console_Printf("\r\nLAB%02u: AUDIO INIT FAILED\r\n", (unsigned)LAB_ID);
    return 1u;
  }

#if VOICE_XPORT_UART
  /* Стартовая скорость линии работы (обходит граблю «baud из .ioc»). SetBaud — DeInit/Init USART2,
   * его MspInit сбрасывает приоритеты в (0,0); следующий Preempt их восстановит. */
  UartPort_SetBaud((cfg != NULL) ? cfg->baud : 460800u);
#endif

  /* После всех MspInit (USART2/DMA/I2S приоритеты уже расставлены): включить вытеснение
   * (аудио-выход старше приёмных ISR). Для радио тоже нужно — аудио-выход не должен опаздывать. */
  Preempt_AudioOutHighest();

#if VOICE_XPORT_UART
  TRACE_LOG("LAB%02u voice: codec=%s rate=%u Hz, block=%u ms, USART2 %lu 8N1", (unsigned)LAB_ID,
            codec_disp_name(g_codec), (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u),
            (unsigned)BLOCK_MS, (unsigned long)UartPort_GetBaud());
  Console_Printf("\r\nLAB%02u ready: codec=%s rate=%u Hz baud=%lu. Hold PA0 to talk. 'help'.\r\n",
                 (unsigned)LAB_ID, codec_disp_name(g_codec),
                 (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u), (unsigned long)UartPort_GetBaud());
#else  /* UWB */
  TRACE_LOG("LAB%02u voice/UWB: codec=%s rate=%u Hz, block=%u ms (run 'uwbinit')", (unsigned)LAB_ID,
            codec_disp_name(g_codec), (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u), (unsigned)BLOCK_MS);
  Console_Printf("\r\nLAB%02u UWB voice ready: codec=%s rate=%u Hz. Run 'uwbinit', hold PA0 to talk. 'help'.\r\n",
                 (unsigned)LAB_ID, codec_disp_name(g_codec),
                 (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u));
#endif
  return 0u;
}

void Voice_Process(void)
{
  static uint32_t lastBlink = 0u;
  static uint8_t  pttRawLast = 0u;
  static uint32_t pttChangeTick = 0u;
  static uint8_t  pttBtn = 0u;
  static uint8_t  pttPrev = 0u;
  uint32_t now = HAL_GetTick();
  uint8_t  raw;

  /* Замер фактического периода вызова Voice_Process (макс. интервал = худшая добавленная
   * задержка разбора кадра). Только чтение CYCCNT + разность; печатается как loop_max. */
  {
    uint32_t cyc = DWT->CYCCNT;
    if (loopHavePrev != 0u) { uint32_t d = (uint32_t)(cyc - loopPrevCyc); if (d > loopMaxCyc) { loopMaxCyc = d; } }
    else { loopHavePrev = 1u; }
    loopPrevCyc = cyc;
  }

  Console_Process();

  raw = (BSP_PB_GetState(BUTTON_KEY) != 0u) ? 1u : 0u;
  if (raw != pttRawLast) { pttRawLast = raw; pttChangeTick = now; }
  else if (((uint32_t)(now - pttChangeTick) >= 20u) && (pttBtn != raw)) { pttBtn = raw; }
  g_ptt = (uint8_t)((pttBtn != 0u) || (g_pttCmd != 0u));

  /* Смена PTT: сбросить накопители передачи, чтобы недособранный кусок не перетёк в следующий
   * сеанс (TASK п.2). Фронт 0→1 — чистим 5-мс накопитель и кольцо Codec2 под запретом прерываний
   * (стартуем с нуля). Фронт 1→0 — достаточно опустошить кольцо со стороны потребителя (ISR уже
   * не пишет: Audio_OnCapture выходит при g_ptt==0). */
  if ((g_ptt != 0u) && (pttPrev == 0u))
  {
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    accN = 0u; c2TxHead = 0u; c2TxTail = 0u;
    if (prim == 0u) { __enable_irq(); }
  }
  else if ((g_ptt == 0u) && (pttPrev != 0u))
  {
    c2TxTail = c2TxHead;
#if VOICE_XPORT_UWB
    Dw3000Port_VoiceRxArm();   /* полудуплекс: после передачи явно вернуть приёмник в приём */
#endif
  }
  pttPrev = g_ptt;

#if VOICE_XPORT_UWB
  /* Приём радиокадров опросом (пока не PTT): драйвер отдаёт целый payload -> в ту же очередь. */
  if (g_ptt == 0u) { Dw3000Port_VoicePoll(frame_enqueue); }
#endif
  rx_drain();          /* offload: разбор принятых кадров здесь, а не в приёмном ISR */
  c2_tx_process();     /* codec2: кодирование накопленных отсчётов и отправка (тоже главный цикл) */
#if VOICE_XPORT_UART
  UartPort_Rs485Poll();/* сторож направления RS-485: опустить DE, если передача зависла (no-op при TTL) */
#endif

  if (g_ptt != 0u) { BSP_LED_On(LED3); } else { BSP_LED_Off(LED3); }
  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u) { lastBlink = now; BSP_LED_Toggle(LED6); }
  }
  else { BSP_LED_Off(LED6); }
}

#endif /* (LAB_ID == 7) || (LAB_ID == 8) */
