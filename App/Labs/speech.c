/**
  ******************************************************************************
  * @file    App/Labs/speech.c
  * @author  Wagan Sarukhanov
  * @brief   LAB07 «Сжатие речи»: речь по проводу (как LAB04) со сменными кодеками
  *          (raw/µ-law/ADPCM) и частотой 8/16 кГц. Реализует Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только в конфигурации LAB07 (LAB_ID == 7) — весь файл обёрнут ниже.
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
  * тяжёлый разбор (Codec_Decode, апсемплинг, jb_push = voice_sink) вынесен в Lab_Process
  * (rx_drain), чтобы приёмный ISR не задерживал 1-мс пере-взвод аудио-выхода — см.
  * docs/REPORT_rx_isr_offload.md и docs/REPORT_isr_deadline_probe.md.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 7

#include "console.h"
#include "uart_port.h"
#include "frame.h"
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

#define JB_SIZE         2048u
#define JB_MASK         (JB_SIZE - 1u)
#define JB_PREFILL      320u                     /* 20 мс @16 кГц */
#define LOSS_FILL_CAP   8u

#define RATE_8K         0u
#define RATE_16K        1u

/* Антиалиас-фильтр децимации 16→8 (см. docs/REPORT_codec2_recon.md и REPORT_aa_decim.md).
 * Старый способ (усреднение пар) — очень слабый НЧ: тон 5 кГц проходил и отражался в 3 кГц.
 * Новый — линейно-фазовый FIR-НЧ перед прореживанием 2:1. Оба доступны (команда decim). */
#define AA_TAPS         31u                      /* тапов FIR (нечётное → линейная фаза, тип I) */
#define DECIM_AVG       0u                       /* старый способ: усреднение пар */
#define DECIM_FIR       1u                       /* новый: FIR-НЧ ~3.6 кГц + прореживание */

/* ================= НАСТРОЙКИ (по умолчанию) ================= */
static volatile uint8_t  g_codec = (uint8_t)CODEC_ADPCM; /* кодек по умолчанию */
static volatile uint8_t  g_rate  = RATE_16K;             /* частота по умолчанию */
static volatile uint8_t  g_decim = DECIM_FIR;            /* метод децимации 16→8 (по умолч. FIR) */
static volatile uint8_t  g_ptt   = 0u;
static volatile uint8_t  g_tone  = 0u;
static volatile uint16_t g_toneHz = 1000u;              /* частота тона (Гц), по умолчанию 1 кГц */
static uint8_t           g_pttCmd = 0u;

/* ================= ПЕРЕДАЧА (аудио-ISR) ================= */
static int16_t  acc16[SAMP16];
static uint16_t accN = 0u;
static uint16_t txSeq = 0u;
static uint8_t  txPayload[PAYLOAD_MAX];
static uint8_t  txEnc[ENC_MAX];

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
static Frame_Decoder     rxDec;
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
static volatile uint32_t loopMaxCyc = 0u;             /* макс. интервал между вызовами Lab_Process (тактов) */
static uint32_t          loopPrevCyc = 0u;
static uint8_t           loopHavePrev = 0u;

/* ================= ОЧЕРЕДЬ КАДРОВ: вынос разбора из ISR (TASK_rx_isr_offload) =================
 * Приёмный ISR (uart_port RxEventCallback → voice_rx_byte → Frame_DecodeByte) при готовом
 * КАДРЕ лишь КОПИРУЕТ его в очередь (frame_enqueue) и двигает head. Тяжёлый разбор
 * (заголовок, Codec_Decode, апсемплинг, jb_push = voice_sink) выполняется в Lab_Process
 * (rx_drain). Так приёмный ISR перестаёт задерживать 1-мс пере-взвод аудио-выхода.
 *
 * Один производитель (ISR пишет ТОЛЬКО fqHead и слоты) и один потребитель (Lab_Process
 * пишет ТОЛЬКО fqTail) → индексы атомарны по построению, критических секций в ISR нет.
 *
 * Глубина: кадр приходит каждые 5 мс; период Lab_Process — доли мс (консоль неблокирующая:
 * raw_put отбрасывает при полном кольце, tx_drain — один CDC_Transmit_FS без ожидания),
 * т.е. между разгрузками накапливается <1 кадра. FQ_DEPTH=16 = 80 мс буфера — >16× запас
 * даже против маловероятной 5-мс задержки цикла; фактический период показывает loop_max в
 * команде voice. Переполнение считается cQover (молчаливой потери нет). Слот = FRAME_MAX_CONTENT,
 * чтобы никогда не усекать CRC-корректный кадр. */
#define FQ_DEPTH   16u
#define FQ_MASK    (FQ_DEPTH - 1u)
typedef struct { uint16_t len; uint8_t data[FRAME_MAX_CONTENT]; } fq_item_t;
static fq_item_t         fq[FQ_DEPTH];
static volatile uint16_t fqHead = 0u;   /* пишет только ISR-производитель */
static volatile uint16_t fqTail = 0u;   /* пишет только потребитель (Lab_Process) */

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

  /* 2) 8 кГц — децимация 16→8 (FIR-антиалиас или старое усреднение); 16 кГц — без изменений. */
  if (g_rate == RATE_8K)
  {
    if (g_decim == DECIM_FIR) { aa_decimate(src16, work); }
    else { for (i = 0u; i < SAMP8; i++) { work[i] = (int16_t)(((int32_t)src16[2u * i] + src16[2u * i + 1u]) / 2); } }
    nsamp = SAMP8;
  }
  else { for (i = 0u; i < SAMP16; i++) { work[i] = src16[i]; } nsamp = SAMP16; }

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

  {
    uint16_t e = Frame_Encode(txPayload, frameLen, txEnc, (uint16_t)sizeof(txEnc));
    if ((e != 0u) && (UartPort_SendRaw(txEnc, e) == 0u)) { cSent++; }
    else { cTxDrop++; }
  }
  txSeq++; accN = 0u;
}

void Audio_OnCapture(const int16_t *mono, uint16_t n)
{
  uint16_t i;
  cRec++;
  if (g_ptt == 0u) { return; }
  for (i = 0u; i < n; i++)
  {
    /* Накапливаем звук с микрофона; подмена на тон — в tx_flush на рабочей частоте. */
    acc16[accN++] = mono[i];
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
    if (jb_fill() >= JB_PREFILL) { jbPlaying = 1u; }
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
  if (len > (uint16_t)FRAME_MAX_CONTENT) { len = (uint16_t)FRAME_MAX_CONTENT; }
  fq[fqHead].len = len;
  for (i = 0u; i < len; i++) { fq[fqHead].data[i] = payload[i]; }
  __DMB();                                               /* содержимое слота видно до публикации head */
  fqHead = nh;
}

/* ================= ПРИЁМ КАДРОВ (разбор — в Lab_Process через rx_drain) ================= */
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

/* sink в ISR — только постановка кадра в очередь (разбор перенесён в rx_drain/Lab_Process). */
static void voice_rx_byte(uint8_t b) { Frame_DecodeByte(&rxDec, b, frame_enqueue); }

/* ================= КОМАНДЫ (база как в LAB04) ================= */
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
static void cmd_stat(int argc, char **argv)
{
  UartPort_Stats s; (void)argc; (void)argv; UartPort_GetStats(&s);
  Console_Printf("link: baud=%lu bytes tx=%lu rx=%lu | err ore=%lu fe=%lu pe=%lu ne=%lu\r\n",
                 (unsigned long)UartPort_GetBaud(), (unsigned long)s.bytesTx, (unsigned long)s.bytesRx,
                 (unsigned long)s.errOre, (unsigned long)s.errFe, (unsigned long)s.errPe, (unsigned long)s.errNe);
}
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
static void cmd_reset(int argc, char **argv)
{
  (void)argc; (void)argv;
  UartPort_ResetStats();
  cRec = cSent = cTxDrop = cRecv = cLost = cPlayed = cUnder = cOver = 0u;
  encCyc = encCnt = decCyc = decCnt = 0u;
  rxDec.stats.framesRx = rxDec.stats.framesCrc = rxDec.stats.resync = rxDec.stats.bytesDropped = 0u;
  Audio_ResetOutProbe();        /* диагностика A: обнулить и задать базу «секунд от старта» */
  UartPort_ResetRxIsrProbe();   /* диагностика B */
  fqHead = fqTail = 0u; cQover = 0u;           /* очередь кадров offload */
  loopMaxCyc = 0u; loopHavePrev = 0u;          /* probeC: период Lab_Process */
  Console_Write("counters reset\r\n");
}
static void cmd_voice(int argc, char **argv)
{
  UartPort_Stats s; (void)argc; (void)argv; UartPort_GetStats(&s);
  Console_Printf("voice: rec=%lu sent=%lu txdrop=%lu recv=%lu lost=%lu played=%lu under=%lu over=%lu\r\n",
                 (unsigned long)cRec, (unsigned long)cSent, (unsigned long)cTxDrop, (unsigned long)cRecv,
                 (unsigned long)cLost, (unsigned long)cPlayed, (unsigned long)cUnder, (unsigned long)cOver);
  Console_Printf("voice: codec=%s rate=%u Hz decim=%s jb_fill=%u (%u ms) ptt=%s tone=%s | rx err ore=%lu fe=%lu pe=%lu ne=%lu\r\n",
                 Codec_Name((codec_id_t)g_codec), (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u),
                 (g_decim == DECIM_FIR) ? "fir" : "avg",
                 (unsigned)jb_fill(), (unsigned)(jb_fill() / AUDIO_BLOCK_SAMPLES),
                 (g_ptt ? "on" : "off"), (g_tone ? "on" : "off"),
                 (unsigned long)s.errOre, (unsigned long)s.errFe, (unsigned long)s.errPe, (unsigned long)s.errNe);
  /* Диагностика срыва дедлайна аудио-выхода (TASK_isr_deadline_probe) — отдельными строками. */
  {
    Audio_OutProbe ap;
    uint32_t bcalls = 0u, bmax = 0u, bavg = 0u, blong = 0u;
    Audio_GetOutProbe(&ap);
    UartPort_GetRxIsrProbe(&bcalls, &bmax, &bavg, &blong);
    Console_Printf("probeA out-rearm: calls=%lu over(>%luus)=%lu max=%luus first=%lus last=%lus lastmin=%lu%s\r\n",
                   (unsigned long)ap.calls, (unsigned long)ap.threshUs, (unsigned long)ap.over,
                   (unsigned long)ap.maxUs, (unsigned long)ap.firstSec, (unsigned long)ap.lastSec,
                   (unsigned long)ap.lastMinOver, (ap.lastMinCapped != 0u) ? "(capped)" : "");
    Console_Printf("probeB rx-isr: calls=%lu max=%luus avg=%luus long(>%uus)=%lu\r\n",
                   (unsigned long)bcalls, (unsigned long)bmax, (unsigned long)bavg,
                   (unsigned)UARTPORT_RXISR_LONG_US, (unsigned long)blong);
    Console_Printf("probeC offload: qover=%lu qdepth=%u loop_max=%luus\r\n",
                   (unsigned long)cQover, (unsigned)FQ_DEPTH,
                   (unsigned long)(loopMaxCyc / (SystemCoreClock / 1000000u)));
  }
}
static void cmd_proto(int argc, char **argv)
{
  (void)argc; (void)argv;
  Console_Printf("proto: frames=%lu crc_drop=%lu resync=%lu bytes_dropped=%lu\r\n",
                 (unsigned long)rxDec.stats.framesRx, (unsigned long)rxDec.stats.framesCrc,
                 (unsigned long)rxDec.stats.resync, (unsigned long)rxDec.stats.bytesDropped);
}
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

/* codec [raw|ulaw|adpcm] */
static void cmd_codec(int argc, char **argv)
{
  if (argc > 1)
  {
    if      (strcmp(argv[1], "raw")   == 0) { g_codec = (uint8_t)CODEC_RAW; }
    else if (strcmp(argv[1], "ulaw")  == 0) { g_codec = (uint8_t)CODEC_ULAW; }
    else if (strcmp(argv[1], "adpcm") == 0) { g_codec = (uint8_t)CODEC_ADPCM; }
    else { Console_Write("usage: codec raw|ulaw|adpcm\r\n"); return; }
  }
  Console_Printf("codec = %s\r\n", Codec_Name((codec_id_t)g_codec));
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
    if      (r == 8000ul)  { g_rate = RATE_8K; }
    else if (r == 16000ul) { g_rate = RATE_16K; }
    else { Console_Write("usage: rate 8000|16000\r\n"); return; }
  }
  Console_Printf("rate = %u Hz\r\n", (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u));
}

/* load — загрузка ядра кодеком (DWT). µs на блок и % от реального времени блока (BLOCK_MS). */
static void cmd_load(int argc, char **argv)
{
  uint32_t eus, dus, period_us;
  (void)argc; (void)argv;
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
  /* Ориентир: настоящий вокодер Codec2 @ 3200 бит/с (пока не реализован) — открывает
   * дальнобойный режим, недоступный даже ADPCM. */
  {
    uint32_t wire = (3200u * 13u) / 10u;   /* = 4160 */
    for (t = 0; t < 8; t++)
    {
      if (wired[t] >= wire)
      {
        Console_Printf("codec2*    3200   x40+   %-7lu  %-7lu   %-6lu   %d dBm  [plan]\r\n",
                       (unsigned long)wire, (unsigned long)wired[t], (unsigned long)airR[t], sensR[t]);
        break;
      }
    }
  }
}

/* ================= ЗАМЕР ЗАГРУЗКИ CODEC2 (TASK_codec2_port_and_load) =================
 * Голый замер БЕЗ интеграции в тракт: тракт речи (raw/ulaw/adpcm, кадрирование, джиттер)
 * не меняется, Codec2 к нему не подключается. Codec2 собран под __EMBEDDED__ и зовёт внешние
 * codec2_malloc/free — даём аллокатор на СТАТИЧЕСКОМ пуле (без кучи; high-water пула = ОЗУ
 * состояния). bump со сбросом при возврате счётчика аллокаций к нулю (create/destroy
 * сбалансированы); high-water консервативен (включает не переиспользуемые временные буферы
 * времени create). */
#define C2_POOL_BYTES 16384u    /* state ~9.4КБ (замерено); 16К с запасом, освобождает bss под стек */
static uint8_t  c2Pool[C2_POOL_BYTES] __attribute__((aligned(8)));
static uint32_t c2PoolTop = 0u, c2PoolCnt = 0u, c2PoolHigh = 0u;

void *codec2_malloc(size_t size)
{
  uint32_t sz = (uint32_t)((size + 7u) & ~7u);
  void *p;
  if ((c2PoolTop + sz) > C2_POOL_BYTES) { return NULL; }
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

/* Честный (несатурируемый) замер расхода стека. Заливаем ВСЮ свободную область стека — от конца
 * bss/кучи (символ линкера `end`) до текущего SP минус запас — и после операции считаем, докуда
 * стек дошёл вниз. Область гарантированно только стековая (выше bss/пула), поэтому не-стековые
 * записи её не портят; окно = вся свободная RAM под стек (десятки КБ) → результат не насыщается. */
extern char end[];              /* конец bss / начало кучи (линкер) */
extern uint32_t _estack;        /* верх стека (линкер) */
#define STK_PAT    0xA5A5A5A5u
#define STK_GUARD  512u          /* пропуск под кадры самих paint/used (выше заливаемой зоны) */
static uint32_t stk_bottom(void) { return (((uint32_t)end + 0x600u) + 3u) & ~3u; }  /* +куча+запас */
static void stk_paint(uint32_t top)
{
  uint32_t *p = (uint32_t *)stk_bottom();
  while ((uint32_t)p < top) { *p++ = STK_PAT; }
}
static uint32_t stk_used(uint32_t top)   /* байт стека ниже top, задетых после заливки */
{
  uint32_t *p = (uint32_t *)stk_bottom();
  while (((uint32_t)p < top) && (*p == STK_PAT)) { p++; }
  return (top - (uint32_t)p) + STK_GUARD;
}

/* c2load — encode/decode по режимам (замер тактов DWT, вход тишина/тон/шум) + честный расход
 * стека раздельно на create/encode/decode. Тракт не трогает. */
static void cmd_c2load(int argc, char **argv)
{
  static const struct { int mode; const char *name; } ml[] = {
    { CODEC2_MODE_3200, "3200" }, { CODEC2_MODE_2400, "2400" },
    { CODEC2_MODE_1600, "1600" }, { CODEC2_MODE_1300, "1300" },
    { CODEC2_MODE_700C, "700C" },
  };
  int      nml = (int)(sizeof(ml) / sizeof(ml[0]));
  uint32_t cyc_us = SystemCoreClock / 1000000u;
  uint32_t sCre[5] = {0}, sEnc[5] = {0}, sDec[5] = {0}, stRam[5] = {0}, stkMax = 0u;
  int mi;
  (void)argc; (void)argv;

  Console_Write("c2load Codec2 8kHz, 30 iter/input, worst of silence/tone/noise (-O2 hard-float):\r\n");
  Console_Write("mode  enc_max enc_avg dec_max dec_avg frame enc%  dec%  stateRAM\r\n");

  for (mi = 0; mi < nml; mi++)
  {
    struct CODEC2 *c2;
    uint32_t top = (__get_MSP() - STK_GUARD) & ~3u;   /* верх заливаемой зоны (ниже наших кадров) */
    short          sp_in[320], sp_out[320];
    unsigned char  bits[16];
    int nsam, inp, it, i;
    uint32_t frame_us, eMax = 0u, eSum = 0u, dMax = 0u, dSum = 0u, cnt = 0u, seed = 22222u;

    c2PoolHigh = 0u;
    stk_paint(top); c2 = codec2_create(ml[mi].mode); sCre[mi] = stk_used(top);
    if (c2 == NULL) { Console_Printf("%-5s create failed (pool too small)\r\n", ml[mi].name); continue; }
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
      for (i = 0; i < nsam; i++)
      {
        if      (inp == 0) { sp_in[i] = 0; }
        else if (inp == 1) { sp_in[i] = toneLUT[i & 0x0F]; }
        else { seed = seed * 1103515245u + 12345u; sp_in[i] = (short)(seed >> 16); }
      }
      for (it = 0; it < 30; it++)
      {
        uint32_t t0, d;
        t0 = DWT->CYCCNT; codec2_encode(c2, bits, sp_in);  d = (uint32_t)(DWT->CYCCNT - t0);
        if (d > eMax) { eMax = d; } eSum += d;
        t0 = DWT->CYCCNT; codec2_decode(c2, sp_out, bits); d = (uint32_t)(DWT->CYCCNT - t0);
        if (d > dMax) { dMax = d; } dSum += d;
        cnt++;
      }
    }
    {
      uint32_t eMu = eMax / cyc_us, eAu = (eSum / cnt) / cyc_us;
      uint32_t dMu = dMax / cyc_us, dAu = (dSum / cnt) / cyc_us;
      uint32_t fus = (frame_us != 0u) ? frame_us : 1u;
      Console_Printf("%-5s %6lu %6lu %6lu %6lu %3lums %3lu%% %3lu%% %5luB\r\n",
                     ml[mi].name, (unsigned long)eMu, (unsigned long)eAu,
                     (unsigned long)dMu, (unsigned long)dAu, (unsigned long)(frame_us / 1000u),
                     (unsigned long)((eMu * 100u) / fus),
                     (unsigned long)((dMu * 100u) / fus), (unsigned long)stRam[mi]);
    }
    codec2_destroy(c2);
  }

  Console_Write("stack (bytes, honest paint): mode create encode decode\r\n");
  for (mi = 0; mi < nml; mi++)
  {
    Console_Printf("  %-5s %6lu %6lu %6lu\r\n", ml[mi].name,
                   (unsigned long)sCre[mi], (unsigned long)sEnc[mi], (unsigned long)sDec[mi]);
  }
  {
    uint32_t freeStk = (uint32_t)&_estack - ((uint32_t)end + 0x200u);   /* область под стек, байт */
    Console_Printf("c2load: state pool high=%luB/%lu; stack max=%luB; free stack region ~%luB (margin ~%luB)\r\n",
                   (unsigned long)stRam[0], (unsigned long)C2_POOL_BYTES, (unsigned long)stkMax,
                   (unsigned long)freeStk, (unsigned long)(freeStk - stkMax));
  }
}

static const console_cmd_t k_cmds[] =
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
  { "codec",    "codec raw|ulaw|adpcm",                     cmd_codec    },
  { "rate",     "rate 8000|16000",                          cmd_rate     },
  { "decim",    "decim fir|avg (16->8 anti-alias)",         cmd_decim    },
  { "budget",   "channel budget table (HC-12)",             cmd_budget   },
  { "load",     "codec core load (us/block, %)",            cmd_load     },
  { "c2load",   "Codec2 encode/decode load (bench, no path)", cmd_c2load },
};

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  BSP_LED_Init(LED3); BSP_LED_Init(LED4); BSP_LED_Init(LED5); BSP_LED_Init(LED6);
  BSP_LED_Off(LED3); BSP_LED_Off(LED4); BSP_LED_Off(LED5); BSP_LED_Off(LED6);
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Счётчик тактов ядра для замера загрузки (как в pintest). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  Console_Init();
  Console_Register(k_cmds, (uint16_t)(sizeof(k_cmds) / sizeof(k_cmds[0])));

  Frame_DecoderInit(&rxDec);
  UartPort_Init();
  UartPort_SetRxTap(voice_rx_byte);

  if (Audio_Init() != 0u)
  {
    TRACE_ERR("LAB07: audio init failed");
    Console_Write("\r\nLAB07: AUDIO INIT FAILED\r\n");
    return 1u;
  }

  /* После всех MspInit (USART2/DMA/I2S приоритеты уже расставлены): включить вытеснение. */
  Preempt_AudioOutHighest();

  TRACE_LOG("LAB07 speech: codec=%s rate=%u Hz, block=%u ms, USART2 %lu 8N1",
            Codec_Name((codec_id_t)g_codec), (unsigned)((g_rate == RATE_8K) ? 8000u : 16000u),
            (unsigned)BLOCK_MS, (unsigned long)UartPort_GetBaud());
  Console_Write("\r\nLAB07 speech ready. 'codec', 'rate', 'budget', 'load'. Hold PA0 to talk. 'help'.\r\n");
  return 0u;
}

void Lab_Process(void)
{
  static uint32_t lastBlink = 0u;
  static uint8_t  pttRawLast = 0u;
  static uint32_t pttChangeTick = 0u;
  static uint8_t  pttBtn = 0u;
  uint32_t now = HAL_GetTick();
  uint8_t  raw;

  /* Замер фактического периода вызова Lab_Process (макс. интервал = худшая добавленная
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

  rx_drain();   /* offload: разбор принятых кадров здесь, а не в приёмном ISR */

  if (g_ptt != 0u) { BSP_LED_On(LED3); } else { BSP_LED_Off(LED3); }
  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u) { lastBlink = now; BSP_LED_Toggle(LED6); }
  }
  else { BSP_LED_Off(LED6); }
}

#endif /* LAB_ID == 7 */
