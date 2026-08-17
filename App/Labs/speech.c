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
  * Печать из ISR запрещена: аудио-хуки и разбор кадров — только счётчики/буферы.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 7

#include "console.h"
#include "uart_port.h"
#include "frame.h"
#include "audio.h"
#include "codec.h"
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

/* ================= ПРИЁМ КАДРОВ (ISR через RxTap) ================= */
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

static void voice_rx_byte(uint8_t b) { Frame_DecodeByte(&rxDec, b, voice_sink); }

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
  UartPort_SetBaud((uint32_t)b); Console_Printf("baud set to %lu\r\n", (unsigned long)UartPort_GetBaud());
}
static void cmd_reset(int argc, char **argv)
{
  (void)argc; (void)argv;
  UartPort_ResetStats();
  cRec = cSent = cTxDrop = cRecv = cLost = cPlayed = cUnder = cOver = 0u;
  encCyc = encCnt = decCyc = decCnt = 0u;
  rxDec.stats.framesRx = rxDec.stats.framesCrc = rxDec.stats.resync = rxDec.stats.bytesDropped = 0u;
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

  Console_Process();

  raw = (BSP_PB_GetState(BUTTON_KEY) != 0u) ? 1u : 0u;
  if (raw != pttRawLast) { pttRawLast = raw; pttChangeTick = now; }
  else if (((uint32_t)(now - pttChangeTick) >= 20u) && (pttBtn != raw)) { pttBtn = raw; }
  g_ptt = (uint8_t)((pttBtn != 0u) || (g_pttCmd != 0u));

  if (g_ptt != 0u) { BSP_LED_On(LED3); } else { BSP_LED_Off(LED3); }
  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u) { lastBlink = now; BSP_LED_Toggle(LED6); }
  }
  else { BSP_LED_Off(LED6); }
}

#endif /* LAB_ID == 7 */
