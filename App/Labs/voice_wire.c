/**
  ******************************************************************************
  * @file    App/Labs/voice_wire.c
  * @author  Wagan Sarukhanov
  * @brief   LAB04 «Речь по проводу»: PCM 16 кГц с микрофона → SLIP-кадры по USART2 → джиттер-
  *          буфер → кодек другой платы. Полудуплекс с PTT. Реализует Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только в конфигурации LAB04 (LAB_ID == 4) — весь файл обёрнут ниже.
  *
  * Тракт: микрофон (общий модуль App/Common/audio.c даёт моно-блоки по 1 мс) → накопление в
  * речевой кадр (VOICE_MS мс) → упаковка SLIP (App/Common/frame.c) → USART2 (uart_port) →
  * на приёме свой Frame_Decoder → джиттер-буфер (кольцо отсчётов) → воспроизведение по 1 мс.
  * Полудуплекс: пока PTT нажата — плата ПЕРЕДАЁТ и не воспроизводит; отпущена — слушает.
  *
  * Печать из ISR запрещена: аудио-хуки и разбор кадров — только счётчики/буферы; вся печать
  * из Lab_Process (консоль по USB CDC).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 4

#include "console.h"
#include "uart_port.h"
#include "frame.h"
#include "audio.h"
#include "preempt.h"
#include "trace_log.h"
#include "stm32f4xx_hal.h"
#include "stm32f411e_discovery.h"         /* LED + кнопка PA0 (BSP_PB) */
#include <stdlib.h>
#include <string.h>

/* ================= ПАРАМЕТРЫ (обоснование — в docs/REPORT_lab04_voice_wire.md) ================= */
#define VOICE_MS            5u                                   /* мс звука в одном кадре */
#define VOICE_SAMPLES       (AUDIO_BLOCK_SAMPLES * VOICE_MS)     /* = 80 отсчётов (моно) в кадре */
#define VOICE_HDR           2u                                    /* заголовок кадра: seq (u16) */
#define VOICE_PL_BYTES      (VOICE_HDR + VOICE_SAMPLES * 2u)      /* полезная нагрузка кадра, байт */
#define VOICE_ENC_MAX       (2u + 2u * (VOICE_PL_BYTES + 2u))     /* худший случай кодирования SLIP */

#define JB_SIZE             2048u          /* кольцо отсчётов джиттер-буфера (степень двойки) */
#define JB_MASK             (JB_SIZE - 1u)
#define JB_PREFILL          320u           /* порог старта воспроизведения = 20 мс (антиджиттер) */
#define LOSS_FILL_CAP       8u             /* максимум кадров потерь, заполняемых тишиной */

/* ================= СОСТОЯНИЕ ПЕРЕДАЧИ (пишется из аудио-ISR) ================= */
static int16_t  txAccum[VOICE_SAMPLES];
static uint16_t txAccN = 0u;
static uint16_t txSeq  = 0u;
static uint8_t  txEnc[VOICE_ENC_MAX];

/* Тон 1 кГц: 16 отсчётов на период при 16 кГц. */
static const int16_t toneLUT[16] =
{ 0, 3062, 5657, 7391, 8000, 7391, 5657, 3062, 0, -3062, -5657, -7391, -8000, -7391, -5657, -3062 };
static uint8_t  tonePhase = 0u;

/* ================= ПРИЁМ / ДЖИТТЕР-БУФЕР ================= */
static Frame_Decoder    rxDec;
static uint16_t         rxExpSeq = 0u;
static uint8_t          rxHaveSeq = 0u;

static int16_t          jbuf[JB_SIZE];
static volatile uint16_t jbHead = 0u;      /* пишет приёмный ISR */
static volatile uint16_t jbTail = 0u;      /* пишет кодечный ISR */
static volatile uint8_t  jbPlaying = 0u;   /* прошли ли порог prefill */
static int16_t           lastOut[AUDIO_BLOCK_SAMPLES];  /* последний блок — для маскировки провала */

/* ================= УПРАВЛЕНИЕ ================= */
static volatile uint8_t g_ptt   = 0u;      /* эффективный PTT (кнопка || команда) — читается в ISR */
static volatile uint8_t g_tone  = 0u;      /* передавать тон вместо микрофона */
static uint8_t          g_pttCmd = 0u;     /* форсирование передачи командой ptt on */

/* ================= СЧЁТЧИКИ (volatile: пишутся в ISR, читаются в консоли) ================= */
static volatile uint32_t cRec    = 0u;     /* блоков с микрофона */
static volatile uint32_t cSent   = 0u;     /* кадров отправлено */
static volatile uint32_t cTxDrop = 0u;     /* кадров отброшено на передаче (линия занята) */
static volatile uint32_t cRecv   = 0u;     /* кадров принято */
static volatile uint32_t cLost   = 0u;     /* кадров потеряно (разрыв seq) */
static volatile uint32_t cPlayed = 0u;     /* блоков воспроизведено */
static volatile uint32_t cUnder  = 0u;     /* опустошений джиттер-буфера */
static volatile uint32_t cOver   = 0u;     /* переполнений джиттер-буфера */

static uint16_t jb_fill(void) { return (uint16_t)((jbHead - jbTail) & JB_MASK); }

/* Положить отсчёты в джиттер-буфер (SPSC: только приёмный ISR двигает head). Переполнение —
 * входящее отбрасываем (чтобы не двигать tail из чужого ISR). */
static void jb_push(const int16_t *s, uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++)
  {
    uint16_t nx = (uint16_t)((jbHead + 1u) & JB_MASK);
    if (nx == jbTail) { cOver++; return; }
    jbuf[jbHead] = s[i];
    jbHead = nx;
  }
}

static void jb_push_silence(uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++)
  {
    uint16_t nx = (uint16_t)((jbHead + 1u) & JB_MASK);
    if (nx == jbTail) { cOver++; return; }
    jbuf[jbHead] = 0;
    jbHead = nx;
  }
}

/* ================= ПЕРЕДАЧА (аудио-ISR) ================= */
static void tx_flush_frame(void)
{
  uint8_t  pl[VOICE_PL_BYTES];
  uint16_t i;
  uint16_t enc;

  pl[0] = (uint8_t)(txSeq >> 8);
  pl[1] = (uint8_t)(txSeq & 0xFFu);
  for (i = 0u; i < VOICE_SAMPLES; i++)
  {
    pl[VOICE_HDR + 2u * i]      = (uint8_t)((uint16_t)txAccum[i] >> 8);
    pl[VOICE_HDR + 2u * i + 1u] = (uint8_t)((uint16_t)txAccum[i] & 0xFFu);
  }
  enc = Frame_Encode(pl, (uint16_t)VOICE_PL_BYTES, txEnc, (uint16_t)sizeof(txEnc));
  if ((enc != 0u) && (UartPort_SendRaw(txEnc, enc) == 0u)) { cSent++; }
  else { cTxDrop++; }                 /* линия занята/ошибка — кадр отбрасываем (задержка важнее) */
  txSeq++;                            /* seq растёт всегда — приёмник увидит потерю по разрыву */
  txAccN = 0u;
}

void Audio_OnCapture(const int16_t *mono, uint16_t n)
{
  uint16_t i;
  cRec++;
  if (g_ptt == 0u) { return; }        /* передаём только при нажатой PTT */
  for (i = 0u; i < n; i++)
  {
    int16_t s;
    if (g_tone != 0u) { s = toneLUT[tonePhase]; tonePhase = (uint8_t)((tonePhase + 1u) & 0x0Fu); }
    else              { s = mono[i]; }
    txAccum[txAccN++] = s;
    if (txAccN >= VOICE_SAMPLES) { tx_flush_frame(); }
  }
}

/* ================= ВОСПРОИЗВЕДЕНИЕ (аудио-ISR) ================= */
void Audio_FillPlayback(int16_t *mono, uint16_t n)
{
  uint16_t i;

  if (g_ptt != 0u)                    /* при передаче воспроизведение выключено */
  {
    for (i = 0u; i < n; i++) { mono[i] = 0; }
    jbPlaying = 0u;                   /* при возврате в приём — заново набрать prefill */
    return;
  }

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
    /* Опустошение: маскируем провал линейным затуханием последнего блока к нулю (без щелчка),
     * затем — тишина; ждём повторного набора prefill. */
    cUnder++;
    jbPlaying = 0u;
    for (i = 0u; i < n; i++)
    {
      mono[i]   = (int16_t)(((int32_t)lastOut[i] * (int32_t)(n - 1u - i)) / (int32_t)n);
      lastOut[i] = 0;
    }
  }
}

void Audio_OnError(const char *who) { (void)who; BSP_LED_On(LED5); }

/* ================= ПРИЁМ КАДРОВ (ISR через RxTap) ================= */
static void voice_sink(const uint8_t *payload, uint16_t len)
{
  int16_t  samp[VOICE_SAMPLES];
  uint16_t nsamp;
  uint16_t seq;
  uint16_t i;

  if (len < VOICE_HDR) { return; }
  cRecv++;
  BSP_LED_Toggle(LED4);               /* зелёный: активность приёма */
  if (g_ptt != 0u) { return; }        /* при передаче принятое не буферизуем (полудуплекс) */

  seq   = (uint16_t)(((uint16_t)payload[0] << 8) | (uint16_t)payload[1]);
  nsamp = (uint16_t)((len - VOICE_HDR) / 2u);
  if (nsamp > VOICE_SAMPLES) { nsamp = VOICE_SAMPLES; }

  if (rxHaveSeq != 0u)
  {
    uint16_t gap = (uint16_t)(seq - rxExpSeq);         /* потерянные кадры (арифметика u16) */
    if ((gap != 0u) && (gap <= LOSS_FILL_CAP))
    {
      cLost += gap;
      jb_push_silence((uint16_t)(gap * VOICE_SAMPLES)); /* держим timeline: тишина за потерю */
    }
    else if (gap != 0u) { cLost += gap; }              /* большой разрыв — не заполняем (ресинхр.) */
  }
  rxExpSeq  = (uint16_t)(seq + 1u);
  rxHaveSeq = 1u;

  for (i = 0u; i < nsamp; i++)
  {
    samp[i] = (int16_t)(((uint16_t)payload[VOICE_HDR + 2u * i] << 8) |
                        (uint16_t)payload[VOICE_HDR + 2u * i + 1u]);
  }
  jb_push(samp, nsamp);
}

static void voice_rx_byte(uint8_t b) { Frame_DecodeByte(&rxDec, b, voice_sink); }

/* ================= КОМАНДЫ КОНСОЛИ ================= */
static void cmd_send(int argc, char **argv)
{
  unsigned long n = 1ul, i;
  if (argc > 1) { n = strtoul(argv[1], NULL, 0); }
  if (n == 0ul) { n = 1ul; }
  if (n > 1000ul) { n = 1000ul; }
  for (i = 0ul; i < n; i++)
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
  UartPort_Stats s;
  (void)argc; (void)argv;
  UartPort_GetStats(&s);
  Console_Printf("link: baud=%lu bytes tx=%lu rx=%lu | err ore=%lu fe=%lu pe=%lu ne=%lu\r\n",
                 (unsigned long)UartPort_GetBaud(), (unsigned long)s.bytesTx, (unsigned long)s.bytesRx,
                 (unsigned long)s.errOre, (unsigned long)s.errFe, (unsigned long)s.errPe, (unsigned long)s.errNe);
}

static void cmd_reset(int argc, char **argv)
{
  (void)argc; (void)argv;
  UartPort_ResetStats();
  cRec = 0u; cSent = 0u; cTxDrop = 0u; cRecv = 0u; cLost = 0u; cPlayed = 0u; cUnder = 0u; cOver = 0u;
  /* Счётчики протокола нашего приёмного автомата (состояние FSM не трогаем). */
  rxDec.stats.framesRx = 0u; rxDec.stats.framesCrc = 0u;
  rxDec.stats.resync = 0u; rxDec.stats.bytesDropped = 0u;
  Audio_ResetOutProbe();   /* probeA: обнулить и задать базу «секунд от старта» */
  Console_Write("counters reset\r\n");
}

static void cmd_dump(int argc, char **argv)
{
  uint8_t  buf[64];
  uint16_t n, i;
  (void)argc; (void)argv;
  n = UartPort_Dump(buf, (uint16_t)sizeof(buf));
  if (n == 0u) { Console_Write("dump: (nothing received)\r\n"); return; }
  Console_Printf("dump: last %u byte(s), oldest first:\r\n", (unsigned)n);
  for (i = 0u; i < n; i++)
  {
    Console_Printf("%02X%s", (unsigned)buf[i], (((i & 0x0Fu) == 0x0Fu) || (i + 1u == n)) ? "\r\n" : " ");
  }
}

static void cmd_regs(int argc, char **argv)
{
  UartPort_Stats s;
  (void)argc; (void)argv;
  UartPort_GetStats(&s);
  Console_Printf("USART2: SR=%08lX CR1=%08lX CR3=%08lX | bytesRx=%lu\r\n",
                 (unsigned long)USART2->SR, (unsigned long)USART2->CR1, (unsigned long)USART2->CR3,
                 (unsigned long)s.bytesRx);
}

static void cmd_baud(int argc, char **argv)
{
  unsigned long b;
  if (argc < 2) { Console_Printf("baud = %lu\r\n", (unsigned long)UartPort_GetBaud()); return; }
  b = strtoul(argv[1], NULL, 0);
  if ((b < 1200ul) || (b > 3000000ul)) { Console_Write("baud out of range\r\n"); return; }
  UartPort_SetBaud((uint32_t)b);
  Preempt_AudioOutHighest();   /* реинит USART2 сбросил его приоритет — переустановить */
  Console_Printf("baud set to %lu\r\n", (unsigned long)UartPort_GetBaud());
}

/* voice — статистика речевого тракта + ошибки приёмника линии. Оценка задержки — расчётом. */
static void cmd_voice(int argc, char **argv)
{
  unsigned lat_ms = (unsigned)(VOICE_MS + (JB_PREFILL / AUDIO_BLOCK_SAMPLES) + 2u); /* tx-накопл + prefill + кодек */
  UartPort_Stats s;
  (void)argc; (void)argv;
  UartPort_GetStats(&s);
  Console_Printf("voice: rec=%lu sent=%lu txdrop=%lu recv=%lu lost=%lu played=%lu under=%lu over=%lu\r\n",
                 (unsigned long)cRec, (unsigned long)cSent, (unsigned long)cTxDrop, (unsigned long)cRecv,
                 (unsigned long)cLost, (unsigned long)cPlayed, (unsigned long)cUnder, (unsigned long)cOver);
  Console_Printf("voice: jb_fill=%u/%u samples (%u ms), ptt=%s tone=%s, est one-way latency ~%u ms\r\n",
                 (unsigned)jb_fill(), (unsigned)JB_SIZE, (unsigned)(jb_fill() / AUDIO_BLOCK_SAMPLES),
                 (g_ptt ? "on" : "off"), (g_tone ? "on" : "off"), lat_ms);
  Console_Printf("voice: rx errors ore=%lu fe=%lu pe=%lu ne=%lu\r\n",
                 (unsigned long)s.errOre, (unsigned long)s.errFe,
                 (unsigned long)s.errPe, (unsigned long)s.errNe);
  /* probeA (audio.c): срывы 1-мс дедлайна пере-взвода аудио-выхода — критерий вытеснения. */
  {
    Audio_OutProbe ap;
    Audio_GetOutProbe(&ap);
    Console_Printf("probeA out-rearm: calls=%lu over(>%luus)=%lu max=%luus first=%lus last=%lus\r\n",
                   (unsigned long)ap.calls, (unsigned long)ap.threshUs, (unsigned long)ap.over,
                   (unsigned long)ap.maxUs, (unsigned long)ap.firstSec, (unsigned long)ap.lastSec);
  }
}

/* proto — статистика протокола кадрирования на ПРИЁМЕ (наш rxDec). */
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
  if      (strcmp(argv[1], "on")  == 0) { g_pttCmd = 1u; }
  else if (strcmp(argv[1], "off") == 0) { g_pttCmd = 0u; }
  else { Console_Write("usage: ptt on|off\r\n"); return; }
  Console_Printf("ptt(cmd) = %s\r\n", g_pttCmd ? "on" : "off");
}

static void cmd_tone(int argc, char **argv)
{
  if (argc < 2) { Console_Printf("tone = %s (1 kHz)\r\n", g_tone ? "on" : "off"); return; }
  if      (strcmp(argv[1], "on")  == 0) { g_tone = 1u; }
  else if (strcmp(argv[1], "off") == 0) { g_tone = 0u; }
  else { Console_Write("usage: tone on|off\r\n"); return; }
  Console_Printf("tone = %s (1 kHz)\r\n", g_tone ? "on" : "off");
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
  { "tone",     "tone on|off (send 1 kHz instead of mic)",  cmd_tone     },
};

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  BSP_LED_Init(LED3);   /* передача (PTT) */
  BSP_LED_Init(LED4);   /* приём */
  BSP_LED_Init(LED5);   /* ошибка */
  BSP_LED_Init(LED6);   /* USB «жив» */
  BSP_LED_Off(LED3); BSP_LED_Off(LED4); BSP_LED_Off(LED5); BSP_LED_Off(LED6);

  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);   /* кнопка PA0 как PTT */

  /* Счётчик тактов ядра для probe A (диагностика пере-взвода аудио-выхода в audio.c). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  Console_Init();
  Console_Register(k_cmds, (uint16_t)(sizeof(k_cmds) / sizeof(k_cmds[0])));

  Frame_DecoderInit(&rxDec);
  UartPort_Init();                 /* запускает приём USART2 кольцевым DMA */
  UartPort_SetRxTap(voice_rx_byte);/* принятые байты — в наш разбор кадров речи */

  if (Audio_Init() != 0u)          /* микрофон + кодек (ошибку покажет LED5) */
  {
    TRACE_ERR("LAB04: audio init failed");
    Console_Write("\r\nLAB04: AUDIO INIT FAILED\r\n");
    return 1u;
  }

  /* После всех MspInit: вытеснение, аудио-выход выше приёмного USART2 (лечение провалов звука). */
  Preempt_AudioOutHighest();

  TRACE_LOG("LAB04 voice wire: %u kHz mono, frame=%u ms (%u samples), USART2 %lu 8N1",
            (unsigned)(AUDIO_FREQ_HZ / 1000u), (unsigned)VOICE_MS, (unsigned)VOICE_SAMPLES,
            (unsigned long)UartPort_GetBaud());
  Console_Write("\r\nLAB04 voice wire ready. Hold PA0 (or 'ptt on') to talk. Type 'help'.\r\n");
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

  /* PTT: кнопка PA0 с подавлением дребезга (стабильность 20 мс) ИЛИ команда ptt on. */
  raw = (BSP_PB_GetState(BUTTON_KEY) != 0u) ? 1u : 0u;
  if (raw != pttRawLast) { pttRawLast = raw; pttChangeTick = now; }
  else if (((uint32_t)(now - pttChangeTick) >= 20u) && (pttBtn != raw)) { pttBtn = raw; }
  g_ptt = (uint8_t)((pttBtn != 0u) || (g_pttCmd != 0u));

  /* Индикация: LED3 — передача; LED6 — USB «жив»; LED4 переключается в приёмном ISR. */
  if (g_ptt != 0u) { BSP_LED_On(LED3); } else { BSP_LED_Off(LED3); }

  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u) { lastBlink = now; BSP_LED_Toggle(LED6); }
  }
  else { BSP_LED_Off(LED6); }
}

#endif /* LAB_ID == 4 */
