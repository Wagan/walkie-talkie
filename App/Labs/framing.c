/**
  ******************************************************************************
  * @file    App/Labs/framing.c
  * @author  Wagan Sarukhanov
  * @brief   LAB03 «Протокол кадрирования»: SLIP-кадры в непрерывном потоке USART2, поиск
  *          границ без пауз и восстановление после сбоя. Реализует Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только в конфигурации LAB03 (LAB_ID == 3) — весь файл обёрнут ниже.
  *
  * Смысл: старая схема (пакет фиксированной длины) держалась на паузах между пакетами
  * (событие простоя линии совпадало с границей). В непрерывном потоке (flood) она теряет
  * выравнивание. Кадрирование SLIP (маркер END + байт-стаффинг + CRC кадра) находит границы
  * без пауз и восстанавливается после порчи. Транспорт и протокол — общий модуль
  * App/Common/uart_port.c + App/Common/frame.c; консоль — App/Common/console.c (как в LAB05).
  *
  * Команды: send, sendbyte, stat, reset, dump, regs, baud (как в LAB05) + flood, corrupt,
  * proto, mode (переключение старой/новой схемы для сравнения на одном железе).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 3

#include "console.h"
#include "uart_port.h"
#include "trace_log.h"
#include "stm32f4xx_hal.h"          /* HAL_GetTick */
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* */
#include <stdlib.h>                 /* strtoul */
#include <string.h>                 /* strcmp */

/* Дескрипторы, созданные CubeMX в main.c — для диагностической команды regs. */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

/* ================= ОБЩИЕ КОМАНДЫ (как в LAB05) ================= */
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
  if (argc < 2) { Console_Write("usage: sendbyte <value>  (e.g. 0x55 or 85)\r\n"); return; }
  v = strtoul(argv[1], NULL, 0) & 0xFFul;
  if (UartPort_SendByte((uint8_t)v) == 0u) { Console_Printf("sent byte 0x%02lX\r\n", (unsigned long)v); }
  else { Console_Write("sendbyte failed\r\n"); }
}

static void cmd_stat(int argc, char **argv)
{
  UartPort_Stats s;
  (void)argc; (void)argv;
  UartPort_GetStats(&s);
  Console_Printf("baud=%lu tx=%lu rx=%lu lost=%lu crc=%lu resync=%lu\r\n",
                 (unsigned long)UartPort_GetBaud(), (unsigned long)s.tx, (unsigned long)s.rxOk,
                 (unsigned long)s.lost, (unsigned long)s.crcErr, (unsigned long)s.resync);
  Console_Printf("err: ore=%lu fe=%lu pe=%lu ne=%lu | bytes tx=%lu rx=%lu | rtt=%s%lu ms\r\n",
                 (unsigned long)s.errOre, (unsigned long)s.errFe, (unsigned long)s.errPe,
                 (unsigned long)s.errNe, (unsigned long)s.bytesTx, (unsigned long)s.bytesRx,
                 (s.haveRtt ? "" : "n/a "), (unsigned long)s.lastRttMs);
}

static void cmd_reset(int argc, char **argv)
{
  (void)argc; (void)argv;
  UartPort_ResetStats();
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
  Console_Printf("USART2: SR=%08lX CR1=%08lX CR3=%08lX | RxState=%02lX gState=%02lX err=%08lX\r\n",
                 (unsigned long)USART2->SR, (unsigned long)USART2->CR1, (unsigned long)USART2->CR3,
                 (unsigned long)huart2.RxState, (unsigned long)huart2.gState, (unsigned long)huart2.ErrorCode);
  Console_Printf("DMA1S5: CR=%08lX NDTR=%08lX | State=%02lX err=%08lX | rxPos=%u bytesRx=%lu\r\n",
                 (unsigned long)DMA1_Stream5->CR, (unsigned long)DMA1_Stream5->NDTR,
                 (unsigned long)hdma_usart2_rx.State, (unsigned long)hdma_usart2_rx.ErrorCode,
                 (unsigned)UartPort_RxPos(), (unsigned long)s.bytesRx);
}

static void cmd_baud(int argc, char **argv)
{
  unsigned long b;
  if (argc < 2) { Console_Printf("baud = %lu\r\n", (unsigned long)UartPort_GetBaud()); return; }
  b = strtoul(argv[1], NULL, 0);
  if ((b < 1200ul) || (b > 3000000ul)) { Console_Write("baud out of range (1200..3000000)\r\n"); return; }
  UartPort_SetBaud((uint32_t)b);
  Console_Printf("baud set to %lu, rx restarted\r\n", (unsigned long)UartPort_GetBaud());
}

/* ================= КОМАНДЫ LAB03 ================= */

/* mode [legacy|slip] — показать/переключить схему обмена. */
static void cmd_mode(int argc, char **argv)
{
  if (argc > 1)
  {
    if      (strcmp(argv[1], "legacy") == 0) { UartPort_SetFraming(0u); }
    else if (strcmp(argv[1], "slip")   == 0) { UartPort_SetFraming(1u); }
    else { Console_Write("usage: mode legacy|slip\r\n"); return; }
  }
  Console_Printf("mode = %s\r\n", (UartPort_GetFraming() != 0u) ? "slip (framing)" : "legacy (fixed-length)");
}

/* flood <n> — отправить n пакетов/кадров подряд без пауз. Главный опыт: старая схема здесь
 * теряет выравнивание (см. resync в stat), новая (slip) — нет (см. proto). */
static void cmd_flood(int argc, char **argv)
{
  unsigned long n, i;
  if (argc < 2) { Console_Write("usage: flood <n>\r\n"); return; }
  /* Нагрузочный опыт рассчитан на встречную передачу с обеих плат. На RS-485 (полудуплекс) это
   * физически невозможно — драйверы столкнутся. Внятно отказываем, а не роняем/вешаем линию. */
  if (UartPort_PhyAllowsContention() == 0u)
  {
    Console_Write("flood: not applicable on RS-485 (half-duplex): both sides cannot transmit at once. Use 'phy ttl'.\r\n");
    return;
  }
  n = strtoul(argv[1], NULL, 0);
  if (n == 0ul) { n = 1ul; }
  if (n > 5000ul) { n = 5000ul; }             /* защита от зависания цикла */

  Console_Printf("flood: sending %lu back-to-back in %s mode...\r\n",
                 n, (UartPort_GetFraming() != 0u) ? "slip" : "legacy");
  Console_Flush();

  for (i = 0ul; i < n; i++)
  {
    uint32_t t0 = HAL_GetTick();
    /* ждём готовности передатчика — но НЕ вставляем паузы между пакетами (шлём вплотную) */
    while ((UartPort_TxBusy() != 0u) && ((uint32_t)(HAL_GetTick() - t0) < 50u)) { }
    if (UartPort_SendPacket() != 0u) { break; }
  }
  Console_Printf("flood: sent %lu\r\n", (unsigned long)i);
}

/* corrupt on|off [k] — портить каждый k-й отправляемый байт (по умолчанию каждый 32-й).
 * Способ порчи — инверсия байта (XOR 0xFF); наглядно ломает и данные, и маркеры/CRC. */
static void cmd_corrupt(int argc, char **argv)
{
  if (argc < 2) { Console_Printf("corrupt = %s, every %u\r\n",
                                 (UartPort_GetCorrupt() != 0u) ? "on" : "off",
                                 (unsigned)UartPort_GetCorruptK()); return; }
  if (strcmp(argv[1], "on") == 0)
  {
    uint16_t k = (argc > 2) ? (uint16_t)strtoul(argv[2], NULL, 0) : 0u;
    UartPort_SetCorrupt(1u, k);
    Console_Printf("corrupt on, every %u byte(s)\r\n", (unsigned)UartPort_GetCorruptK());
  }
  else if (strcmp(argv[1], "off") == 0)
  {
    UartPort_SetCorrupt(0u, 0u);
    Console_Write("corrupt off\r\n");
  }
  else { Console_Write("usage: corrupt on|off [k]\r\n"); }
}

/* proto — статистика протокола кадрирования. */
static void cmd_proto(int argc, char **argv)
{
  UartPort_ProtoStats p;
  (void)argc; (void)argv;
  UartPort_GetProto(&p);
  Console_Printf("proto(%s): frames=%lu crc_drop=%lu resync=%lu bytes_dropped=%lu\r\n",
                 (UartPort_GetFraming() != 0u) ? "slip" : "legacy-off",
                 (unsigned long)p.framesRx, (unsigned long)p.framesCrc,
                 (unsigned long)p.resync, (unsigned long)p.bytesDropped);
}

/* phy [ttl|rs485] — физический слой линии. На RS-485 нагрузочный flood недоступен (полудуплекс). */
static void cmd_phy(int argc, char **argv)
{
  if (argc > 1)
  {
    if      (strcmp(argv[1], "ttl")   == 0) { UartPort_SetPhy(UARTPORT_PHY_TTL); }
    else if (strcmp(argv[1], "rs485") == 0) { UartPort_SetPhy(UARTPORT_PHY_RS485); }
    else { Console_Write("usage: phy ttl|rs485\r\n"); return; }
  }
  Console_Printf("phy = %s%s\r\n", UartPort_PhyName(),
                 (UartPort_PhyAllowsContention() == 0u) ? " (half-duplex: 'flood' N/A)" : "");
}

static const console_cmd_t k_cmds[] =
{
  { "send",     "send [n] test packets (default 1)",        cmd_send     },
  { "sendbyte", "send one raw byte (dec or 0x..)",          cmd_sendbyte },
  { "stat",     "print link statistics",                    cmd_stat     },
  { "reset",    "reset counters",                           cmd_reset    },
  { "dump",     "hex of last raw received bytes",           cmd_dump     },
  { "regs",     "dump USART2/DMA registers & rx state",     cmd_regs     },
  { "baud",     "show/set UART baud on the fly",            cmd_baud     },
  { "mode",     "show/set legacy|slip framing",             cmd_mode     },
  { "flood",    "send <n> back-to-back (no pauses)",        cmd_flood    },
  { "phy",      "phy ttl|rs485 (flood N/A on rs485)",       cmd_phy      },
  { "corrupt",  "corrupt on|off [k] every k-th tx byte",    cmd_corrupt  },
  { "proto",    "framing protocol statistics",              cmd_proto    },
};

/* ================= ХУКИ ТРАНСПОРТА (ISR — только светодиод) ================= */
void UartPort_OnRxOk(void) { BSP_LED_Toggle(LED4); }   /* зелёный: корректный приём */
void UartPort_OnError(void) { BSP_LED_On(LED5); }      /* красный: сбой на линии/CRC */

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  BSP_LED_Init(LED3);               /* USB «жив» (heartbeat) */
  BSP_LED_Init(LED4);               /* корректный приём */
  BSP_LED_Init(LED5);               /* ошибка/CRC */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);

  Console_Init();
  Console_Register(k_cmds, (uint16_t)(sizeof(k_cmds) / sizeof(k_cmds[0])));

  UartPort_Init();
  UartPort_SetFraming(1u);          /* по умолчанию LAB03 работает по НОВОЙ схеме (SLIP) */

  TRACE_LOG("LAB03 framing (SLIP): USART2 %lu 8N1. Console over USB CDC, type 'help'.",
            (unsigned long)UartPort_GetBaud());
  Console_Write("\r\nLAB03 framing ready (mode=slip). Type 'help'.\r\n");
  Console_Write("Experiment: 'mode legacy' + 'flood 1000' vs 'mode slip' + 'flood 1000'.\r\n");
  return 0u;
}

void Lab_Process(void)
{
  static uint32_t lastBlink = 0u;
  uint32_t now = HAL_GetTick();

  Console_Process();

  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u) { lastBlink = now; BSP_LED_Toggle(LED3); }
  }
  else { BSP_LED_Off(LED3); }
}

#endif /* LAB_ID == 3 */
