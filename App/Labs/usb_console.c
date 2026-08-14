/**
  ******************************************************************************
  * @file    App/Labs/usb_console.c
  * @author  Wagan Sarukhanov
  * @brief   LAB05 «Консоль по USB CDC»: двусторонний канал с ПК для ручного управления
  *          обменом по USART2 и наблюдения за линией. Реализует Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только в конфигурации LAB05 (LAB_ID == 5) — весь файл обёрнут ниже.
  *
  * Цель работы — дать инструмент отладки провода: вместо угадывания по светодиодам можно
  * подавать команды и видеть, что реально приходит по USART2 (команда dump — сырые байты
  * ДО разбора). Приём USART2 — общий модуль App/Common/uart_port.c (кольцевой DMA + IDLE),
  * тот же, что в LAB02. Консоль — общий модуль App/Common/console.c поверх USB CDC.
  *
  * NB: команды send/sendbyte могут коротко подождать готовности передатчика (ограниченный
  * спин с таймаутом) — это ручные отладочные команды из основного цикла, не из ISR.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 5

#include "console.h"
#include "uart_port.h"
#include "trace_log.h"
#include "stm32f4xx_hal.h"          /* HAL_GetTick */
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* */
#include <stdlib.h>                 /* strtoul */

/* Дескрипторы, созданные CubeMX в main.c — для диагностической команды regs. */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_rx;

/* ================= ОБРАБОТЧИКИ КОМАНД ================= */

/* send [n] — отправить n тестовых пакетов (по умолчанию 1). */
static void cmd_send(int argc, char **argv)
{
  unsigned long n = 1ul;
  unsigned long i;

  if (argc > 1) { n = strtoul(argv[1], NULL, 0); }
  if (n == 0ul) { n = 1ul; }
  if (n > 1000ul) { n = 1000ul; }   /* защита от опечатки/зависания цикла */

  for (i = 0ul; i < n; i++)
  {
    uint32_t t0 = HAL_GetTick();
    /* дождаться готовности передатчика (ограниченно), затем отправить */
    while ((UartPort_TxBusy() != 0u) && ((uint32_t)(HAL_GetTick() - t0) < 50u)) { }
    if (UartPort_SendPacket() != 0u) { break; }   /* не удалось поставить — прекращаем */
  }
  Console_Printf("sent %lu packet(s)\r\n", (unsigned long)i);
}

/* sendbyte <value> — отправить один произвольный байт (dec или 0x..). */
static void cmd_sendbyte(int argc, char **argv)
{
  unsigned long v;

  if (argc < 2)
  {
    Console_Write("usage: sendbyte <value>  (e.g. 0x55 or 85)\r\n");
    return;
  }
  v = strtoul(argv[1], NULL, 0) & 0xFFul;
  if (UartPort_SendByte((uint8_t)v) == 0u)
  {
    Console_Printf("sent byte 0x%02lX\r\n", (unsigned long)v);
  }
  else
  {
    Console_Write("sendbyte failed\r\n");
  }
}

/* stat — текущая статистика. */
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

/* reset — обнулить счётчики. */
static void cmd_reset(int argc, char **argv)
{
  (void)argc; (void)argv;
  UartPort_ResetStats();
  Console_Write("counters reset\r\n");
}

/* dump — последние принятые СЫРЫЕ байты (до разбора на пакеты), в hex. */
static void cmd_dump(int argc, char **argv)
{
  uint8_t  buf[64];
  uint16_t n;
  uint16_t i;
  (void)argc; (void)argv;

  n = UartPort_Dump(buf, (uint16_t)sizeof(buf));
  if (n == 0u)
  {
    Console_Write("dump: (nothing received)\r\n");
    return;
  }
  Console_Printf("dump: last %u byte(s), oldest first:\r\n", (unsigned)n);
  for (i = 0u; i < n; i++)
  {
    Console_Printf("%02X%s", (unsigned)buf[i], (((i & 0x0Fu) == 0x0Fu) || (i + 1u == n)) ? "\r\n" : " ");
  }
}

/* baud [speed] — показать/сменить скорость UART на лету. */
static void cmd_baud(int argc, char **argv)
{
  unsigned long b;

  if (argc < 2)
  {
    Console_Printf("baud = %lu\r\n", (unsigned long)UartPort_GetBaud());
    return;
  }
  b = strtoul(argv[1], NULL, 0);
  if ((b < 1200ul) || (b > 3000000ul))
  {
    Console_Write("baud out of range (1200..3000000)\r\n");
    return;
  }
  UartPort_SetBaud((uint32_t)b);
  Console_Printf("baud set to %lu, rx restarted\r\n", (unsigned long)UartPort_GetBaud());
}

/* regs — регистры и состояние приёмника USART2/DMA в hex (плата печатает их сама, т.к.
 * при остановке ядра отладчиком отваливается виртуальный COM). Имена — из CMSIS. */
static void cmd_regs(int argc, char **argv)
{
  UartPort_Stats s;
  (void)argc; (void)argv;
  UartPort_GetStats(&s);
  Console_Printf("USART2: SR=%08lX CR1=%08lX CR3=%08lX | RxState=%02lX gState=%02lX err=%08lX\r\n",
                 (unsigned long)USART2->SR, (unsigned long)USART2->CR1, (unsigned long)USART2->CR3,
                 (unsigned long)huart2.RxState, (unsigned long)huart2.gState,
                 (unsigned long)huart2.ErrorCode);
  Console_Printf("DMA1S5: CR=%08lX NDTR=%08lX | State=%02lX err=%08lX | rxPos=%u bytesRx=%lu\r\n",
                 (unsigned long)DMA1_Stream5->CR, (unsigned long)DMA1_Stream5->NDTR,
                 (unsigned long)hdma_usart2_rx.State, (unsigned long)hdma_usart2_rx.ErrorCode,
                 (unsigned)UartPort_RxPos(), (unsigned long)s.bytesRx);
}

/* pintest [pin] — проверка ФИЗИЧЕСКОГО вывода отдельно от UART. Вывод PAx переводится в
 * обычный выход push-pull (отцепляется от AF) и ~5 с переключается ~1 кГц — ловится
 * осциллографом. Если меандр на пине есть, а посылок UART нет — виноват тракт передатчика,
 * а не пин (так и выяснилось про PA9: его держит остаточная VBUS-цепь платы).
 * По умолчанию проверяется PA2 (USART2_TX). Можно указать другой вывод GPIOA: `pintest 9`.
 * По окончании PA2/PA3 возвращаются в AF7 USART2 переинициализацией UART (пины восстанавливает
 * MspInit, приём перезапускается); прочие проверенные выводы остаются в режиме выхода.
 * ВАЖНО: во время теста по USART2 ничего не передаётся (иначе конфликт за пин). */
static void cmd_pintest(int argc, char **argv)
{
  GPIO_InitTypeDef g = {0};
  unsigned long pin = 2ul;                        /* по умолчанию PA2 (USART2_TX) */
  uint16_t pinMask;
  uint32_t ticks;
  uint32_t t0;
  uint32_t k;

  if (argc > 1) { pin = strtoul(argv[1], NULL, 0); }
  if (pin > 15ul)
  {
    Console_Write("pintest: pin must be 0..15 (GPIOA)\r\n");
    return;
  }
  pinMask = (uint16_t)(1u << pin);

  Console_Printf("pintest: PA%lu -> GPIO push-pull, ~1kHz toggle for ~5s (scope PA%lu). No USART2 TX.\r\n",
                 pin, pin);
  Console_Flush();                                /* сообщение должно уйти ДО блокирующего теста */

  /* PAx как обычный выход push-pull (перекрывает конфигурацию AF). */
  g.Pin   = pinMask;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &g);

  /* Точная задержка через счётчик циклов DWT (ядро 96 МГц): полупериод 500 мкс -> ~1 кГц. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  ticks = (SystemCoreClock / 1000000u) * 500u;    /* циклов на 500 мкс */

  for (k = 0u; k < 5000u; k++)                     /* 5000 * (0.5+0.5 мс) = ~5 с */
  {
    HAL_GPIO_WritePin(GPIOA, pinMask, GPIO_PIN_SET);
    t0 = DWT->CYCCNT; while ((DWT->CYCCNT - t0) < ticks) { }
    HAL_GPIO_WritePin(GPIOA, pinMask, GPIO_PIN_RESET);
    t0 = DWT->CYCCNT; while ((DWT->CYCCNT - t0) < ticks) { }
  }

  /* Вернуть PA2/PA3 в AF7 USART2 и переинициализировать UART (пины восстанавливает MspInit,
   * приём перезапускается) — на текущей скорости, без её изменения. */
  UartPort_SetBaud(UartPort_GetBaud());

  Console_Printf("pintest: done. USART2 (PA2/PA3) restored, UART reinitialized.\r\n");
}

/* Таблица команд (help — встроенная в console.c, здесь не регистрируется). */
static const console_cmd_t k_cmds[] =
{
  { "send",     "send [n] test packets (default 1)",        cmd_send     },
  { "sendbyte", "send one raw byte (dec or 0x..)",          cmd_sendbyte },
  { "stat",     "print current statistics",                 cmd_stat     },
  { "reset",    "reset counters",                           cmd_reset    },
  { "dump",     "hex of last raw received bytes",           cmd_dump     },
  { "baud",     "show/set UART baud on the fly",            cmd_baud     },
  { "regs",     "dump USART2/DMA registers & rx state",     cmd_regs     },
  { "pintest",  "toggle PA[n] GPIO ~1kHz/5s (def PA2)",     cmd_pintest  },
};

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  /* LED4 (зел) — признак «USB сконфигурирован» (индикация для отладки USB, Задача D). */
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);

  Console_Init();
  Console_Register(k_cmds, (uint16_t)(sizeof(k_cmds) / sizeof(k_cmds[0])));

  /* Приём USART2 — общий кольцевой DMA (как в LAB02). Прерывания включены генератором. */
  UartPort_Init();

  /* Баннер: в SWO (сразу) и в USB (выведется, как только хост откроет порт). */
  TRACE_LOG("LAB05 USB console: type 'help'. USART2 %lu 8N1, packet=%u B",
            (unsigned long)UartPort_GetBaud(), (unsigned)UartPort_PacketSize());
  Console_Write("\r\nLAB05 USB console ready. Type 'help'.\r\n");
  return 0u;
}

void Lab_Process(void)
{
  static uint32_t lastBlink = 0u;
  uint32_t now = HAL_GetTick();

  /* Разбор команд + выталкивание вывода в USB. */
  Console_Process();

  /* Индикация USB: порт открыт хостом -> LED4 мигает ~2 Гц; не сконфигурирован -> выключен. */
  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u)
    {
      lastBlink = now;
      BSP_LED_Toggle(LED4);
    }
  }
  else
  {
    BSP_LED_Off(LED4);
  }
}

/* Индикация ошибок приёмника из общего транспорта (контекст ISR — только светодиод). */
void UartPort_OnError(void)
{
  BSP_LED_On(LED5);                 /* красный: ошибка/несход CRC на линии */
}

#endif /* LAB_ID == 5 */
