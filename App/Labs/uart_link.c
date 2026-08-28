/**
  ******************************************************************************
  * @file    App/Labs/uart_link.c
  * @author  Wagan Sarukhanov
  * @brief   LAB02 «Провод между китами»: байтовый обмен пакетами по USART2, статистика и
  *          измерение пропускной способности/задержки. Реализует Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только в конфигурации LAB02 (LAB_ID == 2) — весь файл обёрнут ниже.
  *
  * Весь транспорт по USART2 (приём кольцевым DMA+IDLE, сборка пакетов, ресинхронизация,
  * статистика, передача) вынесен в общий модуль App/Common/uart_port.c, чтобы им
  * пользовалась и LAB05 (консоль). Здесь остаётся только специфика LAB02: индикация
  * светодиодами, режимы PERIODIC/LOAD и печать статистики по SWO.
  *
  * NB: печать из ISR запрещена. В хуках UartPort_On*() — только счётчики/светодиоды;
  * вся печать — из Lab_Process (main-цикл).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 2

#include "uart_port.h"              /* общий транспорт USART2 */
#include "stm32f4xx_hal.h"          /* HAL_GetTick */
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* */
#include "trace_log.h"              /* TRACE_LOG (ASCII, только вне ISR) */

/* ================= ПАРАМЕТРЫ ОПЫТА (можно менять) ================= */
#define LAB02_TX_INTERVAL_MS   100u    /* период отправки в режиме PERIODIC */
#define LAB02_LOAD_MS          5000u   /* длительность нагрузочного измерения */

#define LAB02_MODE_PERIODIC    0
#define LAB02_MODE_LOAD        1
#define LAB02_MODE             LAB02_MODE_PERIODIC   /* <-- переключатель режима работы */

/* Состояние нагрузочного конвейера (loadActive читается в хуке OnTxDone — контекст ISR). */
static volatile uint8_t loadActive = 0u;
#if (LAB02_MODE == LAB02_MODE_LOAD)
static volatile uint8_t loadDone   = 0u;
static uint32_t loadStartMs = 0u;
#endif

/* Отправка одного пакета + индикация (оранжевый — факт отправки). */
static void lab02_send(void)
{
  if (UartPort_SendPacket() == 0u)
  {
    BSP_LED_Toggle(LED3);
  }
}

/* ================= ХУКИ ТРАНСПОРТА (контекст ISR — без печати) ================= */
void UartPort_OnRxOk(void)
{
  BSP_LED_Toggle(LED4);             /* зелёный: корректный приём */
}

void UartPort_OnError(void)
{
  BSP_LED_On(LED5);                 /* красный: ошибка/несход CRC */
}

void UartPort_OnTxDone(void)
{
  if (loadActive != 0u)
  {
    lab02_send();                   /* конвейер нагрузки: сразу шлём следующий пакет */
  }
}

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  /* Стартовый баннер: номер работы из LAB_ID + время сборки (SWO; консоли у LAB02 нет). */
  TRACE_LOG("=== LAB%02u build %s %s ===", (unsigned)(LAB_ID), __DATE__, __TIME__);

  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);

  /* Прерывания USART2 и DMA1 Stream5 включены сгенерированным кодом. Запускаем приём. */
  UartPort_Init();

  TRACE_LOG("LAB02 uart link (RX: circular DMA + IDLE): 115200 8N1, packet=%u B (payload %u), mode=%s",
            (unsigned)UartPort_PacketSize(), (unsigned)UARTPORT_PAYLOAD_SIZE,
            (LAB02_MODE == LAB02_MODE_LOAD) ? "LOAD" : "PERIODIC");

#if (LAB02_MODE == LAB02_MODE_LOAD)
  loadStartMs = HAL_GetTick();
  loadActive  = 1u;
  lab02_send();                     /* запускаем конвейер; далее его гонит OnTxDone */
#endif
  return 0u;
}

void Lab_Process(void)
{
  uint32_t now = HAL_GetTick();
  UartPort_Stats s;

#if (LAB02_MODE == LAB02_MODE_PERIODIC)
  static uint32_t lastTx   = 0u;
  static uint32_t lastStat = 0u;

  if (((uint32_t)(now - lastTx) >= LAB02_TX_INTERVAL_MS) && (UartPort_TxBusy() == 0u))
  {
    lastTx = now;
    lab02_send();
  }

  if ((uint32_t)(now - lastStat) >= 1000u)
  {
    lastStat = now;
    UartPort_GetStats(&s);
    TRACE_LOG("stat: tx=%u rx=%u lost=%u crc=%u resync=%u ore=%u fe=%u pe=%u ne=%u rtt=%s%u ms",
              (unsigned)s.tx, (unsigned)s.rxOk, (unsigned)s.lost, (unsigned)s.crcErr, (unsigned)s.resync,
              (unsigned)s.errOre, (unsigned)s.errFe, (unsigned)s.errPe, (unsigned)s.errNe,
              (s.haveRtt ? "" : "n/a "), (unsigned)s.lastRttMs);
  }
#else  /* LAB02_MODE_LOAD */
  if ((loadActive != 0u) && ((uint32_t)(now - loadStartMs) >= LAB02_LOAD_MS))
  {
    loadActive = 0u;
    loadDone   = 1u;
  }

  if (loadDone != 0u)
  {
    uint32_t elapsed = (uint32_t)(now - loadStartMs);
    uint32_t bps;
    UartPort_GetStats(&s);
    bps = (elapsed != 0u) ? (uint32_t)(((uint64_t)s.bytesRx * 1000u) / elapsed) : 0u;
    loadDone = 0u;
    TRACE_LOG("load done: elapsed=%u ms, tx_bytes=%u, rx_bytes=%u, rate=%u B/s (%u bit/s), lost=%u, crc_err=%u, resync=%u",
              (unsigned)elapsed, (unsigned)s.bytesTx, (unsigned)s.bytesRx,
              (unsigned)bps, (unsigned)(bps * 8u), (unsigned)s.lost, (unsigned)s.crcErr, (unsigned)s.resync);
    if (s.haveRtt != 0u)
    {
      TRACE_LOG("load rtt(last)=%u ms", (unsigned)s.lastRttMs);
    }
  }
#endif
}

#endif /* LAB_ID == 2 */
