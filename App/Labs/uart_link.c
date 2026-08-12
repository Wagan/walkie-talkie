/**
  ******************************************************************************
  * @file    App/Labs/uart_link.c
  * @author  Wagan Sarukhanov
  * @brief   LAB02 «Провод между китами»: байтовый обмен пакетами по USART1 (115200 8N1),
  *          статистика и измерение пропускной способности/задержки. Реализует Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только в конфигурации LAB02 (LAB_ID == 2) — весь файл обёрнут ниже.
  *
  * ВЫБОР СПОСОБА ПРИЁМА (Задача C1): приём по ПРЕРЫВАНИЮ (HAL_UART_Receive_IT), не по DMA.
  *  - В текущем .ioc для USART1 DMA не сконфигурирован, а править .ioc нельзя (правило 2);
  *    поднимать DMA целиком в прикладном коде (выбор потока/канала DMA2, circular, линковка)
  *    в обход CubeMX — хрупко. Поэтому берём приём по прерыванию: он неблокирующий, процессор
  *    не ждёт, и на 115200 (и в нагрузочном тесте) данные не теряются.
  *  - Приём построен на callback'ах (RxCplt/Error) — та же архитектура, что и у DMA. В LAB04,
  *    когда по этому каналу пойдёт непрерывный аудиопоток, приём переведут на DMA circular
  *    (включив его уже в CubeMX/.ioc), не переписывая логику пакетов и ошибок.
  *
  * NB: печать из обработчиков прерываний ЗАПРЕЩЕНА (правило C3). В callback'ах — только
  * счётчики (volatile) и переключение светодиодов; вся печать — из Lab_Process (main-цикл).
  ******************************************************************************
  */

#include "lab.h"                    /* единый интерфейс Lab_Init/Lab_Process + проверка LAB_ID */

#if LAB_ID == 2

#include "stm32f4xx_hal.h"          /* HAL UART + HAL_GetTick */
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* */
#include "trace_log.h"              /* TRACE_LOG / TRACE_ERR (ASCII, только вне ISR) */
#include <string.h>

/* huart1 создан CubeMX в main.c (USART1, 115200 8N1, OVER16); берём его как внешний. */
extern UART_HandleTypeDef huart1;

/* ================= ПАРАМЕТРЫ ОПЫТА (можно менять) ================= */
#define LAB02_PAYLOAD_SIZE     16u     /* размер полезной нагрузки; меняет размер пакета */
#define LAB02_TX_INTERVAL_MS   100u    /* период отправки в режиме PERIODIC */
#define LAB02_LOAD_MS          5000u   /* длительность нагрузочного измерения */

#define LAB02_MODE_PERIODIC    0
#define LAB02_MODE_LOAD        1
#define LAB02_MODE             LAB02_MODE_PERIODIC   /* <-- переключатель режима работы */

/* ================= ФОРМАТ ПАКЕТА (Задача B) =================
 * Пакет фиксированного размера. Поля (все — фиксированной ширины, порядок от больших к
 * меньшим, структура __packed — поэтому раскладка байт одинакова при любой оптимизации и
 * без «дыр» выравнивания). Оба кита — один и тот же STM32F411 (little-endian), поэтому
 * сырые байты структуры идут по проводу как есть (порядок байт совпадает).
 *
 *   seq     [u32] — порядковый номер пакета у ОТПРАВИТЕЛЯ, +1 на каждый пакет.
 *                   По разрывам в seq на приёме считаются потери.
 *   t_tx    [u32] — метка времени отправителя (HAL_GetTick, мс) в момент отправки.
 *   t_echo  [u32] — ЭХО: последняя принятая нами t_tx соседа, отражённая обратно.
 *                   Когда наша t_tx возвращается в t_echo встречного пакета, считаем
 *                   RTT = now - t_echo (часы не синхронизированы, поэтому меряем именно
 *                   round-trip, а не одностороннюю задержку). 0 — если эха ещё нет.
 *   payload [байты фикс. размера] — наполнитель (здесь — узнаваемый паттерн).
 *   crc     [u16] — контрольная сумма ПО ВСЕМУ ПАКЕТУ, КРОМЕ самого crc.
 *
 * ВНИМАНИЕ: это ещё не протокол. Кадрирования/поиска начала кадра/ресинхронизации НЕТ —
 * это LAB03. Здесь пакеты фиксированного размера просто «ходят и считаются»: приём
 * набирает ровно LAB02_PACKET_SIZE байт на пакет. Выравнивание держится, пока не потерян
 * байт; при потере (например, ORE на завышенной скорости) пойдут ошибки CRC — это и должно
 * быть видно студенту. */
typedef struct __attribute__((packed))
{
  uint32_t seq;
  uint32_t t_tx;
  uint32_t t_echo;
  uint8_t  payload[LAB02_PAYLOAD_SIZE];
  uint16_t crc;
} lab02_pkt_t;

#define LAB02_PACKET_SIZE   ((uint16_t)sizeof(lab02_pkt_t))
/* 4+4+4 (u32) + payload + 2 (u16) — проверяем, что упаковка без выравнивающих дыр. */
_Static_assert(sizeof(lab02_pkt_t) == (14u + LAB02_PAYLOAD_SIZE), "lab02_pkt_t must be tightly packed");

/* ================= БУФЕРЫ ================= */
static lab02_pkt_t txpkt;   /* исходящий пакет */
static lab02_pkt_t rxpkt;   /* приёмный буфер (одно приёмное задание за раз) */

/* ===== Разделяемое с прерываниями — volatile (пишется в ISR, читается в Lab_Process) ===== */
static volatile uint8_t  txBusy      = 0u;   /* идёт неблокирующая передача */
static volatile uint32_t cTx         = 0u;   /* отправлено пакетов */
static volatile uint32_t cRxOk       = 0u;   /* принято корректных пакетов */
static volatile uint32_t cCrcErr     = 0u;   /* пакетов с неверной CRC */
static volatile uint32_t cLost       = 0u;   /* потеряно по разрывам seq */
static volatile uint32_t cErrOre     = 0u;   /* ошибок приёмника: переполнение */
static volatile uint32_t cErrFe      = 0u;   /* ошибок кадра */
static volatile uint32_t cErrPe      = 0u;   /* ошибок чётности */
static volatile uint32_t cErrNe      = 0u;   /* ошибок шума */
static volatile uint32_t bytesTx     = 0u;   /* всего отправлено байт (нагрузочный режим) */
static volatile uint32_t bytesRx     = 0u;   /* всего принято байт (полных приёмных заданий) */
static volatile uint32_t lastRttMs   = 0u;   /* последняя измеренная round-trip задержка, мс */
static volatile uint8_t  haveRtt     = 0u;
static volatile uint32_t remoteTx    = 0u;   /* последняя принятая t_tx соседа (для эха) */
static volatile uint8_t  haveRemote  = 0u;
static volatile uint32_t expRemoteSeq = 0u;  /* ожидаемый следующий seq соседа */
static volatile uint8_t  haveRemoteSeq = 0u;
static volatile uint8_t  loadActive  = 0u;   /* идёт нагрузочная отправка */
static volatile uint8_t  loadDone    = 0u;   /* нагрузка завершена, пора печатать итог */
static uint32_t loadStartMs = 0u;

/* ================= CRC ================= */
/* CRC-16/CCITT (полином 0x1021, init 0xFFFF). Выбор: простая табличная-независимая
 * реализация (несколько строк), но заметно надёжнее простой суммы/XOR — ловит все
 * одиночные и двойные битовые ошибки и любые пакеты ошибок длиной до 16 бит. На 115200
 * стоимость (несколько десятков байт на пакет) ничтожна. Простая аддитивная сумма
 * пропускала бы перестановки байт и часть многобайтовых искажений — для «честной» оценки
 * канала этого мало. */
static uint16_t lab02_crc16(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFu;
  uint32_t i;
  uint8_t  b;
  for (i = 0u; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (b = 0u; b < 8u; b++)
    {
      crc = (uint16_t)((crc & 0x8000u) ? ((crc << 1) ^ 0x1021u) : (crc << 1));
    }
  }
  return crc;
}

/* ================= ПЕРЕДАЧА (неблокирующая) ================= */
static void lab02_send(void)
{
  uint16_t i;

  if (txBusy != 0u)
  {
    return;                       /* предыдущая передача ещё идёт */
  }

  txpkt.seq    = cTx;
  txpkt.t_tx   = HAL_GetTick();
  txpkt.t_echo = (haveRemote != 0u) ? remoteTx : 0u;
  for (i = 0u; i < LAB02_PAYLOAD_SIZE; i++)
  {
    txpkt.payload[i] = (uint8_t)(0xA0u + (i & 0x0Fu));   /* узнаваемый наполнитель */
  }
  txpkt.crc = lab02_crc16((const uint8_t *)&txpkt, (uint32_t)(LAB02_PACKET_SIZE - 2u));

  txBusy = 1u;
  if (HAL_UART_Transmit_IT(&huart1, (const uint8_t *)&txpkt, LAB02_PACKET_SIZE) != HAL_OK)
  {
    txBusy = 0u;                  /* не удалось поставить передачу */
    return;
  }
  cTx++;
  bytesTx += LAB02_PACKET_SIZE;
  BSP_LED_Toggle(LED3);           /* оранжевый: факт отправки */
}

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  /* Светодиоды: LED3 (оранж) — отправка, LED4 (зел) — корректный приём, LED5 (кр) — ошибка. */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);

  /* Включаем прерывание USART1 (в .ioc оно не включалось — делаем в коде, .ioc не трогаем). */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Ставим первое приёмное задание на ровно один пакет. */
  (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxpkt, LAB02_PACKET_SIZE);

  TRACE_LOG("LAB02 uart link: 115200 8N1, packet=%u B (payload %u), mode=%s",
            (unsigned)LAB02_PACKET_SIZE, (unsigned)LAB02_PAYLOAD_SIZE,
            (LAB02_MODE == LAB02_MODE_LOAD) ? "LOAD" : "PERIODIC");

#if (LAB02_MODE == LAB02_MODE_LOAD)
  loadStartMs = HAL_GetTick();
  loadActive  = 1u;
  lab02_send();                   /* запускаем конвейер нагрузки; далее его гонит TxCplt */
#endif
  return 0u;
}

void Lab_Process(void)
{
  uint32_t now = HAL_GetTick();

#if (LAB02_MODE == LAB02_MODE_PERIODIC)
  static uint32_t lastTx   = 0u;
  static uint32_t lastStat = 0u;

  /* Периодическая отправка раз в интервал (неблокирующе). */
  if (((uint32_t)(now - lastTx) >= LAB02_TX_INTERVAL_MS) && (txBusy == 0u))
  {
    lastTx = now;
    lab02_send();
  }

  /* Статистика раз в секунду (кумулятивно). Печать — только здесь, не в ISR. */
  if ((uint32_t)(now - lastStat) >= 1000u)
  {
    lastStat = now;
    TRACE_LOG("stat: tx=%u rx=%u lost=%u crc=%u ore=%u fe=%u pe=%u ne=%u rtt=%s%u ms",
              (unsigned)cTx, (unsigned)cRxOk, (unsigned)cLost, (unsigned)cCrcErr,
              (unsigned)cErrOre, (unsigned)cErrFe, (unsigned)cErrPe, (unsigned)cErrNe,
              (haveRtt ? "" : "n/a "), (unsigned)lastRttMs);
  }
#else  /* LAB02_MODE_LOAD */
  /* Окончание окна измерения фиксируем здесь (TxCplt перестанет слать при loadActive==0). */
  if ((loadActive != 0u) && ((uint32_t)(now - loadStartMs) >= LAB02_LOAD_MS))
  {
    loadActive = 0u;
    loadDone   = 1u;
  }

  if (loadDone != 0u)
  {
    uint32_t elapsed = (uint32_t)(now - loadStartMs);
    uint32_t bps  = (elapsed != 0u) ? (uint32_t)(((uint64_t)bytesRx * 1000u) / elapsed) : 0u;
    loadDone = 0u;
    /* Что ограничивает скорость: на 115200 8N1 предел линии = 115200/10 = 11520 байт/с
     * (10 бит на байт со старт/стоп). Обработка приёма (~11.5k IRQ/с) и CRC на ядре 96 МГц
     * ничтожны, а печать идёт ПОСЛЕ окна измерения — поэтому число отражает КАНАЛ, а не
     * диагностику. Итоговая скорость по принятым байтам (bytes/elapsed). */
    TRACE_LOG("load done: elapsed=%u ms, tx_bytes=%u, rx_bytes=%u, rate=%u B/s (%u bit/s), lost=%u, crc_err=%u",
              (unsigned)elapsed, (unsigned)bytesTx, (unsigned)bytesRx,
              (unsigned)bps, (unsigned)(bps * 8u), (unsigned)cLost, (unsigned)cCrcErr);
    if (haveRtt != 0u)
    {
      TRACE_LOG("load rtt(last)=%u ms", (unsigned)lastRttMs);
    }
  }
#endif
}

/* ================= CALLBACK'И HAL (контекст прерывания — БЕЗ печати) ================= */

/* Обработчик прерывания USART1. В .ioc он не генерировался — определяем здесь (перекрывает
 * слабый вектор из startup). NVIC включён в Lab_Init. */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }
  txBusy = 0u;
  if (loadActive != 0u)
  {
    lab02_send();                 /* конвейер нагрузки: сразу шлём следующий пакет */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t calc;
  uint32_t seq;
  uint32_t techo;
  uint32_t ttx;

  if (huart->Instance != USART1)
  {
    return;
  }

  /* Полный пакет принят — байты доставлены (учитываем и «битые» для оценки канала). */
  bytesRx += LAB02_PACKET_SIZE;

  /* Снимаем нужные поля ДО перезапуска приёма в тот же буфер. */
  seq   = rxpkt.seq;
  ttx   = rxpkt.t_tx;
  techo = rxpkt.t_echo;
  calc  = lab02_crc16((const uint8_t *)&rxpkt, (uint32_t)(LAB02_PACKET_SIZE - 2u));

  if (calc == rxpkt.crc)
  {
    cRxOk++;
    BSP_LED_Toggle(LED4);         /* зелёный: корректный приём */

    /* Потери по разрывам нумерации. */
    if (haveRemoteSeq == 0u)
    {
      haveRemoteSeq = 1u;
    }
    else if (seq > expRemoteSeq)
    {
      cLost += (seq - expRemoteSeq);
    }
    expRemoteSeq = seq + 1u;

    /* RTT: если сосед отразил нашу метку — считаем round-trip. */
    if (techo != 0u)
    {
      lastRttMs = (uint32_t)(HAL_GetTick() - techo);
      haveRtt   = 1u;
    }

    /* Запоминаем t_tx соседа, чтобы отразить её в нашем следующем пакете. */
    remoteTx   = ttx;
    haveRemote = 1u;
  }
  else
  {
    cCrcErr++;
    BSP_LED_On(LED5);             /* красный: ошибка */
  }

  /* Ставим следующее приёмное задание. */
  (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxpkt, LAB02_PACKET_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t ec;

  if (huart->Instance != USART1)
  {
    return;
  }
  ec = huart->ErrorCode;
  if ((ec & HAL_UART_ERROR_ORE) != 0u) { cErrOre++; }   /* переполнение */
  if ((ec & HAL_UART_ERROR_FE)  != 0u) { cErrFe++;  }   /* ошибка кадра */
  if ((ec & HAL_UART_ERROR_PE)  != 0u) { cErrPe++;  }   /* ошибка чётности */
  if ((ec & HAL_UART_ERROR_NE)  != 0u) { cErrNe++;  }   /* ошибка шума */
  BSP_LED_On(LED5);                                     /* красный: ошибка */

  /* Ошибка обрывает приём по IT — перезапускаем приёмное задание. */
  (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxpkt, LAB02_PACKET_SIZE);
}

#endif /* LAB_ID == 2 */
