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
  * ВЫБОР СПОСОБА ПРИЁМА: приём по DMA в КОЛЬЦЕВОЙ буфер с событием простоя линии (IDLE),
  * через HAL_UARTEx_ReceiveToIdle_DMA — по эталону ST UART_ReceptionToIdle_CircularDMA.
  *  - Почему не «приём на пакет» (HAL_UART_Receive_IT на LAB02_PACKET_SIZE): такой приём
  *    фиксированной длины НЕ ИМЕЕТ точки ресинхронизации. Стоит один раз потерять/вставить
  *    байт (например, ORE), и 30-байтовая граница пакета «съезжает» навсегда: CRC перестаёт
  *    сходиться у ВСЕХ последующих пакетов (наблюдался симптом rx=0 при растущих fe/crc).
  *    Разбор в TASK_uart_rx_reference это подтвердил: причина была не в настройках UART и не
  *    в обработке ошибок, а в пакетной рассинхронизации.
  *  - Кольцевой DMA принимает НЕПРЕРЫВНО: между пакетами нет окна, где теряются байты, а
  *    событие IDLE (пауза на линии) естественно совпадает с границей пакета в режиме PERIODIC
  *    и даёт точку синхронизации. Это же — целевая архитектура для LAB04 (непрерывный звук).
  *
  * NB: печать из обработчиков прерываний ЗАПРЕЩЕНА. В callback'ах (RxEvent/Error/TxCplt) —
  * только разбор потока, счётчики (volatile) и переключение светодиодов; вся печать —
  * из Lab_Process (main-цикл).
  *
  * NB: USART1_IRQHandler и DMA2_Stream2_IRQHandler теперь генерирует CubeMX (в Core/Src/
  * stm32f4xx_it.c), т.к. в .ioc включены прерывание USART1 (для IDLE) и DMA2 Stream2.
  * Поэтому здесь СВОЕГО USART1_IRQHandler больше нет и NVIC вручную не включаем.
  ******************************************************************************
  */

#include "lab.h"                    /* единый интерфейс Lab_Init/Lab_Process + проверка LAB_ID */

#if LAB_ID == 2

#include "stm32f4xx_hal.h"          /* HAL UART/UARTEx + HAL_GetTick */
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* */
#include "trace_log.h"              /* TRACE_LOG / TRACE_ERR (ASCII, только вне ISR) */
#include <string.h>

/* huart1 создан CubeMX в main.c (USART1, 115200 8N1, OVER16, DMA2 Stream2 RX circular). */
extern UART_HandleTypeDef huart1;

/* ================= ПАРАМЕТРЫ ОПЫТА (можно менять) ================= */
#define LAB02_PAYLOAD_SIZE     16u     /* размер полезной нагрузки; меняет размер пакета */
#define LAB02_TX_INTERVAL_MS   100u    /* период отправки в режиме PERIODIC */
#define LAB02_LOAD_MS          5000u   /* длительность нагрузочного измерения */

#define LAB02_MODE_PERIODIC    0
#define LAB02_MODE_LOAD        1
#define LAB02_MODE             LAB02_MODE_PERIODIC   /* <-- переключатель режима работы */

/* ================= ФОРМАТ ПАКЕТА =================
 * Пакет фиксированного размера. Поля фиксированной ширины, структура __packed — раскладка
 * байт одинакова при любой оптимизации и без «дыр». Оба кита — один STM32F411 (little-endian),
 * сырые байты структуры идут по проводу как есть.
 *
 *   seq     [u32] — порядковый номер пакета у ОТПРАВИТЕЛЯ, +1 на каждый пакет (по разрывам — потери).
 *   t_tx    [u32] — метка времени отправителя (HAL_GetTick, мс) в момент отправки.
 *   t_echo  [u32] — ЭХО: последняя принятая нами t_tx соседа, отражённая обратно (для RTT). 0 — если нет.
 *   payload [байты] — узнаваемый наполнитель.
 *   crc     [u16] — CRC-16/CCITT ПО ВСЕМУ ПАКЕТУ, КРОМЕ самого crc.
 *
 * ВНИМАНИЕ: это ещё не протокол. Полноценного кадрирования с маркерами/экранированием НЕТ —
 * это LAB03. Здесь — минимальная ресинхронизация по CRC (см. lab02_feed_byte): после сбоя
 * приёмник сдвигается на один байт и пробует снова, пока CRC не сойдётся. */
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

/* ================= ПРИЁМНЫЙ КОЛЬЦЕВОЙ БУФЕР (DMA) =================
 * Размер выбран осознанно: он должен с запасом вмещать несколько пакетов, чтобы DMA не
 * «догнал» разбор. Пакет = 30 байт (14 + payload 16). Берём 256 байт (~8 пакетов):
 *  - на 115200 8N1 линия даёт 11520 байт/с (10 бит/байт); половина кольца (128 байт) при
 *    самом плотном (нагрузочном) потоке набегает за ~11 мс, а её разбор в callback'е — это
 *    десятки микросекунд (CRC на ядре 96 МГц), т.е. DMA физически не может обогнать разбор;
 *  - в режиме PERIODIC между пакетами пауза, callback приходит по IDLE ровно на границе
 *    пакета, кольцо почти всегда пустое — запас тем более избыточный;
 *  - степень двойки удобна и оставляет большой резерв на будущее увеличение payload.
 * Приём CIRCULAR — бесконечный: HAL_UARTEx_ReceiveToIdle_DMA запускается один раз в Lab_Init. */
#define LAB02_RX_RING_SIZE   256u
static uint8_t rxRing[LAB02_RX_RING_SIZE];
static uint16_t rxOldPos = 0u;   /* позиция последнего разобранного байта в кольце (0..RING) */

/* Сборщик пакета из потока: линейный буфер на один пакет + текущая длина. Трогается только
 * в контексте прерывания (RxEvent/Error), поэтому без volatile. Union даёт удобный доступ и
 * побайтно (bytes), и полями пакета (pkt). */
static union
{
  lab02_pkt_t pkt;
  uint8_t     bytes[LAB02_PACKET_SIZE];
} asmBuf;
static uint16_t asmLen   = 0u;   /* сколько байт уже накоплено в asmBuf */
static uint8_t  wasSynced = 0u;  /* был ли недавно корректный пакет (для счётчика эпизодов ресинхр.) */

/* ================= ПЕРЕДАЧА ================= */
static lab02_pkt_t txpkt;   /* исходящий пакет */

/* ===== Разделяемое с прерываниями — volatile (пишется в ISR, читается в Lab_Process) ===== */
static volatile uint8_t  txBusy      = 0u;   /* идёт неблокирующая передача */
static volatile uint32_t cTx         = 0u;   /* отправлено пакетов */
static volatile uint32_t cRxOk       = 0u;   /* принято корректных пакетов */
static volatile uint32_t cCrcErr     = 0u;   /* неудачных проверок CRC при сборке */
static volatile uint32_t cResync     = 0u;   /* эпизодов ресинхронизации (сколько раз теряли синхронизацию) */
static volatile uint32_t cLost       = 0u;   /* потеряно по разрывам seq */
static volatile uint32_t cErrOre     = 0u;   /* ошибок приёмника: переполнение */
static volatile uint32_t cErrFe      = 0u;   /* ошибок кадра */
static volatile uint32_t cErrPe      = 0u;   /* ошибок чётности */
static volatile uint32_t cErrNe      = 0u;   /* ошибок шума */
static volatile uint32_t bytesTx     = 0u;   /* всего отправлено байт (нагрузочный режим) */
static volatile uint32_t bytesRx     = 0u;   /* всего принято байт из потока (нагрузочный режим) */
static volatile uint32_t lastRttMs   = 0u;   /* последняя измеренная round-trip задержка, мс */
static volatile uint8_t  haveRtt     = 0u;
static volatile uint32_t remoteTx    = 0u;   /* последняя принятая t_tx соседа (для эха) */
static volatile uint8_t  haveRemote  = 0u;
static volatile uint32_t expRemoteSeq = 0u;  /* ожидаемый следующий seq соседа */
static volatile uint8_t  haveRemoteSeq = 0u;
static volatile uint8_t  loadActive  = 0u;   /* идёт нагрузочная отправка (читается в TxCplt в обоих режимах) */
#if (LAB02_MODE == LAB02_MODE_LOAD)
static volatile uint8_t  loadDone    = 0u;   /* нагрузка завершена, пора печатать итог */
static uint32_t loadStartMs = 0u;            /* начало окна измерения (только LOAD) */
#endif

/* ================= CRC ================= */
/* CRC-16/CCITT (полином 0x1021, init 0xFFFF). Ловит все одиночные/двойные и пакеты ошибок
 * до 16 бит; на 115200 стоимость ничтожна. Простая сумма/XOR пропускала бы перестановки. */
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

/* ================= РАЗБОР ПРИНЯТОГО ПАКЕТА (контекст прерывания) ================= */
static void lab02_handle_packet(void)
{
  uint32_t seq   = asmBuf.pkt.seq;
  uint32_t ttx   = asmBuf.pkt.t_tx;
  uint32_t techo = asmBuf.pkt.t_echo;

  cRxOk++;
  BSP_LED_Toggle(LED4);           /* зелёный: корректный приём */

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

  /* RTT: если сосед отразил нашу метку — считаем round-trip (часы не синхронизированы). */
  if (techo != 0u)
  {
    lastRttMs = (uint32_t)(HAL_GetTick() - techo);
    haveRtt   = 1u;
  }

  /* Запоминаем t_tx соседа, чтобы отразить её в нашем следующем пакете. */
  remoteTx   = ttx;
  haveRemote = 1u;
}

/* ================= СБОРКА ПАКЕТОВ ИЗ ПОТОКА (контекст прерывания) =================
 * Накапливаем байты в asmBuf. Как только набрали полный пакет — проверяем CRC:
 *  - CRC сошлась  -> пакет принят, начинаем следующий с нуля;
 *  - CRC не сошлась -> МИНИМАЛЬНАЯ РЕСИНХРОНИЗАЦИЯ: выбрасываем самый старый байт, сдвигаем
 *    буфер на один байт, длина = PACKET_SIZE-1, и ждём следующий байт. Так за один входящий
 *    байт делается ровно один сдвиг (работа O(1) на байт, число попыток естественно
 *    ограничено: приёмник дошагает до истинной границы не позже, чем за PACKET_SIZE байт).
 * Полное кадрирование с маркерами — это LAB03; здесь достаточно ресинхронизации по CRC. */
static void lab02_feed_byte(uint8_t b)
{
  asmBuf.bytes[asmLen] = b;
  asmLen++;

  if (asmLen < LAB02_PACKET_SIZE)
  {
    return;                        /* пакет ещё не набран */
  }

  /* Набран полный пакет — проверяем CRC (по всему пакету, кроме поля crc). */
  if (lab02_crc16(asmBuf.bytes, (uint32_t)(LAB02_PACKET_SIZE - 2u)) == asmBuf.pkt.crc)
  {
    lab02_handle_packet();
    asmLen    = 0u;                /* начинаем следующий пакет с чистого листа */
    wasSynced = 1u;
  }
  else
  {
    cCrcErr++;
    if (wasSynced != 0u)
    {
      cResync++;                   /* считаем эпизод: были синхронизированы и потеряли синхронизацию */
      wasSynced = 0u;
    }
    BSP_LED_On(LED5);              /* красный: сбой приёма/рассинхронизация */
    /* сдвиг на один байт: [1..N-1] -> [0..N-2] */
    memmove(&asmBuf.bytes[0], &asmBuf.bytes[1], (size_t)(LAB02_PACKET_SIZE - 1u));
    asmLen = (uint16_t)(LAB02_PACKET_SIZE - 1u);
  }
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

/* ================= ЗАПУСК ПРИЁМА ================= */
static void lab02_start_rx(void)
{
  /* Сбрасываем разбор: после (пере)запуска DMA пишет с начала кольца (NDTR = полный размер). */
  rxOldPos  = 0u;
  asmLen    = 0u;
  wasSynced = 0u;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxRing, LAB02_RX_RING_SIZE);
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

  /* Прерывания USART1 и DMA2 Stream2 включены сгенерированным кодом (CubeMX/.ioc);
   * здесь NVIC руками НЕ трогаем. Запускаем бесконечный кольцевой приём по IDLE. */
  lab02_start_rx();

  TRACE_LOG("LAB02 uart link (RX: circular DMA + IDLE): 115200 8N1, packet=%u B (payload %u), mode=%s",
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
    TRACE_LOG("stat: tx=%u rx=%u lost=%u crc=%u resync=%u ore=%u fe=%u pe=%u ne=%u rtt=%s%u ms",
              (unsigned)cTx, (unsigned)cRxOk, (unsigned)cLost, (unsigned)cCrcErr, (unsigned)cResync,
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
    /* Скорость по фактически принятым байтам потока (bytesRx растёт в RxEventCallback на
     * каждый доставленный DMA байт — независимо от кадрирования). На 115200 8N1 предел линии
     * = 11520 байт/с; DMA-приём и CRC на 96 МГц ничтожны, печать идёт ПОСЛЕ окна — число
     * отражает КАНАЛ, а не диагностику. */
    uint32_t bps  = (elapsed != 0u) ? (uint32_t)(((uint64_t)bytesRx * 1000u) / elapsed) : 0u;
    loadDone = 0u;
    TRACE_LOG("load done: elapsed=%u ms, tx_bytes=%u, rx_bytes=%u, rate=%u B/s (%u bit/s), lost=%u, crc_err=%u, resync=%u",
              (unsigned)elapsed, (unsigned)bytesTx, (unsigned)bytesRx,
              (unsigned)bps, (unsigned)(bps * 8u), (unsigned)cLost, (unsigned)cCrcErr, (unsigned)cResync);
    if (haveRtt != 0u)
    {
      TRACE_LOG("load rtt(last)=%u ms", (unsigned)lastRttMs);
    }
  }
#endif
}

/* ================= CALLBACK'И HAL (контекст прерывания — БЕЗ печати) ================= */

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

/* Событие приёма: вызывается на HALF-transfer, TRANSFER-COMPLETE и IDLE. Size — АБСОЛЮТНАЯ
 * позиция в кольце (0..RING), до которой данные уже лежат. Обрабатываем все три события
 * одинаково: скармливаем сборщику новые байты [rxOldPos..Size) с учётом заворота кольца
 * (архитектура — как в эталоне UART_ReceptionToIdle_CircularDMA). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  if (huart->Instance != USART1)
  {
    return;
  }

  if (Size != rxOldPos)
  {
    if (Size > rxOldPos)
    {
      /* обычный случай: индекс вырос, новый участок непрерывен */
      bytesRx += (uint32_t)(Size - rxOldPos);
      for (i = rxOldPos; i < Size; i++)
      {
        lab02_feed_byte(rxRing[i]);
      }
    }
    else
    {
      /* достигнут конец кольца: сначала «хвост» [rxOldPos..RING), потом «голова» [0..Size) */
      bytesRx += (uint32_t)(LAB02_RX_RING_SIZE - rxOldPos) + (uint32_t)Size;
      for (i = rxOldPos; i < LAB02_RX_RING_SIZE; i++)
      {
        lab02_feed_byte(rxRing[i]);
      }
      for (i = 0u; i < Size; i++)
      {
        lab02_feed_byte(rxRing[i]);
      }
    }
    rxOldPos = Size;
  }

  /* IDLE — граница пакета: линия замолчала. Если в сборщике остался НЕполный пакет (потеряли
   * байт внутри кадра), это самая надёжная точка ресинхронизации — сбрасываем «хвост», чтобы
   * следующий пакет начать с нуля. В нагрузочном режиме пауз нет, IDLE не приходит — там
   * работает только сдвиговая ресинхронизация по CRC в lab02_feed_byte. */
  if ((HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) && (asmLen != 0u))
  {
    if (wasSynced != 0u)
    {
      cResync++;
      wasSynced = 0u;
    }
    asmLen = 0u;
  }
}

/* Ошибка приёмника. В DMA-режиме (проверено по коду HAL: HAL_UART_IRQHandler при DMAR любую
 * ошибку — FE/NE/PE/ORE — считает блокирующей и делает UART_EndRxTransfer + abort DMA, а
 * UART_DMAError на ошибке контроллера — то же самое; см. stm32f4xx_hal_uart.c) приём
 * ОСТАНАВЛИВАЕТСЯ. Поэтому здесь его обязательно перезапускаем. */
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

  /* Перезапуск кольцевого приёма (сбрасывает и позицию разбора, и сборщик пакета). */
  lab02_start_rx();
}

#endif /* LAB_ID == 2 */
