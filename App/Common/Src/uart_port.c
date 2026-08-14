/**
  ******************************************************************************
  * @file    App/Common/Src/uart_port.c
  * @author  Wagan Sarukhanov
  * @brief   Общий транспорт по USART2 (реализация). См. uart_port.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Приём — по DMA в кольцевой буфер с событием простоя линии (IDLE), через
  * HAL_UARTEx_ReceiveToIdle_DMA (эталон ST UART_ReceptionToIdle_CircularDMA). Логика
  * приёма/сборки/ресинхронизации перенесена из LAB02 без изменения поведения. Печать из
  * ISR запрещена — здесь только счётчики (volatile), сырой dump-буфер и слабые хуки.
  ******************************************************************************
  */

#include "lab.h"   /* LAB_ID + его проверка (единая точка правды) */

#if (LAB_ID == 2) || (LAB_ID == 3) || (LAB_ID == 5)

#include "uart_port.h"
#include "frame.h"                  /* кадрирование SLIP (режим LAB03) */
#include "stm32f4xx_hal.h"          /* HAL UART/UARTEx + HAL_GetTick */
#include <string.h>

/* huart2 создан CubeMX в main.c (USART2, 8N1, OVER16, DMA1 Stream5 RX circular). */
extern UART_HandleTypeDef huart2;

/* ================= ФОРМАТ ПАКЕТА =================
 * Пакет фиксированного размера, поля фиксированной ширины, структура __packed (раскладка
 * байт одинакова при любой оптимизации, оба кита little-endian). Подробности семантики —
 * в исходном LAB02. */
typedef struct __attribute__((packed))
{
  uint32_t seq;      /* порядковый номер у отправителя (+1 на пакет) */
  uint32_t t_tx;     /* метка времени отправителя (HAL_GetTick, мс) */
  uint32_t t_echo;   /* эхо последней принятой t_tx соседа (для RTT); 0 — если нет */
  uint8_t  payload[UARTPORT_PAYLOAD_SIZE];
  uint16_t crc;      /* CRC-16/CCITT по всему пакету, кроме самого crc */
} uartport_pkt_t;

#define PKT_SIZE   ((uint16_t)sizeof(uartport_pkt_t))
_Static_assert(sizeof(uartport_pkt_t) == (14u + UARTPORT_PAYLOAD_SIZE), "uartport_pkt_t must be tightly packed");

/* ================= КАДРИРОВАНИЕ (LAB03) =================
 * Полезная нагрузка кадра — те же поля, что в пакете, но БЕЗ собственной CRC: целостность
 * обеспечивает CRC самого кадра (frame.c). Режим кадрирования включается UartPort_SetFraming;
 * по умолчанию 0 — старая схема пакетов фиксированной длины (для LAB02/LAB05 поведение прежнее). */
typedef struct __attribute__((packed))
{
  uint32_t seq;
  uint32_t t_tx;
  uint32_t t_echo;
  uint8_t  payload[UARTPORT_PAYLOAD_SIZE];
} uartport_fpl_t;

#define FPL_SIZE       ((uint16_t)sizeof(uartport_fpl_t))
/* Худший случай кодирования: END + каждый из (FPL_SIZE+2 CRC) байт экранирован (2 байта) + END. */
#define FRAME_ENC_MAX  ((uint16_t)(2u + (2u * (FPL_SIZE + 2u))))

static uint8_t        g_framing   = 0u;    /* 0 — пакет фикс. длины (LAB02), 1 — SLIP-кадры (LAB03) */
static Frame_Decoder  g_dec;               /* приёмный автомат протокола */
static uint8_t        g_corrupt   = 0u;    /* портить каждый K-й отправляемый байт (демонстрация) */
static uint16_t       g_corruptK  = 32u;
static uint32_t       g_txByteCtr = 0u;    /* сквозной счётчик отправленных байт (для порчи) */
static uartport_fpl_t g_ftx;               /* исходящая нагрузка кадра */
static uint8_t        g_ftxBuf[FRAME_ENC_MAX];  /* закодированный кадр (живёт во время Transmit_IT) */

/* ================= ПРИЁМНЫЙ КОЛЬЦЕВОЙ БУФЕР (DMA) =================
 * 256 байт (~8 пакетов): половина кольца (128 Б) на пределе линии 11520 Б/с набегает за
 * ~11 мс, а разбор в колбэке — десятки мкс, поэтому DMA не обгоняет разбор. Обоснование
 * подробно — в docs/REPORT_lab02_rx_dma.md. */
#define RX_RING_SIZE   256u
static uint8_t  rxRing[RX_RING_SIZE];
static uint16_t rxOldPos = 0u;

/* Сборщик одного пакета из потока (только контекст ISR, без volatile). */
static union
{
  uartport_pkt_t pkt;
  uint8_t        bytes[PKT_SIZE];
} asmBuf;
static uint16_t asmLen    = 0u;
static uint8_t  wasSynced = 0u;

/* Сырой dump-буфер: последние принятые байты ДО разбора на пакеты (для команды dump). */
#define DUMP_SIZE   256u
static uint8_t           dumpBuf[DUMP_SIZE];
static volatile uint16_t dumpHead  = 0u;   /* индекс следующей записи (кольцо) */
static volatile uint32_t dumpCount = 0u;   /* всего записано байт (для определения заполнения) */

/* Исходящий пакет. */
static uartport_pkt_t txpkt;

/* ===== Счётчики/состояние, общие с ISR (volatile) ===== */
static volatile uint8_t  txBusy      = 0u;
static volatile uint32_t cTx         = 0u;
static volatile uint32_t cRxOk       = 0u;
static volatile uint32_t cCrcErr     = 0u;
static volatile uint32_t cResync     = 0u;
static volatile uint32_t cLost       = 0u;
static volatile uint32_t cErrOre     = 0u;
static volatile uint32_t cErrFe      = 0u;
static volatile uint32_t cErrPe      = 0u;
static volatile uint32_t cErrNe      = 0u;
static volatile uint32_t bytesTx     = 0u;
static volatile uint32_t bytesRx     = 0u;
static volatile uint32_t lastRttMs   = 0u;
static volatile uint8_t  haveRtt     = 0u;
static volatile uint32_t remoteTx    = 0u;
static volatile uint8_t  haveRemote  = 0u;
static volatile uint32_t expRemoteSeq = 0u;
static volatile uint8_t  haveRemoteSeq = 0u;

/* ================= СЛАБЫЕ ХУКИ (по умолчанию пустые) ================= */
__attribute__((weak)) void UartPort_OnRxOk(void)   { }
__attribute__((weak)) void UartPort_OnError(void)  { }
__attribute__((weak)) void UartPort_OnTxDone(void) { }

/* ================= CRC-16/CCITT (полином 0x1021, init 0xFFFF) ================= */
static uint16_t crc16(const uint8_t *data, uint32_t len)
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

/* ================= УЧЁТ ПРИНЯТОЙ НАГРУЗКИ (ISR) =================
 * Общая бухгалтерия для обеих схем (пакет фикс. длины и кадр SLIP): потери по разрывам seq,
 * RTT по эху, запоминание метки соседа, счётчик корректных, индикация. */
static void account_rx(uint32_t seq, uint32_t ttx, uint32_t techo)
{
  cRxOk++;

  if (haveRemoteSeq == 0u)
  {
    haveRemoteSeq = 1u;
  }
  else if (seq > expRemoteSeq)
  {
    cLost += (seq - expRemoteSeq);
  }
  expRemoteSeq = seq + 1u;

  if (techo != 0u)
  {
    lastRttMs = (uint32_t)(HAL_GetTick() - techo);
    haveRtt   = 1u;
  }

  remoteTx   = ttx;
  haveRemote = 1u;

  UartPort_OnRxOk();
}

/* Старая схема: полный пакет собран и CRC сошлась. */
static void handle_packet(void)
{
  account_rx(asmBuf.pkt.seq, asmBuf.pkt.t_tx, asmBuf.pkt.t_echo);
}

/* Приёмник корректного кадра (вызывается из Frame_DecodeByte в контексте ISR). */
static void frame_sink(const uint8_t *payload, uint16_t len)
{
  if (len == FPL_SIZE)
  {
    const uartport_fpl_t *p = (const uartport_fpl_t *)(const void *)payload;
    account_rx(p->seq, p->t_tx, p->t_echo);
  }
  /* иной размер — не наш кадр, игнорируем (CRC уже подтвердила целостность содержимого) */
}

/* ================= СБОРКА ПАКЕТОВ ИЗ ПОТОКА (ISR) =================
 * Набрали полный пакет — проверяем CRC. Сошлась → пакет принят. Не сошлась → минимальная
 * ресинхронизация: выбрасываем самый старый байт, сдвигаем на один и ждём следующий
 * (O(1) на байт, выравнивание восстанавливается не позже, чем за длину пакета). */
static void feed_byte(uint8_t b)
{
  asmBuf.bytes[asmLen] = b;
  asmLen++;

  if (asmLen < PKT_SIZE)
  {
    return;
  }

  if (crc16(asmBuf.bytes, (uint32_t)(PKT_SIZE - 2u)) == asmBuf.pkt.crc)
  {
    handle_packet();
    asmLen    = 0u;
    wasSynced = 1u;
  }
  else
  {
    cCrcErr++;
    if (wasSynced != 0u)
    {
      cResync++;
      wasSynced = 0u;
    }
    UartPort_OnError();
    memmove(&asmBuf.bytes[0], &asmBuf.bytes[1], (size_t)(PKT_SIZE - 1u));
    asmLen = (uint16_t)(PKT_SIZE - 1u);
  }
}

/* Записать сырой байт в dump-кольцо (ISR). */
static void dump_put(uint8_t b)
{
  dumpBuf[dumpHead] = b;
  dumpHead = (uint16_t)((dumpHead + 1u) % DUMP_SIZE);
  dumpCount++;
}

/* Обработать один принятый байт: в dump + по активной схеме (кадры SLIP или пакет фикс. длины). */
static void rx_byte(uint8_t b)
{
  dump_put(b);
  if (g_framing != 0u) { Frame_DecodeByte(&g_dec, b, frame_sink); }
  else                 { feed_byte(b); }
}

/* ================= ПЕРЕДАЧА ================= */
/* Внести порчу: перевернуть (XOR 0xFF) каждый K-й отправляемый байт по сквозному счётчику.
 * Простой и наглядный способ: студент видит, как приёмник восстанавливается после сбоя. */
static void apply_corrupt(uint8_t *buf, uint16_t len)
{
  uint16_t i;
  for (i = 0u; i < len; i++)
  {
    g_txByteCtr++;
    if ((g_corrupt != 0u) && (g_corruptK != 0u) && ((g_txByteCtr % g_corruptK) == 0u))
    {
      buf[i] ^= 0xFFu;
    }
  }
}

/* Заполнить общие поля исходящей нагрузки ПРЯМЫМ присваиванием (P — lvalue структуры с полями
 * seq/t_tx/t_echo/payload). Через макрос, а не функцию с указателями: взятие адреса поля
 * packed-структуры даёт предупреждение -Waddress-of-packed-member, а прямое присваивание — нет. */
#define FILL_TX_FIELDS(P) do {                                                       \
    uint16_t _i;                                                                     \
    (P).seq    = cTx;                                                                \
    (P).t_tx   = HAL_GetTick();                                                      \
    (P).t_echo = (haveRemote != 0u) ? remoteTx : 0u;                                 \
    for (_i = 0u; _i < UARTPORT_PAYLOAD_SIZE; _i++)                                   \
    { (P).payload[_i] = (uint8_t)(0xA0u + (_i & 0x0Fu)); }                            \
  } while (0)

/* Старая схема: пакет фиксированной длины с собственной CRC. */
static uint8_t send_legacy(void)
{
  FILL_TX_FIELDS(txpkt);
  txpkt.crc = crc16((const uint8_t *)&txpkt, (uint32_t)(PKT_SIZE - 2u));

  apply_corrupt((uint8_t *)&txpkt, PKT_SIZE);
  txBusy = 1u;
  if (HAL_UART_Transmit_IT(&huart2, (const uint8_t *)&txpkt, PKT_SIZE) != HAL_OK)
  {
    txBusy = 0u;
    return 2u;
  }
  cTx++;
  bytesTx += PKT_SIZE;
  return 0u;
}

/* Новая схема: SLIP-кадр (целостность — CRC кадра). */
static uint8_t send_framed(void)
{
  uint16_t enc;
  FILL_TX_FIELDS(g_ftx);

  enc = Frame_Encode((const uint8_t *)&g_ftx, FPL_SIZE, g_ftxBuf, (uint16_t)sizeof(g_ftxBuf));
  if (enc == 0u) { return 2u; }

  apply_corrupt(g_ftxBuf, enc);
  txBusy = 1u;
  if (HAL_UART_Transmit_IT(&huart2, g_ftxBuf, enc) != HAL_OK)
  {
    txBusy = 0u;
    return 2u;
  }
  cTx++;
  bytesTx += enc;
  return 0u;
}

uint8_t UartPort_SendPacket(void)
{
  if (txBusy != 0u)
  {
    return 1u;                     /* предыдущая передача ещё идёт */
  }
  return (g_framing != 0u) ? send_framed() : send_legacy();
}

uint8_t UartPort_SendByte(uint8_t b)
{
  /* Один байт — блокирующе (при 115200 ~87 мкс), чтобы не конфликтовать с txBusy пакетной
   * передачи. Приём по DMA при этом продолжается (полный дуплекс). */
  if (HAL_UART_Transmit(&huart2, &b, 1u, 100u) != HAL_OK)
  {
    return 1u;
  }
  bytesTx += 1u;
  return 0u;
}

uint8_t  UartPort_TxBusy(void)     { return txBusy; }
uint16_t UartPort_PacketSize(void) { return PKT_SIZE; }
uint32_t UartPort_GetBaud(void)    { return huart2.Init.BaudRate; }
uint16_t UartPort_RxPos(void)      { return rxOldPos; }

/* ================= КАДРИРОВАНИЕ: управление и статистика ================= */
void UartPort_SetFraming(uint8_t on)
{
  g_framing = (on != 0u) ? 1u : 0u;
  Frame_DecoderInit(&g_dec);        /* переключение режима — с чистого приёмного автомата */
}

uint8_t UartPort_GetFraming(void) { return g_framing; }

void UartPort_SetCorrupt(uint8_t on, uint16_t everyK)
{
  g_corrupt = (on != 0u) ? 1u : 0u;
  if (everyK != 0u) { g_corruptK = everyK; }
}

uint8_t  UartPort_GetCorrupt(void)  { return g_corrupt; }
uint16_t UartPort_GetCorruptK(void) { return g_corruptK; }

void UartPort_GetProto(UartPort_ProtoStats *out)
{
  if (out == NULL) { return; }
  out->framesRx     = g_dec.stats.framesRx;
  out->framesCrc    = g_dec.stats.framesCrc;
  out->resync       = g_dec.stats.resync;
  out->bytesDropped = g_dec.stats.bytesDropped;
}

/* ================= ЗАПУСК/ПЕРЕЗАПУСК ПРИЁМА ================= */
static void start_rx(void)
{
  rxOldPos  = 0u;
  asmLen    = 0u;
  wasSynced = 0u;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxRing, RX_RING_SIZE);
}

void UartPort_Init(void)
{
  Frame_DecoderInit(&g_dec);
  g_txByteCtr = 0u;
  start_rx();
}

void UartPort_SetBaud(uint32_t baud)
{
  /* Полный корректный реинит: DeInit (MspDeInit: DMA/NVIC снимаются) -> смена скорости ->
   * Init (MspInit: DMA/NVIC/пины восстанавливаются генерируемым кодом) -> перезапуск приёма. */
  (void)HAL_UART_DeInit(&huart2);
  huart2.Init.BaudRate = baud;
  (void)HAL_UART_Init(&huart2);
  start_rx();
}

/* ================= СТАТИСТИКА / DUMP ================= */
void UartPort_GetStats(UartPort_Stats *out)
{
  if (out == NULL) { return; }
  out->tx        = cTx;
  out->rxOk      = cRxOk;
  out->crcErr    = cCrcErr;
  out->resync    = cResync;
  out->lost      = cLost;
  out->errOre    = cErrOre;
  out->errFe     = cErrFe;
  out->errPe     = cErrPe;
  out->errNe     = cErrNe;
  out->bytesTx   = bytesTx;
  out->bytesRx   = bytesRx;
  out->lastRttMs = lastRttMs;
  out->haveRtt   = haveRtt;
}

void UartPort_ResetStats(void)
{
  /* Сбрасываем только счётчики/диагностику; приём не трогаем. Порядок и краткость важны,
   * т.к. значения могут обновиться из ISR между присваиваниями — для отладочной команды
   * reset это допустимо. */
  cTx = 0u; cRxOk = 0u; cCrcErr = 0u; cResync = 0u; cLost = 0u;
  cErrOre = 0u; cErrFe = 0u; cErrPe = 0u; cErrNe = 0u;
  bytesTx = 0u; bytesRx = 0u; lastRttMs = 0u; haveRtt = 0u;
  dumpCount = 0u; dumpHead = 0u;
  /* Счётчики протокола тоже обнуляем (состояние приёмного автомата не трогаем — иначе
   * можно разорвать текущий кадр в потоке). */
  g_dec.stats.framesRx = 0u; g_dec.stats.framesCrc = 0u;
  g_dec.stats.resync = 0u; g_dec.stats.bytesDropped = 0u;
}

uint16_t UartPort_Dump(uint8_t *dst, uint16_t max)
{
  uint32_t have;
  uint16_t n;
  uint16_t i;
  uint16_t start;

  if ((dst == NULL) || (max == 0u)) { return 0u; }

  have = dumpCount;
  if (have > DUMP_SIZE) { have = DUMP_SIZE; }     /* в кольце не больше DUMP_SIZE */
  n = (have < (uint32_t)max) ? (uint16_t)have : max;

  /* Последние n байт: они заканчиваются на (dumpHead-1). Начало — dumpHead-n по модулю. */
  start = (uint16_t)(((uint32_t)dumpHead + DUMP_SIZE - n) % DUMP_SIZE);
  for (i = 0u; i < n; i++)
  {
    dst[i] = dumpBuf[(uint16_t)((start + i) % DUMP_SIZE)];
  }
  return n;
}

/* ================= HAL-КОЛБЭКИ USART2 (ISR — БЕЗ печати) ================= */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) { return; }
  txBusy = 0u;
  UartPort_OnTxDone();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t i;

  if (huart->Instance != USART2) { return; }

  if (Size != rxOldPos)
  {
    if (Size > rxOldPos)
    {
      bytesRx += (uint32_t)(Size - rxOldPos);
      for (i = rxOldPos; i < Size; i++)
      {
        rx_byte(rxRing[i]);
      }
    }
    else
    {
      bytesRx += (uint32_t)(RX_RING_SIZE - rxOldPos) + (uint32_t)Size;
      for (i = rxOldPos; i < RX_RING_SIZE; i++)
      {
        rx_byte(rxRing[i]);
      }
      for (i = 0u; i < Size; i++)
      {
        rx_byte(rxRing[i]);
      }
    }
    rxOldPos = Size;
  }

  /* IDLE — граница пакета в СТАРОЙ схеме: сбрасываем неполный хвост. В режиме кадрирования
   * границу задаёт маркер END, поэтому IDLE не используется. */
  if ((g_framing == 0u) &&
      (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE) && (asmLen != 0u))
  {
    if (wasSynced != 0u)
    {
      cResync++;
      wasSynced = 0u;
    }
    asmLen = 0u;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t ec;

  if (huart->Instance != USART2) { return; }
  ec = huart->ErrorCode;
  if ((ec & HAL_UART_ERROR_ORE) != 0u) { cErrOre++; }
  if ((ec & HAL_UART_ERROR_FE)  != 0u) { cErrFe++;  }
  if ((ec & HAL_UART_ERROR_PE)  != 0u) { cErrPe++;  }
  if ((ec & HAL_UART_ERROR_NE)  != 0u) { cErrNe++;  }

  UartPort_OnError();

  /* В DMA-режиме любая ошибка приёмника останавливает приём (проверено по коду HAL) —
   * перезапускаем (сбрасывая позицию разбора и сборщик). */
  start_rx();
}

#endif /* (LAB_ID == 2) || (LAB_ID == 5) */
