/**
  ******************************************************************************
  * @file    App/Common/Inc/uart_port.h
  * @author  Wagan Sarukhanov
  * @brief   Общий транспорт по USART2 для нескольких работ: приём кольцевым DMA с
  *          событием простоя (IDLE), сборка пакетов с ресинхронизацией, статистика,
  *          сырой dump-буфер, передача пакетов/байт, смена скорости на лету.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Вынесено из LAB02 (App/Labs/uart_link.c), чтобы одним приёмом пользовались и LAB02,
  * и LAB05 (консоль). Компилируется только там, где нужен провод по USART2 — файл .c
  * обёрнут в `#if (LAB_ID == 2) || (LAB_ID == 5)`.
  *
  * Разделение обязанностей: этот модуль владеет ВСЕЙ работой с huart2 (RX/TX/ошибки/
  * статистика/CRC/ресинхронизация) и определяет HAL-колбэки USART2. Лаборатория-потребитель
  * добавляет только своё: индикацию, режимы, печать — через слабые хуки UartPort_On*()
  * ниже (по умолчанию пустые), не трогая логику приёма.
  ******************************************************************************
  */

#ifndef UART_PORT_H
#define UART_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Размер полезной нагрузки тестового пакета (как в LAB02). */
#define UARTPORT_PAYLOAD_SIZE   16u

/* Снимок счётчиков для печати статистики (копируется атомарно-достаточно в GetStats). */
typedef struct
{
  uint32_t tx;         /* отправлено пакетов */
  uint32_t rxOk;       /* принято корректных пакетов */
  uint32_t crcErr;     /* неудачных проверок CRC при сборке */
  uint32_t resync;     /* эпизодов ресинхронизации */
  uint32_t lost;       /* потеряно по разрывам seq */
  uint32_t errOre;     /* ошибок приёмника: переполнение */
  uint32_t errFe;      /* ошибок кадра */
  uint32_t errPe;      /* ошибок чётности */
  uint32_t errNe;      /* ошибок шума */
  uint32_t bytesTx;    /* всего отправлено байт */
  uint32_t bytesRx;    /* всего принято байт из потока */
  uint32_t lastRttMs;  /* последняя round-trip задержка, мс */
  uint8_t  haveRtt;    /* было ли измерение RTT */
} UartPort_Stats;

/* --- Управление --- */
void     UartPort_Init(void);                 /* запустить бесконечный кольцевой приём (DMA+IDLE) */
uint16_t UartPort_PacketSize(void);           /* размер пакета в байтах */
uint8_t  UartPort_TxBusy(void);               /* идёт неблокирующая передача пакета */
uint8_t  UartPort_SendPacket(void);           /* собрать и отправить один тестовый пакет; 0 — поставлено */
uint8_t  UartPort_SendByte(uint8_t b);        /* отправить один произвольный байт (блокирующе, 1 байт); 0 — ок */
uint8_t  UartPort_SendRaw(const uint8_t *buf, uint16_t len); /* отправить готовый кадр (LAB04); 0 ок/1 занято/2 ошибка */
void     UartPort_SetRxTap(void (*cb)(uint8_t)); /* перехват принятых байт (LAB04 — свой разбор кадров) */
void     UartPort_SetBaud(uint32_t baud);     /* переинициализировать USART2 на новую скорость и перезапустить приём */
uint32_t UartPort_GetBaud(void);              /* текущая скорость */
uint16_t UartPort_RxPos(void);                /* позиция разбора кольца (0..RING) для диагностики */

/* --- Кадрирование (LAB03): переключение схемы и порча для демонстрации --- */
void     UartPort_SetFraming(uint8_t on);        /* 0 — пакет фикс. длины (LAB02), 1 — SLIP-кадры */
uint8_t  UartPort_GetFraming(void);
void     UartPort_SetCorrupt(uint8_t on, uint16_t everyK);  /* портить каждый K-й отправляемый байт */
uint8_t  UartPort_GetCorrupt(void);
uint16_t UartPort_GetCorruptK(void);

/* Счётчики протокола кадрирования (копия). */
typedef struct
{
  uint32_t framesRx;      /* принято корректных кадров */
  uint32_t framesCrc;     /* кадров отброшено по CRC */
  uint32_t resync;        /* восстановлений синхронизации */
  uint32_t bytesDropped;  /* байт отброшено при поиске границы */
} UartPort_ProtoStats;
void     UartPort_GetProto(UartPort_ProtoStats *out);

/* --- Статистика и диагностика --- */
void     UartPort_GetStats(UartPort_Stats *out);
void     UartPort_ResetStats(void);
/* Скопировать последние принятые СЫРЫЕ байты (до разбора на пакеты), самый новый — последним.
 * Возвращает число скопированных байт (<= max и <= фактически накопленного). */
uint16_t UartPort_Dump(uint8_t *dst, uint16_t max);

/* --- Диагностика B: длительность приёмного ISR (HAL_UARTEx_RxEventCallback), где идёт весь
 * разбор кадра. Замер DWT от входа до выхода; нужно знать запас относительно 1-мс дедлайна
 * пере-взвода аудио-выхода. См. docs/REPORT_isr_deadline_probe.md. Счётчики накапливаются. */
#define UARTPORT_RXISR_LONG_US   250u   /* порог «длинного» ISR (мкс), ~1/4 от 1-мс дедлайна */
void UartPort_GetRxIsrProbe(uint32_t *calls, uint32_t *maxUs, uint32_t *avgUs, uint32_t *longCnt);
void UartPort_ResetRxIsrProbe(void);

/* --- Слабые хуки для лаборатории (по умолчанию пустые; переопределять при необходимости) --- */
void UartPort_OnRxOk(void);    /* вызывается из ISR при корректно принятом пакете */
void UartPort_OnError(void);   /* вызывается из ISR при ошибке приёмника/несходе CRC */
void UartPort_OnTxDone(void);  /* вызывается из ISR по завершении передачи пакета */

#ifdef __cplusplus
}
#endif

#endif /* UART_PORT_H */
