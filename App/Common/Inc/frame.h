/**
  ******************************************************************************
  * @file    App/Common/Inc/frame.h
  * @author  Wagan Sarukhanov
  * @brief   Кадрирование байтового потока: байт-стаффинг с маркером кадра (SLIP, RFC 1055)
  *          + контроль целостности CRC-16/CCITT. Находит границы кадра без пауз на линии.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * ПОЧЕМУ SLIP (а не «синхрослово + длина») — см. docs/REPORT_lab03_framing.md. Кратко:
  * маркер END однозначно и мгновенно задаёт границу кадра, а приёмник самосинхронизируется —
  * после любого сбоя достаточно дождаться следующего END. Length-based доверяет полю длины,
  * и при его повреждении приёмник «разъезжается»; для непрерывного речевого потока LAB04 это
  * недопустимо.
  *
  * ФОРМАТ КАДРА в линии:
  *   END  <stuffed(payload)>  <stuffed(crc16(payload), big-endian)>  END
  * где stuffing (экранирование) внутри содержимого:
  *   байт END (0xC0) -> ESC(0xDB), ESC_END(0xDC)
  *   байт ESC (0xDB) -> ESC(0xDB), ESC_ESC(0xDD)
  *   прочие байты — как есть.
  * CRC-16/CCITT (полином 0x1021, init 0xFFFF) считается по НЕэкранированному payload и
  * добавляется в содержимое перед стаффингом (тоже экранируется).
  ******************************************************************************
  */

#ifndef FRAME_H
#define FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Спецсимволы SLIP (RFC 1055). */
#define FRAME_END       0xC0u
#define FRAME_ESC       0xDBu
#define FRAME_ESC_END   0xDCu
#define FRAME_ESC_ESC   0xDDu

/* Максимум НЕэкранированного содержимого (payload + 2 байта CRC) между маркерами END.
 * Ограничение защищает от бесконечного накопления при потерянном маркере/порче. 256 байт
 * вмещают речевой кадр LAB04 (заголовок + до ~120 отсчётов PCM); тестовые кадры LAB03 (30 Б)
 * заведомо укладываются. */
#define FRAME_MAX_CONTENT   256u

/* Счётчики диагностики протокола (только чтение снаружи). */
typedef struct
{
  uint32_t framesRx;      /* принято корректных кадров (CRC сошлась) */
  uint32_t framesCrc;     /* кадров отброшено по несходу CRC */
  uint32_t resync;        /* восстановлений синхронизации (после переполнения/битого escape) */
  uint32_t bytesDropped;  /* байт отброшено при поиске границы (до синхр. / при переполнении) */
} Frame_Stats;

/* Состояние конечного автомата разбора (одно на приёмник). Поля — приватные, снаружи не
 * трогать; для чтения счётчиков есть поле stats. */
typedef struct
{
  Frame_Stats stats;
  uint8_t  buf[FRAME_MAX_CONTENT];  /* накопитель НЕэкранированного содержимого */
  uint16_t n;                       /* сколько байт в buf */
  uint8_t  synced;                  /* увидели ли хотя бы одну границу END */
  uint8_t  escaped;                 /* предыдущий байт был ESC */
  uint8_t  discarding;              /* кадр испорчен — ждём следующий END, всё отбрасываем */
} Frame_Decoder;

/* Обработчик готового корректного кадра: payload — НЕэкранированные данные без CRC, len — их длина.
 * Вызывается из Frame_DecodeByte (у нас — в контексте прерывания приёма). */
typedef void (*Frame_Sink)(const uint8_t *payload, uint16_t len);

/* Сбросить приёмник (счётчики и состояние). */
void Frame_DecoderInit(Frame_Decoder *d);

/* Скормить один принятый байт. На каждый корректный кадр вызывает sink(payload, len). */
void Frame_DecodeByte(Frame_Decoder *d, uint8_t c, Frame_Sink sink);

/* Упаковать payload в готовый к отправке кадр (END..stuffed(payload+crc)..END).
 * Возвращает длину кадра в out или 0, если не поместилось в outCap. */
uint16_t Frame_Encode(const uint8_t *payload, uint16_t plen, uint8_t *out, uint16_t outCap);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_H */
