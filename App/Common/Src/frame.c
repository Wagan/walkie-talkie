/**
  ******************************************************************************
  * @file    App/Common/Src/frame.c
  * @author  Wagan Sarukhanov
  * @brief   Кадрирование SLIP + CRC-16 (реализация). См. frame.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Чистая логика без HAL. Компилируется там же, где нужен транспорт по USART2
  * (LAB02/LAB03/LAB05) — обёрнут в тот же guard, что uart_port.
  ******************************************************************************
  */

#include "lab.h"

#if (LAB_ID == 2) || (LAB_ID == 3) || (LAB_ID == 4) || (LAB_ID == 5) || (LAB_ID == 7)

#include "frame.h"

/* CRC-16/CCITT (полином 0x1021, init 0xFFFF) — тот же, что в uart_port. */
static uint16_t frame_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  uint16_t i;
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

void Frame_DecoderInit(Frame_Decoder *d)
{
  d->stats.framesRx     = 0u;
  d->stats.framesCrc    = 0u;
  d->stats.resync       = 0u;
  d->stats.bytesDropped = 0u;
  d->n          = 0u;
  d->synced     = 0u;
  d->escaped    = 0u;
  d->discarding = 0u;
}

/* Завершение кадра по маркеру END: проверить CRC, при успехе выдать payload в sink. */
static void frame_end(Frame_Decoder *d, Frame_Sink sink)
{
  if (d->n >= 2u)                          /* минимум — 2 байта CRC */
  {
    uint16_t plen = (uint16_t)(d->n - 2u);
    uint16_t rx   = (uint16_t)(((uint16_t)d->buf[plen] << 8) | (uint16_t)d->buf[plen + 1u]);
    if (frame_crc16(d->buf, plen) == rx)
    {
      d->stats.framesRx++;
      if (sink != 0) { sink(d->buf, plen); }
    }
    else
    {
      d->stats.framesCrc++;                /* граница найдена штатно, но содержимое битое */
    }
  }
  /* n < 2 (пустой/куцый кадр между двумя END) — просто игнорируем. */
  d->n = 0u;
  d->escaped = 0u;
}

/* Отбросить текущий недокадр и ждать следующий END (переполнение / битый escape). */
static void frame_abort(Frame_Decoder *d)
{
  d->stats.bytesDropped += d->n;
  d->n = 0u;
  d->escaped = 0u;
  d->discarding = 1u;
}

/* ================= КОНЕЧНЫЙ АВТОМАТ РАЗБОРА =================
 * Состояния (неявно, через флаги):
 *   [HUNT]   synced==0 или discarding==1 — ищем границу: каждый байт, кроме END, отбрасываем;
 *            END переводит в [DATA] (первый END = начальная синхронизация; END в discarding =
 *            восстановление синхронизации, resync++).
 *   [DATA]   synced==1, discarding==0, escaped==0 — набираем содержимое; ESC -> [ESC];
 *            END -> конец кадра (проверка CRC); переполнение -> [HUNT] (abort).
 *   [ESC]    escaped==1 — следующий байт де-экранируется (ESC_END->END, ESC_ESC->ESC);
 *            любой другой байт после ESC — нарушение, кадр отбрасывается -> [HUNT].
 * Так приёмник НАХОДИТ границу в потоке без пауз и ВОССТАНАВЛИВАЕТСЯ после сбоя: любой сбой
 * заканчивается ожиданием следующего END, после чего разбор продолжается с чистого листа. */
void Frame_DecodeByte(Frame_Decoder *d, uint8_t c, Frame_Sink sink)
{
  if (c == (uint8_t)FRAME_END)
  {
    if (d->discarding != 0u)
    {
      d->stats.resync++;                   /* восстановили синхронизацию на этой границе */
      d->discarding = 0u;
      d->n = 0u;
      d->escaped = 0u;
      d->synced = 1u;
      return;
    }
    if (d->synced == 0u)
    {
      d->synced = 1u;                      /* первая граница — начальная синхронизация */
      d->n = 0u;
      d->escaped = 0u;
      return;
    }
    frame_end(d, sink);                    /* конец кадра */
    return;
  }

  if ((d->synced == 0u) || (d->discarding != 0u))
  {
    d->stats.bytesDropped++;               /* мусор до синхронизации / в режиме отбрасывания */
    return;
  }

  if (d->escaped != 0u)
  {
    uint8_t b;
    d->escaped = 0u;
    if      (c == (uint8_t)FRAME_ESC_END) { b = (uint8_t)FRAME_END; }
    else if (c == (uint8_t)FRAME_ESC_ESC) { b = (uint8_t)FRAME_ESC; }
    else { d->stats.bytesDropped++; frame_abort(d); return; }  /* битый escape */
    if (d->n >= FRAME_MAX_CONTENT) { frame_abort(d); return; } /* переполнение */
    d->buf[d->n++] = b;
    return;
  }

  if (c == (uint8_t)FRAME_ESC) { d->escaped = 1u; return; }

  if (d->n >= FRAME_MAX_CONTENT) { frame_abort(d); return; }   /* переполнение */
  d->buf[d->n++] = c;
}

/* ================= УПАКОВКА ================= */
static uint16_t frame_stuff(uint8_t b, uint8_t *out, uint16_t o, uint16_t cap)
{
  if ((b == (uint8_t)FRAME_END) || (b == (uint8_t)FRAME_ESC))
  {
    if ((o + 2u) > cap) { return 0xFFFFu; }             /* сигнал переполнения */
    out[o++] = (uint8_t)FRAME_ESC;
    out[o++] = (b == (uint8_t)FRAME_END) ? (uint8_t)FRAME_ESC_END : (uint8_t)FRAME_ESC_ESC;
  }
  else
  {
    if ((o + 1u) > cap) { return 0xFFFFu; }
    out[o++] = b;
  }
  return o;
}

uint16_t Frame_Encode(const uint8_t *payload, uint16_t plen, uint8_t *out, uint16_t outCap)
{
  uint16_t crc = frame_crc16(payload, plen);
  uint16_t o = 0u;
  uint16_t i;

  if (outCap == 0u) { return 0u; }
  out[o++] = (uint8_t)FRAME_END;

  for (i = 0u; i < plen; i++)
  {
    o = frame_stuff(payload[i], out, o, outCap);
    if (o == 0xFFFFu) { return 0u; }
  }
  o = frame_stuff((uint8_t)(crc >> 8), out, o, outCap);
  if (o == 0xFFFFu) { return 0u; }
  o = frame_stuff((uint8_t)(crc & 0xFFu), out, o, outCap);
  if (o == 0xFFFFu) { return 0u; }

  if ((o + 1u) > outCap) { return 0u; }
  out[o++] = (uint8_t)FRAME_END;
  return o;
}

#endif /* LAB_ID == 2 || 3 || 5 */
