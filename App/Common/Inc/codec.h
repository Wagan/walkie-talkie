/**
  ******************************************************************************
  * @file    App/Common/Inc/codec.h
  * @author  Wagan Sarukhanov
  * @brief   Речевые кодеки: сырой PCM, µ-law (G.711), IMA ADPCM. Единый интерфейс
  *          кодирования/декодирования блока отсчётов; кодек выбирается на лету.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Каждый БЛОК кодируется НЕЗАВИСИМО (кодер/декодер не хранят состояние между вызовами):
  * для ADPCM начальные предсказатель и индекс шага передаются в заголовке блока. Это
  * принципиально для радиоканала — потеря кадра не рассинхронизирует последующие (см.
  * docs/REPORT_lab07_speech_compression.md, задача B.3).
  *
  * Источники алгоритмов:
  *  - µ-law: ITU-T G.711 (эталон — g711.c Sun Microsystems, BIAS 0x84, CLIP 32635);
  *  - ADPCM: IMA/DVI ADPCM (таблицы шага и индекса IMA), поблочно как в WAV IMA ADPCM.
  ******************************************************************************
  */

#ifndef CODEC_H
#define CODEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  CODEC_RAW   = 0,   /* сырой PCM 16 бит — точка отсчёта */
  CODEC_ULAW  = 1,   /* µ-law, 8 бит/отсчёт */
  CODEC_ADPCM = 2,   /* IMA ADPCM, 4 бит/отсчёт (+ маленький заголовок блока) */
  CODEC_COUNT = 3
} codec_id_t;

/* Закодировать n отсчётов из in в out (<= cap байт). Возвращает число записанных байт (0 при
 * нехватке места/ошибке). */
uint16_t Codec_Encode(codec_id_t id, const int16_t *in, uint16_t n, uint8_t *out, uint16_t cap);

/* Раскодировать len байт из in в out (<= cap отсчётов). Возвращает число отсчётов. */
uint16_t Codec_Decode(codec_id_t id, const uint8_t *in, uint16_t len, int16_t *out, uint16_t cap);

/* Верхняя оценка размера закодированного блока из n отсчётов (для выделения буферов). */
uint16_t Codec_MaxEncoded(codec_id_t id, uint16_t n);

/* Имя кодека ("raw"/"ulaw"/"adpcm"). */
const char *Codec_Name(codec_id_t id);

/* Номинальная скорость кодека, БИТ на отсчёт × 10 (raw=160, ulaw=80, adpcm=40) — для budget. */
uint16_t Codec_BitsX10(codec_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* CODEC_H */
