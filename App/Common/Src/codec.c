/**
  ******************************************************************************
  * @file    App/Common/Src/codec.c
  * @author  Wagan Sarukhanov
  * @brief   Речевые кодеки (реализация). См. codec.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Чистая логика без HAL. Источники: µ-law — ITU-T G.711 (вычислительная, 16-бит вариант,
  * BIAS 0x84, CLIP 32635); ADPCM — IMA/DVI ADPCM (таблицы IMA), кодируется ПОБЛОЧНО с
  * заголовком (предсказатель int16 + индекс шага u8), поэтому блоки независимы и потеря
  * кадра не рассинхронизирует последующие.
  ******************************************************************************
  */

#include "lab.h"

#if (LAB_ID == 7) || (LAB_ID == 8)

#include "codec.h"

/* ================= µ-law (ITU-T G.711), вычислительно ================= */
#define ULAW_BIAS   0x84
#define ULAW_CLIP   32635

static uint8_t ulaw_encode(int16_t s)
{
  uint8_t  sign = (s < 0) ? 0x80u : 0x00u;
  int32_t  m    = (sign != 0u) ? -(int32_t)s : (int32_t)s;
  int32_t  mask;
  int      exp;
  int      mant;

  if (m > ULAW_CLIP) { m = ULAW_CLIP; }
  m += ULAW_BIAS;
  exp = 7; mask = 0x4000;
  while (((m & mask) == 0) && (exp > 0)) { exp--; mask >>= 1; }
  mant = (int)((m >> (exp + 3)) & 0x0F);
  return (uint8_t)(~(sign | (uint8_t)(exp << 4) | (uint8_t)mant));
}

static int16_t ulaw_decode(uint8_t u)
{
  int32_t m;
  int     sign, exp, mant;
  u = (uint8_t)~u;
  sign = u & 0x80;
  exp  = (u >> 4) & 0x07;
  mant = u & 0x0F;
  m = (((int32_t)mant << 3) + ULAW_BIAS) << exp;
  m -= ULAW_BIAS;
  return (int16_t)((sign != 0) ? -m : m);
}

/* ================= IMA ADPCM ================= */
static const int8_t  ima_index[16] =
{ -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8 };
static const int16_t ima_step[89] =
{
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,
  130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,
  1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
  5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,
  27086,29794,32767
};

static void ima_clamp_pred(int32_t *p)
{
  if (*p > 32767)  { *p = 32767; }
  if (*p < -32768) { *p = -32768; }
}

static uint16_t adpcm_encode(const int16_t *in, uint16_t n, uint8_t *out, uint16_t cap)
{
  int32_t  pred;
  int      idx = 0;
  uint16_t o, k;
  uint8_t  half = 0u, nib = 0u;

  if ((n == 0u) || (cap < 3u)) { return 0u; }
  pred = in[0];
  out[0] = (uint8_t)(pred & 0xFF);
  out[1] = (uint8_t)((pred >> 8) & 0xFF);
  out[2] = (uint8_t)idx;
  o = 3u;

  for (k = 1u; k < n; k++)
  {
    int32_t step = ima_step[idx];
    int32_t diff = (int32_t)in[k] - pred;
    int     code = 0;
    int32_t diffq;

    if (diff < 0) { code = 8; diff = -diff; }
    if (diff >= step)      { code |= 4; diff -= step; }
    step >>= 1; if (diff >= step) { code |= 2; diff -= step; }
    step >>= 1; if (diff >= step) { code |= 1; }

    /* реконструкция (кодер повторяет декодер, чтобы предсказатель совпадал) */
    step  = ima_step[idx];
    diffq = step >> 3;
    if (code & 4) { diffq += step; }
    if (code & 2) { diffq += step >> 1; }
    if (code & 1) { diffq += step >> 2; }
    if (code & 8) { pred -= diffq; } else { pred += diffq; }
    ima_clamp_pred(&pred);
    idx += ima_index[code & 0x0F];
    if (idx < 0) { idx = 0; }
    if (idx > 88) { idx = 88; }

    if (half == 0u) { nib = (uint8_t)(code & 0x0F); half = 1u; }
    else
    {
      if (o >= cap) { return 0u; }
      out[o++] = (uint8_t)(nib | (uint8_t)((code & 0x0F) << 4));
      half = 0u;
    }
  }
  if (half != 0u)
  {
    if (o >= cap) { return 0u; }
    out[o++] = nib;
  }
  return o;
}

static uint16_t adpcm_decode(const uint8_t *in, uint16_t len, int16_t *out, uint16_t cap)
{
  int32_t  pred;
  int      idx;
  uint16_t so, bi;
  uint8_t  high = 0u;

  if ((len < 3u) || (cap == 0u)) { return 0u; }
  pred = (int16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
  idx  = in[2];
  if (idx < 0) { idx = 0; } if (idx > 88) { idx = 88; }

  out[0] = (int16_t)pred;
  so = 1u; bi = 3u;
  while ((so < cap) && (bi < len))
  {
    int     code;
    int32_t step = ima_step[idx];
    int32_t diffq;

    if (high == 0u) { code = in[bi] & 0x0F; high = 1u; }
    else            { code = (in[bi] >> 4) & 0x0F; bi++; high = 0u; }

    diffq = step >> 3;
    if (code & 4) { diffq += step; }
    if (code & 2) { diffq += step >> 1; }
    if (code & 1) { diffq += step >> 2; }
    if (code & 8) { pred -= diffq; } else { pred += diffq; }
    ima_clamp_pred(&pred);
    idx += ima_index[code & 0x0F];
    if (idx < 0) { idx = 0; }
    if (idx > 88) { idx = 88; }
    out[so++] = (int16_t)pred;
  }
  return so;
}

/* ================= Единый интерфейс ================= */
uint16_t Codec_Encode(codec_id_t id, const int16_t *in, uint16_t n, uint8_t *out, uint16_t cap)
{
  uint16_t i;
  switch (id)
  {
    case CODEC_RAW:
      if ((uint32_t)n * 2u > cap) { return 0u; }
      for (i = 0u; i < n; i++)
      {
        out[2u * i]      = (uint8_t)((uint16_t)in[i] & 0xFF);
        out[2u * i + 1u] = (uint8_t)((uint16_t)in[i] >> 8);
      }
      return (uint16_t)(n * 2u);
    case CODEC_ULAW:
      if (n > cap) { return 0u; }
      for (i = 0u; i < n; i++) { out[i] = ulaw_encode(in[i]); }
      return n;
    case CODEC_ADPCM:
      return adpcm_encode(in, n, out, cap);
    default:
      return 0u;
  }
}

uint16_t Codec_Decode(codec_id_t id, const uint8_t *in, uint16_t len, int16_t *out, uint16_t cap)
{
  uint16_t i, n;
  switch (id)
  {
    case CODEC_RAW:
      n = (uint16_t)(len / 2u);
      if (n > cap) { n = cap; }
      for (i = 0u; i < n; i++)
      {
        out[i] = (int16_t)((uint16_t)in[2u * i] | ((uint16_t)in[2u * i + 1u] << 8));
      }
      return n;
    case CODEC_ULAW:
      n = (len > cap) ? cap : len;
      for (i = 0u; i < n; i++) { out[i] = ulaw_decode(in[i]); }
      return n;
    case CODEC_ADPCM:
      return adpcm_decode(in, len, out, cap);
    default:
      return 0u;
  }
}

uint16_t Codec_MaxEncoded(codec_id_t id, uint16_t n)
{
  switch (id)
  {
    case CODEC_RAW:   return (uint16_t)(n * 2u);
    case CODEC_ULAW:  return n;
    case CODEC_ADPCM: return (uint16_t)(3u + (n / 2u) + 1u);
    default:          return 0u;
  }
}

const char *Codec_Name(codec_id_t id)
{
  switch (id)
  {
    case CODEC_RAW:   return "raw";
    case CODEC_ULAW:  return "ulaw";
    case CODEC_ADPCM: return "adpcm";
    default:          return "?";
  }
}

uint16_t Codec_BitsX10(codec_id_t id)
{
  switch (id)
  {
    case CODEC_RAW:   return 160u;   /* 16 бит/отсчёт */
    case CODEC_ULAW:  return 80u;    /* 8 бит/отсчёт */
    case CODEC_ADPCM: return 40u;    /* 4 бит/отсчёт (номинал, без учёта заголовка блока) */
    default:          return 0u;
  }
}

#endif /* LAB_ID == 7 */
