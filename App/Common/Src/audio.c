/**
  ******************************************************************************
  * @file    App/Common/Src/audio.c
  * @author  Wagan Sarukhanov
  * @brief   Общий аудиотракт (реализация). См. audio.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Тракт данных 1:1 с прежним LAB00 (эталон Audio_playback_and_record, пересчёт под 16 кГц):
  *   MP45DT02 --PDM--> I2S2 (RX) --кольцевой DMA1_Stream3--> pdmBuf[INTERNAL_BUFF_SIZE]
  *     --BSP_AUDIO_IN_PDMToPCM (децимация 64)--> стерео-слот PCM_OUT_SIZE*2 слов
  *     --> МОНО (левый канал, микрофон один) --> хук Audio_OnCapture.
  *   Воспроизведение: хук Audio_FillPlayback даёт моно-блок --> расширяем в стерео --> кодек
  *     (I2S3 TX, DMA NORMAL) через BSP_AUDIO_OUT_ChangeBuffer.
  * Печать из ISR запрещена — здесь только конвертация/копирование и вызовы слабых хуков.
  ******************************************************************************
  */

#include "lab.h"

#if (LAB_ID == 0) || (LAB_ID == 4) || (LAB_ID == 7)

#include "audio.h"
#include "stm32f411e_discovery_audio.h"   /* BSP аудио + макросы буферов */

/* Сверяем предположение о размере блока с фактическим макросом BSP. */
_Static_assert(PCM_OUT_SIZE == (int)AUDIO_BLOCK_SAMPLES, "AUDIO_BLOCK_SAMPLES must equal PCM_OUT_SIZE");

#define VOLUME_DEFAULT   70u

/* Приёмный кольцевой буфер PDM (см. расчёт размера в шапке прежнего LAB00). */
static uint16_t pdmBuf[INTERNAL_BUFF_SIZE];
/* Рабочий стерео-слот для результата PDM→PCM (16 стерео-кадров = 32 слова). */
static uint16_t pcmStereo[PCM_OUT_SIZE * 2];
/* Моно-блок с микрофона (левый канал). */
static int16_t  monoCap[AUDIO_BLOCK_SAMPLES];
/* Выходной стерео-слот для кодека (живёт во время DMA-передачи NORMAL). */
static uint16_t playStereo[PCM_OUT_SIZE * 2];

/* ===== Диагностика A: задержка пере-взвода воспроизведения (см. docs/REPORT_isr_deadline_probe.md) =====
 * Порог дедлайна = номинал 1 мс (выходной буфер PCM_OUT_SIZE*2 слов при 16 кГц) + один период
 * I2S-слова. У I2S нет глубокого FIFO (один DR + тень), слов/с = 2*Fs = 32000 → 31.25 мкс/слово,
 * поэтому пере-взвод, опоздавший больше чем на одно слово, гарантированно оставляет тракт без
 * данных = слышимый провал; меньшая задержка (короткий ISR микрофона) укладывается в одно слово.
 * Порог считается один раз в Audio_Init от SystemCoreClock — в ISR деления нет. Переполнение
 * 32-битного CYCCNT (~44.7 с при 96 МГц) для интервалов ~1 мс безопасно: беззнаковая разность
 * корректна, пока интервал < 2^32 тактов. */
#define AO_RING  64u                          /* кольцо меток превышений (степень двойки) для окна «минута» */
static volatile uint32_t aoThreshCyc = 0u;    /* порог интервала, тактов */
static volatile uint32_t aoPrevCyc   = 0u;
static volatile uint8_t  aoHavePrev  = 0u;
static volatile uint32_t aoStartTick = 0u;    /* тик HAL (мс) reset/старта — база «секунд от старта» */
static volatile uint32_t aoCalls     = 0u;
static volatile uint32_t aoOver      = 0u;
static volatile uint32_t aoMaxCyc    = 0u;
static volatile uint8_t  aoHaveFirst = 0u;
static volatile uint32_t aoFirstTick = 0u;
static volatile uint32_t aoLastTick  = 0u;
static volatile uint32_t aoRingTick[AO_RING]; /* тики (мс) последних превышений */
static volatile uint32_t aoRingHead  = 0u;

/* ================= СЛАБЫЕ ХУКИ ================= */
__attribute__((weak)) void Audio_OnCapture(const int16_t *mono, uint16_t n) { (void)mono; (void)n; }
__attribute__((weak)) void Audio_OnError(const char *who) { (void)who; }
__attribute__((weak)) void Audio_FillPlayback(int16_t *mono, uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++) { mono[i] = 0; }   /* по умолчанию — тишина */
}

/* Одна половина PDM-буфера → PCM → выдать моно-блок потребителю (контекст ISR). */
static void convert_block(uint16_t *pdmHalf)
{
  uint16_t i;
  BSP_AUDIO_IN_PDMToPCM(pdmHalf, pcmStereo);          /* стерео-слот: [L0,R0,L1,R1,...] */
  for (i = 0u; i < AUDIO_BLOCK_SAMPLES; i++)
  {
    monoCap[i] = (int16_t)pcmStereo[2u * i];          /* левый канал (микрофон один → L==R) */
  }
  Audio_OnCapture(monoCap, AUDIO_BLOCK_SAMPLES);
}

uint8_t Audio_Init(void)
{
  uint16_t i;
  for (i = 0u; i < (PCM_OUT_SIZE * 2); i++) { playStereo[i] = 0u; }   /* стартовая тишина */

  /* Порог диагностики A: 1 мс + один период I2S-слова (деление один раз, не в ISR). */
  aoThreshCyc = (SystemCoreClock / 1000u) + (SystemCoreClock / (2u * AUDIO_FREQ_HZ));
  aoStartTick = HAL_GetTick();

  /* Порядок как в LAB00: кодек (выход) первым, затем вход (оба ClockConfig ставят один PLLI2S). */
  if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, VOLUME_DEFAULT, AUDIO_FREQ_HZ) != AUDIO_OK)
  {
    Audio_OnError("BSP_AUDIO_OUT_Init");
    return AUDIO_ERROR;
  }
  if (BSP_AUDIO_IN_Init(AUDIO_FREQ_HZ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
  {
    Audio_OnError("BSP_AUDIO_IN_Init");
    return AUDIO_ERROR;
  }
  /* Завести передачу кодека тишиной (размер — в БАЙТАХ). */
  if (BSP_AUDIO_OUT_Play(playStereo, (uint32_t)(PCM_OUT_SIZE * 2 * AUDIODATA_SIZE)) != AUDIO_OK)
  {
    Audio_OnError("BSP_AUDIO_OUT_Play");
    return AUDIO_ERROR;
  }
  /* Запустить приём с микрофона кольцевым DMA на весь pdmBuf (размер — в СЛОВАХ). */
  if (BSP_AUDIO_IN_Record(pdmBuf, (uint32_t)INTERNAL_BUFF_SIZE) != AUDIO_OK)
  {
    Audio_OnError("BSP_AUDIO_IN_Record");
    return AUDIO_ERROR;
  }
  return AUDIO_OK;
}

/* ===== Колбэки BSP (перекрывают __weak из stm32f411e_discovery_audio.c) ===== */

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  convert_block(&pdmBuf[0]);                          /* первая половина */
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  convert_block(&pdmBuf[INTERNAL_BUFF_SIZE / 2]);     /* вторая половина */
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  int16_t  mono[AUDIO_BLOCK_SAMPLES];
  uint16_t i;

  /* Диагностика A (только чтение CYCCNT + арифметика, без деления): интервал между
   * пере-взводами. Превышение порога = поздний пере-взвод NORMAL-DMA = кандидат в провал. */
  {
    uint32_t cyc = DWT->CYCCNT;
    if (aoHavePrev != 0u)
    {
      uint32_t d = (uint32_t)(cyc - aoPrevCyc);       /* беззнаковая разность корректна при одном обороте */
      aoCalls++;
      if (d > aoMaxCyc) { aoMaxCyc = d; }
      if ((aoThreshCyc != 0u) && (d > aoThreshCyc))
      {
        uint32_t tick = HAL_GetTick();
        aoOver++;
        if (aoHaveFirst == 0u) { aoHaveFirst = 1u; aoFirstTick = tick; }
        aoLastTick = tick;
        aoRingTick[aoRingHead & (AO_RING - 1u)] = tick;  /* & — не деление (AO_RING = степень двойки) */
        aoRingHead++;
      }
    }
    else { aoHavePrev = 1u; }
    aoPrevCyc = cyc;
  }

  Audio_FillPlayback(mono, AUDIO_BLOCK_SAMPLES);      /* потребитель даёт следующий моно-блок */
  for (i = 0u; i < AUDIO_BLOCK_SAMPLES; i++)
  {
    playStereo[2u * i]      = (uint16_t)mono[i];      /* моно → оба канала (L и R) */
    playStereo[2u * i + 1u] = (uint16_t)mono[i];
  }
  BSP_AUDIO_OUT_ChangeBuffer(playStereo, (uint16_t)(PCM_OUT_SIZE * 2));
}

/* Имена — точно как __weak в BSP: у входа Error_Callback, у выхода Error_CallBack. */
void BSP_AUDIO_IN_Error_Callback(void)  { Audio_OnError("BSP_AUDIO_IN_Error_Callback"); }
void BSP_AUDIO_OUT_Error_CallBack(void) { Audio_OnError("BSP_AUDIO_OUT_Error_CallBack"); }

/* ===== Диагностика A: снимок и сброс (вне ISR; деления только здесь) ===== */
void Audio_GetOutProbe(Audio_OutProbe *out)
{
  uint32_t now = HAL_GetTick();
  uint32_t cycPerUs = SystemCoreClock / 1000000u;
  uint32_t i, cnt = 0u;
  if (out == 0) { return; }
  out->calls    = aoCalls;
  out->over     = aoOver;
  out->threshUs = (cycPerUs != 0u) ? (aoThreshCyc / cycPerUs) : 0u;
  out->maxUs    = (cycPerUs != 0u) ? (aoMaxCyc / cycPerUs) : 0u;
  out->firstSec = (aoHaveFirst != 0u) ? ((uint32_t)(aoFirstTick - aoStartTick) / 1000u) : 0u;
  out->lastSec  = (aoHaveFirst != 0u) ? ((uint32_t)(aoLastTick  - aoStartTick) / 1000u) : 0u;
  for (i = 0u; i < AO_RING; i++)
  {
    uint32_t t = aoRingTick[i];
    if ((t != 0u) && ((uint32_t)(now - t) < 60000u)) { cnt++; }
  }
  out->lastMinOver   = cnt;
  out->lastMinCapped = (cnt >= AO_RING) ? 1u : 0u;   /* все слоты кольца попали в окно → занижено */
}

void Audio_ResetOutProbe(void)
{
  uint32_t i;
  aoCalls = 0u; aoOver = 0u; aoMaxCyc = 0u;
  aoHaveFirst = 0u; aoFirstTick = 0u; aoLastTick = 0u;
  aoRingHead = 0u;
  for (i = 0u; i < AO_RING; i++) { aoRingTick[i] = 0u; }
  aoStartTick = HAL_GetTick();
  /* aoHavePrev/aoPrevCyc не трогаем: колбэк идёт непрерывно, следующий интервал нормальный. */
}

#endif /* LAB_ID == 0 || 4 */
