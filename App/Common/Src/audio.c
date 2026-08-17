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

#endif /* LAB_ID == 0 || 4 */
