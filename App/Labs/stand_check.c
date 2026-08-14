/**
  ******************************************************************************
  * @file    App/Labs/stand_check.c
  * @author  Wagan Sarukhanov
  * @brief   LAB00 «Проверка стенда»: микрофон MP45DT02 → кодек CS43L22, Fs = 16 кГц,
  *          + heartbeat и статус по SWO. Реализует единый интерфейс Lab_Init/Lab_Process.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Работа компилируется только в конфигурации LAB00 (LAB_ID == 0) — весь файл обёрнут ниже.
  *
  * Аудиотракт вынесен в общий модуль App/Common/audio.c (чтобы им пользовалась и LAB04).
  * Здесь остаётся только специфика LAB00: «петля» микрофон→кодек (двойной буфер), индикация
  * светодиодами, статус и heartbeat по SWO. Поведение (звук, LED, вывод) прежнее.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 0

#include "audio.h"                        /* общий аудиотракт: Audio_Init + хуки */
#include "stm32f411e_discovery_audio.h"   /* макросы для информационной строки (INTERNAL_BUFF_SIZE, ...) */
#include "stm32f411e_discovery.h"         /* светодиоды BSP_LED_* */
#include "trace_log.h"                    /* TRACE_LOG / TRACE_ERR (печать только вне ISR) */
#include "trace_swo.h"                    /* Trace_SWO_PutString для heartbeat */
#include <string.h>

/* Двойной моно-буфер для петли: микрофонный хук пишет один слот, кодечный читает другой. */
static int16_t  loopSlot[2][AUDIO_BLOCK_SAMPLES];
static volatile uint8_t   loopFill  = 0u;
static int16_t * volatile pLoopReady = NULL;

/* Диагностика (меняется в ISR-хуках, читается в основном цикле). */
static volatile uint8_t  loopbackError = 0u;
static volatile uint32_t blockCount    = 0u;
static const char * volatile lbErrWho  = NULL;

static uint32_t lbStatusTick  = 0u;
static uint32_t lbBlocksShown = 0u;
static uint8_t  lbErrReported = 0u;

static uint32_t swoHbTick = 0u;
static uint32_t swoHbSeq  = 0u;

/* ===== Хуки общего аудиотракта (контекст прерываний — без печати) ===== */
void Audio_OnCapture(const int16_t *mono, uint16_t n)
{
  uint16_t i;
  for (i = 0u; i < n; i++) { loopSlot[loopFill][i] = mono[i]; }
  pLoopReady = loopSlot[loopFill];      /* публикуем заполненный слот */
  loopFill  ^= 1u;
  blockCount++;
  BSP_LED_Toggle(LED6);                 /* синий: индикация обработки блоков */
}

void Audio_FillPlayback(int16_t *mono, uint16_t n)
{
  int16_t *src = pLoopReady;            /* снимок указателя (атомарно на M4) */
  uint16_t i;
  if (src != NULL) { for (i = 0u; i < n; i++) { mono[i] = src[i]; } }
  else             { for (i = 0u; i < n; i++) { mono[i] = 0; } }
}

void Audio_OnError(const char *who)
{
  lbErrWho = who;
  loopbackError = 1u;
  BSP_LED_On(LED5);
}

uint8_t Lab_Init(void)
{
  /* Светодиоды: LED4 зелёный — init OK, LED6 синий — обработка, LED5 красный — ошибка. */
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);

  memset((void *)loopSlot, 0, sizeof(loopSlot));
  loopFill      = 0u;
  pLoopReady    = loopSlot[0];
  blockCount    = 0u;
  loopbackError = 0u;
  lbErrWho      = NULL;
  lbErrReported = 0u;
  lbBlocksShown = 0u;

  if (Audio_Init() != 0u)               /* ошибку зафиксировал Audio_OnError (LED5, флаг) */
  {
    const char *who = lbErrWho;
    TRACE_ERR("audio init failed at %s", (who != NULL) ? who : "(unknown)");
    return 1u;
  }

  BSP_LED_On(LED4);                      /* зелёный: инициализация прошла успешно */
  TRACE_LOG("audio init OK: Fs=%u Hz, in ch=%u, pdmBuf=%u words, pcm slot=%u words x2, codec=OK mic=OK",
            (unsigned)AUDIO_FREQ_HZ, (unsigned)DEFAULT_AUDIO_IN_CHANNEL_NBR,
            (unsigned)INTERNAL_BUFF_SIZE, (unsigned)(PCM_OUT_SIZE * 2));
  return 0u;
}

void Lab_Process(void)
{
  if (loopbackError != 0u)
  {
    BSP_LED_On(LED5);
    if (lbErrReported == 0u)
    {
      const char *who = lbErrWho;
      TRACE_ERR("audio error from %s", (who != NULL) ? who : "(unknown)");
      lbErrReported = 1u;
    }
  }

  if ((uint32_t)(HAL_GetTick() - lbStatusTick) >= 1000u)
  {
    uint32_t now   = blockCount;
    uint32_t delta = now - lbBlocksShown;
    lbBlocksShown = now;
    lbStatusTick  = HAL_GetTick();
    TRACE_LOG("status: blocks=%u err=%u", (unsigned)delta, (unsigned)loopbackError);
  }

  /* SWO heartbeat: раз в ~1 c строка "SWO <n>", форматируется вручную (как прежде). */
  if ((uint32_t)(HAL_GetTick() - swoHbTick) >= 1000u)
  {
    char buf[20];
    char rev[10];
    uint32_t n = swoHbSeq++;
    uint8_t i = 0u;
    uint8_t k = 0u;
    uint8_t j;

    swoHbTick = HAL_GetTick();
    buf[i++] = 'S'; buf[i++] = 'W'; buf[i++] = 'O'; buf[i++] = ' ';
    do { rev[k++] = (char)('0' + (n % 10u)); n /= 10u; } while (n != 0u);
    for (j = k; j > 0u; j--) { buf[i++] = rev[j - 1u]; }
    buf[i++] = '\r'; buf[i++] = '\n'; buf[i] = '\0';
    Trace_SWO_PutString(buf);
  }
}

#endif /* LAB_ID == 0 */
