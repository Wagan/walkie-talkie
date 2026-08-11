/**
  ******************************************************************************
  * @file    App/Src/audio_loopback.c
  * @author  Wagan Sarukhanov
  * @brief   Демо loopback «микрофон MP45DT02 → кодек CS43L22», Fs = 16 кГц.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * ТРАКТ ДАННЫХ (по эталону Audio_playback_and_record, пересчёт под Fs = 16000):
  *
  *  MP45DT02 --PDM--> I2S2 (master RX, MCLK off, прогр. 2·Fs=32000)
  *        --кольцевой DMA1_Stream3--> pdmBuf[INTERNAL_BUFF_SIZE]
  *        --BSP_AUDIO_IN_PDMToPCM (децимация 64, HTONS внутри)--> pcmBuf[slot]
  *        --BSP_AUDIO_OUT_ChangeBuffer--> I2S3 (master TX, MCLK on)
  *        --DMA1_Stream7--> CS43L22 --> джек 3.5 мм.
  *
  * РАЗМЕРЫ БУФЕРОВ (все — из макросов BSP, раскрытие показано):
  *   INTERNAL_BUFF_SIZE = 128 · DEFAULT_AUDIO_IN_FREQ/16000 · DEFAULT_AUDIO_IN_CHANNEL_NBR
  *                      = 128 · 16000/16000 · 1 = 128 слов (u16) — кольцевой приёмный буфер PDM.
  *   Половина буфера = INTERNAL_BUFF_SIZE/2 = 64 слова PDM → один вызов PDMToPCM.
  *   PCM_OUT_SIZE       = DEFAULT_AUDIO_IN_FREQ/1000 = 16 — число PCM-отсчётов на канал за 1 вызов
  *                        (16 отсчётов при 16 кГц = 1 мс звука).
  *   PCM-слот           = PCM_OUT_SIZE·2 = 32 слова — стерео (моно-отсчёт дублируется в 2 канала
  *                        внутри BSP_AUDIO_IN_PDMToPCM, т.к. микрофон один).
  *
  * СХЕМА БУФЕРИЗАЦИИ / РАЗВЯЗКА ПРИЁМА И ПЕРЕДАЧИ:
  *   Приём — кольцевой DMA (как в эталоне): прерывание половины отдаёт pdmBuf[0..63],
  *   прерывание конца — pdmBuf[64..127]. Каждое даёт 1 мс звука (16 стерео-кадров).
  *   Выход — ДВОЙНОЙ буфер pcmBuf[2][32]. Микрофонный колбэк пишет в слот pcmFillIdx,
  *   затем публикует его в pPcmReady и переключает pcmFillIdx на другой слот.
  *   Колбэк завершения передачи кодека отдаёт в DMA слот pPcmReady. Так слот, который
  *   играет кодек, НЕ переписывается: микрофон в этот момент пишет ДРУГОЙ слот.
  *   Оба тракта тактируются одним PLLI2S (86 МГц, см. bsp_audio_clock.c) → строго 16 кГц
  *   и там, и там, без расхождения частот, поэтому двух слотов достаточно (задержка ≈1–2 мс).
  *   Публикация pPcmReady — запись/чтение выровненного указателя, атомарна на Cortex-M4.
  *
  * РЕЖИМ DMA ПЕРЕДАЧИ: у BSP выход настроен как DMA_NORMAL (не кольцевой,
  *   BSP_AUDIO_OUT_MspInit), поэтому следующий блок подаётся в колбэке завершения
  *   передачи через BSP_AUDIO_OUT_ChangeBuffer — тем же способом, что у ST (waveplayer).
  *
  * ТАКТИРОВАНИЕ CRC: PDM-библиотека разблокируется тактированием CRC. Проверено по коду
  *   BSP: PDMDecoder_Init() (вызывается из BSP_AUDIO_IN_Init) делает __HAL_RCC_CRC_CLK_ENABLE()
  *   — отдельно включать в нашем коде НЕ требуется.
  ******************************************************************************
  */

#include "audio_loopback.h"
#include "stm32f411e_discovery_audio.h"   /* BSP аудио + константы (INTERNAL_BUFF_SIZE, ...) */
#include "stm32f411e_discovery.h"         /* светодиоды BSP_LED_* */
#include "trace_log.h"                    /* TRACE_LOG / TRACE_ERR (печать только вне ISR) */
#include <string.h>

/* Частота дискретизации демо и умеренная громкость кодека (0..100). */
#define LOOPBACK_AUDIO_FREQ   I2S_AUDIOFREQ_16K   /* = 16000 Гц */
#define LOOPBACK_VOLUME       70U

/* Приёмный кольцевой буфер PDM (см. расчёт размера в шапке). */
static uint16_t pdmBuf[INTERNAL_BUFF_SIZE];

/* Двойной выходной буфер PCM (стерео), 2 слота по PCM_OUT_SIZE*2 слов. */
static uint16_t pcmBuf[2][PCM_OUT_SIZE * 2];

/* Индекс слота, в который микрофонный колбэк пишет сейчас. */
static volatile uint8_t pcmFillIdx = 0U;
/* Последний готовый слот для кодека (публикуется микрофонным колбэком). */
static uint16_t * volatile pPcmReady = NULL;

/* Диагностика. Переменные, меняющиеся в прерывании, объявлены volatile.
   loopbackError и blockCount пишутся в ISR-колбэках, читаются в основном цикле.
   lbErrWho — имя функции BSP, вернувшей ошибку (указатель на строковый литерал;
   запись/чтение выровненного указателя атомарны на Cortex-M4). */
static volatile uint8_t  loopbackError = 0U;
static volatile uint32_t blockCount    = 0U;
static const char * volatile lbErrWho  = NULL;

/* Только основной цикл (не ISR): темп статуса ~1 Гц, снимок счётчиков, флаг доклада. */
static uint32_t lbStatusTick   = 0U;
static uint32_t lbBlocksShown  = 0U;
static uint8_t  lbErrReported  = 0U;

/**
  * @brief  Преобразовать одну половину PDM-буфера в PCM и опубликовать для кодека.
  * @param  pdmHalf: указатель на начало обрабатываемой половины pdmBuf.
  */
static void Loopback_ConvertBlock(uint16_t *pdmHalf)
{
  /* HTONS-подготовка данных перед фильтром выполняется ВНУТРИ BSP_AUDIO_IN_PDMToPCM,
     параметры фильтра (децимация 64, 1 вход → 2 выхода) заданы в PDMDecoder_Init. */
  BSP_AUDIO_IN_PDMToPCM(pdmHalf, pcmBuf[pcmFillIdx]);

  /* Публикуем заполненный слот и переключаемся на другой. */
  pPcmReady  = pcmBuf[pcmFillIdx];
  pcmFillIdx ^= 1U;

  blockCount++;
  BSP_LED_Toggle(LED6);   /* синий: индикация обработки блоков */
}

uint8_t AudioLoopback_Init(void)
{
  /* Светодиоды-индикаторы: LED4 зелёный — init OK, LED6 синий — обработка, LED5 красный — ошибка. */
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);

  /* Стартовая тишина в обоих слотах, начальные указатели. */
  memset((void *)pcmBuf, 0, sizeof(pcmBuf));
  pcmFillIdx    = 0U;
  pPcmReady     = pcmBuf[0];
  blockCount    = 0U;
  loopbackError = 0U;
  lbErrWho      = NULL;
  lbErrReported = 0U;
  lbBlocksShown = 0U;

  /* ПОРЯДОК ИНИЦИАЛИЗАЦИИ (обоснование по исходнику BSP):
     оба BSP_AUDIO_*_ClockConfig перекрыты и ставят один и тот же PLLI2S (86 МГц),
     поэтому вторая инициализация не сбивает частоту первой. Кодек (выход) поднимаем
     первым: BSP_AUDIO_OUT_Init настраивает CS43L22 по I2C и I2S3; затем вход —
     BSP_AUDIO_IN_Init настраивает I2S2 и PDM-фильтр (и включает тактирование CRC). */
  if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, LOOPBACK_VOLUME, LOOPBACK_AUDIO_FREQ) != AUDIO_OK)
  {
    lbErrWho = "BSP_AUDIO_OUT_Init";
    loopbackError = 1U;
    BSP_LED_On(LED5);
    return AUDIO_ERROR;
  }

  if (BSP_AUDIO_IN_Init(LOOPBACK_AUDIO_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION,
                        DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
  {
    lbErrWho = "BSP_AUDIO_IN_Init";
    loopbackError = 1U;
    BSP_LED_On(LED5);
    return AUDIO_ERROR;
  }

  /* Заводим передачу кодека тишиной: DMA передачи в режиме NORMAL начнёт выдавать
     прерывания завершения (TransferComplete), в которых мы подаём свежий PCM.
     Размер BSP_AUDIO_OUT_Play — в БАЙТАХ (внутри делится на AUDIODATA_SIZE). */
  if (BSP_AUDIO_OUT_Play(pcmBuf[0], (uint32_t)(PCM_OUT_SIZE * 2 * AUDIODATA_SIZE)) != AUDIO_OK)
  {
    lbErrWho = "BSP_AUDIO_OUT_Play";
    loopbackError = 1U;
    BSP_LED_On(LED5);
    return AUDIO_ERROR;
  }

  /* Запускаем приём с микрофона кольцевым DMA на весь pdmBuf.
     Размер BSP_AUDIO_IN_Record — в СЛОВАХ (u16), передаётся в HAL_I2S_Receive_DMA. */
  if (BSP_AUDIO_IN_Record(pdmBuf, (uint32_t)INTERNAL_BUFF_SIZE) != AUDIO_OK)
  {
    lbErrWho = "BSP_AUDIO_IN_Record";
    loopbackError = 1U;
    BSP_LED_On(LED5);
    return AUDIO_ERROR;
  }

  BSP_LED_On(LED4);   /* зелёный: инициализация прошла успешно */

  /* Одно сообщение при инициализации с фактическими параметрами (main-контекст, не ISR). */
  TRACE_LOG("audio init OK: Fs=%u Hz, in ch=%u, pdmBuf=%u words, pcm slot=%u words x2, codec=OK mic=OK",
            (unsigned)LOOPBACK_AUDIO_FREQ, (unsigned)DEFAULT_AUDIO_IN_CHANNEL_NBR,
            (unsigned)INTERNAL_BUFF_SIZE, (unsigned)(PCM_OUT_SIZE * 2));
  return AUDIO_OK;
}

void AudioLoopback_Process(void)
{
  /* Вся обработка звука — в прерываниях DMA. Здесь (основной цикл, не ISR) печатаем
     статус не чаще раза в секунду и один раз докладываем об ошибке. */
  if (loopbackError != 0U)
  {
    BSP_LED_On(LED5);
    if (lbErrReported == 0U)
    {
      const char *who = lbErrWho;                 /* снимок указателя (атомарно на M4) */
      TRACE_ERR("audio error from %s", (who != NULL) ? who : "(unknown)");
      lbErrReported = 1U;
    }
  }

  if ((uint32_t)(HAL_GetTick() - lbStatusTick) >= 1000U)
  {
    uint32_t now = blockCount;                    /* снимок счётчика из ISR (атомарно) */
    uint32_t delta = now - lbBlocksShown;
    lbBlocksShown = now;
    lbStatusTick  = HAL_GetTick();
    TRACE_LOG("status: blocks=%u err=%u", (unsigned)delta, (unsigned)loopbackError);
  }
}

uint8_t AudioLoopback_HasError(void)
{
  return loopbackError;
}

/* ===== Колбэки BSP (перекрывают __weak из stm32f411e_discovery_audio.c) ===== */

/* Приём: половина кольцевого буфера готова → обрабатываем pdmBuf[0..63]. */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  Loopback_ConvertBlock(&pdmBuf[0]);
}

/* Приём: буфер заполнен → обрабатываем вторую половину pdmBuf[64..127]. */
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  Loopback_ConvertBlock(&pdmBuf[INTERNAL_BUFF_SIZE / 2]);
}

/* Передача завершена → подаём кодеку последний готовый PCM-слот.
   Размер ChangeBuffer — в СЛОВАХ (u16), передаётся в HAL_I2S_Transmit_DMA напрямую. */
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)pPcmReady, (uint16_t)(PCM_OUT_SIZE * 2));
}

/* Ошибки трактов делаем видимыми (имена — точно как __weak в BSP:
   у входа Error_Callback, у выхода Error_CallBack). */
void BSP_AUDIO_IN_Error_Callback(void)
{
  /* ISR: только фиксируем источник и флаг; печать — в AudioLoopback_Process (основной цикл). */
  lbErrWho = "BSP_AUDIO_IN_Error_Callback";
  loopbackError = 1U;
  BSP_LED_On(LED5);
}

void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* ISR: только фиксируем источник и флаг; печать — в AudioLoopback_Process (основной цикл). */
  lbErrWho = "BSP_AUDIO_OUT_Error_CallBack";
  loopbackError = 1U;
  BSP_LED_On(LED5);
}
