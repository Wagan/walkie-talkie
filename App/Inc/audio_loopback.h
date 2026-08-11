/**
  ******************************************************************************
  * @file    App/Inc/audio_loopback.h
  * @brief   Демо loopback «микрофон → кодек» на Fs = 16 кГц (нулевая лаба курса).
  ******************************************************************************
  */

#ifndef AUDIO_LOOPBACK_H
#define AUDIO_LOOPBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  Инициализирует и запускает loopback: приём с микрофона MP45DT02 и
  *         вывод на кодек CS43L22, Fs = 16 кГц, вход моно.
  * @retval 0 (AUDIO_OK) при успехе, 1 (AUDIO_ERROR) если какая-либо функция BSP
  *         вернула ошибку. При ошибке также поднимается внутренний флаг
  *         (см. AudioLoopback_HasError) и загорается красный светодиод.
  */
uint8_t AudioLoopback_Init(void);

/**
  * @brief  Обслуживание из основного цикла. Вся обработка звука идёт в прерываниях
  *         DMA; здесь только неспешная фоновая работа (индикация состояния ошибки).
  */
void AudioLoopback_Process(void);

/**
  * @brief  Флаг ошибки, видимый снаружи (для диагностики без отладчика).
  * @retval 0 — ошибок не было, 1 — была ошибка BSP (инициализация или тракт).
  */
uint8_t AudioLoopback_HasError(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_LOOPBACK_H */
