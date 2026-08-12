/**
  ******************************************************************************
  * @file    App/Common/Src/bsp_audio_clock.c
  * @author  Wagan Sarukhanov
  * @brief   Единое тактирование PLLI2S для обоих аудиотрактов (loopback-демо).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * ЗАЧЕМ ЭТОТ ФАЙЛ.
  * PLLI2S на кристалле STM32F411 ОДИН, и от него тактируются оба тракта — вход
  * (микрофон, I2S2) и выход (кодек, I2S3). Штатные __weak-функции ST ставят для
  * них РАЗНЫЕ значения PLLI2S: BSP_AUDIO_IN_ClockConfig → I2SCLK 32 МГц
  * (M=8, N=192, R=6), BSP_AUDIO_OUT_ClockConfig → I2SCLK 86 МГц (M=8, N=258, R=3).
  * При одновременной работе двух трактов вторая по времени инициализация
  * перепрограммирует PLLI2S, и делитель первого тракта, посчитанный под старую
  * частоту, начинает выдавать неверную Fs (вплоть до −63 %).
  *
  * РЕШЕНИЕ: здесь заданы СИЛЬНЫЕ одноимённые функции, перекрывающие слабые из BSP.
  * Обе программируют ОДИН И ТОТ ЖЕ PLLI2S — I2SCLK = 86 МГц (PLLI2SM=8, PLLI2SN=258,
  * PLLI2SR=3), решение владельца из docs/REPORT_i2s_clock_analysis.md. Способ
  * программирования (RCC_PeriphCLKInitTypeDef + HAL_RCCEx_*) повторяет оригинал ST,
  * чтобы не появилось второго стиля работы с RCC.
  *
  * РАСЧЁТ ФАКТИЧЕСКИХ ЧАСТОТ (формула RM0383 §20.4.4, 16-битный кадр):
  *   вход  (MCLK выкл): FS = I2SCLK / (32 · D),   D = 2·I2SDIV + ODD; прогр. FS = 2·Fs;
  *   выход (MCLK вкл):  FS = I2SCLK / (256 · D).
  * При I2SCLK = 86 МГц:
  *   Fs = 16000:
  *     выход: D=86e6/(256·16000)=20.996 → I2SDIV=10, ODD=1 → 15997.02 Гц, ошибка −0.0186 %
  *     вход : прогр.32000, D=86e6/(32·32000)=83.98 → I2SDIV=42, ODD=0 → 31994.05 Гц
  *            (аудио 15997.02 Гц), ошибка −0.0186 %
  *   Fs = 8000 (пригодность на будущее, этап радиоканала):
  *     выход: D=86e6/(256·8000)=41.99 → I2SDIV=21, ODD=0 → 7998.51 Гц, ошибка −0.0186 %
  *     вход : прогр.16000, D=86e6/(32·16000)=167.97 → I2SDIV=84, ODD=0 → 15997.02 Гц
  *            (аудио 7998.51 Гц), ошибка −0.0186 %
  * Вывод: на 86 МГц оба тракта дают ≈ −0.019 % и на 16 кГц, и на 8 кГц — приемлемо,
  * то же значение PLLI2S годится для будущего перехода на 8 кГц.
  ******************************************************************************
  */

/* Прототипы BSP_AUDIO_*_ClockConfig и типы HAL берутся отсюда (тот же заголовок,
   что использует BSP), чтобы сигнатуры перекрытий совпадали точно. */
#include "stm32f411e_discovery_audio.h"

/* Единое значение PLLI2S для обоих трактов (I2SCLK = 8МГц/M · N / R = 86 МГц). */
#define LOOPBACK_PLLI2SM   8U
#define LOOPBACK_PLLI2SN   258U
#define LOOPBACK_PLLI2SR   3U

/**
  * @brief  Программирует единый PLLI2S для аудиотрактов.
  * @note   Локальная функция, вызывается из обоих перекрытий, чтобы значение
  *         задавалось в одном месте.
  */
static void Loopback_SetUnifiedPLLI2S(void)
{
  RCC_PeriphCLKInitTypeDef rccclkinit;

  /* Тот же порядок работы с RCC, что и в оригинальных функциях ST. */
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
  rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  rccclkinit.PLLI2S.PLLI2SM = LOOPBACK_PLLI2SM;
  rccclkinit.PLLI2S.PLLI2SN = LOOPBACK_PLLI2SN;
  rccclkinit.PLLI2S.PLLI2SR = LOOPBACK_PLLI2SR;
  HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
}

/**
  * @brief  Перекрытие слабой BSP_AUDIO_IN_ClockConfig (микрофон, I2S2).
  * @note   Сигнатура точно как в stm32f411e_discovery_audio.c. AudioFreq и Params
  *         намеренно не используются: PLLI2S единый и не зависит от запрошенной Fs.
  */
void BSP_AUDIO_IN_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params)
{
  (void)hi2s;
  (void)AudioFreq;
  (void)Params;
  Loopback_SetUnifiedPLLI2S();
}

/**
  * @brief  Перекрытие слабой BSP_AUDIO_OUT_ClockConfig (кодек, I2S3).
  * @note   Сигнатура точно как в stm32f411e_discovery_audio.c.
  */
void BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params)
{
  (void)hi2s;
  (void)AudioFreq;
  (void)Params;
  Loopback_SetUnifiedPLLI2S();
}
