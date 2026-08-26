/**
  ******************************************************************************
  * @file    App/Common/Src/preempt.c
  * @author  Wagan Sarukhanov
  * @brief   Вытеснение прерываний: аудио-выход выше приёмного USART2. См. preempt.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Компилируется только там, где есть аудио-выход с 1-мс пере-взводом DMA (LAB04/LAB07).
  ******************************************************************************
  */

#include "lab.h"

#if (LAB_ID == 4) || (LAB_ID == 7) || (LAB_ID == 8)

#include "preempt.h"
#include "stm32f4xx_hal.h"

/* По умолчанию проект стоит на NVIC_PRIORITYGROUP_0 (вытеснения нет вовсе, stm32f4xx_hal_msp.c:75),
 * поэтому колбэк пере-взвода аудио-выхода (DMA NORMAL, каждую 1 мс) не может прервать длинный
 * приёмный ISR → срыв дедлайна → щелчки. Включаем GROUP_4 и уровни (0 = высший):
 *   аудио-выход DMA1_Stream7 = 0  — ДОЛЖЕН прерывать приёмный ISR (это и есть лечение);
 *   SysTick               = 1  — ниже выхода (не задерживает пере-взвод), выше остальных
 *                                (HAL_GetTick идёт во время USART/USB/аудио-входа);
 *   аудио-вход DMA1_Stream3 = 2 и USART2 + DMA1_Stream5 (RX) = 2 — ОДИН уровень: приёмный колбэк
 *                                не реентерабелен, а аудио-вход на ПЕРЕДАТЧИКЕ зовёт
 *                                HAL_UART_Transmit_IT — при равном уровне нет реентерабельного
 *                                доступа к huart2;
 *   USB OTG_FS            = 3  — ниже всех (не задерживает аудио).
 * Пути аудио-ISR проверены: без HAL_Delay/HAL_GetTick-ожиданий (I2S DMA-старт неблокирующий,
 * SendRaw = HAL_UART_Transmit_IT), поэтому уровни выше SysTick безопасны. Джиттер-буфер во всех
 * работах — корректный SPSC (индекс публикуется после записи данных), безопасен при вытеснении. */
void Preempt_AudioOutHighest(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0u, 0u);   /* аудио-выход I2S3 — высший */
  HAL_NVIC_SetPriority(SysTick_IRQn,      1u, 0u);
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2u, 0u);   /* аудио-вход I2S2 */
  HAL_NVIC_SetPriority(USART2_IRQn,       2u, 0u);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2u, 0u);   /* приём USART2 RX DMA — как USART2 */
  HAL_NVIC_SetPriority(OTG_FS_IRQn,       3u, 0u);   /* USB — низший */
  if (primask == 0u) { __enable_irq(); }
}

#endif /* (LAB_ID == 4) || (LAB_ID == 7) */
