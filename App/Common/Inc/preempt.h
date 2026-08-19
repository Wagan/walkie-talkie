/**
  ******************************************************************************
  * @file    App/Common/Inc/preempt.h
  * @author  Wagan Sarukhanov
  * @brief   Включение вытеснения прерываний с приоритетом аудио-выхода выше приёмного USART2.
  *          Лечение провалов звука на приёме (поздний пере-взвод аудио-выхода).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Общий для работ с аудио-выходом (LAB04/LAB07), чтобы не дублировать раскладку приоритетов.
  * Подробности и разведка — docs/REPORT_nvic_preemption.md и docs/REPORT_preemption_other_labs.md.
  ******************************************************************************
  */

#ifndef PREEMPT_H
#define PREEMPT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Перевести NVIC в режим с вытеснением (GROUP_4) и расставить приоритеты: аудио-выход — высший,
 * ниже него SysTick, затем аудио-вход вместе с USART2 и его RX-DMA на одном уровне, ниже всех USB.
 * Вызывать из Lab_Init работы ПОСЛЕ всех MspInit и ПОВТОРНО после смены baud (реинит USART2
 * сбрасывает его приоритет генерируемым MspInit). Обёрнута в критическую секцию. ОТКАТ — убрать
 * вызовы: вернётся NVIC_PRIORITYGROUP_0 из stm32f4xx_hal_msp.c и приоритеты из MspInit. */
void Preempt_AudioOutHighest(void);

#ifdef __cplusplus
}
#endif

#endif /* PREEMPT_H */
