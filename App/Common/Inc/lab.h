/**
  ******************************************************************************
  * @file    App/Common/Inc/lab.h
  * @author  Wagan Sarukhanov
  * @brief   Единый интерфейс лабораторной работы + выбор работы по LAB_ID.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Курс собран одним проектом CubeIDE: конкретная лабораторная выбирается КОНФИГУРАЦИЕЙ
  * СБОРКИ через символ препроцессора LAB_ID (напр. конфигурация LAB00 задаёт LAB_ID=0).
  * Каждый файл работы в App/Labs/ обёрнут в `#if LAB_ID == <n>` и реализует две функции
  * ниже; main.c вызывает их единообразно и при добавлении новой работы не меняется.
  *
  * Список реализованных LAB_ID держится здесь (единая точка правды). При добавлении
  * новой работы дополнить условие проверки ниже.
  ******************************************************************************
  */

#ifndef LAB_H
#define LAB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --- Защита от неверной конфигурации сборки (внятная ошибка вместо ошибок компоновки) --- */
#if !defined(LAB_ID)
# error "LAB_ID is not defined: the active build configuration does not match any lab. Select a lab build configuration (for example LAB00 -> LAB_ID=0)."
#elif (LAB_ID != 0) && (LAB_ID != 2) && (LAB_ID != 5)
# error "No lab code for the selected build configuration (LAB_ID is not in the implemented list). Implemented: LAB00 (LAB_ID=0), LAB02 (LAB_ID=2), LAB05 (LAB_ID=5). Check the active build configuration."
#endif

/**
  * @brief  Инициализация выбранной лабораторной работы.
  * @retval 0 при успехе, ненулевое значение при ошибке.
  */
uint8_t Lab_Init(void);

/**
  * @brief  Обслуживание работы в основном цикле (вызывать в while(1)).
  * @note   Вся тяжёлая/прерывательная обработка — внутри работы; здесь только
  *         неспешные задачи (индикация, вывод статуса).
  */
void Lab_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* LAB_H */
