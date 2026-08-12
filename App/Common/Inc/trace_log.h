/**
  ******************************************************************************
  * @file    App/Common/Inc/trace_log.h
  * @author  Wagan Sarukhanov
  * @brief   Простой интерфейс логирования с уровнями поверх printf (вывод в SWO).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * printf перенаправлен в SWO/ITM через __io_putchar() (см. App/Src/trace_swo.c).
  * Без подключённого отладчика вывод отбрасывается и НЕ блокирует ядро.
  * ВНИМАНИЕ: не вызывать из обработчиков прерываний (printf медленный) — только из
  * основного цикла и из кода инициализации.
  ******************************************************************************
  */

#ifndef TRACE_LOG_H
#define TRACE_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

/* Два уровня: обычное сообщение [I] и ошибка [E]. Перевод строки добавляется сам.
   ##__VA_ARGS__ — расширение GCC (наш тулчейн arm-none-eabi-gcc), корректно убирает
   запятую при вызове без аргументов формата. */
#define TRACE_LOG(fmt, ...)   printf("[I] " fmt "\r\n", ##__VA_ARGS__)
#define TRACE_ERR(fmt, ...)   printf("[E] " fmt "\r\n", ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* TRACE_LOG_H */
