/**
  ******************************************************************************
  * @file    App/Common/Inc/trace_swo.h
  * @author  Wagan Sarukhanov
  * @brief   Минимальный односторонний вывод диагностики через SWO (ITM), Часть 1.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  */

#ifndef TRACE_SWO_H
#define TRACE_SWO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Номер ITM-стимул-порта. Штатная CMSIS-функция ITM_SendChar() жёстко работает с
   портом 0 (см. core_cm4.h), поэтому в SWV ITM Data Console нужно включать именно
   этот порт. Константа введена для наглядности и как единая точка правды. */
#define TRACE_SWO_ITM_PORT   0U

/**
  * @brief  Вывести нуль-терминированную строку в SWO через ITM (порт 0).
  * @note   Безопасно при отсутствии отладчика/выключенной трассировке: ничего не
  *         выводит и НЕ блокирует ядро (подробности в trace_swo.c).
  */
void Trace_SWO_PutString(const char *s);

/**
  * @brief  Хук дополнительного приёмника вывода printf (по умолчанию — пустой).
  * @param  ch  символ, только что выведенный через __io_putchar (после SWO).
  * @note   Слабая реализация в trace_swo.c ничего не делает, поэтому у работ без консоли
  *         (LAB00/LAB02) вывод идёт только в SWO — поведение не меняется. Модуль консоли
  *         (LAB05) даёт СИЛЬНОЕ определение, направляя те же символы ещё и в USB CDC.
  *         Вызывается из основного цикла (printf), не из ISR.
  */
void Trace_UsbPutChar(int ch);

#ifdef __cplusplus
}
#endif

#endif /* TRACE_SWO_H */
