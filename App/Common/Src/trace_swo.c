/**
  ******************************************************************************
  * @file    App/Common/Src/trace_swo.c
  * @author  Wagan Sarukhanov
  * @brief   Минимальный вывод строки в SWO через ITM (CMSIS ITM_SendChar), Часть 1.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * ПОВЕДЕНИЕ БЕЗ ОТЛАДЧИКА / ПРИ ВЫКЛЮЧЕННОЙ ТРАССИРОВКЕ (важно для автономного питания).
  * Используется штатная CMSIS-функция ITM_SendChar() из core_cm4.h. По её исходнику
  * (Drivers/CMSIS/Include/core_cm4.h) она устроена так:
  *
  *   if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & 1))  // ITM включён И порт 0 включён
  *   {
  *       while (ITM->PORT[0].u32 == 0) { __NOP(); }          // ждать готовности FIFO
  *       ITM->PORT[0].u8 = (uint8_t)ch;                      // записать байт
  *   }
  *   return ch;
  *
  * Биты TCR.ITMENA и TER(порт 0) выставляет ОТЛАДЧИК при включении SWV. Поэтому:
  *  - без подключённого отладчика (или при выключенной трассировке) условие ЛОЖНО →
  *    функция НЕ входит в цикл ожидания, НЕ пишет в FIFO и СРАЗУ возвращает управление,
  *    просто отбрасывая символ. Ядро не блокируется — прошивка работает автономно.
  *  - busy-wait `while(PORT[0].u32==0)` достижим ТОЛЬКО когда отладчик уже включил ITM
  *    и порт 0; тогда возможна кратковременная задержка, если приёмник SWO не успевает
  *    вычитывать. Это штатное поведение ITM и на автономный кит не влияет.
  ******************************************************************************
  */

#include "trace_swo.h"
#include "stm32f4xx.h"   /* Device Peripheral Access Layer → подключает core_cm4.h с ITM_SendChar() */
#include <stddef.h>

void Trace_SWO_PutString(const char *s)
{
  if (s == NULL)
  {
    return;
  }
  while (*s != '\0')
  {
    (void)ITM_SendChar((uint32_t)(uint8_t)(*s));
    s++;
  }
}

/**
  * @brief  Перенаправление printf в SWO без правки syscalls.c.
  * @note   Сгенерированный Core/Src/syscalls.c содержит слабый _write(), который в цикле
  *         вызывает __io_putchar(), объявленную там как `extern __attribute__((weak))`
  *         без сильного определения. Дав здесь СИЛЬНУЮ __io_putchar(), мы штатно
  *         перенаправляем printf в ITM (порт 0), не изменяя syscalls.c.
  *         При отсутствии отладчика ITM_SendChar() возвращает сразу (см. выше) — printf
  *         не блокирует ядро.
  */
/**
  * @brief  Слабый хук второго приёмника вывода printf. По умолчанию — пусто.
  * @note   Переопределяется модулем консоли (App/Common/console.c, только LAB05) для
  *         дублирования вывода в USB CDC. Для LAB00/LAB02 остаётся пустым — SWO без изменений.
  */
__attribute__((weak)) void Trace_UsbPutChar(int ch)
{
  (void)ch;
}

int __io_putchar(int ch)
{
  (void)ITM_SendChar((uint32_t)(uint8_t)ch);
  Trace_UsbPutChar(ch);         /* доп. приёмник (USB в LAB05); по умолчанию no-op */
  return ch;
}
