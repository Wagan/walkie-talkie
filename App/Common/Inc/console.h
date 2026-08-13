/**
  ******************************************************************************
  * @file    App/Common/Inc/console.h
  * @author  Wagan Sarukhanov
  * @brief   Консоль поверх USB CDC (Virtual COM): приём строк, разбор на команды,
  *          таблица команд, вывод. Общий модуль для всех работ, которым нужна консоль.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Модель работы (без блокировок и без печати из ISR):
  *  - из колбэка USB (CDC_Receive_FS) вызывается Console_RxFromISR() — она ТОЛЬКО
  *    складывает принятые байты в кольцевой буфер;
  *  - Console_Process() вызывается в основном цикле: собирает строки (учитывая разные
  *    окончания строк), разбирает на имя команды и аргументы, вызывает обработчик, а также
  *    выталкивает накопленный вывод в USB порциями (неблокирующе).
  * Команды регистрируются таблицей — добавление новой команды не требует правок разбора.
  ******************************************************************************
  */

#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Обработчик команды: argc/argv как в main() (argv[0] — имя команды). */
typedef void (*console_cmd_fn)(int argc, char **argv);

typedef struct
{
  const char     *name;   /* имя команды */
  const char     *help;   /* краткое описание для команды help */
  console_cmd_fn  fn;      /* обработчик */
} console_cmd_t;

/* Инициализация консоли (сброс буферов). Вызывать до Console_Process. */
void Console_Init(void);

/* Обслуживание в основном цикле: разбор принятых строк + выталкивание вывода в USB. */
void Console_Process(void);

/* Приём байтов из колбэка USB CDC (контекст прерывания). Только буферизация. */
void Console_RxFromISR(const uint8_t *data, uint32_t len);

/* Регистрация таблицы команд (указатель должен жить всё время работы, напр. static const). */
void Console_Register(const console_cmd_t *cmds, uint16_t count);

/* Форматированный вывод в консоль (в USB; попадает и в SWO, если строка шла через printf). */
int  Console_Printf(const char *fmt, ...);

/* Вывод готовой строки в консоль. */
void Console_Write(const char *s);

/* 1, если USB Device сконфигурирован хостом (порт открыт на стороне ПК как COM). */
uint8_t Console_IsConfigured(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
