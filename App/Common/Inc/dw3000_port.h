/**
  ******************************************************************************
  * @file    App/Common/Inc/dw3000_port.h
  * @author  Wagan Sarukhanov
  * @brief   LAB09: платформенный слой Qorvo dwt_uwb_driver под STM32F411 (SPI4).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Это НАШ код (не производный от драйвера Qorvo): он реализует интерфейс, который
  * драйвер ожидает от платформы (SPI-обмен, задержки, критическая секция, пробуждение),
  * и предоставляет прикладные обёртки. Используется только в сборке DW3000
  * (LAB_ID==9 && UWB_CHIP_DW3000). SPI4/выводы CSn(PE11)/RSTn(PE10) — общие с uwb_core.c.
  ******************************************************************************
  */

#ifndef DW3000_PORT_H
#define DW3000_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Инициализация платформенного слоя (счётчик циклов DWT для мкс-задержек). */
void Dw3000Port_Init(void);

/* Однократный dwt_probe() (выбор драйвера чтением DEV_ID через наш SPI). 0 при успехе. */
int  Dw3000Port_Probe(void);

/* Прочитать DEV_ID ЧЕРЕЗ драйвер Qorvo (dwt_readdevid). 1 при успехе (probe прошёл), иначе 0. */
uint8_t Dw3000Port_ReadDevId(uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DW3000_PORT_H */
