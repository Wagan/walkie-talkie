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
#include "console.h"   /* console_cmd_t */

/* Аппаратный сброс модуля по RSTn (реализован в uwb_core.c; тот же механизм, что команда dwreset).
 * uwbinit зовёт его первым шагом, иначе повторная инициализация застревает на ожидании IDLE_RC. */
void Uwb_HardReset(void);

/* Инициализация платформенного слоя (счётчик циклов DWT для мкс-задержек). */
void Dw3000Port_Init(void);

/* Однократный dwt_probe() (выбор драйвера чтением DEV_ID через наш SPI). 0 при успехе. */
int  Dw3000Port_Probe(void);

/* Прочитать DEV_ID ЧЕРЕЗ драйвер Qorvo (dwt_readdevid). 1 при успехе (probe прошёл), иначе 0. */
uint8_t Dw3000Port_ReadDevId(uint32_t *out);

/* --- Радио LAB09 (первый кадр) --- реализовано в dw3000_port.c, вызывается адаптером DW3000. */
const console_cmd_t *Dw3000Port_Cmds(uint16_t *count);   /* набор команд uwbinit/uwbcfg/... */
void Dw3000Port_Poll(void);                              /* опрос приёма (из Lab_Process) */

/* --- Транспорт для голосового движка (voice.c, VOICE_XPORT_UWB): «отдать кадр / принять кадр» --- */
uint8_t Dw3000Port_IsInited(void);                       /* 1, если uwbinit прошёл */
uint8_t Dw3000Port_VoiceTx(const uint8_t *payload, uint16_t len);  /* 0 = TXFRS подтверждён, иначе 1 */
void Dw3000Port_VoicePoll(void (*sink)(const uint8_t *payload, uint16_t len)); /* опрос приёма -> sink */
void Dw3000Port_VoiceRxArm(void);                        /* вернуть приёмник в приём (после передачи) */
void Dw3000Port_GetVoiceStats(uint32_t *tx, uint32_t *rx, uint32_t *crc,
                              uint32_t *phe, uint32_t *to, uint32_t *sw);
void Dw3000Port_ResetVoiceStats(void);

/* Единый источник состояния приёмника для печати: "off" / "armed(voice)" / "armed(diag)"
 * (дефект B: uwbstat и строка radio:/link: в voice не должны расходиться). */
const char *Dw3000Port_RxStateStr(void);
/* Число первичных взводов приёмника (конец uwbinit), отдельно от switch (TX->RX). */
uint32_t Dw3000Port_GetArmedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* DW3000_PORT_H */
