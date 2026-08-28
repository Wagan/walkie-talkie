/**
  ******************************************************************************
  * @file    App/Common/Inc/uwb_chip.h
  * @author  Wagan Sarukhanov
  * @brief   LAB09: интерфейс тонкого адаптера поколения UWB-чипа (DW1000/DW3000).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Общий движок LAB09 — App/Common/Src/uwb_core.c (сброс, обмен по SPI, каркас команд,
  * печать и диагностика ответа). Над ним ровно один адаптер (App/Labs/uwb_dw3000.c или
  * uwb_dw1000.c, выбор флагом сборки, см. uwb_config.h), реализующий функции ниже —
  * то немногое, что зависит от поколения чипа: построение заголовка чтения регистра,
  * ожидаемый идентификатор и его толкование, потолок частоты SPI.
  ******************************************************************************
  */

#ifndef UWB_CHIP_H
#define UWB_CHIP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>   /* NULL */
#include "uwb_config.h"

/* Общие константы (одинаковы у обоих поколений). */
#define UWB_REG_DEV_ID   0x00u        /* адрес регистра идентификатора устройства */
#define UWB_DEADDEAD     0xDEADDEADu  /* маркер неиспользуемого/зарезервированного регистра */

/* --- Реализуется адаптером выбранного семейства --- */

/* Короткое имя семейства для печати ("DW1000" / "DW3000"). */
const char *UwbChip_Family(void);

/* Построить заголовок ЧТЕНИЯ регистра reg в hdr[] (>= 2 байт), вернуть длину заголовка (байт).
 * Формат заголовка зависит от поколения (см. RECON_dw3000.md / DW3000 User Manual §2.3). */
uint8_t UwbChip_BuildReadHeader(uint8_t reg, uint8_t *hdr);

/* Основное ожидаемое значение DEV_ID (для печати "expected 0x..."). */
uint32_t UwbChip_DevIdExpected(void);

/* Если devid — валидный «живой» идентификатор этого семейства, вернуть 1 и записать в
 * *variant строку исполнения ("" если одно; напр. "non-PDoA"/"PDoA" у DW3000). Иначе 0. */
uint8_t UwbChip_IsAlive(uint32_t devid, const char **variant);

/* Строка про потолок частоты SPI в текущем (после сброса) состоянии — для spistat. */
const char *UwbChip_SpiCeilingNote(void);

/* Прочитать DEV_ID ЧЕРЕЗ драйвер производителя (если он есть у семейства). Возврат 1 и значение
 * в *out, если получено драйвером; 0 если драйвера нет (тогда общий движок читает напрямую).
 * У DW3000 — через Qorvo dwt_uwb_driver (платформенный слой dw3000_port); у DW1000 — нет. */
uint8_t UwbChip_ReadDevIdViaDriver(uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* UWB_CHIP_H */
