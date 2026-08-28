/**
  ******************************************************************************
  * @file    App/Labs/uwb_dw1000.c
  * @author  Wagan Sarukhanov
  * @brief   LAB09 адаптер поколения DW1000 (модуль DWM1000) над общим движком
  *          App/Common/Src/uwb_core.c. Сохраняемый путь — включается флагом
  *          UWB_CHIP_DW1000 (см. uwb_config.h). Компилируется при LAB_ID == 9.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Факты (DW1000 User Manual v2.17 + DWM1000 Data Sheet v1.8, проверено железом на шаге 1):
  *  - DEV_ID (0x00) = 0xDECA0130 (UM §7.2.2, стр.64), октеты младшим вперёд;
  *  - заголовок ЧТЕНИЯ: bit7 = 0 (read), bit6 = 0 (без суб-индекса), биты [5:0] = адрес
  *    (UM §2.2.1.2, Figure 2). Для DEV_ID заголовок = 0x00;
  *  - потолок SPI: INIT (после сброса) <= 3 МГц, до 20 МГц в IDLE (DW1000 UM Table 1).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 9

#include "uwb_chip.h"   /* тянет uwb_config.h -> разрешает флаг по умолчанию */

#if defined(UWB_CHIP_DW1000)

#define DW1000_DEVID  0xDECA0130u

const char *UwbChip_Family(void)
{
  return "DW1000";
}

uint8_t UwbChip_BuildReadHeader(uint8_t reg, uint8_t *hdr)
{
  /* bit7=0 (read), bit6=0 (без суб-индекса), биты [5:0] = адрес регистра. */
  hdr[0] = (uint8_t)(reg & 0x3Fu);
  return 1u;
}

uint32_t UwbChip_DevIdExpected(void)
{
  return DW1000_DEVID;
}

uint8_t UwbChip_IsAlive(uint32_t devid, const char **variant)
{
  if (devid == DW1000_DEVID)
  {
    if (variant != NULL) { *variant = ""; }   /* единственное исполнение */
    return 1u;
  }
  return 0u;
}

const char *UwbChip_SpiCeilingNote(void)
{
  return "SPI ceiling: 3 MHz in INIT state after reset, up to 20 MHz in IDLE (DW1000 UM Table 1).";
}

uint8_t UwbChip_ReadDevIdViaDriver(uint32_t *out)
{
  /* Для DW1000 драйвер не перенесён — чтение только прямой транзакцией общего движка. */
  (void)out;
  return 0u;
}

#endif /* UWB_CHIP_DW1000 */

#endif /* LAB_ID == 9 */
