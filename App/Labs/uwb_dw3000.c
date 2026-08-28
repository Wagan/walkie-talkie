/**
  ******************************************************************************
  * @file    App/Labs/uwb_dw3000.c
  * @author  Wagan Sarukhanov
  * @brief   LAB09 адаптер поколения DW3000 (модуль DWM3000, чип DW3110) над общим
  *          движком App/Common/Src/uwb_core.c. Компилируется при LAB_ID == 9 и
  *          выбранном флаге UWB_CHIP_DW3000 (умолчание, см. uwb_config.h).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Факты (docs/RECON_dw3000.md, перепроверено по DW3000 User Manual v1.1 в docs/reference):
  *  - DEV_ID (0x00:00, 4 октета RO) = 0xDECA0302 (non-PDoA) или 0xDECA0312 (PDoA)
  *    (UM §8.2.2.1 стр.74; §1.1 стр.7). Привязку DW3110 к одному из них документы не дают,
  *    поэтому оба валидны, различаем и печатаем исполнение;
  *  - заголовок ЧТЕНИЯ короткого адреса — 1 октет: bit0 = 0 (read), bit1 = 0 (короткий,
  *    1-октетный заголовок), биты [5:1] = 5-битный базовый адрес регистрового файла
  *    (UM §2.3.1.2 / Figure 4, стр.13-14). Для DEV_ID (файл 0x00) заголовок = 0x00;
  *  - потолок SPI: INIT_RC (сразу после сброса) <= 7 МГц, до 38 МГц в IDLE_PLL
  *    (UM §2.4 Table 4, стр.19).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 9

#include "uwb_chip.h"   /* тянет uwb_config.h -> разрешает флаг по умолчанию */

#if defined(UWB_CHIP_DW3000)

#include "dw3000_port.h"   /* платформенный слой Qorvo dwt_uwb_driver (наш код) */

#define DW3000_DEVID_NONPDOA  0xDECA0302u
#define DW3000_DEVID_PDOA     0xDECA0312u

const char *UwbChip_Family(void)
{
  return "DW3000";
}

uint8_t UwbChip_BuildReadHeader(uint8_t reg, uint8_t *hdr)
{
  /* Короткий адрес, 1 октет: bit0=0 (read), bit1=0 (short), биты [5:1] = адрес файла. */
  hdr[0] = (uint8_t)((reg & 0x1Fu) << 1);
  return 1u;
}

uint32_t UwbChip_DevIdExpected(void)
{
  return DW3000_DEVID_NONPDOA;   /* основное для печати "expected"; PDoA-вариант тоже валиден */
}

uint8_t UwbChip_IsAlive(uint32_t devid, const char **variant)
{
  if (devid == DW3000_DEVID_NONPDOA)
  {
    if (variant != NULL) { *variant = "non-PDoA"; }
    return 1u;
  }
  if (devid == DW3000_DEVID_PDOA)
  {
    if (variant != NULL) { *variant = "PDoA"; }
    return 1u;
  }
  return 0u;
}

const char *UwbChip_SpiCeilingNote(void)
{
  return "SPI ceiling: 7 MHz in INIT_RC after reset, up to 38 MHz in IDLE_PLL (DW3000 UM Table 4).";
}

uint8_t UwbChip_ReadDevIdViaDriver(uint32_t *out)
{
  /* Через Qorvo dwt_uwb_driver (dwt_probe + dwt_readdevid) поверх нашего платформенного слоя. */
  return Dw3000Port_ReadDevId(out);
}

const console_cmd_t *UwbChip_ExtraCmds(uint16_t *count)
{
  return Dw3000Port_Cmds(count);        /* uwbinit/uwbcfg/uwbtx/uwbrx/uwbstat */
}

void UwbChip_Poll(void)
{
  Dw3000Port_Poll();                    /* опрос приёма кадра */
}

#endif /* UWB_CHIP_DW3000 */

#endif /* LAB_ID == 9 */
