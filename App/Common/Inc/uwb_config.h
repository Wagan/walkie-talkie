/**
  ******************************************************************************
  * @file    App/Common/Inc/uwb_config.h
  * @author  Wagan Sarukhanov
  * @brief   LAB09: выбор поколения UWB-чипа флагом сборки (по умолчанию DW3000).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * LAB09 — ОДНА работа курса (LAB_ID==9). Радиомодуль может быть двух поколений:
  * серийный DWM3000 (чип DW3110) — основной, и снятый с эксплуатации DWM1000 (DW1000) —
  * сохраняемый путь. Выбор — символом препроцессора в конфигурации сборки (по образцу
  * board_config.h из mks-firmware): по умолчанию DW3000; чтобы собрать путь DW1000 —
  * задать в конфигурации LAB09 символ UWB_CHIP_DW1000 (одна строка: -D UWB_CHIP_DW1000),
  * либо раскомментировать строку ниже. Ровно одно семейство должно быть выбрано.
  ******************************************************************************
  */

#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H

/* Раскомментировать для локальной сборки пути DW1000 (эквивалент -D UWB_CHIP_DW1000): */
/* #define UWB_CHIP_DW1000 */

/* Умолчание — DW3000 (задаётся явно, если флаг не пришёл извне). */
#if !defined(UWB_CHIP_DW1000) && !defined(UWB_CHIP_DW3000)
#define UWB_CHIP_DW3000
#endif

#if defined(UWB_CHIP_DW1000) && defined(UWB_CHIP_DW3000)
#error "LAB09: select exactly ONE UWB chip family (UWB_CHIP_DW1000 or UWB_CHIP_DW3000)."
#endif

#endif /* UWB_CONFIG_H */
