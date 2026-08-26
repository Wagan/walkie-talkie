/**
  ******************************************************************************
  * @file    App/Labs/radio_voice.c
  * @author  Wagan Sarukhanov
  * @brief   LAB08 «Речь по радио» (HC-12) — тонкий адаптер над движком voice.c.
  *          Умолчания эфира (Codec2 3200 / 8 кГц / 9600 / ttl) + узкий полевой набор команд.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Этап 2 разделения LAB07/LAB08 (TASK_lab08_split_recon): весь речевой тракт — в общем движке
  * App/Common/voice.{h,c}; здесь только стартовый конфиг и выбор набора команд. LAB08 — рация:
  * плата стартует готовой к эфиру без команд с консоли (питание в поле от USB-адаптера, PTT на
  * кнопке PA0, индикация на радиомодуле). HC-12 — полудуплексный UART без вывода направления, поэтому
  * физслой ttl (умолчание движка). Узкий набор команд: codec/c2mode/prefill/budget/voice/ptt/stat.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 8

#include "voice.h"

/* Умолчания эфира: вокодер Codec2 3200, 8 кГц, линия 9600 (HC-12 FU3). Физслой ttl — умолчание. */
static const VoiceConfig k_airCfg = { VOICE_CODEC_CODEC2, 1u /*8 кГц*/, 0u /*Codec2 3200*/, 9600u };

uint8_t Lab_Init(void)
{
  uint16_t n;
  const console_cmd_t *cmds = Voice_CmdsLab08(&n);
  return Voice_Init(&k_airCfg, cmds, n);
}

void Lab_Process(void) { Voice_Process(); }

#endif /* LAB_ID == 8 */
