/**
  ******************************************************************************
  * @file    App/Labs/speech.c
  * @author  Wagan Sarukhanov
  * @brief   LAB07 «Сжатие речи на проводе» — тонкий адаптер над движком voice.c.
  *          Умолчания провода (raw / 16 кГц / 921600) + набор команд лесенки.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Этап 2 разделения LAB07/LAB08 (TASK_lab08_split_recon): весь тракт — в App/Common/voice.{h,c};
  * здесь только стартовый конфиг и выбор набора команд. Радио — отдельный адаптер radio_voice.c
  * (LAB08). LAB07 сравнивает кодеки на чистом проводе, поэтому по умолчанию сырой PCM 16 кГц на
  * 921600, и команды лесенки (codec/rate/decim/headroom/load/budget) + отладка.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 7

#include "voice.h"

/* Умолчания провода: сырой PCM, 16 кГц, линия 921600 (запас для raw 16 кГц). */
static const VoiceConfig k_wireCfg = { VOICE_CODEC_RAW, 0u /*16 кГц*/, 0u /*Codec2 3200*/, 921600u };

uint8_t Lab_Init(void)
{
  uint16_t n;
  const console_cmd_t *cmds = Voice_CmdsLab07(&n);
  return Voice_Init(&k_wireCfg, cmds, n);
}

void Lab_Process(void) { Voice_Process(); }

#endif /* LAB_ID == 7 */
