/**
  ******************************************************************************
  * @file    App/Labs/speech.c
  * @author  Wagan Sarukhanov
  * @brief   LAB07 «Сжатие речи» — тонкий адаптер над общим голосовым движком (voice.c).
  *          Реализует Lab_Init/Lab_Process, делегируя всю работу движку.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Этап 1 разделения LAB07/LAB08 (TASK_lab08_split_recon): весь речевой тракт вынесен в
  * App/Common/voice.{h,c} БЕЗ изменения поведения. Пока движок обслуживает обе работы, поэтому
  * этот файл-адаптер компилируется и в LAB07, и в LAB08 (guard ниже). На этапе 2 появится
  * отдельный адаптер LAB08 (App/Labs/radio_voice.c, #if LAB_ID==8), а этот guard вернётся к == 7.
  * Умолчания и наборы команд (провод vs эфир) разведутся на этапе 3.
  ******************************************************************************
  */

#include "lab.h"

#if (LAB_ID == 7) || (LAB_ID == 8)

#include "voice.h"

uint8_t Lab_Init(void)   { return Voice_Init(); }
void    Lab_Process(void) { Voice_Process(); }

#endif /* (LAB_ID == 7) || (LAB_ID == 8) */
