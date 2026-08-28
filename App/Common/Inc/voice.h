/**
  ******************************************************************************
  * @file    App/Common/Inc/voice.h
  * @author  Wagan Sarukhanov
  * @brief   Общий голосовой движок: инициализация и обслуживание речевого тракта
  *          (кодеки, децимация, headroom, джиттер-буфер, Codec2, PTT, кадрирование).
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Реализация — App/Common/Src/voice.c (компилируется в LAB07 и LAB08). Работы-адаптеры
  * (LAB07 «сжатие», LAB08 «радио») зовут эти две функции из своих Lab_Init/Lab_Process.
  * Вынесено из App/Labs/speech.c без изменения поведения (этап 1, TASK_lab08_split_recon).
  ******************************************************************************
  */

#ifndef VOICE_H
#define VOICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "console.h"   /* console_cmd_t для набора команд работы */

/* Коды кодеков для VoiceConfig (совпадают с внутренними codec_id_t / WIRE_CODEC2). */
#define VOICE_CODEC_RAW     0u
#define VOICE_CODEC_ULAW    1u
#define VOICE_CODEC_ADPCM   2u
#define VOICE_CODEC_CODEC2  3u

/* Стартовые умолчания работы (провод LAB07 vs эфир LAB08). */
typedef struct
{
  uint8_t  codec;   /* VOICE_CODEC_* */
  uint8_t  rate8k;  /* 1 = 8 кГц, 0 = 16 кГц (при codec == CODEC2 всегда 8 кГц) */
  uint8_t  c2mode;  /* индекс режима Codec2 (0 = 3200) */
  uint32_t baud;    /* стартовая скорость линии, бод */
} VoiceConfig;

/* Curated-наборы команд по работам (обработчики — в движке). Адаптер берёт свой. */
const console_cmd_t *Voice_CmdsLab07(uint16_t *count);  /* провод: лесенка + отладка */
const console_cmd_t *Voice_CmdsLab08(uint16_t *count);  /* радио HC-12: узкий полевой набор */
const console_cmd_t *Voice_CmdsLab09(uint16_t *count);  /* радио UWB: узкий полевой набор */

/* Инициализация голосового тракта: BSP (LED/кнопка), DWT, консоль + регистрация cmds, транспорт
 * USART2, аудио, вытеснение, стартовые умолчания из cfg (кодек/частота/режим/скорость). Печатает
 * шапку с номером работы из LAB_ID. Возврат 0 при успехе, ненулевое при ошибке аудио. */
uint8_t Voice_Init(const VoiceConfig *cfg, const console_cmd_t *cmds, uint16_t ncmds);

/* Обслуживание в основном цикле: разбор принятых кадров (offload), кодирование/отправка Codec2,
 * сторож RS-485, антидребезг PTT, индикация. Вызывать в while(1) через Lab_Process работы. */
void Voice_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_H */
