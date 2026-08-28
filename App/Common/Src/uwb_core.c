/**
  ******************************************************************************
  * @file    App/Common/Src/uwb_core.c
  * @author  Wagan Sarukhanov
  * @brief   LAB09 «Проверка монтажа и живости UWB-модуля»: общий движок поверх SPI4 —
  *          аппаратный сброс по RSTn, чтение регистра DEV_ID, каркас консольных команд
  *          (devid/dwreset/spistat), печать и диагностика ответа. Поколение чипа
  *          (DW1000/DW3000) подставляет тонкий адаптер (см. uwb_chip.h). Реализует
  *          Lab_Init/Lab_Process; компилируется только при LAB_ID == 9.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Радиообмена и речевого тракта здесь нет — только контроль SPI-связи с модулем.
  * Что относится к КОНКРЕТНОМУ поколению (формат заголовка чтения, ожидаемый DEV_ID и его
  * толкование, потолок частоты SPI) — вынесено в адаптер uwb_dw3000.c / uwb_dw1000.c и
  * зовётся здесь через UwbChip_*(). Всё остальное (выводы, сброс, механика SPI, консоль,
  * диагностика по характеру ответа) — общее и живёт здесь.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 9

#include "uwb_chip.h"
#include "console.h"
#include "trace_log.h"
#include "stm32f4xx_hal.h"
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* (индикация USB) */

/* Дескриптор SPI4, созданный CubeMX в main.c. */
extern SPI_HandleTypeDef hspi4;

/* ================= Выводы модуля (docs/PINOUT_dwm1000_lab09.md; проверены железом) ==========
 * Раскладка одна для DWM1000 и DWM3000EVB (pin/pitch-совместимы; на шилде RSTn выведен на D7). */
#define UWB_CSN_PORT    GPIOE
#define UWB_CSN_PIN     GPIO_PIN_11   /* SPICSn: программный выбор кристалла, push-pull */
#define UWB_RST_PORT    GPIOE
#define UWB_RST_PIN     GPIO_PIN_10   /* RSTn: выход с ОТКРЫТЫМ СТОКОМ, только притяжка вниз */

/* ================= Тайминги аппаратного сброса (общие, с запасом на оба поколения) =========
 * RSTn — активно-низкий open-drain, высоким извне не подавать (DWM1000 DS §1.2; DWM3000 DS
 * Table 2). Минимум низкого импульса: DW1000 = 10 нс (DWM1000 DS §1.2); для DW3000 числом в
 * доступных документах не задан. HAL_Delay работает в мс, поэтому держим низким 2 мс (>>
 * минимумов) и ждём 5 мс после отпускания: у DW1000 выход в INIT ~4 мс (UM Table 1), у DW3000
 * POR ~1 мс (XTAL) + ~70 мкс (OTP boot) до готовности SPI (DW3000 UM Figure 8 / DWM3000 DS
 * Figure 1) — 5 мс покрывает оба. Значения легко изменить здесь. */
#define UWB_RST_LOW_MS    2u
#define UWB_RST_WAIT_MS   5u

/* ================= Низкоуровневые операции ================= */

static inline void uwb_cs_low(void)  { HAL_GPIO_WritePin(UWB_CSN_PORT, UWB_CSN_PIN, GPIO_PIN_RESET); }
static inline void uwb_cs_high(void) { HAL_GPIO_WritePin(UWB_CSN_PORT, UWB_CSN_PIN, GPIO_PIN_SET);   }

/* Аппаратный сброс модуля по RSTn (open-drain: запись 0 тянет вниз, запись 1 отпускает в
 * высокоимпедансное состояние; активный высокий извне не подаём). CSn на время сброса неактивен. */
static void uwb_hard_reset(void)
{
  uwb_cs_high();
  HAL_GPIO_WritePin(UWB_RST_PORT, UWB_RST_PIN, GPIO_PIN_RESET);  /* RSTn -> 0: тянем вниз */
  HAL_Delay(UWB_RST_LOW_MS);
  HAL_GPIO_WritePin(UWB_RST_PORT, UWB_RST_PIN, GPIO_PIN_SET);    /* отпускаем: OD -> высокоимпеданс */
  HAL_Delay(UWB_RST_WAIT_MS);                                     /* ждём готовности SPI */
}

/* Чтение регистра: заголовок строит адаптер выбранного семейства, затем len байт данных с MISO.
 * CSn удерживается низким на ВСЮ транзакцию. buf[] — байты в порядке приёма (младший октет первым
 * у обоих поколений). */
static HAL_StatusTypeDef uwb_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t hdr[2] = { 0u, 0u };
  uint8_t hlen;
  HAL_StatusTypeDef st;

  hlen = UwbChip_BuildReadHeader(reg, hdr);

  uwb_cs_low();
  st = HAL_SPI_Transmit(&hspi4, hdr, (uint16_t)hlen, 100u);
  if (st == HAL_OK)
  {
    st = HAL_SPI_Receive(&hspi4, buf, len, 100u);   /* мастер тактирует SCK и читает MISO */
  }
  uwb_cs_high();
  return st;
}

/* Фактическая частота SCK: PCLK2 (APB2) делённая на делитель из BaudRatePrescaler (не константой). */
static uint32_t uwb_sck_hz(uint32_t *outDiv)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t idx   = (hspi4.Init.BaudRatePrescaler >> 3) & 0x7u;   /* 0..7 */
  uint32_t div   = 1u << (idx + 1u);                             /* 2..256 */
  if (outDiv != NULL) { *outDiv = div; }
  return pclk2 / div;
}

/* ================= Обработчики команд ================= */

/* devid — прочитать регистр идентификатора, напечатать сырые байты, значение и вердикт.
 * Вердикт сравнивает с семейством, под которое собрана прошивка (UwbChip_*). */
static void cmd_devid(int argc, char **argv)
{
  uint8_t b[4] = { 0u, 0u, 0u, 0u };
  HAL_StatusTypeDef st;
  uint32_t val;
  const char *variant = "";
  (void)argc; (void)argv;

  st = uwb_read_reg(UWB_REG_DEV_ID, b, 4u);
  if (st != HAL_OK)
  {
    Console_Printf("devid: SPI transfer error (HAL status=%d)\r\n", (int)st);
    return;
  }

  /* Октеты младшим вперёд: b[0]=младший ... b[3]=старший. */
  val = ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[0];

  Console_Printf("devid raw (bus order, low octet first): %02X %02X %02X %02X\r\n",
                 (unsigned)b[0], (unsigned)b[1], (unsigned)b[2], (unsigned)b[3]);
  Console_Printf("family=%s devid=0x%08lX (expected 0x%08lX)\r\n",
                 UwbChip_Family(), (unsigned long)val, (unsigned long)UwbChip_DevIdExpected());

  if (UwbChip_IsAlive(val, &variant) != 0u)
  {
    if ((variant != NULL) && (variant[0] != '\0'))
    {
      Console_Printf("verdict: OK - %s alive (%s), SPI link works.\r\n", UwbChip_Family(), variant);
    }
    else
    {
      Console_Printf("verdict: OK - %s alive, SPI link works.\r\n", UwbChip_Family());
    }
  }
  else if (val == UWB_DEADDEAD)
  {
    Console_Write("verdict: 0xDEADDEAD - SPI works but address is unused/reserved (wrong reg).\r\n");
  }
  else if (val == 0x00000000ul)
  {
    Console_Write("verdict: all zeros - no data on MISO (check MISO wire, power, reset, CSn).\r\n");
  }
  else if (val == 0xFFFFFFFFul)
  {
    Console_Write("verdict: all ones - MISO stuck high (miswire / module absent / no power).\r\n");
  }
  else
  {
    Console_Printf("verdict: unexpected value - not the %s id (check wiring, SCK, SPI mode 0).\r\n",
                   UwbChip_Family());
  }
}

/* dwreset — повторить процедуру аппаратного сброса модуля. */
static void cmd_dwreset(int argc, char **argv)
{
  (void)argc; (void)argv;
  Console_Write("dwreset: pulsing RSTn low, then releasing...\r\n");
  Console_Flush();                 /* сообщение должно уйти ДО блокирующих задержек */
  uwb_hard_reset();
  Console_Printf("dwreset: done (low %u ms, wait %u ms). Try 'devid'.\r\n",
                 (unsigned)UWB_RST_LOW_MS, (unsigned)UWB_RST_WAIT_MS);
}

/* spistat — фактическая частота SCK, режим CPOL/CPHA и потолок SPI для собранного семейства. */
static void cmd_spistat(int argc, char **argv)
{
  uint32_t div = 0u;
  uint32_t sck = uwb_sck_hz(&div);
  unsigned cpol = (hspi4.Init.CLKPolarity == SPI_POLARITY_HIGH) ? 1u : 0u;
  unsigned cpha = (hspi4.Init.CLKPhase    == SPI_PHASE_2EDGE)   ? 1u : 0u;
  (void)argc; (void)argv;

  Console_Printf("SPI4: PCLK2=%lu Hz / div=%lu -> SCK=%lu Hz (%lu.%03lu MHz)\r\n",
                 (unsigned long)HAL_RCC_GetPCLK2Freq(), (unsigned long)div,
                 (unsigned long)sck, (unsigned long)(sck / 1000000ul),
                 (unsigned long)((sck % 1000000ul) / 1000ul));
  Console_Printf("SPI4: CPOL=%u CPHA=%u (mode 0), MSB first, 8-bit, NSS software.\r\n", cpol, cpha);
  Console_Printf("family=%s: %s\r\n", UwbChip_Family(), UwbChip_SpiCeilingNote());
}

/* Таблица команд (help — встроенная в console.c). */
static const console_cmd_t k_cmds[] =
{
  { "devid",   "read DEV_ID and verify vs the built-for chip family", cmd_devid   },
  { "dwreset", "hardware-reset the UWB module via RSTn",              cmd_dwreset },
  { "spistat", "show actual SCK frequency and SPI mode/ceiling",     cmd_spistat },
};

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  HAL_StatusTypeDef st;

  BSP_LED_Init(LED4);   /* зелёный: признак «USB сконфигурирован» */
  BSP_LED_Init(LED5);   /* красный: ошибка */
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);

  /* Понизить SCK до <= 3 МГц: годится обоим поколениям (DW1000 INIT <= 3 МГц; DW3000 INIT_RC
   * <= 7 МГц). PCLK2 = APB2 = 96 МГц, делитель 64 -> 1.5 МГц (запас на длинный жгут).
   * Переинициализируем ТОЛЬКО экземпляр hspi4 в прикладном коде — .ioc и MX_SPI4_Init не трогаем. */
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  st = HAL_SPI_Init(&hspi4);

  Console_Init();
  Console_Register(k_cmds, (uint16_t)(sizeof(k_cmds) / sizeof(k_cmds[0])));

  if (st != HAL_OK)
  {
    BSP_LED_On(LED5);
    TRACE_ERR("LAB09 SPI4 reinit failed (HAL=%d)", (int)st);
  }

  /* Первичный аппаратный сброс, чтобы к первой команде модуль был в рабочем состоянии. */
  uwb_hard_reset();

  /* Баннер: в SWO сразу, в USB — как только хост откроет порт. Только ASCII. */
  {
    uint32_t div = 0u;
    uint32_t sck = uwb_sck_hz(&div);
    TRACE_LOG("LAB09 UWB (%s) devid check: SPI4 SCK=%lu Hz, mode 0",
              UwbChip_Family(), (unsigned long)sck);
  }
  Console_Printf("\r\nLAB09 UWB (%s) devid check ready. Type 'help'.\r\n", UwbChip_Family());
  Console_Printf("Run 'devid' - expect 0x%08lX.\r\n", (unsigned long)UwbChip_DevIdExpected());
  return 0u;
}

void Lab_Process(void)
{
  static uint32_t lastBlink = 0u;
  uint32_t now = HAL_GetTick();

  /* Разбор команд консоли + выталкивание вывода в USB. */
  Console_Process();

  /* Индикация USB: порт открыт хостом -> LED4 мигает ~2 Гц; иначе выключен. */
  if (Console_IsConfigured() != 0u)
  {
    if ((uint32_t)(now - lastBlink) >= 250u)
    {
      lastBlink = now;
      BSP_LED_Toggle(LED4);
    }
  }
  else
  {
    BSP_LED_Off(LED4);
  }
}

#endif /* LAB_ID == 9 */
