/**
  ******************************************************************************
  * @file    App/Labs/uwb_link.c
  * @author  Wagan Sarukhanov
  * @brief   LAB09, шаг 1 «Проверка монтажа и живости DWM1000»: аппаратный сброс
  *          модуля по RSTn и чтение регистра идентификатора DEV_ID (0x00) по SPI4.
  *          Радиообмена и речевого тракта здесь НЕТ — только контроль SPI-связи.
  *          Реализует Lab_Init/Lab_Process. Компилируется только при LAB_ID == 9.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * Обоснование метода (DW1000 User Manual v2.17, §7.2.2): регистр DEV_ID идеально
  * подходит хосту для проверки работоспособности SPI — прочитать его и сверить с
  * паспортным значением ПЕРЕД тем, как пользоваться микросхемой.
  *
  * Факты из документации (см. docs/PINOUT_dwm1000_lab09.md и REPORT_lab09_devid.md):
  *  - DEV_ID продукционного DW1000 = 0xDECA0130 (UM §7.2.2);
  *  - октеты выдаются младшим вперёд -> на шине 30 01 CA DE (UM §2.2.1.2, Figure 3);
  *  - зарезервированные/неиспользуемые регистры читаются как 0xDEADDEAD (UM §7.2);
  *  - заголовок чтения одиночного регистра: бит7=0 (read), бит6=0 (без суб-адреса),
  *    младшие 6 бит = адрес; для DEV_ID это 0x00 (UM §2.2.1.2, Figure 2);
  *  - режим SPI 0 (CPOL=0, CPHA=0, DWM1000 DS Figure 2); старший бит вперёд (DS §1.3);
  *    CSn низкий всю транзакцию;
  *  - потолок SPICLK пока CLKPLL не залочен (INIT, после сброса) <= 3 МГц, в IDLE до 20 МГц
  *    (DWM1000 DS Table 2; UM стр.15).
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 9

#include "console.h"
#include "trace_log.h"
#include "stm32f4xx_hal.h"
#include "stm32f411e_discovery.h"   /* светодиоды BSP_LED_* (индикация USB) */

/* Дескриптор SPI4, созданный CubeMX в main.c. */
extern SPI_HandleTypeDef hspi4;

/* ================= Выводы DWM1000 (docs/PINOUT_dwm1000_lab09.md) ================= */
#define DW_CSN_PORT    GPIOE
#define DW_CSN_PIN     GPIO_PIN_11   /* SPICSn: программный выбор кристалла, push-pull */
#define DW_RST_PORT    GPIOE
#define DW_RST_PIN     GPIO_PIN_10   /* RSTn: выход с ОТКРЫТЫМ СТОКОМ, только притяжка вниз */

/* ================= Тайминги аппаратного сброса =================
 * Источники: DWM1000 Data Sheet v1.8 §1.2 (стр.5) и DW1000 User Manual v2.17 (стр.14-15).
 *   - DWM1000 DS §1.2: "An external circuit can reset the DWM1000 by asserting RSTn for a
 *     minimum of 10 ns... RSTn should never be driven high by an external source" -> минимум
 *     импульса низкого уровня 10 нс; отпускать в high-Z (наш open-drain это и обеспечивает);
 *   - после отпускания RSTn модуль входит в INIT ~за 4 мс, INIT->IDLE +5 мкс (UM Table 1);
 *     в INIT уже допустим SPI <= 3 МГц (DWM1000 DS Table 2 / UM стр.15).
 * HAL_Delay работает в миллисекундах, поэтому держим низким 2 мс (>> минимума 10 нс, запас на
 * дребезг/шум жгута) и ждём 5 мс (> 4 мс до INIT). Обе величины легко изменить здесь. */
#define DW_RST_LOW_MS    2u    /* удержание RSTn низким: минимум по DS 10 нс, держим 2 мс с запасом */
#define DW_RST_WAIT_MS   5u    /* ожидание после отпускания RSTn: > ~4 мс до INIT (UM Table 1) */

/* ================= Регистр и эталоны ================= */
#define DW_REG_DEV_ID       0x00u
#define DW_DEVID_EXPECTED   0xDECA0130u
#define DW_DEADDEAD         0xDEADDEADu

/* ================= Низкоуровневые операции ================= */

static inline void dw_cs_low(void)  { HAL_GPIO_WritePin(DW_CSN_PORT, DW_CSN_PIN, GPIO_PIN_RESET); }
static inline void dw_cs_high(void) { HAL_GPIO_WritePin(DW_CSN_PORT, DW_CSN_PIN, GPIO_PIN_SET);   }

/* Аппаратный сброс модуля по RSTn. Вывод — открытый сток: запись 0 тянет линию вниз
 * (активный сброс), запись 1 отпускает её в высокоимпедансное состояние (высокий уровень
 * извне НЕ подаём — этого требует даташит модуля, см. PINOUT). CSn на время сброса неактивен. */
static void dw_hard_reset(void)
{
  dw_cs_high();
  HAL_GPIO_WritePin(DW_RST_PORT, DW_RST_PIN, GPIO_PIN_RESET);  /* RSTn -> 0: тянем вниз */
  HAL_Delay(DW_RST_LOW_MS);
  HAL_GPIO_WritePin(DW_RST_PORT, DW_RST_PIN, GPIO_PIN_SET);    /* отпускаем: OD -> высокоимпеданс */
  HAL_Delay(DW_RST_WAIT_MS);                                    /* ждём выхода в INIT/IDLE */
}

/* Чтение регистра без суб-адреса: один заголовочный октет, затем len байт данных с MISO.
 * CSn удерживается низким на ВСЮ транзакцию (иначе она обрывается). buf[] — байты в
 * порядке приёма с шины (младший октет значения первым). */
static HAL_StatusTypeDef dw_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t hdr = (uint8_t)(reg & 0x3Fu);   /* бит7=0 read, бит6=0 без суб-индекса */
  HAL_StatusTypeDef st;

  dw_cs_low();
  st = HAL_SPI_Transmit(&hspi4, &hdr, 1u, 100u);
  if (st == HAL_OK)
  {
    st = HAL_SPI_Receive(&hspi4, buf, len, 100u);   /* мастер тактирует SCK и читает MISO */
  }
  dw_cs_high();
  return st;
}

/* Фактическая частота SCK: PCLK2 (APB2) делённая на делитель SPI из BaudRatePrescaler,
 * а не константой. BaudRatePrescaler кодирует делитель битами BR[2:0] в позиции 3..5. */
static uint32_t dw_sck_hz(uint32_t *outDiv)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t idx   = (hspi4.Init.BaudRatePrescaler >> 3) & 0x7u;   /* 0..7 */
  uint32_t div   = 1u << (idx + 1u);                             /* 2..256 */
  if (outDiv != NULL) { *outDiv = div; }
  return pclk2 / div;
}

/* ================= Обработчики команд ================= */

/* devid — прочитать регистр 0x00, напечатать сырые байты, собранное значение и вердикт. */
static void cmd_devid(int argc, char **argv)
{
  uint8_t b[4] = { 0u, 0u, 0u, 0u };
  HAL_StatusTypeDef st;
  uint32_t val;
  (void)argc; (void)argv;

  st = dw_read_reg(DW_REG_DEV_ID, b, 4u);
  if (st != HAL_OK)
  {
    Console_Printf("devid: SPI transfer error (HAL status=%d)\r\n", (int)st);
    return;
  }

  /* Октеты младшим вперёд: b[0]=0x30 ... b[3]=0xDE  ->  0xDECA0130. */
  val = ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[0];

  Console_Printf("devid raw (bus order, low octet first): %02X %02X %02X %02X\r\n",
                 (unsigned)b[0], (unsigned)b[1], (unsigned)b[2], (unsigned)b[3]);
  Console_Printf("devid = 0x%08lX  (expected 0x%08lX)\r\n",
                 (unsigned long)val, (unsigned long)DW_DEVID_EXPECTED);

  if (val == DW_DEVID_EXPECTED)
  {
    Console_Write("verdict: OK - DW1000 alive, SPI link works.\r\n");
  }
  else if (val == DW_DEADDEAD)
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
    Console_Write("verdict: unexpected value - garbage (check wiring, SCK<=3MHz, SPI mode 0).\r\n");
  }
}

/* dwreset — повторить процедуру аппаратного сброса модуля. */
static void cmd_dwreset(int argc, char **argv)
{
  (void)argc; (void)argv;
  Console_Write("dwreset: pulsing RSTn low, then releasing...\r\n");
  Console_Flush();                 /* сообщение должно уйти ДО блокирующих задержек */
  dw_hard_reset();
  Console_Printf("dwreset: done (low %u ms, wait %u ms). Try 'devid'.\r\n",
                 (unsigned)DW_RST_LOW_MS, (unsigned)DW_RST_WAIT_MS);
}

/* spistat — фактическая частота SCK (вычислена из шины и делителя) и режим CPOL/CPHA. */
static void cmd_spistat(int argc, char **argv)
{
  uint32_t div = 0u;
  uint32_t sck = dw_sck_hz(&div);
  unsigned cpol = (hspi4.Init.CLKPolarity == SPI_POLARITY_HIGH) ? 1u : 0u;
  unsigned cpha = (hspi4.Init.CLKPhase    == SPI_PHASE_2EDGE)   ? 1u : 0u;
  (void)argc; (void)argv;

  Console_Printf("SPI4: PCLK2=%lu Hz / div=%lu -> SCK=%lu Hz (%lu.%03lu MHz)\r\n",
                 (unsigned long)HAL_RCC_GetPCLK2Freq(), (unsigned long)div,
                 (unsigned long)sck, (unsigned long)(sck / 1000000ul),
                 (unsigned long)((sck % 1000000ul) / 1000ul));
  Console_Printf("SPI4: CPOL=%u CPHA=%u (mode 0), MSB first, 8-bit, NSS software.\r\n", cpol, cpha);
  Console_Write("note: INIT state ceiling is 3 MHz (DW1000 UM Table 1).\r\n");
}

/* Таблица команд (help — встроенная в console.c). */
static const console_cmd_t k_cmds[] =
{
  { "devid",   "read DEV_ID (0x00) and verify vs 0xDECA0130", cmd_devid   },
  { "dwreset", "hardware-reset the DWM1000 via RSTn",         cmd_dwreset },
  { "spistat", "show actual SCK frequency and SPI mode",      cmd_spistat },
};

/* ================= ИНТЕРФЕЙС ЛАБОРАТОРНОЙ ================= */
uint8_t Lab_Init(void)
{
  HAL_StatusTypeDef st;

  BSP_LED_Init(LED4);   /* зелёный: признак «USB сконфигурирован» */
  BSP_LED_Init(LED5);   /* красный: ошибка */
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);

  /* Понизить SCK до <= 3 МГц (потолок INIT, UM Table 1). PCLK2 = APB2 = 96 МГц,
   * делитель 64 -> 1.5 МГц (с запасом на длинный жгут переходной платы).
   * Переинициализируем ТОЛЬКО экземпляр hspi4 в прикладном коде — walkie-talkie.ioc и
   * MX_SPI4_Init не трогаем (аналогично тому, как pintest переинициализирует UART). */
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  st = HAL_SPI_Init(&hspi4);

  Console_Init();
  Console_Register(k_cmds, (uint16_t)(sizeof(k_cmds) / sizeof(k_cmds[0])));

  if (st != HAL_OK)
  {
    BSP_LED_On(LED5);
    TRACE_ERR("LAB09 SPI4 reinit failed (HAL=%d)", (int)st);
  }

  /* Первичный аппаратный сброс модуля, чтобы к первой команде он был в рабочем состоянии. */
  dw_hard_reset();

  /* Баннер: в SWO сразу, в USB — как только хост откроет порт. Только ASCII. */
  {
    uint32_t div = 0u;
    uint32_t sck = dw_sck_hz(&div);
    TRACE_LOG("LAB09 UWB DWM1000 devid check: SPI4 SCK=%lu Hz (<=3MHz), mode 0",
              (unsigned long)sck);
  }
  Console_Write("\r\nLAB09 UWB DWM1000 devid check ready. Type 'help'.\r\n");
  Console_Write("Run 'devid' - expect 0xDECA0130.\r\n");
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
