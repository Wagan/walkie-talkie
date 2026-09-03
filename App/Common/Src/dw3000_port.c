/**
  ******************************************************************************
  * @file    App/Common/Src/dw3000_port.c
  * @author  Wagan Sarukhanov
  * @brief   LAB09: платформенный слой Qorvo dwt_uwb_driver под STM32F411/SPI4. См. dw3000_port.h.
  *
  * Copyright (c) 1991-2026 NCPR LLC (Flexlab). All rights reserved.
  ******************************************************************************
  * НАШ код: реализует функции, которые драйвер Qorvo ожидает от платформы, и вызывает его
  * публичный API (dwt_probe/dwt_readdevid). Драйвер лежит в ThirdParty/dw3000_driver
  * (собран в libdw3000_cm4.a). Компилируется только при LAB_ID==9 && UWB_CHIP_DW3000.
  *
  * Интерфейс SPI драйвера (struct dwt_spi_s): readfromspi/writetospi/writetospiwithcrc +
  * setslowrate/setfastrate. Плюс глобальные deca_sleep/deca_usleep/decamutexon/decamutexoff,
  * которые драйвер зовёт напрямую. wakeup_device_with_io передаётся через probe.
  ******************************************************************************
  */

#include "lab.h"

#if LAB_ID == 9
#include "uwb_config.h"

#if defined(UWB_CHIP_DW3000)

#include "dw3000_port.h"
#include "stm32f4xx_hal.h"
#define CONFIG_DW3000_CHIP_DW3000        /* согласовано с сборкой libdw3000_cm4.a */
#include "deca_device_api.h"
#include "deca_interface.h"
#include "trace_log.h"
#include "voice.h"          /* Voice_IsPtt(): взводить приёмник в конце uwbinit, если не держим PTT */
#include "stm32f411e_discovery.h"  /* BSP_LED_* : LED4 готовность, LED5 отказ автозапуска */
#include <string.h>
#include <stdlib.h>

/* Дескриптор SPI4 (CubeMX, main.c) и выводы модуля — те же, что в uwb_core.c (проверены железом). */
extern SPI_HandleTypeDef hspi4;
#define DW_CSN_PORT   GPIOE
#define DW_CSN_PIN    GPIO_PIN_11

/* Дескриптор драйвера DW3000 определён в libdw3000_cm4.a (dw3000_device.c). Объявляем сами:
 * штатный extern в deca_device_api.h спрятан за USE_DRV_DW3000. */
extern const struct dwt_driver_s dw3000_driver;

/* ================= Задержки ================= */
/* Включить счётчик тактов DWT для мкс-задержек драйвера (deca_usleep). CYCCNT здесь НЕ обнуляем:
 * это ОБЩИЙ на весь проект свободнобегущий счётчик, его читают probeA (audio.c), loop_max/enc/dec
 * (voice.c), мкс-задержки и разворот RS-485 (uart_port.c), c2load. Функция зовётся не только на
 * старте, но и на КАЖДОМ uwbinit (через Dw3000Port_Probe при s_probed=0). Обнуление в середине
 * прогона делало висящую беззнаковую разность `now - prev` заворотом почти на полный оборот
 * (~44.7 с при 96 МГц) -> probeA max и loop_max показывали десятки секунд (дефект A,
 * TASK_lab09_diag_fixes §2). deca_usleep пользуется ОТНОСИТЕЛЬНОЙ разностью (CYCCNT - start),
 * которая переживает заворот сама и не зависит от абсолютного значения, поэтому обнуление не нужно.
 * Однократное включение+обнуление на старте делает Voice_Init (voice.c). |= идемпотентно. */
static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void deca_sleep(unsigned int time_ms)
{
  HAL_Delay((uint32_t)time_ms);
}

void deca_usleep(unsigned long time_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock / 1000000u) * (uint32_t)time_us;
  while ((DWT->CYCCNT - start) < ticks) { }
}

/* ================= Критическая секция (без RTOS) ================= */
decaIrqStatus_t decamutexon(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return (decaIrqStatus_t)primask;         /* 0 = прерывания были разрешены */
}

void decamutexoff(decaIrqStatus_t s)
{
  if (s == 0) { __enable_irq(); }
}

/* ================= SPI-обмен ================= */
static inline void cs_low(void)  { HAL_GPIO_WritePin(DW_CSN_PORT, DW_CSN_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(DW_CSN_PORT, DW_CSN_PIN, GPIO_PIN_SET);   }

/* CSn низкий на всю транзакцию: сначала заголовок, затем данные (чтение или запись). */
static int32_t spi_read(uint16_t hlen, uint8_t *hbuf, uint16_t rlen, uint8_t *rbuf)
{
  HAL_StatusTypeDef st;
  cs_low();
  st = HAL_SPI_Transmit(&hspi4, hbuf, hlen, 100u);
  if ((st == HAL_OK) && (rlen != 0u)) { st = HAL_SPI_Receive(&hspi4, rbuf, rlen, 100u); }
  cs_high();
  return (st == HAL_OK) ? (int32_t)DWT_SUCCESS : (int32_t)DWT_ERROR;
}

static int32_t spi_write(uint16_t hlen, const uint8_t *hbuf, uint16_t blen, const uint8_t *bbuf)
{
  HAL_StatusTypeDef st;
  cs_low();
  st = HAL_SPI_Transmit(&hspi4, (uint8_t *)hbuf, hlen, 100u);
  if ((st == HAL_OK) && (blen != 0u)) { st = HAL_SPI_Transmit(&hspi4, (uint8_t *)bbuf, blen, 100u); }
  cs_high();
  return (st == HAL_OK) ? (int32_t)DWT_SUCCESS : (int32_t)DWT_ERROR;
}

static int32_t spi_write_crc(uint16_t hlen, const uint8_t *hbuf, uint16_t blen, const uint8_t *bbuf, uint8_t crc8)
{
  /* SPI-CRC режим не используем; полная реализация на случай вызова драйвером. */
  HAL_StatusTypeDef st;
  cs_low();
  st = HAL_SPI_Transmit(&hspi4, (uint8_t *)hbuf, hlen, 100u);
  if ((st == HAL_OK) && (blen != 0u)) { st = HAL_SPI_Transmit(&hspi4, (uint8_t *)bbuf, blen, 100u); }
  if (st == HAL_OK) { st = HAL_SPI_Transmit(&hspi4, &crc8, 1u, 100u); }
  cs_high();
  return (st == HAL_OK) ? (int32_t)DWT_SUCCESS : (int32_t)DWT_ERROR;
}

/* Смена скорости SPI. Драйвер зовёт setslowrate ДО и setfastrate ПОСЛЕ выхода в рабочее
 * состояние: в INIT_RC (сразу после сброса) SPICLK <= 7 МГц, в IDLE_PLL допустимо до 38 МГц
 * (DW3000 UM Table 4). PCLK2=APB2=96 МГц. slow: /64 = 1.5 МГц; fast: /8 = 12 МГц (консервативно,
 * с запасом на жгут переходной платы; на шаге devid НЕ достигается — обмен остаётся slow). */
static void spi_set_prescaler(uint32_t presc)
{
  hspi4.Init.BaudRatePrescaler = presc;
  (void)HAL_SPI_Init(&hspi4);
}
static void spi_slow(void) { spi_set_prescaler(SPI_BAUDRATEPRESCALER_64); }
static void spi_fast(void) { spi_set_prescaler(SPI_BAUDRATEPRESCALER_16); }  /* 6 МГц: см. §5 отчёта */

/* Пробуждение по IO: на шаге devid режимы сна не используются, модуль держится активным
 * (сброс делает uwb_core в Lab_Init) — пустая операция. */
static void port_wakeup(void) { }

/* Таблица SPI-функций для драйвера. */
static const struct dwt_spi_s s_spi =
{
  .readfromspi       = spi_read,
  .writetospi        = spi_write,
  .writetospiwithcrc = spi_write_crc,
  .setslowrate       = spi_slow,
  .setfastrate       = spi_fast,
};

/* ================= Прикладные обёртки ================= */
static uint8_t s_probed = 0u;

void Dw3000Port_Init(void)
{
  dwt_delay_init();
  s_probed = 0u;
}

int Dw3000Port_Probe(void)
{
  static struct dwt_driver_s *drv_list[1];
  struct dwt_probe_s probe;

  if (s_probed != 0u) { return 0; }

  dwt_delay_init();                                  /* мкс-задержки могут понадобиться драйверу */
  drv_list[0] = (struct dwt_driver_s *)&dw3000_driver;
  (void)memset(&probe, 0, sizeof(probe));
  probe.dw                   = NULL;                 /* использовать внутреннюю структуру драйвера */
  probe.spi                  = (void *)&s_spi;
  probe.wakeup_device_with_io = port_wakeup;
  probe.driver_list          = drv_list;
  probe.dw_driver_num        = 1u;

  if (dwt_probe(&probe) != (int32_t)DWT_SUCCESS) { return -1; }
  s_probed = 1u;
  return 0;
}

uint8_t Dw3000Port_ReadDevId(uint32_t *out)
{
  if (out == NULL) { return 0u; }
  if (Dw3000Port_Probe() != 0) { return 0u; }
  *out = dwt_readdevid();
  return 1u;
}

/* =====================================================================================
 *  Радио LAB09 (первый кадр): полная инициализация + TX/RX одного кадра опросом.
 *  Через публичный API драйвера Qorvo. Речевого тракта здесь нет. Роли — командами.
 * =====================================================================================*/

/* Умолчания канала (см. REPORT_lab09_firstframe.md §2; обоснование по DW3000 UM):
 * chan 5 (нижний из двух, лучше проходит), 850 кбит/с (больше энергии на бит -> надёжнее приём),
 * преамбула 128 + PAC8 (рекоменд. для plen<=128), код 9/9 (PRF 64 МГц, канал 5), SFD DW-8,
 * PHR standard, STS off, PDOA off. sfdTO = plen + 1 + SFD(8) - PAC(8) = 129. */
static dwt_config_t s_cfg =
{
  5,                 /* chan */
  DWT_PLEN_128,      /* txPreambLength */
  DWT_PAC8,          /* rxPAC */
  9,                 /* txCode (PRF 64) */
  9,                 /* rxCode */
  1,                 /* sfdType: DW 8-bit */
  DWT_BR_850K,       /* dataRate */
  DWT_PHRMODE_STD,   /* phrMode */
  DWT_PHRRATE_STD,   /* phrRate */
  129,               /* sfdTO */
  DWT_STS_MODE_OFF,  /* stsMode */
  DWT_STS_LEN_64,    /* stsLength (не используется при STS off) */
  DWT_PDOA_M0        /* pdoaMode */
};

static uint8_t  s_inited   = 0u;
static uint8_t  s_rx_active = 0u;
static uint8_t  s_tx_seq   = 0u;

/* Автозапуск радио при старте (полевой режим, TASK_lab09_autoinit2). Значения — предложение,
 * к утверждению владельцем; меняются одной строкой каждое. Индикация без консоли (LED по решению
 * владельца): готовность — вспышки LED4 (зелёный), отказ — редкое мигание LED5 (красный). */
#define AUTOINIT_TRIES     3u     /* попыток uwbinit при холодном старте (процедура начинается со сброса -> повтор безопасен) */
#define AUTOINIT_RETRY_MS  200u   /* пауза между попытками, мс */
#define READY_FLASH_N      3u     /* число вспышек готовности */
#define READY_FLASH_MS     100u   /* длительность вспышки/паузы готовности, мс */
#define FAIL_BLINK_MS      500u   /* полупериод мигания отказа, мс (~1 Гц) */

static uint8_t  s_autoFail = 0u;  /* автозапуск исчерпал попытки: мигать LED5 из главного цикла */

/* Счётчики. */
static uint32_t cnt_tx = 0u, cnt_rx = 0u, cnt_crc = 0u, cnt_phe = 0u, cnt_to = 0u;
static uint32_t cnt_switch = 0u;    /* переключений TX->RX (реальный разворот полудуплекса) */
static uint32_t cnt_armed  = 0u;    /* первичных взводов приёмника (конец uwbinit; НЕ TX->RX) */
static uint8_t  s_rx_armed = 0u;    /* приёмник взведён (для голосового опроса) */

/* Тест-кадр: маркер "WT" + порядковый номер + заполнение 0xA5. */
#define TF_LEN     12u
#define TF_FILL    0xA5u
static uint8_t s_txbuf[TF_LEN];
static uint8_t s_rxbuf[128];   /* вмещает голосовой payload (Codec2 малый; raw 8к ~84 Б) */

/* Наборы масок статуса приёма. */
#define RX_ERR_MASK  ((uint32_t)DWT_INT_RXPHE_BIT_MASK | (uint32_t)DWT_INT_RXFCE_BIT_MASK | \
                      (uint32_t)DWT_INT_RXFSL_BIT_MASK)
#define RX_TO_MASK   ((uint32_t)DWT_INT_RXFTO_BIT_MASK | (uint32_t)DWT_INT_RXPTO_BIT_MASK | \
                      (uint32_t)DWT_INT_RXSTO_BIT_MASK)

static const char *rate_str(void) { return (s_cfg.dataRate == DWT_BR_6M8) ? "6M8" : "850k"; }

/* Взвести приёмник (сброс TRX -> очистка статуса -> включить RX, без таймаута = непрерывно). */
static void rx_arm(void)
{
  dwt_forcetrxoff();
  dwt_writesysstatuslo(RX_ERR_MASK | RX_TO_MASK | (uint32_t)DWT_INT_RXFCG_BIT_MASK |
                       (uint32_t)DWT_INT_RXFR_BIT_MASK);
  (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* ---- Полная инициализация с проверкой каждого этапа ---- */
static int radio_init(void)
{
  uint32_t t0;

  s_inited   = 0u;
  s_rx_armed = 0u;             /* сброс чипа снимает взвод приёмника — отразим в нашем состоянии */

  /* Пункт 1: аппаратный сброс модуля ПЕРВЫМ шагом (тот же RSTn-механизм, что команда dwreset).
   * Без него повторный uwbinit застревает на ожидании IDLE_RC: после configure чип в IDLE_PLL,
   * событие RCINIT в SYS_STATUS больше не выставляется, dwt_checkidlerc() не проходит (таймаут).
   * Сброс возвращает INIT_RC->IDLE_RC и заново выставляет RCINIT. См. REPORT_lab09_init_fixes §3. */
  Console_Write("uwbinit: hard reset (RSTn)...\r\n");
  Console_Flush();
  Uwb_HardReset();
  spi_slow();                  /* INIT_RC после сброса: SCK <= 7 МГц до dwt_initialise (UM Table 4) */
  s_probed = 0u;               /* заново выбрать драйвер чтением DEV_ID на свежесброшенном чипе */

  Console_Write("uwbinit: probe (DEV_ID)...\r\n");
  Console_Flush();
  if (Dw3000Port_Probe() != 0)
  {
    Console_Write("uwbinit: FAIL - probe (no DEV_ID over SPI)\r\n");
    return -1;
  }

  /* Дождаться IDLE_RC (готовность после сброса, ~единицы мс). */
  t0 = HAL_GetTick();
  while (dwt_checkidlerc() == 0u)
  {
    if ((uint32_t)(HAL_GetTick() - t0) > 10u)
    {
      Console_Write("uwbinit: FAIL - IDLE_RC not reached (timeout)\r\n");
      return -1;
    }
  }
  Console_Write("uwbinit: IDLE_RC ok\r\n");

  /* Загрузка калибровок из OTP; здесь драйвер переводит SPI на fast rate. */
  if (dwt_initialise(DWT_DW_INIT) == (int32_t)DWT_ERROR)
  {
    Console_Write("uwbinit: FAIL - dwt_initialise (OTP/PLL)\r\n");
    return -1;
  }
  Console_Write("uwbinit: initialise ok (OTP loaded, SPI->fast 6 MHz)\r\n");

  /* Конфигурация канала: включает захват PLL (IDLE_PLL) и калибровку приёмника (PGF). */
  if (dwt_configure(&s_cfg) != (int32_t)DWT_SUCCESS)
  {
    Console_Write("uwbinit: FAIL - dwt_configure (PLL lock / PGF cal)\r\n");
    return -1;
  }
  Console_Write("uwbinit: configure ok (IDLE_PLL, channel/PGF)\r\n");

  dwt_setrxtimeout(0u);           /* приём без таймаута (ждём кадр непрерывно) */
  s_inited = 1u;

  /* Пункт 2: взвести приёмник по окончании инициализации, чтобы плата, ни разу не передававшая,
   * уже слышала встречную (раньше приём включался только на фронте PTT 1->0). Диагностический путь
   * uwbrx имеет свой флаг s_rx_active; голосовой путь взводим, только если оператор не держит PTT
   * (режим передачи приоритетнее — там взвод произойдёт штатно на отпускании PTT). Самовзвод в
   * Dw3000Port_VoicePoll остаётся страховкой. */
  if (s_rx_active != 0u) { rx_arm(); }
  else if (Voice_IsPtt() == 0u) { rx_arm(); s_rx_armed = 1u; cnt_armed++; }  /* первичный взвод: не TX->RX */

  /* Успешная инициализация (авто или ручная) снимает мигание отказа автозапуска. Гасим LED5 только
   * если сами его зажигали — иначе затёрли бы индикацию ошибки аудио (LED5 общий, Audio_OnError). */
  if (s_autoFail != 0u) { s_autoFail = 0u; BSP_LED_Off(LED5); }

  Console_Printf("uwbinit: DONE - chan=%u rate=%s plen=128 pac=8 code=9 (PRF64) sfd=DW8\r\n",
                 (unsigned)s_cfg.chan, rate_str());
  return 0;
}

/* ---- Автозапуск радио при старте (полевой режим, без консоли) ---- */

/* Серия вспышек готовности на LED4 (зелёный). Зовётся ДО старта аудио, поэтому блокирующая
 * HAL_Delay безопасна (дедлайна аудиовыхода ещё нет). После серии LED4 погашен — далее он штатно
 * работает индикатором приёма. */
static void ready_flash(void)
{
  uint8_t i;
  for (i = 0u; i < READY_FLASH_N; i++)
  {
    BSP_LED_On(LED4);  HAL_Delay(READY_FLASH_MS);
    BSP_LED_Off(LED4); HAL_Delay(READY_FLASH_MS);
  }
}

/* Автоматически поднять радио при старте: та же процедура, что uwbinit, до AUTOINIT_TRIES попыток
 * с паузой AUTOINIT_RETRY_MS. Приёмник взводится внутри radio_init (если не держим PTT) -> первичный
 * взвод идёт в armed (не switch). Ход печатается в консоль как у ручного вызова. Вызывается из
 * Voice_Init ДО Audio_Init: блокирующие выдержки (сброс/IDLE_RC) не задевают дедлайн аудиовыхода. */
void Dw3000Port_AutoInit(void)
{
  uint8_t attempt;
  int rc = -1;

  Console_Write("\r\nuwb autoinit: bringing up radio (field mode)...\r\n");
  Console_Flush();
  for (attempt = 0u; attempt < AUTOINIT_TRIES; attempt++)
  {
    Console_Printf("uwb autoinit: attempt %u/%u\r\n", (unsigned)(attempt + 1u), (unsigned)AUTOINIT_TRIES);
    Console_Flush();
    rc = radio_init();
    if (rc == 0) { break; }
    if ((uint8_t)(attempt + 1u) < AUTOINIT_TRIES) { HAL_Delay(AUTOINIT_RETRY_MS); }
  }

  if (rc == 0)
  {
    s_autoFail = 0u;
    Console_Write("uwb autoinit: OK - receiver armed, ready.\r\n");
    ready_flash();                 /* готовность: вспышки LED4 (зелёный), до старта аудио */
  }
  else
  {
    s_autoFail = 1u;               /* отказ: редкое мигание LED5 (красный) из главного цикла */
    BSP_LED_Off(LED4);
    Console_Write("uwb autoinit: FAILED after all attempts - red LED blinking. Try 'uwbinit'.\r\n");
  }
  Console_Flush();
}

/* Мигание индикатора отказа автозапуска (LED5, красный, ~1 Гц) из главного цикла. Без блокировок,
 * по HAL_GetTick. Ничего не делает, пока s_autoFail==0 (LED5 тогда за Audio_OnError). Успешный
 * ручной uwbinit снимает s_autoFail и гасит LED5 (см. radio_init). */
void Dw3000Port_FailBlinkPoll(void)
{
  static uint32_t last = 0u;
  static uint8_t  on   = 0u;
  uint32_t now;

  if (s_autoFail == 0u) { return; }
  now = HAL_GetTick();
  if ((uint32_t)(now - last) >= FAIL_BLINK_MS)
  {
    last = now;
    on = (uint8_t)(on == 0u);
    if (on != 0u) { BSP_LED_On(LED5); } else { BSP_LED_Off(LED5); }
  }
}

/* ---- Команды ---- */
static void cmd_uwbinit(int argc, char **argv)
{
  (void)argc; (void)argv;
  (void)radio_init();
}

static void cmd_uwbcfg(int argc, char **argv)
{
  if (argc >= 3)
  {
    if (strcmp(argv[1], "chan") == 0)
    {
      unsigned long c = strtoul(argv[2], NULL, 0);
      if ((c == 5ul) || (c == 9ul)) { s_cfg.chan = (uint8_t)c; s_inited = 0u; }
      else { Console_Write("uwbcfg: chan must be 5 or 9\r\n"); return; }
    }
    else if (strcmp(argv[1], "rate") == 0)
    {
      unsigned long r = strtoul(argv[2], NULL, 0);
      if (r == 850ul)       { s_cfg.dataRate = DWT_BR_850K; s_inited = 0u; }
      else if (r == 6800ul) { s_cfg.dataRate = DWT_BR_6M8;  s_inited = 0u; }
      else { Console_Write("uwbcfg: rate must be 850 or 6800\r\n"); return; }
    }
    else { Console_Write("uwbcfg: usage: uwbcfg [chan 5|9] [rate 850|6800]\r\n"); return; }
    Console_Write("uwbcfg: set - run 'uwbinit' to apply\r\n");
  }
  Console_Printf("uwbcfg: chan=%u rate=%s plen=128 pac=8 txcode=%u rxcode=%u sfd=DW8 sts=off pdoa=off (inited=%u)\r\n",
                 (unsigned)s_cfg.chan, rate_str(), (unsigned)s_cfg.txCode, (unsigned)s_cfg.rxCode,
                 (unsigned)s_inited);
}

static void cmd_uwbtx(int argc, char **argv)
{
  unsigned long n = 1ul;
  unsigned long i;
  uint32_t confirmed = 0u;

  if (s_inited == 0u) { Console_Write("uwbtx: run 'uwbinit' first\r\n"); return; }
  if (argc > 1) { n = strtoul(argv[1], NULL, 0); }
  if (n == 0ul) { n = 1ul; }
  if (n > 1000ul) { n = 1000ul; }

  for (i = 0ul; i < n; i++)
  {
    uint32_t t0;
    s_txbuf[0] = (uint8_t)'W';
    s_txbuf[1] = (uint8_t)'T';
    s_txbuf[2] = s_tx_seq++;
    (void)memset(&s_txbuf[3], TF_FILL, TF_LEN - 3u);

    dwt_forcetrxoff();
    dwt_writesysstatuslo((uint32_t)DWT_INT_TXFRS_BIT_MASK);
    (void)dwt_writetxdata(TF_LEN, s_txbuf, 0u);
    dwt_writetxfctrl((uint16_t)(TF_LEN + 2u), 0u, 0u);   /* +2 = автодобавляемый FCS */

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != (int32_t)DWT_SUCCESS) { continue; }

    t0 = HAL_GetTick();
    while (((dwt_readsysstatuslo() & (uint32_t)DWT_INT_TXFRS_BIT_MASK) == 0u) &&
           ((uint32_t)(HAL_GetTick() - t0) < 10u)) { }
    if ((dwt_readsysstatuslo() & (uint32_t)DWT_INT_TXFRS_BIT_MASK) != 0u)
    {
      dwt_writesysstatuslo((uint32_t)DWT_INT_TXFRS_BIT_MASK);
      confirmed++;
      cnt_tx++;
    }
  }
  Console_Printf("uwbtx: %lu/%lu confirmed (TXFRS)\r\n", (unsigned long)confirmed, n);
  if (s_rx_active != 0u) { rx_arm(); }        /* вернуть приёмник, если он был включён */
}

static void cmd_uwbrx(int argc, char **argv)
{
  if (argc < 2)
  {
    Console_Printf("uwbrx: %s (usage: uwbrx on|off)\r\n", (s_rx_active != 0u) ? "on" : "off");
    return;
  }
  if (strcmp(argv[1], "on") == 0)
  {
    if (s_inited == 0u) { Console_Write("uwbrx: run 'uwbinit' first\r\n"); return; }
    s_rx_active = 1u;
    rx_arm();
    Console_Write("uwbrx: on (polling)\r\n");
  }
  else if (strcmp(argv[1], "off") == 0)
  {
    s_rx_active = 0u;
    dwt_forcetrxoff();
    Console_Write("uwbrx: off\r\n");
  }
  else { Console_Write("uwbrx: usage: uwbrx on|off\r\n"); }
}

static void cmd_uwbstat(int argc, char **argv)
{
  if ((argc > 1) && (strcmp(argv[1], "reset") == 0))
  {
    cnt_tx = 0u; cnt_rx = 0u; cnt_crc = 0u; cnt_phe = 0u; cnt_to = 0u;
    Console_Write("uwbstat: counters reset\r\n");
    return;
  }
  Console_Printf("uwbstat: tx=%lu rx=%lu crcErr=%lu phrErr=%lu rxTimeout=%lu | inited=%u rxState=%s armed=%lu switch=%lu\r\n",
                 (unsigned long)cnt_tx, (unsigned long)cnt_rx, (unsigned long)cnt_crc,
                 (unsigned long)cnt_phe, (unsigned long)cnt_to,
                 (unsigned)s_inited, Dw3000Port_RxStateStr(),
                 (unsigned long)cnt_armed, (unsigned long)cnt_switch);
}

static const console_cmd_t s_cmds[] =
{
  { "uwbinit", "full DW3000 init (probe/IDLE_RC/OTP/PLL/PGF)", cmd_uwbinit },
  { "uwbcfg",  "show/set channel params (chan 5|9, rate 850|6800)", cmd_uwbcfg },
  { "uwbtx",   "transmit [n] test frames (default 1)",         cmd_uwbtx   },
  { "uwbrx",   "receiver on|off (polled)",                     cmd_uwbrx   },
  { "uwbstat", "radio counters (uwbstat reset to clear)",      cmd_uwbstat },
};

const console_cmd_t *Dw3000Port_Cmds(uint16_t *count)
{
  if (count != NULL) { *count = (uint16_t)(sizeof(s_cmds) / sizeof(s_cmds[0])); }
  return s_cmds;
}

/* Опрос приёма из главного цикла (Lab_Process). Без прерывания: EXTI по IRQ (PE9) не включён. */
void Dw3000Port_Poll(void)
{
  uint32_t status;

  if ((s_inited == 0u) || (s_rx_active == 0u)) { return; }

  status = dwt_readsysstatuslo();

  if ((status & (uint32_t)DWT_INT_RXFCG_BIT_MASK) != 0u)
  {
    uint8_t  rng   = 0u;
    uint16_t flen  = dwt_getframelength(&rng);
    uint16_t plen  = (flen > 2u) ? (uint16_t)(flen - 2u) : 0u;   /* минус 2 байта FCS */
    uint16_t j;
    uint8_t  match;

    if (plen > (uint16_t)sizeof(s_rxbuf)) { plen = (uint16_t)sizeof(s_rxbuf); }
    dwt_readrxdata(s_rxbuf, plen, 0u);
    dwt_writesysstatuslo((uint32_t)DWT_INT_RXFCG_BIT_MASK | (uint32_t)DWT_INT_RXFR_BIT_MASK);
    cnt_rx++;

    /* Сверка с образцом: маркер "WT" + заполнение 0xA5 (байт [2] — порядковый номер). */
    match = ((plen == TF_LEN) && (s_rxbuf[0] == (uint8_t)'W') && (s_rxbuf[1] == (uint8_t)'T')) ? 1u : 0u;
    for (j = 3u; (j < plen) && (match != 0u); j++)
    {
      if (s_rxbuf[j] != TF_FILL) { match = 0u; }
    }

    Console_Printf("uwbrx: #%lu len=%u seq=%u first=%02X %02X %02X match=%s\r\n",
                   (unsigned long)cnt_rx, (unsigned)plen, (unsigned)s_rxbuf[2],
                   (unsigned)s_rxbuf[0], (unsigned)s_rxbuf[1], (unsigned)s_rxbuf[2],
                   (match != 0u) ? "yes" : "no");
    rx_arm();
  }
  else if ((status & RX_ERR_MASK) != 0u)
  {
    if ((status & (uint32_t)DWT_INT_RXPHE_BIT_MASK) != 0u) { cnt_phe++; }
    else { cnt_crc++; }
    dwt_writesysstatuslo(RX_ERR_MASK);
    rx_arm();
  }
  else if ((status & RX_TO_MASK) != 0u)
  {
    cnt_to++;
    dwt_writesysstatuslo(RX_TO_MASK);
    rx_arm();
  }
}

/* =====================================================================================
 *  Транспорт для голосового движка (voice.c): «отдать кадр / принять кадр».
 * =====================================================================================*/

uint8_t Dw3000Port_IsInited(void) { return s_inited; }

/* Передать один payload как радиокадр (полудуплекс, полагаемся на FCS радио — SLIP не нужен). */
uint8_t Dw3000Port_VoiceTx(const uint8_t *payload, uint16_t len)
{
  uint32_t t0;
  if ((s_inited == 0u) || (payload == NULL)) { return 1u; }

  s_rx_armed = 0u;                        /* передача выключает приём (полудуплекс) */
  dwt_forcetrxoff();
  dwt_writesysstatuslo((uint32_t)DWT_INT_TXFRS_BIT_MASK);
  (void)dwt_writetxdata(len, (uint8_t *)payload, 0u);
  dwt_writetxfctrl((uint16_t)(len + 2u), 0u, 0u);   /* +2 = автодобавляемый FCS */
  if (dwt_starttx(DWT_START_TX_IMMEDIATE) != (int32_t)DWT_SUCCESS) { return 1u; }

  t0 = HAL_GetTick();
  while (((dwt_readsysstatuslo() & (uint32_t)DWT_INT_TXFRS_BIT_MASK) == 0u) &&
         ((uint32_t)(HAL_GetTick() - t0) < 10u)) { }
  if ((dwt_readsysstatuslo() & (uint32_t)DWT_INT_TXFRS_BIT_MASK) != 0u)
  {
    dwt_writesysstatuslo((uint32_t)DWT_INT_TXFRS_BIT_MASK);
    cnt_tx++;
    return 0u;
  }
  return 1u;
}

/* Опрос приёма из главного цикла: самовзвод приёмника + при готовом кадре — в sink (frame_enqueue). */
void Dw3000Port_VoicePoll(void (*sink)(const uint8_t *payload, uint16_t len))
{
  uint32_t status;
  if (s_inited == 0u) { return; }
  if (s_rx_armed == 0u) { rx_arm(); s_rx_armed = 1u; cnt_switch++; }

  status = dwt_readsysstatuslo();
  if ((status & (uint32_t)DWT_INT_RXFCG_BIT_MASK) != 0u)
  {
    uint8_t  rng  = 0u;
    uint16_t flen = dwt_getframelength(&rng);
    uint16_t plen = (flen > 2u) ? (uint16_t)(flen - 2u) : 0u;
    if (plen > (uint16_t)sizeof(s_rxbuf)) { plen = (uint16_t)sizeof(s_rxbuf); }
    dwt_readrxdata(s_rxbuf, plen, 0u);
    dwt_writesysstatuslo((uint32_t)DWT_INT_RXFCG_BIT_MASK | (uint32_t)DWT_INT_RXFR_BIT_MASK);
    cnt_rx++;
    if (sink != NULL) { sink(s_rxbuf, plen); }
    rx_arm();
  }
  else if ((status & RX_ERR_MASK) != 0u)
  {
    if ((status & (uint32_t)DWT_INT_RXPHE_BIT_MASK) != 0u) { cnt_phe++; } else { cnt_crc++; }
    dwt_writesysstatuslo(RX_ERR_MASK);
    rx_arm();
  }
  else if ((status & RX_TO_MASK) != 0u)
  {
    cnt_to++;
    dwt_writesysstatuslo(RX_TO_MASK);
    rx_arm();
  }
}

void Dw3000Port_VoiceRxArm(void)
{
  if (s_inited != 0u) { rx_arm(); s_rx_armed = 1u; cnt_switch++; }
}

void Dw3000Port_GetVoiceStats(uint32_t *tx, uint32_t *rx, uint32_t *crc,
                              uint32_t *phe, uint32_t *to, uint32_t *sw)
{
  if (tx != NULL)  { *tx  = cnt_tx;  }
  if (rx != NULL)  { *rx  = cnt_rx;  }
  if (crc != NULL) { *crc = cnt_crc; }
  if (phe != NULL) { *phe = cnt_phe; }
  if (to != NULL)  { *to  = cnt_to;  }
  if (sw != NULL)  { *sw  = cnt_switch; }
}

void Dw3000Port_ResetVoiceStats(void)
{
  cnt_tx = 0u; cnt_rx = 0u; cnt_crc = 0u; cnt_phe = 0u; cnt_to = 0u; cnt_switch = 0u; cnt_armed = 0u;
}

/* Единый источник состояния приёмника (дефект B): и uwbstat, и строка radio:/link: в voice
 * печатают ровно это, чтобы не расходиться. Диагностический путь (uwbrx on) имеет приоритет над
 * голосовым при отображении, так как удерживает приёмник явно. Только ASCII. */
const char *Dw3000Port_RxStateStr(void)
{
  if (s_rx_active != 0u) { return "armed(diag)";  }
  if (s_rx_armed  != 0u) { return "armed(voice)"; }
  return "off";
}

/* Число первичных взводов приёмника (конец uwbinit), отдельно от switch (TX->RX). */
uint32_t Dw3000Port_GetArmedCount(void) { return cnt_armed; }

#endif /* UWB_CHIP_DW3000 */
#endif /* LAB_ID == 9 */
