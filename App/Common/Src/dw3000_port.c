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
#include <string.h>

/* Дескриптор SPI4 (CubeMX, main.c) и выводы модуля — те же, что в uwb_core.c (проверены железом). */
extern SPI_HandleTypeDef hspi4;
#define DW_CSN_PORT   GPIOE
#define DW_CSN_PIN    GPIO_PIN_11

/* Дескриптор драйвера DW3000 определён в libdw3000_cm4.a (dw3000_device.c). Объявляем сами:
 * штатный extern в deca_device_api.h спрятан за USE_DRV_DW3000. */
extern const struct dwt_driver_s dw3000_driver;

/* ================= Задержки ================= */
static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
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
static void spi_fast(void) { spi_set_prescaler(SPI_BAUDRATEPRESCALER_8);  }

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

#endif /* UWB_CHIP_DW3000 */
#endif /* LAB_ID == 9 */
