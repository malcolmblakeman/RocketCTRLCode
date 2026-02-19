



#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "BAR.h"

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"



/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        5 //ms

#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float_t pressure_hPa;
static float_t temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

static   stmdev_ctx_t dev_ctx;
static uint8_t lps22hh_drdy_event;

/* Main Example --------------------------------------------------------------*/

int main(void)
{
  lps22hh_read_data_drdy();
}
void lps22hh_read_data_drdy_handler(void)
{
  lps22hh_drdy_event = 1;
}

void lps22hh_read_data_drdy(void)
{
  lps22hh_pin_int_route_t int_route;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  whoamI = 0;
  lps22hh_device_id_get(&dev_ctx, &whoamI);

  if ( whoamI != LPS22HH_ID )
    while (1); /*manage here device not found */

  /* Restore default configuration */
  lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lps22hh_reset_get(&dev_ctx, &rst);
  } while (rst);

  lps22hh_int_notification_set(&dev_ctx, LPS22HH_INT_PULSED);

  lps22hh_pin_int_route_get(&dev_ctx, &int_route);
  int_route.drdy_pres = PROPERTY_ENABLE;
  lps22hh_pin_int_route_set(&dev_ctx, int_route);

  /* Enable Block Data Update */
  lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

  /* Read samples on drdy event */
  while (1) {
    if (lps22hh_drdy_event) {
      uint8_t p_drdy;
      uint8_t t_drdy;

      lps22hh_drdy_event = 0;

      /* Read output only if new value is available */
      lps22hh_press_flag_data_ready_get(&dev_ctx, &p_drdy);
      lps22hh_temp_flag_data_ready_get(&dev_ctx, &t_drdy);

      if (p_drdy) {
        memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
        lps22hh_pressure_raw_get(&dev_ctx, &data_raw_pressure);
        pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_pressure);
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "pressure [hPa]:%6.2f\r\n", pressure_hPa);
        tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
      }

      if (t_drdy) {
        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
        lps22hh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
        temperature_degC = lps22hh_from_lsb_to_celsius(
                             data_raw_temperature );
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "temperature [degC]:%6.2f\r\n",
                temperature_degC );
        tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
      }
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{

  CDC_Transmit_FS(tx_buffer, len);

}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);

}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
}
