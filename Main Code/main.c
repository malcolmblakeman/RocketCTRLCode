
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
#include "lps22hh_reg.h"
#include "lps22hh.h"
#include "lsm6dsv80x_reg.h"
#include "sensor.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SPI_READ_BIT   			(0x80)
#define SPI_AUTO_INC   			(0x40)

#define BAROM_REG_WHO_AM_I  	(0x0F)
#define BAROM_WHO_AM_I_VAL   	(0xB3)

#define IMU_REG_WHO_AM_I    	(0x0F)
#define IMU_WHO_AM_I_VAL  		(0x73)


/* Private macro -------------------------------------------------------------*/
#define PIN_LOW(port, pin)   		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define PIN_HIGH(port, pin)  		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

#define FLASH_WP_ENABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_RESET)
#define FLASH_WP_DISABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_SET)

#define GPS_FIX_READ()  	 		(HAL_GPIO_ReadPin(GPS_FIX_GPIO_Port, GPS_FIX_Pin) == GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef hpcd_USB_DRD_FS;
LPS22HH_Object_t baro;
stmdev_ctx_t imu_ctx;

//Test Variables
static uint8_t  	rx_byte;
static char     	nmea_line[128];
static volatile 	uint16_t nmea_idx = 0;
static volatile 	uint8_t  nmea_line_ready = 0;

//Real Program Variables
static volatile 	uint8_t g_pyroActive = 0;
static volatile 	uint8_t g_pyroTicks = 0;   		//Determines how long pyro channels on
static volatile 	uint8_t g_rfTicks = 0;			//Determines how long between every rf tx
static volatile 	uint16_t g_timeStamp = 0;		//Keeps track of time in 100's of ms
static volatile 	uint8_t g_LoRaTx = 0;				//Transmit to LoRa module
static volatile 	uint8_t g_getData = 1;
static volatile     uint32_t flash_write_addr = 0;


/* Private function prototypes -----------------------------------------------*/
//Initialization Functions
void systemclock_config(void);
static void mx_gpio_init(void);
static void mx_icache_init(void);
static void mx_spi1_init(void);
static void mx_spi2_init(void);
static void mx_usart2_uart_init(void);
static void mx_usb_pcd_init(void);
static void mx_tim3_init(void);

//Sensor test functions
void test_all(void);
void pyro_test(void);
void flash_read_id(uint8_t id[3]);
void gps_grab_line(void);
static bool barom_test(uint8_t *whoami_out);
static bool imu_test(uint8_t *who_out);
int gps_readoneline(char *out, int maxlen, uint32_t timeout_ms);
bool rf_read_register(uint16_t addr, uint8_t* val_out);
static bool llcc68_wait_while_busy(uint32_t timeout_ms);

//User defined functions
int barom_init(void);
int barom_read(float *p_pres, float *p_temp);
int imu_init(void);
int imu_read(int16_t lowG[3], int16_t highG[3], int16_t gyro[3]);
void flash_page_program(uint32_t addr, uint8_t *data, uint16_t len);
void flash_write_enable(void);
static void light_apogee_pyro(void);
static void light_main_pyro(void);
static inline uint16_t float_to_u16(float x, float scale, float offset);
static inline void pack_buf(uint8_t tx_buf[32],
                          const int16_t lowG[3],
                          const int16_t highG[3],
                          const int16_t gyro[3],
                          uint16_t pres,
                          uint16_t temp);
static uint8_t flash_read_status(void);
static void flash_wait_busy(void);
void flash_read(uint32_t addr, uint8_t *data, uint16_t len);
void flash_init(void);
void flash_sector_erase(uint32_t addr);

/*-------------------------------------------------------------------------------*/


/////////////////////////
///* Begin Main Code *///
/////////////////////////

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  systemclock_config();
  mx_icache_init();

  /* Initialize all configured peripherals */
  mx_gpio_init();

  mx_spi1_init();
  mx_spi2_init();					//Initialized in mode 3
  mx_usart2_uart_init();
  //mx_usb_pcd_init();
  mx_tim3_init();

  //test_all();						//Test sensors

  /* Initialize all sensors */
  barom_init();
  imu_init();
  flash_write_enable();



  /* Initialize variables, test, and start timer */
  uint8_t dataReady = 0;				//Transmit to flash chip
    uint8_t tx_buf[32] = {
      0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
      0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF0, 0x0F,
      0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89,
      0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0x5A, 0xA5
  };				//Save data to buffer to transmit. Random values for test
  uint8_t rx_buf[32] = {0};
	
  uint16_t pres_u16 = 0;
  uint16_t temp_u16 = 0;

  int16_t s_lowG[3] = {0};
  int16_t s_highG[3] = {0};
  int16_t s_gyro[3] = {0};

  float p_hpa = 0;
  float p_tc = 0;


  HAL_TIM_Base_Start_IT(&htim3);	//start timer interrupt

  /* START FLIGHT LOOP */
  while (1)
  {
	  //TODO: Come up with formula for calculating when to ignite parachutes
	  /* if(Altitude and acceleration == ?)
	   * {
	   * light_apogee_pyro();
	   * }
	   */

	  //Should this be a timing thing? After apogee parachute is lit, wait 10s then ignite main?
	  /* if(Altitude and acceleration == ? && g_pyroActive == 0)
	   * {
	   * light_main_pyro();
	   * }
	   */

	  if (g_getData)
	  {
		  //TODO: add functions for communicating with sensors and saving the obtained data

		  //Read Barom
		  if (barom_read(&p_hpa, &p_tc) == 0)
		  {
		    // At sea level you should see ~1000 hPa
			// Should see ~20-30 degrees C

			pres_u16 = float_to_u16(p_hpa, 10.0f, 0.0f);
			//   input units: hPa
			//   u16 = round(p_hPa * 10 + 0)
			//   resolution: 0.1 hPa / LSB
			//   expected raw range: ~9500..10500 for 950..1050 hPa (weather/altitude dependent)
			temp_u16 = float_to_u16(p_tc, 100.0f, 0.0f);
			//   input units: degC
			//   u16 = round(temp_C * 100 + 0)
			//   resolution: 0.01 degC / LSB
			//   expected range: ~1500..3500 for 15..35C indoor
		  }
		  else
		  {
			  //error checking for if getting barom data is unsuccessful
			  //set error flag
		  }


		  //Read IMU
		  if (imu_read(s_lowG, s_highG, s_gyro) == 0)
		  {
			  // lowG ~ (0,0, +something) when sitting still
			  // gyro ~ (0,0,0) when not rotating
			  // highG ~ (0,0,0) unless you smack it / vibrate it
		  }
		  else
		  {
			  //error checking for if getting IMU data is unsuccessful
			  //set error flag
		  }


		  //Read GPS

		  dataReady = 1;		//add error checking to set data ready to 0 in case of incorrect/corrupted data
		  g_getData = 0;
	  }
		
	  if (dataReady)
	  {
		  //Create buffer to be sent
		  pack_buf(tx_buf, s_lowG, s_highG, s_gyro, pres_u16, temp_u16);

		  if(g_LoRaTx)
		  {
			  //TODO: Function for transmitting data over LoRa
			  //Transmit latest data packet over LoRa
			  g_LoRaTx = 0;
		  }

		  //TODO: Function for transmitting data to flash
		  //Transmit latest data packet to flash chip
		  if (flash_write_addr < 0xFFFFFF)
		  {
			  flash_page_program(flash_write_addr, tx_buf, sizeof(tx_buf));
			  flash_write_addr += sizeof(tx_buf);
		  }

		  memset(tx_buf, 0, sizeof(tx_buf));		//Clear data buffer

		  dataReady = 0;
	  }
	  flash_read(0x000000, rx_buf, 32);
	  __NOP();
  }
}


/*-------------------------------------------------------------------------------*/


////////////////////////////////////
///* ALL USER DEFINED FUNCTIONS *///
////////////////////////////////////

static void light_apogee_pyro(void)
{
  // Set PC6-9 HIGH
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  g_pyroActive = 1;
  g_pyroTicks = 0;
}

static void light_main_pyro(void)
{
  // Set PC6-9 HIGH
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  g_pyroActive = 1;
  g_pyroTicks = 0;
}

// Converts value to uint16 using: round(value * scale) + offset
// Then clamps into [0, 65535].
static inline uint16_t float_to_u16(float value, float scale, float offset)
{
  double scaled_value = (double)value * (double)scale + (double)offset;
  long u16_value = lround(scaled_value);

  if (u16_value < 0) u16_value = 0;
  if (u16_value > 65535) u16_value = 65535;

  return (uint16_t)u16_value;
}

static inline void pack_buf(uint8_t tx_buf[32],
                          const int16_t lowG[3],
                          const int16_t highG[3],
                          const int16_t gyro[3],
                          uint16_t pres,
                          uint16_t temp)
{
    uint16_t index = 0;

    tx_buf[index++] = (uint8_t)(g_timeStamp & 0xFF);
    tx_buf[index++] = (uint8_t)(g_timeStamp >> 8); 												 // 2

    memcpy(&tx_buf[index], lowG,  sizeof(int16_t) * 3); index += sizeof(int16_t) * 3;        // +6 = 8
    memcpy(&tx_buf[index], highG, sizeof(int16_t) * 3); index += sizeof(int16_t) * 3;        // +6 = 14
    memcpy(&tx_buf[index], gyro,  sizeof(int16_t) * 3); index += sizeof(int16_t) * 3;        // +6 = 20

    memcpy(&tx_buf[index], &pres, sizeof(pres)); index += sizeof(pres);                      // +2 = 22
    memcpy(&tx_buf[index], &temp, sizeof(temp)); index += sizeof(temp);                      // +2 = 24

    // remaining bytes are free for flags, checksum, and other data
}

/*------------------ Barometer functions -----------------*/

static int32_t barom_platform_read(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  (void)Address;

  uint8_t addr = (uint8_t)((uint8_t)Reg | SPI_READ_BIT);

  PIN_LOW(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);

  if (HAL_SPI_Transmit(&hspi1, &addr, 1, 100) != HAL_OK) goto err;
  if (HAL_SPI_Receive(&hspi1, pData, Length, 100) != HAL_OK) goto err;

  PIN_HIGH(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);
  return 0;

err:
  PIN_HIGH(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);
  return -1;
}

static int32_t barom_platform_write(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  (void)Address;

  uint8_t addr = (uint8_t)Reg; // no bits added

  PIN_LOW(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);

  if (HAL_SPI_Transmit(&hspi1, &addr, 1, 100) != HAL_OK) goto err;
  if (HAL_SPI_Transmit(&hspi1, pData, Length, 100) != HAL_OK) goto err;

  PIN_HIGH(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);
  return 0;

err:
  PIN_HIGH(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);
  return -1;
}



int barom_init(void)
{
  LPS22HH_IO_t io = {
    .Init     = 0,
    .DeInit   = 0,
    .BusType  = LPS22HH_SPI_4WIRES_BUS,
    .Address  = 0, // ignored for SPI
    .WriteReg = barom_platform_write,
    .ReadReg  = barom_platform_read,
    .GetTick  = (int32_t(*)(void))HAL_GetTick,
    .Delay    = HAL_Delay
  };

  if (LPS22HH_RegisterBusIO(&baro, &io) != LPS22HH_OK) return -1;
  if (LPS22HH_Init(&baro) != LPS22HH_OK) return -2;

  // Pick an ODR (Hz). 25 Hz is a good start
  if (LPS22HH_PRESS_SetOutputDataRate(&baro, 50.0f) != LPS22HH_OK) return -3;	//Pressure and Temp share the same ODR

  // This starts continuous conversions
  if (LPS22HH_PRESS_Enable(&baro) != LPS22HH_OK) return -4;
  if (LPS22HH_TEMP_Enable(&baro) != LPS22HH_OK) return -5;

  return 0; // initialized (note: still power-down until BAROM_Start called)
}


int barom_read(float *p_hpa, float *p_tc)
{
  if (!p_hpa || !p_tc) return -1;

  if (LPS22HH_PRESS_GetPressure(&baro, p_hpa) != LPS22HH_OK) return -2;
  if (LPS22HH_TEMP_GetTemperature(&baro, p_tc) != LPS22HH_OK) return -3;

  return 0;
}

/*------------------ IMU functions -----------------*/

static int32_t imu_platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  (void)handle;

  // For ST SPI: reg | 0x80 => read, reg | 0x40 => auto-inc (multi-byte)
  uint8_t addr = reg | SPI_READ_BIT;
  if (len > 1U) addr |= SPI_AUTO_INC;

  PIN_LOW(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);

  if (HAL_SPI_Transmit(&hspi1, &addr, 1, 100) != HAL_OK) goto err;
  if (HAL_SPI_Receive(&hspi1, data, len, 100) != HAL_OK) goto err;

  PIN_HIGH(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);
  return 0;

err:
  PIN_HIGH(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);
  return -1;
}

static int32_t imu_platform_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  (void)handle;

  uint8_t addr = reg;
  if (len > 1U) addr |= SPI_AUTO_INC;

  PIN_LOW(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);

  if (HAL_SPI_Transmit(&hspi1, &addr, 1, 100) != HAL_OK) goto err;
  if (HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, 100) != HAL_OK) goto err;

  PIN_HIGH(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);
  return 0;

err:
  PIN_HIGH(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);
  return -1;
}

int imu_init(void)
{
  imu_ctx.write_reg = imu_platform_write;
  imu_ctx.read_reg  = imu_platform_read;
  imu_ctx.mdelay    = (stmdev_mdelay_ptr)HAL_Delay;
  imu_ctx.handle    = NULL;

  lsm6dsv80x_block_data_update_set(&imu_ctx, 1);

  // Set output data rates (pick something simple for testing)
  // Low-g accel ODR
  if (lsm6dsv80x_xl_setup(&imu_ctx, LSM6DSV80X_ODR_AT_120Hz, LSM6DSV80X_XL_HIGH_PERFORMANCE_MD) != 0) return -1; // enum is in reg.h
  //High-g accel ODR
  if (lsm6dsv80x_hg_xl_data_rate_set(&imu_ctx, LSM6DSV80X_ODR_AT_240Hz, 1) != 0) return -2;
  // Gyro ODR
  if (lsm6dsv80x_gy_setup(&imu_ctx, LSM6DSV80X_ODR_AT_120Hz, LSM6DSV80X_GY_HIGH_PERFORMANCE_MD) != 0) return -3;

  //set scale for each measurement (full scale)
  if (lsm6dsv80x_xl_full_scale_set(&imu_ctx, LSM6DSV80X_16g) != 0) return -4;
  if (lsm6dsv80x_gy_full_scale_set(&imu_ctx, LSM6DSV80X_2000dps) != 0) return -5;
  if (lsm6dsv80x_hg_xl_full_scale_set(&imu_ctx, LSM6DSV80X_80g) != 0) return -6;

  return 0;
}

int imu_read(int16_t lowG[3], int16_t highG[3], int16_t gyro[3])
{
  if (!lowG || !highG || !gyro) return -1;


  if (lsm6dsv80x_acceleration_raw_get(&imu_ctx, lowG) != 0) return -3;
  if (lsm6dsv80x_angular_rate_raw_get(&imu_ctx, gyro) != 0) return -4;
  if (lsm6dsv80x_hg_acceleration_raw_get(&imu_ctx, highG) != 0) return -5;

  return 0;
}
/*------------------ Flash functions -----------------*/

void flash_init(void)
{
    uint8_t cmd;
    uint8_t jedec[3] = {0};

    // Disable hardware write protection
    FLASH_WP_DISABLE_PROTECT();

    __NOP();

    flash_write_enable();

    flash_wait_busy();

    // Global Block Protection Unlock
    cmd = 0x98;   //Global Block Protection Unlock Command
    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    // Wait until device is no longer busy
    flash_wait_busy();

    //sanity check: read JEDEC ID
    cmd = 0x9F;   // JEDEC ID Read Command
    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, jedec, 3, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    __NOP();		//Check ID Here
}

void flash_sector_erase(uint32_t addr)
{
    uint8_t cmd[4];

    cmd[0] = 0x20;   // 4 KB Sector Erase
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    flash_write_enable();

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    flash_wait_busy();
}

void flash_page_program(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t cmd[4];

    if (len == 0 || len > 256)
    {
        return;
    }

    flash_wait_busy();

    cmd[0] = 0x02;   // Page Program
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    flash_write_enable();

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
    HAL_SPI_Transmit(&hspi2, data, len, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
}

void flash_read(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t cmd[4];

    flash_wait_busy();

    cmd[0] = 0x03;   // Read
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
    HAL_SPI_Receive(&hspi2, data, len, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
}

void flash_write_enable(void)
{
    uint8_t cmd = 0x06;   // Write Enable

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
}

static void flash_wait_busy(void)
{
    while (flash_read_status() & 0x01)   // BUSY bit
    {
        __NOP();
    }
}


static uint8_t flash_read_status(void)
{
	uint8_t cmd = 0x05;   // Read Status Register
	uint8_t status = 0;

	PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
	HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
	HAL_SPI_Receive(&hspi2, &status, 1, 100);
	PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

	return status;
}

/*------------------ GPS functions -----------------*/


/*------------------ LoRa functions -----------------*/


/*-------------------------------------------------------------------------------*/


////////////////////////////
///* Interrupt Handlers *///
////////////////////////////

/* 100ms Delay Logic */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	g_timeStamp++;		//Keeps track of time in 100's of ms
	g_getData = 1;

	/* Pyro channel control logic */
	if (g_pyroActive)
	{
		g_pyroTicks++;

		if (g_pyroTicks >= 50) // 50 * 10ms = 0.5s
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
			g_pyroActive = 0;
			g_pyroTicks = 0;
		}
	}

	/* RF transmission logic */
	g_rfTicks++;
	if (g_rfTicks >= 50){
		g_LoRaTx = 1;
		g_rfTicks = 0;
	}
	// Toggle LED1
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);	//Probe and test with scope if actually 10ms
}


/*-------------------------------------------------------------------------------*/

//////////////////////////////////////////////////
///* MICROCONTROLLER INITIALIZATION FUNCTIONS *///
//////////////////////////////////////////////////

/* Configure system clock */
void systemclock_config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure the programming delay */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}


/* Configure icache for debug efficiency */
void mx_icache_init(void)
{

  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
}


/* Configure SPI1 for IMU, Barometer, and RF module */
static void mx_spi1_init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}


/* Configure SPI2 for Flash Chip */
static void mx_spi2_init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}


/* Configure Timer for Interrupts to Transmit and Save Data */
static void mx_tim3_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 31999;                 // 32MHz/32000 = 1kHz (1ms tick)
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 9;                       // 100ms
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}


/* Configure USART2 for GPS */
static void mx_usart2_uart_init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}


/* Configure USB for Debugging in Field and Data Transfering */
static void mx_usb_pcd_init(void)
{
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
}
/* GPIO Configurations */
static void mx_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IMU_INT_1_Pin|IMU_INT_2_Pin|LoRa_INT_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_RF_Pin|SPI2_CS_Flash_Pin|SPI1_CS_BAROM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RSFW_V2_Pin|Pyro_CTRL_Master_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IMU_INT_1_Pin IMU_INT_2_Pin LoRa_INT_1_Pin */
  GPIO_InitStruct.Pin = IMU_INT_1_Pin|IMU_INT_2_Pin|LoRa_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Busy_Pin */
  GPIO_InitStruct.Pin = Busy_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Busy_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAROM_INT_Pin */
  GPIO_InitStruct.Pin = BAROM_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAROM_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RSFW_V2_Pin Pyro_CTRL_Master_Pin */
  GPIO_InitStruct.Pin = RSFW_V2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RSFW_V1_Pin */
  GPIO_InitStruct.Pin = RSFW_V1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RSFW_V1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_FIX_Pin */
  GPIO_InitStruct.Pin = GPS_FIX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_FIX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_WP_Pin */
  GPIO_InitStruct.Pin = FLASH_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_WP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_IMU_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_IMU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_RF_Pin|SPI2_CS_Flash_Pin|SPI1_CS_BAROM_Pin*/
  GPIO_InitStruct.Pin = SPI1_CS_RF_Pin|SPI2_CS_Flash_Pin|SPI1_CS_BAROM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Float_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Pyro_CTRL_Master_Pin */
  GPIO_InitStruct.Pin = Pyro_CTRL_Master_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pyro_CONT_1_Pin, Pyro_CONT_2_Pin, Pyro_CONT_3_Pin, Pyro_CONT_4_Pin */
  GPIO_InitStruct.Pin = Pyro_CONT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pyro_CONT_1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Pyro_CONT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pyro_CONT_2_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Pyro_CONT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pyro_CONT_3_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Pyro_CONT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pyro_CONT_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pyro_CTRL_1_Pin, Pyro_CTRL_2_Pin, Pyro_CTRL_3_Pin, Pyro_CTRL_4_Pin */
  GPIO_InitStruct.Pin = Pyro_CTRL_1_Pin|Pyro_CTRL_2_Pin|Pyro_CTRL_3_Pin|Pyro_CTRL_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, Pyro_CTRL_1_Pin | Pyro_CTRL_2_Pin | Pyro_CTRL_3_Pin | Pyro_CTRL_4_Pin, GPIO_PIN_RESET);	//Ensure low for pyro safety
}


/*-------------------------------------------------------------------------------*/

///////////////////////////////////
///* ALL SENSOR TEST FUNCTIONS *///
///////////////////////////////////

void test_all(void){

	  /* FLASH TEST--------------------------*/
	  uint8_t id[3] = {0};
	  flash_read_id(id);			//Function for reading ID for flash chip
	  __NOP();


	 /* BAROMETER TEST--------------------------*/
	  uint8_t whoBarom = 0;
	  bool okBarom = barom_test(&whoBarom);

	  // Debug: breakpoint here
	  // Expect: who == 0xB3 and ok == true
	  __NOP();
	  (void)okBarom;
	  (void)whoBarom;



	  /* IMU TEST-------------------------------*/
	  uint8_t whoIMU = 0;
	  bool okIMU = imu_test(&whoIMU);

	  // Debug: breakpoint here
	  // Expect: who == 0x73 and ok == true
	   __NOP();
	  (void)okIMU;
	  (void)whoIMU;



	  const char *cmd = "$PMTK104*37\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);


	  /* GPS TEST-------------------------------*/
	  while(1){
		  char line[128];
		  uint8_t gpsTest;

		  if (gps_readoneline(line, sizeof(line), 3000))
		  {
		      gpsTest = 1;      // line contains a clean "$G....\r\n"
		      // breakpoint: inspect line
		  }
		  else
		  {
		      gpsTest = 0;
		  }

		  if (HAL_GPIO_ReadPin(GPS_FIX_GPIO_Port, GPS_FIX_Pin) == 1)
		  {
			  HAL_GPIO_WritePin((LED1_GPIO_Port), (LED1_Pin), GPIO_PIN_SET);
		  }

	 	  __NOP();
	  }



	  /* RF TEST--------------------------*/
	  uint8_t reg_val = 0;
	  bool ok = rf_read_register(0x08AC, &reg_val);
	  //reg_val should commonly be 0x94 or 0x96
	  __NOP();


	  /* PYRO TEST--------------------------*/
	  __NOP();
	  pyro_test();
	  __NOP();
}

void pyro_test(void){

	uint8_t cont1 = HAL_GPIO_ReadPin(Pyro_CONT_1_GPIO_Port, Pyro_CONT_1_Pin);
	PIN_HIGH(Pyro_CTRL_1_GPIO_Port, Pyro_CTRL_1_Pin);
	cont1 = HAL_GPIO_ReadPin(Pyro_CONT_1_GPIO_Port, Pyro_CONT_1_Pin);
	PIN_LOW(Pyro_CTRL_1_GPIO_Port, Pyro_CTRL_1_Pin);
	cont1 = HAL_GPIO_ReadPin(Pyro_CONT_1_GPIO_Port, Pyro_CONT_1_Pin);
	__NOP();

	uint8_t cont2 = HAL_GPIO_ReadPin(Pyro_CONT_2_GPIO_Port, Pyro_CONT_2_Pin);
	PIN_HIGH(Pyro_CTRL_2_GPIO_Port, Pyro_CTRL_2_Pin);
	cont2 = HAL_GPIO_ReadPin(Pyro_CONT_2_GPIO_Port, Pyro_CONT_2_Pin);
	PIN_LOW(Pyro_CTRL_2_GPIO_Port, Pyro_CTRL_2_Pin);
	__NOP();

	uint8_t cont3 = HAL_GPIO_ReadPin(Pyro_CONT_3_GPIO_Port, Pyro_CONT_3_Pin);
	__NOP();

	uint8_t cont4 = HAL_GPIO_ReadPin(Pyro_CONT_4_GPIO_Port, Pyro_CONT_4_Pin);
	__NOP();

	__NOP();
	PIN_HIGH(Pyro_CTRL_Master_GPIO_Port, Pyro_CTRL_Master_Pin);
	PIN_LOW(Pyro_CTRL_Master_GPIO_Port, Pyro_CTRL_Master_Pin);
	__NOP();

	cont1 = HAL_GPIO_ReadPin(Pyro_CONT_1_GPIO_Port, Pyro_CONT_1_Pin);
	cont2 = HAL_GPIO_ReadPin(Pyro_CONT_2_GPIO_Port, Pyro_CONT_2_Pin);
	cont3 = HAL_GPIO_ReadPin(Pyro_CONT_3_GPIO_Port, Pyro_CONT_3_Pin);
	cont4 = HAL_GPIO_ReadPin(Pyro_CONT_4_GPIO_Port, Pyro_CONT_4_Pin);
	__NOP();

	return;
}

void flash_read_id(uint8_t id[3])
{
	FLASH_WP_DISABLE_PROTECT();	//Disable protect to write to flash chip
    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    for (volatile int i = 0; i < 200; i++) __NOP(); // ~ a few µs depending on clock

    uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 100);

    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    id[0] = rx[1];
    id[1] = rx[2];
    id[2] = rx[3];

    //FLASH_WP_ENABLE_PROTECT();	//Disable protect to write to flash chip
    return (st == HAL_OK);

}

static bool barom_test(uint8_t *whoami_out)
{
    //Read WHO_AM_I
    uint8_t tx[2] = { (uint8_t)(BAROM_REG_WHO_AM_I | 0x80)};
    uint8_t rx[2] = {0};


    PIN_LOW(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);

    for (volatile int i = 0; i < 200; i++) __NOP(); // ~ a few µs depending on clock

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

    PIN_HIGH(SPI1_CS_BAROM_GPIO_Port, SPI1_CS_BAROM_Pin);

    uint8_t who = rx[1];
    uint8_t who0 = rx[0];

    if (whoami_out) *whoami_out = who;

    return (st == HAL_OK) && (who == BAROM_WHO_AM_I_VAL);
}


static bool imu_test(uint8_t *who_out)
{
  // ---- Read WHO_AM_I (single transaction) ----
  uint8_t tx[2] = { (uint8_t)(IMU_REG_WHO_AM_I | 0x80)};
  uint8_t rx[2] = { 0 };

  PIN_LOW(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);
  for (volatile int i = 0; i < 200; i++) __NOP(); // ~ a few µs depending on clock


  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

  PIN_HIGH(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);

  uint8_t who = rx[1];
  if (who_out) *who_out = who;

  return (st == HAL_OK) && (who == IMU_WHO_AM_I_VAL);
}


int gps_readoneline(char *out, int maxlen, uint32_t timeout_ms)
{
    uint8_t c;
    int idx = 0;
    uint32_t t0 = HAL_GetTick();

    if (maxlen <= 0) return 0;
    memset(out, 0, maxlen);

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart2, &c, 1, 10) == HAL_OK)
        {
            if (c == '$')
            {
                // hard resync: start a new sentence right here
                idx = 0;
                out[idx++] = '$';
                continue;
            }

            // ignore everything until we've seen a '$'
            if (idx == 0) continue;

            if (idx < (maxlen - 1))
                out[idx++] = (char)c;
            else
                idx = 0; // overflow -> resync

            if (c == '\n')
            {
                out[idx] = '\0';
                return 1;
            }
        }
    }
    out[0] = '\0';
    return 0;
}


bool rf_read_register(uint16_t addr, uint8_t* val_out)
{
    llcc68_wait_while_busy(100);

    uint8_t tx[5] = {
        0x1D,                  // ReadRegister
        (uint8_t)(addr >> 8),  // addr MSB
        (uint8_t)(addr & 0xFF),// addr LSB
        0x00,                  // dummy
        0x00                   // clock 1 byte out
    };
    uint8_t rx[5] = {0};

    PIN_LOW(SPI1_CS_RF_GPIO_Port, SPI1_CS_RF_Pin);
    for (volatile int i = 0; i < 200; i++) __NOP();

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 100);

    PIN_HIGH(SPI1_CS_RF_GPIO_Port, SPI1_CS_RF_Pin);

    llcc68_wait_while_busy(100);

    if (val_out) *val_out = rx[4];
    return (st == HAL_OK);
}

static bool llcc68_wait_while_busy(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_0) == GPIO_PIN_SET) // PH0 BUSY
  {
    if ((HAL_GetTick() - t0) >= timeout_ms) return false;
  }
  return true;
}


/*-------------------------------------------------------------------------------*/


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
