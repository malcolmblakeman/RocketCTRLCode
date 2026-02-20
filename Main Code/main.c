
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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LED1_GPIO_Port 	GPIOB
#define LED1_Pin       	GPIO_PIN_15

#define LED2_GPIO_Port 	GPIOB
#define LED2_Pin       	GPIO_PIN_14

#define LED3_GPIO_Port 	GPIOB
#define LED3_Pin       	GPIO_PIN_13

#define LED4_GPIO_Port 	GPIOB
#define LED4_Pin       	GPIO_PIN_12

#define FLASH_CS_GPIO 	GPIOC
#define FLASH_CS_PIN  	GPIO_PIN_11

#define BAROM_REG_WHO_AM_I  	(0x0F)
#define BAROM_WHO_AM_I_VAL   	(0xB3)
#define BAROM_READ_BIT			(0x80)

#define IMU_REG_WHO_AM_I    	(0x0F)
#define IMU_WHO_AM_I_VAL  	(0x73)
#define IMU_READ_BIT			(0x80)

/* Private macro -------------------------------------------------------------*/
#define PIN_LOW(port, pin)   		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define PIN_HIGH(port, pin)  		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

#define FLASH_WP_ENABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_RESET)
#define FLASH_WP_DISABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_SET)

#define GPS_FIX_READ()  	 		(HAL_GPIO_ReadPin(GPS_FIX_GPIO_Port, GPS_FIX_Pin) == GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

static uint8_t  	rx_byte;
static char     	nmea_line[128];
static volatile 	uint16_t nmea_idx = 0;
static volatile 	uint8_t  nmea_line_ready = 0;

/* Private function prototypes -----------------------------------------------*/
void systemclock_config(void);
static void debug_swd_only_disable_jtag(void);
static void mx_gpio_init(void);
static void mx_adc1_init(void);
static void mx_icache_init(void);
static void mx_spi1_init(void);
static void mx_spi2_init(void);
static void mx_usart2_uart_init(void);
static void mx_usb_pcd_init(void);
static void mx_tim3_init(void);

void flash_read_id(uint8_t id[3]);

void gps_grab_line(void);

static bool barom_test(uint8_t *whoami_out);
static bool imu_test(uint8_t *who_out);

int gps_readoneline(char *out, int maxlen, uint32_t timeout_ms);

bool rf_read_register(uint16_t addr, uint8_t* val_out);
static bool llcc68_wait_while_busy(uint32_t timeout_ms);


/*-------------------------------------------------------------------------------*/


/////////////////////////
///* Begin Main Code *///
/////////////////////////

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  systemclock_config();

  /* Initialize all configured peripherals */
  mx_gpio_init();
  mx_spi1_init();
  mx_spi2_init();				//Initialized in mode 3
  mx_usart2_uart_init();
  //mx_icache_init();
  //mx_usb_pcd_init();
  //mx_tim3_init();


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


  /* GPS TEST-------------------------------*/
  char line[128];
  uint8_t gpsTest = 0;
  if (gps_readoneline(line, sizeof(line), 3000))
  {
      // SUCCESS: put breakpoint here and inspect "line"
	  gpsTest = 1;
  }
  else
  {
      // FAIL: no bytes/line within 3 seconds
	  gpsTest = 0;
  }
  __NOP();


  /* RF TEST--------------------------*/
  uint8_t reg_val = 0;
  bool ok = rf_read_register(0x08AC, &reg_val);
  //reg_val should commonly be 0x94 or 0x96
  __NOP();


  /* PYRO TEST--------------------------*/
  __NOP();
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


  /* START FLIGHT LOOP */
  while (1)
  {
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_Delay(150);

	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  HAL_Delay(150);

	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(150);

	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  HAL_Delay(150);
  }
}


/*-------------------------------------------------------------------------------*/


////////////////////////////////////
///* ALL USER DEFINED FUNCTIONS *///
////////////////////////////////////

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
    uint8_t nmeaEnd;
    int idx = 0;
    uint32_t tick = HAL_GetTick();

    while ((HAL_GetTick() - tick) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart2, &nmeaEnd, 1, 10) == HAL_OK)
        {
            if (idx < (maxlen - 1))
                out[idx++] = (char)nmeaEnd;

            if (nmeaEnd == '\n')   // end of NMEA sentence
            {
                out[idx] = '\0';
                return 1;    // got a line
            }
        }
    }
    uint8_t gpsFix = HAL_GPIO_ReadPin(GPS_FIX_GPIO_Port, GPS_FIX_Pin);
    if (maxlen > 0) out[0] = '\0';
    return 0; // timed out
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
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

}



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
