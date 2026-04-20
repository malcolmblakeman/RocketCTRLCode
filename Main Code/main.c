
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
  *TODO: Check GPS code once antenna is working
  *TODO: Test page append commad to place tx buffer in a 256byte page buffer
  *TODO: Make GPS RX with DMA
  *TODO: Test baro fluctuation and AB filter
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

//STATE (BIT enables)
/* 0 = PAD
1 = BOOST
2 = COAST
3 = APOGEE_DETECTED
4 = DROGUE_FIRED
5 = DESCENT
6 = MAIN_FIRED
7 = LANDED / DONE */
#define STATE          	0
#define CONT          	1

#define LAT_DEG         2
#define LAT_MIN_LSB     3
#define LAT_MIN_MSB     4
#define LON_DEG         5
#define LON_MIN_LSB     6
#define LON_MIN_MSB     7

#define VELO_LSB        8
#define VELO_MSB        9
#define ALT_LSB        10
#define ALT_MSB        11

#define LG_X_LSB       12
#define LG_X_MSB       13
#define LG_Y_LSB       14
#define LG_Y_MSB       15
#define LG_Z_LSB       16
#define LG_Z_MSB       17

#define HG_X_LSB       18
#define HG_X_MSB       19
#define HG_Y_LSB       20
#define HG_Y_MSB       21
#define HG_Z_LSB       22
#define HG_Z_MSB       23

#define GY_P_LSB       24
#define GY_P_MSB       25
#define GY_R_LSB       26
#define GY_R_MSB       27
#define GY_Y_LSB       28
#define GY_Y_MSB       29

#define T_STAMP_1      30
#define T_STAMP_2      31
#define T_STAMP_3      32
#define T_STAMP_4      33

#define FLAGS          34		//Bits 0-3 continuity checking, Bits 4-7 are rocket state
#define CRC8		   35

#define PKT_LEN        36


/* Private macro -------------------------------------------------------------*/
#define PIN_LOW(port, pin)   		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define PIN_HIGH(port, pin)  		HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

#define GPS_FIX_READ()  	 		(HAL_GPIO_ReadPin(GPS_FIX_GPIO_Port, GPS_FIX_Pin) == GPIO_PIN_SET)

#define PAK_U16(buf, idx_lsb, v) do {              \
  (buf)[(idx_lsb)]     = (uint8_t)((v) & 0xFF);    \
  (buf)[(idx_lsb) + 1] = (uint8_t)((v) >> 8);      \
} while(0)

#define PAK_U32(buf, idx0, v) do {                 \
  (buf)[(idx0) + 0] = (uint8_t)((v) & 0xFF);       \
  (buf)[(idx0) + 1] = (uint8_t)(((v) >> 8) & 0xFF);\
  (buf)[(idx0) + 2] = (uint8_t)(((v) >> 16)&0xFF); \
  (buf)[(idx0) + 3] = (uint8_t)(((v) >> 24)&0xFF); \
} while(0)

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

extern LPS22HH_Object_t baro;

typedef enum
{
    FS_PAD = 0,
    FS_BOOST,
    FS_COAST,
    FS_APOGEE,
    FS_DROGUE_DESCENT,
    FS_MAIN_DESCENT,
    FS_LANDED
} flight_state_t;

typedef struct {
  float x;      // altitude estimate (m)
  float v;      // vertical velocity estimate (m/s)
  float alpha;
  float beta;
  uint8_t inited;
} ABFilter;

static flight_state_t g_flightState = FS_PAD;

//Test Variables
static uint8_t  	rx_byte;
static char     	nmea_line[128];
static volatile 	uint16_t nmea_idx = 0;
static volatile 	uint8_t  nmea_line_ready = 0;

//Real Program Variables
static volatile uint8_t 	g_pyroActive = 0;
static volatile	uint8_t 	g_pyroTicks = 0;   		//Determines how long pyro channels on
static volatile	uint8_t 	g_rfTicks = 0;			//Determines how long between every rf tx
static volatile uint32_t 	g_timeStamp = 0;		//Keeps track of time in 100's of ms
static volatile uint8_t 	g_LoRaTx = 0;			//Transmit to LoRa module
static volatile uint16_t 	g_pendingSamples = 0;
static volatile uint32_t 	flash_write_addr = 0;
static uint8_t 				tx_buf[PKT_LEN] = {0};
static uint8_t 				lora_buf[PKT_LEN];
static 						ABFilter g_ab;
static float 				g_p0_hpa = 0.0f;
static uint8_t 				g_p0_set = 0;





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
void tim3_set_period_counts(uint16_t arr);

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
static void light_apogee_pyro(void);
static void light_main_pyro(void);
static inline uint16_t float_to_u16(float x, float scale, float offset);
static inline void pack_buf(uint8_t tx_buf[PKT_LEN],
                            uint8_t lat_deg, uint16_t lat_min,
                            uint8_t lon_deg, uint16_t lon_min,
                            const int16_t lowG[3],
                            const int16_t highG[3],
                            const int16_t gyro[3],
							float vel,
							float alt,
                            uint8_t flags);
static inline void ab_init(ABFilter *f, float x0, float v0, float alpha, float beta);
static inline void ab_update(ABFilter *f, float z_meas, float dt);
static inline float pressure_to_alt(float p_hpa, float p0_hpa);

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
  gps_init();

  test_all();						//Test sensors
  pyro_test();

  /* Initialize all sensors */
  barom_init();
  imu_init();
  flash_init();
  flash_sector_erase(0x000000);
  //flash_erase_entire_chip();
  flash_wait_busy();
  ring_buf_init();



  /* Initialize variables, test, and start timer */
  uint16_t pres_u16 = 0;

  int16_t s_lowG[3] = {0};
  int16_t s_highG[3] = {0};
  int16_t s_gyro[3] = {0};

  float p_hpa = 0;

  char gga[128];
  uint8_t  latD = 0xFF, lonD = 0xFF;      // Latitude and longitude degrees
  uint16_t latM = 0xFFFF, lonM = 0xFFFF;  // Latitude and longitude minutes

  g_p0_set = 0;
  g_ab.inited = 0;

  HAL_TIM_Base_Start_IT(&htim3);	//start timer interrupt at 10ms


  /* START FLIGHT LOOP */
  while (1)
  {
	  if(g_pendingSamples > 0)
	  {
		  g_pendingSamples--;

		  //Read Barom
		  if (barom_read(&p_hpa) == 0)
		  {
		    // AB filter update
			const float dt = 0.01f;

		    // latch ground/reference pressure once
		    if (!g_p0_set) {
		      g_p0_hpa = p_hpa;
		      g_p0_set = 1;

		      // good starting gains for 10ms
		      ab_init(&g_ab, 0.0f, 0.0f, 0.20f, 0.05f);
		    }

		    // convert pressure -> altitude measurement (meters relative to p0)
		    float alt_meas_m = pressure_to_alt(p_hpa, g_p0_hpa);

		    // update filter
		    ab_update(&g_ab, alt_meas_m, dt);
		  }


		  //Read IMU
		  if (imu_read(s_lowG, s_highG, s_gyro) == 0)
		  {
			  // lowG ~ (0,0, +something) when sitting still
			  // gyro ~ (0,0,0) when not rotating
			  // highG ~ (0,0,0) unless you smack it / vibrate it
		  }


		 //TODO: add a "gps_service" function that lets us know every time a new gps line has been parsed. We will only get new gps data every ~100ms
//		 if (gps_get_data(gga, sizeof(gga), 3000))
//		 {
//		     // gga now has some NMEA line
//		 }
//		 if (!strncmp(gga, "$GNGGA,", 7) || !strncmp(gga, "$GPGGA,", 7))
//		 {
//		     if (gps_gga(gga, &latD, &latM, &lonD, &lonM))
//		     {
//		         // pack
//		     }
//		 }
			float vel = g_ab.v; //in m/s
		  float altitude = alt_meas_m; //in m
		  float accel = s_lowG[0] * 0.488 * 9.8 * 1/1000; //in m/s^2
		  
		  //Flight state machine
		  switch (g_flightState)
		  	  {
		  	  	  case FS_PAD:
					static int acc_launch = 0;
		  	  	  if (accel > 20)	//s_lowG[0] > 0
		  		  {
		  			  acc_launch += 1;
		  		  }
				else
				  {
					acc_launch = 0;
				  }
					if(acc_launch > 3)
					{
						g_flightState = FS_BOOST;
					}
					  
		  		  break;

		  		case FS_BOOST:
				static int acc_coast = 0;
		  	  	  if (accel < -0.5)	//s_lowG[0] > 0
		  		  {
		  			  acc_coast += 1;
		  		  }
				else
				  {
					acc_coast = 0;
				  }
					if(acc_coast > 3)
					{
						g_flightState = FS_COAST;
					}
		  		  }
		  		  break;

		  		case FS_COAST:
		  		  static int move_apogee = 0;
		  	  	  if (altitude > 3000 && vel < -100)	
		  		  {
		  			  move_apogee += 1;
		  		  }
				else
				  {
					move_apogee = 0;
				  }
					if(move_apogee > 3)
					{
						g_flightState = FS_APOGEE;
					}
		  		  }
		  		  break;

		  		case FS_APOGEE:
		  		  light_apogee_pyro();
		  		  tim3_set_period_counts(99);
		  		  g_flightState = FS_DROGUE_DESCENT;
		  		  break;

		  		case FS_DROGUE_DESCENT:

					static int alt_deploy = 0;
		  	  	  if (altitude < 500)
		  		  {
		  			  alt_deploy += 1;
		  		  }
				else
				  {
					alt_deploy = 0;
				  }
					if(alt_deploy > 3)
					{
						g_flightState = FS_MAIN_DESCENT;
					}
		  		  break;

		  		case FS_MAIN_DESCENT:
{
    float altitude_est = g_ab.x;   // filtered altitude
    float velocity_est = g_ab.v;

    // magnitude of acceleration vector (simple)
    float accel_mag = sqrtf(
        s_lowG[0]*s_lowG[0] +
        s_lowG[1]*s_lowG[1] +
        s_lowG[2]*s_lowG[2]
    );

    // Check landing conditions
    if ((altitude_est < LAND_ALT_THRESH) &&
        (fabsf(velocity_est) < LAND_VEL_THRESH) &&
        (fabsf(accel_mag - 1.0f) < LAND_ACCEL_THRESH))
    {
        landed_counter++;

        if (landed_counter >= LAND_COUNT_THRESH)
        {
            g_flightState = FS_LANDED;
            landed_counter = 0;
        }
    }
    else
    {
        landed_counter = 0; // reset if unstable
    }
}
break;

		  		case FS_LANDED:
		  			 tim3_set_period_counts(5000);  // much slower loop / telemetry

				    // Optional: indicate landed (LED, buzzer, etc.)
				    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		  		default:
		  			break;
		  	  }

		  //Create buffer to be sent
		  if (g_flightState != FS_PAD)
		  {
			  pack_buf(tx_buf, latD, latM, lonD, lonM, s_lowG, s_highG, s_gyro, g_ab.v, g_ab.x, (uint8_t)g_flightState);
		      write_ring_buf(tx_buf, PKT_LEN);
		  }
		  if (flash_ring_overflowed())
		  {
			  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		  }

		  if((g_flightState != FS_PAD) && g_LoRaTx)
		  {
			  //TODO: Function for transmitting data over LoRa
			  //Transmit latest data packet over LoRa
			  memcpy(lora_buf, tx_buf, PKT_LEN);
			  g_LoRaTx = 0;
		  }

		  memset(tx_buf, 0xFF, PKT_LEN);
	  }

	  //gps_service();
	  //LoRa_Service();
	  service_ring();
  }

}


/*-------------------------------------------------------------------------------*/


////////////////////////////////////
///* ALL USER DEFINED FUNCTIONS *///
////////////////////////////////////

static void light_apogee_pyro(void)
{
  // Set PC6 and 7 HIGH
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  g_pyroActive = 1;
  g_pyroTicks = 0;
}

static void light_main_pyro(void)
{
  // Set PC8 and 9 HIGH
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

static inline void pack_buf(uint8_t tx_buf[PKT_LEN],
                            uint8_t lat_deg, uint16_t lat_min,
                            uint8_t lon_deg, uint16_t lon_min,
                            const int16_t lowG[3],
                            const int16_t highG[3],
                            const int16_t gyro[3],
							float vel,
							float alt,
                            uint8_t flags)
{
    // while debugging: start from a known state
	memset(tx_buf, 0, PKT_LEN);

	// ID
	tx_buf[STATE] = (uint8_t)g_flightState;;
	tx_buf[CONT] = 0xBB;

	// GPS degrees/minutes
	tx_buf[LAT_DEG] = lat_deg;
	PAK_U16(tx_buf, LAT_MIN_LSB, lat_min);

	tx_buf[LON_DEG] = lon_deg;
	PAK_U16(tx_buf, LON_MIN_LSB, lon_min);

	// Altitude / Velocity
	int16_t vel_cms = (int16_t)lroundf(vel * 100.0f);  // m/s -> cm/s
	int16_t alt_cm  = (int16_t)lroundf(alt * 100.0f);  // m -> cm

	uint16_t vel_u16 = (uint16_t)vel_cms; // preserve bits
	uint16_t alt_u16 = (uint16_t)alt_cm;

	PAK_U16(tx_buf, VELO_LSB, vel_u16);
	PAK_U16(tx_buf, ALT_LSB, alt_u16);

	// IMU vectors (STM32 is little-endian; memcpy is fine and fast)
	memcpy(&tx_buf[LG_X_LSB], lowG,  3 * sizeof(int16_t));   // X/Y/Z
	memcpy(&tx_buf[HG_X_LSB], highG, 3 * sizeof(int16_t));
	memcpy(&tx_buf[GY_P_LSB], gyro,  3 * sizeof(int16_t));

	// Timestamp (4 bytes)
	PAK_U32(tx_buf, T_STAMP_1, g_timeStamp);

	// CONT
	tx_buf[FLAGS] = flags;

	// CRC placeholder
	tx_buf[CRC8] = 0xFF;
}


static inline void ab_init(ABFilter *f, float x0, float v0, float alpha, float beta)
{
  f->x = x0;
  f->v = v0;
  f->alpha = alpha;
  f->beta  = beta;
  f->inited = 1;
}

static inline void ab_update(ABFilter *f, float z_meas, float dt)
{
  // predict
  float x_pred = f->x + f->v * dt;

  // residual
  float r = z_meas - x_pred;

  // correct
  f->x = x_pred + f->alpha * r;
  f->v = f->v + (f->beta * r / dt);
}

static inline float pressure_to_alt(float p_hpa, float p0_hpa)
{
  // altitude relative to p0 (meters)
  return 44330.0f * (1.0f - powf(p_hpa / p0_hpa, 0.190294957f));
}

/*-------------------------------------------------------------------------------*/


////////////////////////////
///* Interrupt Handlers *///
////////////////////////////

/* 100ms Delay Logic */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	g_pendingSamples++;
	g_timeStamp++;		//Keeps track of time in 100's of ms

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
	//Toggle LED1 for testing
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

void tim3_set_period_counts(uint16_t arr)
{
    HAL_TIM_Base_Stop_IT(&htim3);

    __HAL_TIM_SET_PRESCALER(&htim3, 31999);
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    HAL_TIM_Base_Start_IT(&htim3);
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
			 HAL_GPIO_WritePin((LED2_GPIO_Port), (LED2_Pin), GPIO_PIN_SET);
			 HAL_GPIO_WritePin((LED3_GPIO_Port), (LED3_Pin), GPIO_PIN_SET);
			 HAL_GPIO_WritePin((LED4_GPIO_Port), (LED4_Pin), GPIO_PIN_SET);
		 }

		 __NOP();
	 }

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

	PIN_HIGH(Pyro_CTRL_Master_GPIO_Port, Pyro_CTRL_Master_Pin);
	__NOP();
	PIN_HIGH(Pyro_CTRL_1_GPIO_Port, Pyro_CTRL_1_Pin);
	PIN_HIGH(Pyro_CTRL_2_GPIO_Port, Pyro_CTRL_2_Pin);
	PIN_HIGH(Pyro_CTRL_3_GPIO_Port, Pyro_CTRL_3_Pin);
	PIN_HIGH(Pyro_CTRL_4_GPIO_Port, Pyro_CTRL_4_Pin);

	HAL_Delay(1000);		//1 second delay = 1000

	PIN_LOW(Pyro_CTRL_Master_GPIO_Port, Pyro_CTRL_Master_Pin);
	PIN_LOW(Pyro_CTRL_1_GPIO_Port, Pyro_CTRL_1_Pin);
	PIN_LOW(Pyro_CTRL_2_GPIO_Port, Pyro_CTRL_2_Pin);
	PIN_LOW(Pyro_CTRL_3_GPIO_Port, Pyro_CTRL_3_Pin);
	PIN_LOW(Pyro_CTRL_4_GPIO_Port, Pyro_CTRL_4_Pin);


	return;
}

void flash_read_id(uint8_t id[3])
{
    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    for (volatile int i = 0; i < 200; i++) __NOP(); // ~ a few µs depending on clock

    uint8_t tx[4] = {0x9f, 0x00, 0x00};
    uint8_t rx[4] = {0};

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 100);

    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    id[0] = rx[0];
    id[1] = rx[1];
    id[2] = rx[2];


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
