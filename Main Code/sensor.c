/*
 * sensor.c
 *
 *  Created on: Mar 5, 2026
 *      Author: easton
 */
#include "sensor.h"
#include "lps22hh_reg.h"
#include "lps22hh.h"
#include "lsm6dsv80x_reg.h"
#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
static LPS22HH_Object_t baro;
static stmdev_ctx_t imu_ctx;
extern UART_HandleTypeDef huart2;

#define SPI_READ_BIT   (0x80)
#define SPI_AUTO_INC   (0x40)

//Page defines
#define PAGE_SIZE      256
#define PKTS_PER_PAGE  7
#define PAGE_DATA_LEN  (PKT_LEN * PKTS_PER_PAGE)  // 252

// Footer indices:
#define PAGE_SEQ_LSB   252
#define PAGE_SEQ_MSB   253
#define PAGE_CRC_LSB   254
#define PAGE_CRC_MSB   255


#define PIN_LOW(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define PIN_HIGH(port, pin)  HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

#define FLASH_WP_ENABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_RESET)
#define FLASH_WP_DISABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_SET)

void flash_write_enable(void);
static void flash_wait_busy(void);
static int nmea_dm_to_deg_min_u8_u16(const char *dm, int is_lon,
                                     uint8_t *deg_out, uint16_t *min_x1000_out);
static int32_t barom_platform_gettick(void);



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


static int32_t barom_platform_gettick(void)
{
  return (int32_t)HAL_GetTick();
}

static int32_t barom_platform_init(void)
{
  return 0; // LPS22HH_OK
}

static int32_t barom_platform_deinit(void)
{
  return 0;
}

int barom_init(void)
{
  LPS22HH_IO_t io = {
    .Init     = barom_platform_init,
    .DeInit   = barom_platform_deinit,
    .BusType  = LPS22HH_SPI_4WIRES_BUS,
    .Address  = 0, // ignored for SPI
    .WriteReg = barom_platform_write,
    .ReadReg  = barom_platform_read,
    .GetTick  = barom_platform_gettick,
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

  uint8_t addr = (uint8_t)(reg | 0x80u);

  // Build TX: [addr][dummy...]
  // RX:      [junk][data...]
  uint8_t tx[1 + 32];   // 32 is just a cap; increase if you read more
  uint8_t rx[1 + 32];

  if (len > 32) return -1;

  tx[0] = addr;
  for (uint16_t i = 0; i < len; i++) tx[1 + i] = 0x00;

  PIN_LOW(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);

  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(1 + len), 100);

  PIN_HIGH(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin);

  if (st != HAL_OK) return -1;

  // Copy out data bytes (skip the first junk byte)
  for (uint16_t i = 0; i < len; i++) data[i] = rx[1 + i];

  return 0;
}

static int32_t imu_platform_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  (void)handle;

  uint8_t addr = reg;                  // no 0x40

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

  if (lsm6dsv80x_auto_increment_set(&imu_ctx, 1) != 0) return -11;
  if (lsm6dsv80x_block_data_update_set(&imu_ctx, 1) != 0) return -10;

  // Set output data rates (pick something simple for testing)
  // Low-g accel ODR
  if (lsm6dsv80x_xl_setup(&imu_ctx, LSM6DSV80X_ODR_AT_120Hz, LSM6DSV80X_XL_HIGH_PERFORMANCE_MD) != 0) return -1; // enum is in reg.h
  //High-g accel ODR
  if (lsm6dsv80x_hg_xl_data_rate_set(&imu_ctx, LSM6DSV80X_HG_XL_ODR_AT_480Hz, 1) != 0) return -2;
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

    FLASH_WP_DISABLE_PROTECT();

    // HOLD# must be high when not used
    // If you have a HOLD pin, drive it high

    __NOP();

    // Proper WREN before ULBPR
    flash_write_enable();

    __NOP();

    cmd = 0x98;   // Global Block Protection Unlock
    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    __NOP();

	uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};   // JEDEC ID + 3 dummy bytes
	uint8_t rx[4] = {0};
	uint8_t id[3] = {0};

	PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 100);
	PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

	id[0] = rx[1];
	id[1] = rx[2];
	id[2] = rx[3];

	__NOP();   // inspect id[]
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

    //flash_wait_busy();
}

void flash_page_program(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t cmd[4];

    flash_wait_busy();
    HAL_Delay(20);

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

    //flash_wait_busy();
    HAL_Delay(20);

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


uint8_t flash_read_status(void)
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
void gps_init(void)
{
	HAL_Delay(1000);  // let GPS boot

	const char *cmd;

	// full cold start (use when you want to force reacquire)
	// cmd = "$PMTK104*37\r\n";
	// HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
	// HAL_Delay(200);

	// 1 Hz fix interval
	cmd = "$PMTK220,1000*1F\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_Delay(100);

//	// Enable SBAS for better efficiency after able to get a fix
//	cmd = "$PMTK313,1*2E\r\n";
//	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
//	HAL_Delay(100);

	// Output only GGA (1); everything else disabled (0). If want RMC data change value 2
	cmd = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_Delay(100);

	// query release (debug)
	cmd = "$PMTK605*31\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_Delay(100);
}


//Chat GPT function
static int nmea_dm_to_deg_min_u8_u16(const char *dm, int is_lon,
                                     uint8_t *deg_out, uint16_t *min_x1000_out)
{
    if (!dm || dm[0] == '\0') return 0;

    double v = atof(dm);                 // ddmm.mmmm / dddmm.mmmm
    int deg = (int)(v / 100.0);
    double minutes = v - (deg * 100.0);  // 0..60

    if (minutes < 0.0 || minutes >= 60.0) return 0;

    if (!is_lon) { if (deg < 0 || deg > 90)  return 0; }
    else         { if (deg < 0 || deg > 180) return 0; }

    long min_x1000 = lround(minutes * 1000.0); // 0..59999
    if (min_x1000 < 0) min_x1000 = 0;
    if (min_x1000 > 59999) min_x1000 = 59999;

    *deg_out = (uint8_t)deg;
    *min_x1000_out = (uint16_t)min_x1000;
    return 1;
}

// Returns 1 if fix quality > 0 and fields exist; else 0

//Chat GPT function
int gps_gga(const char *gga,
                      uint8_t *lat_deg, uint16_t *lat_min_x1000,
                      uint8_t *lon_deg, uint16_t *lon_min_x1000)
{
    if (!(strncmp(gga, "$GNGGA,", 7) == 0 || strncmp(gga, "$GPGGA,", 7) == 0))
        return 0;

    char buf[160];
    strncpy(buf, gga, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // fields: 2 lat, 3 N/S, 4 lon, 5 E/W, 6 fix quality
    char *save = NULL;
    char *tok = strtok_r(buf, ",", &save);
    int field = 0;

    const char *lat = NULL, *lon = NULL;
    int fixq = 0;

    while (tok)
    {
        if (field == 2) lat = tok;
        if (field == 4) lon = tok;
        if (field == 6) fixq = atoi(tok);

        tok = strtok_r(NULL, ",", &save);
        field++;
    }

    if (fixq <= 0) return 0;                 // no fix -> ignore
    if (!lat || !lon || lat[0]=='\0' || lon[0]=='\0') return 0;

    if (!nmea_dm_to_deg_min_u8_u16(lat, 0, lat_deg, lat_min_x1000)) return 0;
    if (!nmea_dm_to_deg_min_u8_u16(lon, 1, lon_deg, lon_min_x1000)) return 0;

    return 1;
}

int gps_get_data(char *out, int maxlen, uint32_t timeout_ms)
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

/*------------------ LoRa functions -----------------*/
