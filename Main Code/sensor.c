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
#include "ux_api.h"
#include "app_usbx_device.h"
#include "ux_device_cdc_acm.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_api.h"
#include "usb.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
static LPS22HH_Object_t baro;
static stmdev_ctx_t imu_ctx;

#define SPI_READ_BIT   (0x80)
#define SPI_AUTO_INC   (0x40)

//Page defines
#define PAGE_SIZE      256
#define PKTS_PER_PAGE  8
#define PAGE_DATA_LEN  PAGE_SIZE  // 252

#define PRE_FLIGHT_BUF_SIZE   1536
#define PRE_FLIGHT_MAX_PKTS   (PRE_FLIGHT_BUF_SIZE / PKT_LEN)

#define FLASH_TOTAL_SIZE   0x200000UL   // 2 MBytes
#define FLASH_RING_START_ADDR   (6U * PAGE_SIZE)   // 0x600
#define FLASH_DUMP_CHUNK      64U
#define FLASH_DUMP_START_ADDR   0x000000UL
#define FLASH_DUMP_TEST_SIZE    0x001000UL   // 4 KB test
#define FLASH_DUMP_FULL_SIZE    0x200000UL   // 2 MB flash
#define FLASH_DUMP_CHUNK_SIZE   256U
#define FLASH_DUMP_CHUNK_SIZE 256U


volatile uint32_t g_dump_total_size = 0;
volatile uint32_t g_dump_chunks_sent = 0;
volatile uint32_t g_dump_bytes_sent = 0;
static uint32_t preflight_flash_addr = 0x000000UL;
static uint8_t pre_flight_buf[PRE_FLIGHT_BUF_SIZE] = {0};
static uint16_t pre_flight_write_pkt = 0;
static uint16_t pre_flight_pkt_count = 0;
static uint8_t pre_flight_wrapped = 0;
static uint16_t pre_flight_flush_read_pkt = 0;
static uint16_t pre_flight_flush_remaining = 0;
static uint8_t pre_flight_flush_active = 0;

static uint8_t  preflight_page_buf[PAGE_SIZE];

static uint8_t  preflight_page_write_active = 0;

static uint16_t preflight_flush_pkt_index = 0;
static uint16_t preflight_flush_remaining_pkts = 0;

#define PIN_LOW(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)
#define PIN_HIGH(port, pin)  HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

#define FLASH_WP_ENABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_RESET)
#define FLASH_WP_DISABLE_PROTECT()	HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_SET)

void flash_write_enable(void);

static int32_t barom_platform_gettick(void);


/* =========================
   BUFFER STATE
   ========================= */
typedef enum
{
    BUF_EMPTY = 0,
    BUF_FILLING,
    BUF_READY,
    BUF_WRITING
} buf_state_t;


/* =========================
   BUFFER STRUCT (RAM ONLY)
   ========================= */
typedef struct
{
    uint8_t data[PAGE_SIZE];
    uint16_t index;
    uint16_t seq;
    buf_state_t state;
} FlashBuf_t;

static FlashBuf_t g_bufs[4];

static FlashBuf_t *g_fillBuf = NULL;
static FlashBuf_t *g_writeBuf = NULL;

static uint16_t g_nextSeq = 0;
static uint32_t g_flashAddr = 0;

static uint8_t g_writeInProgress = 0;
static uint8_t g_overflow = 0;

//for testing
static uint8_t 	rx_buf[256] = {0};
static uint8_t 	flashBuf[256] = {0};

static uint8_t g_flash_dump_buf[FLASH_DUMP_CHUNK_SIZE];


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
  if (LPS22HH_PRESS_SetOutputDataRate(&baro, 100.0f) != LPS22HH_OK) return -3;	//Pressure and Temp share the same ODR

  // This starts continuous conversions
  if (LPS22HH_PRESS_Enable(&baro) != LPS22HH_OK) return -4;

  return 0; // initialized (note: still power-down until BAROM_Start called)
}


int barom_read(float *p_hpa)
{
  if (!p_hpa) return -1;

  if (LPS22HH_PRESS_GetPressure(&baro, p_hpa) != LPS22HH_OK) return -2;

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

    g_flashAddr = 0;
    ring_buf_init();

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
    flash_wait_busy();


    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
}

void flash_erase_entire_chip(void)
{
    uint32_t addr;

    flash_write_enable();
    flash_wait_busy();

    for (addr = 0; addr < 0x200000; addr += 0x1000)
    {
        flash_sector_erase(addr);
        flash_wait_busy();
    }
}

int flash_write_start(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL) return -1;
    if (len == 0 || len > 256) return -2;
    if (((addr & 0xFF) + len) > 256) return -3;
    if (flash_is_busy()) return -4;

    flash_write_enable();

    uint8_t tx[4 + 256];

    tx[0] = 0x02;   // Page Program
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    memcpy(&tx[4], data, len);

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi2, tx, 4 + len, 100);

    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    if (st != HAL_OK) return -5;

    return 0;
}



int flash_read(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (data == NULL) return -1;
    if (len == 0) return -2;

    flash_wait_busy();

    uint8_t tx[4 + 256] = {0};
    uint8_t rx[4 + 256] = {0};

    if (len > 256) return -3;

    tx[0] = 0x03;   // Read
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4 + len, 100);

    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    if (st != HAL_OK) return -4;

    memcpy(data, &rx[4], len);
    return 0;
}

void flash_write_enable(void)
{
    uint8_t cmd = 0x06;   // Write Enable

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
}

int flash_wait_busy(void)
{
	{
	    uint32_t t0 = HAL_GetTick();

	    while (flash_read_status() & 0x01)
	    {
	        if ((HAL_GetTick() - t0) >= 250)
	        {
	            return -1;
	        }
	    }

	    return 0;
	}
}

uint8_t flash_is_busy(void)
{
    return (flash_read_status() & 0x01);
}


uint8_t flash_read_status(void)
{
    uint8_t tx[2] = {0x05, 0x00};   // Read Status Register
    uint8_t rx[2] = {0};

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    return rx[1];
}

void flash_flush_partial_page(void)
{
    if (g_fillBuf && g_fillBuf->index > 0)
    {
        memset(&g_fillBuf->data[g_fillBuf->index],
               0xFF,
               PAGE_SIZE - g_fillBuf->index);

        g_fillBuf->seq = g_nextSeq++;
        g_fillBuf->state = BUF_READY;
        g_fillBuf = NULL;
    }
}

uint8_t flash_has_pending_pages(void)
{
    if (g_writeInProgress)
    {
        return 1;
    }

    for (int i = 0; i < 4; i++)
    {
        if (g_bufs[i].state == BUF_READY || g_bufs[i].state == BUF_WRITING)
        {
            return 1;
        }
    }

    return 0;
}

/*------------------ LoRa functions -----------------*/

/*------------------ Ring Buffer Functions -----------------*/
/*
1. This design avoids sitting in flash_wait_busy() inside the flight loop.

2. The ring buffer prevents overwriting:
   - PAGE_FILLING  = current packet collection page
   - PAGE_READY    = full page waiting to be written
   - PAGE_WRITING  = page currently being written to flash
   - PAGE_EMPTY    = free page that can be reused

3. A page is only reused after:
   - write started
   - BUSY went low later
   - service function marks page EMPTY

4. If all 4 pages become used up before flash catches up:
   - g_fillPage becomes NULL
   - g_flashOverflow is set
   - new packets cannot be accepted until a page finishes writing

5. You should still erase flash before using it.

6. This does NOT flush partial pages at end of run.
   If you want to save the last incomplete page later,
   add a flush function.
*/
void ring_buf_init(void)
{
    int i;

    for (i = 0; i < 4; i++)
    {
        memset(g_bufs[i].data, 0xFF, PAGE_SIZE);
        g_bufs[i].index = 0;
        g_bufs[i].seq = 0;
        g_bufs[i].state = BUF_EMPTY;
    }

    g_fillBuf = &g_bufs[0];
    g_fillBuf->state = BUF_FILLING;

    g_writeBuf = NULL;
    g_nextSeq = 0;

    /*
     * First 6 pages are reserved for preflight/original data.
     * Start normal ring buffer logging on the 7th page.
     */
    g_flashAddr = FLASH_RING_START_ADDR;   // 0x600

    g_writeInProgress = 0;
    g_overflow = 0;
}

static FlashBuf_t *find_empty_buf(void)
{
    for (int i = 0; i < 4; i++)
    {
        if (g_bufs[i].state == BUF_EMPTY)
        {
            return &g_bufs[i];
        }
    }

    return NULL;
}

static FlashBuf_t *find_oldest_ready_buf(void)
{
    FlashBuf_t *best = NULL;

    for (int i = 0; i < 4; i++)
    {
        if (g_bufs[i].state == BUF_READY)
        {
            if (!best || g_bufs[i].seq < best->seq)
            {
                best = &g_bufs[i];
            }
        }
    }

    return best;
}


uint8_t write_ring_buf(const uint8_t *packet, uint16_t len)
{
    if (!packet || len != PKT_LEN || !g_fillBuf)
    {
        return 0;
    }

    /*
     * With PKT_LEN = 32 and PAGE_DATA_LEN = 256,
     * exactly 8 packets fit per flash page.
     */
    if ((g_fillBuf->index + len) > PAGE_DATA_LEN)
    {
        return 0;
    }

    memcpy(&g_fillBuf->data[g_fillBuf->index], packet, len);
    g_fillBuf->index += len;

    if (g_fillBuf->index == PAGE_DATA_LEN)
    {
        /*
         * No footer anymore.
         * The entire 256-byte page is packet data.
         */

        g_fillBuf->seq = g_nextSeq++;
        g_fillBuf->state = BUF_READY;

        FlashBuf_t *next = find_empty_buf();

        if (!next)
        {
            g_fillBuf = NULL;
            g_overflow = 1;
            return 0;
        }

        memset(next->data, 0xFF, PAGE_SIZE);
        next->index = 0;
        next->seq = 0;
        next->state = BUF_FILLING;

        g_fillBuf = next;
    }

    return 1;
}

void service_ring(void)
{
    FlashBuf_t *buf;

    /*
     * If a write is ongoing, check if flash is still busy.
     */
    if (g_writeInProgress)
    {
        if (flash_is_busy())
        {
            return;
        }

//        /*
//         * Optional readback test.
//         * Since g_flashAddr now starts at FLASH_RING_START_ADDR,
//         * check against that instead of PAGE_SIZE.
//         */
//        if (g_flashAddr > FLASH_RING_START_ADDR)
//        {
//            flash_read(g_flashAddr - PAGE_SIZE, flashReadBack, PAGE_SIZE);
//            // compare flashReadBack[] to g_writeBuf->data[]
//            __NOP();   // inspect flashReadBack here
//        }

        /*
         * Write finished. Reuse this RAM buffer.
         */
        memset(g_writeBuf->data, 0xFF, PAGE_SIZE);
        g_writeBuf->index = 0;
        g_writeBuf->seq = 0;
        g_writeBuf->state = BUF_EMPTY;

        g_writeBuf = NULL;
        g_writeInProgress = 0;

        /*
         * Recover if the fill buffer stalled because all buffers were full.
         */
        if (!g_fillBuf)
        {
            buf = find_empty_buf();

            if (buf)
            {
                memset(buf->data, 0xFF, PAGE_SIZE);
                buf->index = 0;
                buf->seq = 0;
                buf->state = BUF_FILLING;

                g_fillBuf = buf;
                g_overflow = 0;
            }
        }

        return;
    }

    /*
     * Start a new flash write if a full page is ready.
     */
    buf = find_oldest_ready_buf();

    if (!buf)
    {
        return;
    }

    /*
     * Do not write past flash memory.
     */
    if ((g_flashAddr + PAGE_SIZE) > FLASH_TOTAL_SIZE)
    {
        return;
    }

    /*
     * Start writing exactly one 256-byte page.
     */
    if (flash_write_start(g_flashAddr, buf->data, PAGE_SIZE) == 0)
    {
        buf->state = BUF_WRITING;
        g_writeBuf = buf;
        g_writeInProgress = 1;

        g_flashAddr += PAGE_SIZE;
    }
}


uint8_t flash_ring_overflowed(void)
{
    return g_overflow;
}

void preflight_buf_reset(void)
{
    memset(pre_flight_buf, 0xFF, PRE_FLIGHT_BUF_SIZE);

    pre_flight_write_pkt = 0;
    pre_flight_pkt_count = 0;
    pre_flight_wrapped = 0;
}

void preflight_buf_append(const uint8_t *tx_buf)
{
    if (tx_buf == NULL)
    {
        return;
    }

    uint16_t byte_index = pre_flight_write_pkt * PKT_LEN;

    memcpy(&pre_flight_buf[byte_index], tx_buf, PKT_LEN);

    pre_flight_write_pkt++;

    if (pre_flight_write_pkt >= PRE_FLIGHT_MAX_PKTS)
    {
        pre_flight_write_pkt = 0;
        pre_flight_wrapped = 1;
    }

    if (pre_flight_pkt_count < PRE_FLIGHT_MAX_PKTS)
    {
        pre_flight_pkt_count++;
    }
}

void preflight_buf_flush_to_flash(uint32_t *flash_addr)
{
    if (flash_addr == NULL)
    {
        return;
    }

    uint16_t start_pkt;

    if (pre_flight_pkt_count < PRE_FLIGHT_MAX_PKTS)
    {
        // Buffer never wrapped, oldest packet is at slot 0
        start_pkt = 0;
    }
    else
    {
        // Buffer wrapped, oldest packet is where the next write would happen
        start_pkt = pre_flight_write_pkt;
    }

    for (uint16_t i = 0; i < pre_flight_pkt_count; i++)
    {
        uint16_t pkt_index = (start_pkt + i) % PRE_FLIGHT_MAX_PKTS;
        uint16_t byte_index = pkt_index * PKT_LEN;

        flash_wait_busy();
        flash_page_program(*flash_addr, &pre_flight_buf[byte_index], PKT_LEN);
        flash_wait_busy();

        *flash_addr += PKT_LEN;
    }
}

/*------------------ Old Functions -----------------*/
int flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (data == NULL) return -1;
    if (len == 0 || len > 256) return -2;
    if (((addr & 0xFF) + len) > 256) return -3;

    flash_wait_busy();
    flash_write_enable();

    uint8_t tx[4 + 256];

    tx[0] = 0x02;   // Page Program
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    memcpy(&tx[4], data, len);

    PIN_LOW(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi2, tx, 4 + len, 100);

    PIN_HIGH(SPI2_CS_Flash_GPIO_Port, SPI2_CS_Flash_Pin);

    if (st != HAL_OK) return -4;

    return 0;
}

uint8_t preflight_flush_service(uint8_t max_pages)
{
    uint8_t pages_started = 0;

    if (!pre_flight_flush_active)
    {
        return 0;
    }

    /*
     * If a preflight page write was already started, wait for it to finish.
     * Do NOT block. Just return and try again next loop.
     */
    if (preflight_page_write_active)
    {
        if (flash_is_busy())
        {
            return 0;
        }

        preflight_page_write_active = 0;
    }

    while ((pages_started < max_pages) && (preflight_flush_remaining_pkts > 0))
    {
        /*
         * Do not overlap the normal ring area.
         */
        if ((preflight_flash_addr + PAGE_SIZE) > FLASH_RING_START_ADDR)
        {
            pre_flight_flush_active = 0;
            return pages_started;
        }

        /*
         * If the flash is busy because the normal ring started a write,
         * do not block.
         */
        if (flash_is_busy())
        {
            return pages_started;
        }

        /*
         * Build one 256-byte page from up to 8 preflight packets.
         */
        memset(preflight_page_buf, 0xFF, PAGE_SIZE);

        uint16_t packets_this_page = preflight_flush_remaining_pkts;

        if (packets_this_page > PKTS_PER_PAGE)
        {
            packets_this_page = PKTS_PER_PAGE;
        }

        for (uint16_t i = 0; i < packets_this_page; i++)
        {
            uint8_t *src = &pre_flight_buf[preflight_flush_pkt_index * PKT_LEN];
            uint8_t *dst = &preflight_page_buf[i * PKT_LEN];

            memcpy(dst, src, PKT_LEN);

            preflight_flush_pkt_index++;

            if (preflight_flush_pkt_index >= PRE_FLIGHT_MAX_PKTS)
            {
                preflight_flush_pkt_index = 0;
            }

            preflight_flush_remaining_pkts--;
        }

        /*
         * Start one non-blocking page program.
         * flash_write_start() should only launch the program command.
         * It should not wait for completion.
         */
        if (flash_write_start(preflight_flash_addr, preflight_page_buf, PAGE_SIZE) != 0)
        {
            return pages_started;
        }

        preflight_flash_addr += PAGE_SIZE;
        preflight_page_write_active = 1;
        pages_started++;

        /*
         * Important: return after starting one page write.
         * Let future calls wait for busy to clear.
         */
        return pages_started;
    }

    if (preflight_flush_remaining_pkts == 0 && !preflight_page_write_active)
    {
        pre_flight_flush_active = 0;
    }

    return pages_started;
}

uint8_t preflight_flush_done(void)
{
    return ((pre_flight_flush_active == 0U) &&
            (preflight_page_write_active == 0U));
}

void preflight_flush_start(void)
{
    if (pre_flight_wrapped)
    {
        preflight_flush_pkt_index = pre_flight_write_pkt;
    }
    else
    {
        preflight_flush_pkt_index = 0;
    }

    preflight_flush_remaining_pkts = pre_flight_pkt_count;
    pre_flight_flush_active = (preflight_flush_remaining_pkts > 0) ? 1U : 0U;

    preflight_flash_addr = 0x000000UL;

    preflight_page_write_active = 0;
}

void dump_flash_over_usb(uint32_t start_addr, uint32_t total_size)
{
    uint32_t addr = start_addr;
    uint32_t remaining = total_size;

    const char begin_msg[] = "\r\nROCKET_FLASH_DUMP_BEGIN\r\n";
    const char end_msg[]   = "\r\nROCKET_FLASH_DUMP_END\r\n";

    g_dump_total_size = total_size;
    g_dump_chunks_sent = 0;
    g_dump_bytes_sent = 0;

    usb_cdc_write_all((const uint8_t *)begin_msg, sizeof(begin_msg) - 1U);

    while (remaining > 0U)
    {
        uint32_t chunk = remaining;

        if (chunk > FLASH_DUMP_CHUNK_SIZE)
        {
            chunk = FLASH_DUMP_CHUNK_SIZE;
        }

        flash_read(addr, g_flash_dump_buf, chunk);

        if (usb_cdc_write_all(g_flash_dump_buf, chunk) != UX_SUCCESS)
        {
            const char err_msg[] = "\r\nROCKET_FLASH_DUMP_ABORTED\r\n";
            usb_cdc_write_all((const uint8_t *)err_msg, sizeof(err_msg) - 1U);
            return;
        }

        addr += chunk;
        remaining -= chunk;

        g_dump_chunks_sent++;
        g_dump_bytes_sent += chunk;
    }

    usb_cdc_write_all((const uint8_t *)end_msg, sizeof(end_msg) - 1U);
}

void usb_init(void)
{
	  MX_USBX_Device_Standalone_Init();
	  MX_USB_PCD_Init();
	  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00, PCD_SNG_BUF, 0x40);
	  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80, PCD_SNG_BUF, 0x80);
	  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x03, PCD_SNG_BUF, 0x140);
	  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x83, PCD_SNG_BUF, 0x180);
	  HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x82, PCD_SNG_BUF, 0x1C0);
	  _ux_dcd_stm32_initialize((ULONG)USB_DRD_FS,
	                           (ULONG)&hpcd_USB_DRD_FS);
	  HAL_NVIC_ClearPendingIRQ(USB_DRD_FS_IRQn);
	  HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
	  HAL_PCD_Start(&hpcd_USB_DRD_FS);
}

void usb_dump(uint32_t timeout)
{
	  uint8_t dump_done = 0;
	  uint32_t usb_start_time = HAL_GetTick();


	  while (timeout > 0)
	  {
		  timeout--;
	      _ux_system_tasks_run();

	      if ((dump_done == 0U) &&
	          ((HAL_GetTick() - usb_start_time) > 8000U))
	      {
	          dump_done = 1U;

	          // First test: only 256 bytes
	          dump_flash_over_usb(0x00000UL, 0x200000UL);
	      }

	      HAL_Delay(10);
	  }
}
