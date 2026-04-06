/*
 * sensor.h
 *
 *  Created on: Mar 5, 2026
 *      Author: easton
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#pragma once
#include <stdint.h>

int barom_init(void);
int barom_read(float *p_hpa);

int imu_init(void);
int imu_read(int16_t lowG[3], int16_t highG[3], int16_t gyro[3]);

int flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len);
int flash_read(uint32_t addr, uint8_t *data, uint16_t len);
void flash_init(void);
void flash_sector_erase(uint32_t addr);
uint8_t flash_read_status(void);
void flash_erase_entire_chip(void);
void flash_wait_busy(void);
uint8_t flash_append_packet(const uint8_t *packet, uint16_t len);
uint8_t flash_is_busy(void);
int flash_write_start(uint32_t addr, const uint8_t *data, uint16_t len);

void service_ring(void);
void ring_buf_init(void);
uint8_t write_ring_buf(const uint8_t *packet, uint16_t len);
uint8_t flash_ring_overflowed(void);

void gps_init(void);
int gps_gga(const char *gga,
                      uint8_t *lat_deg, uint16_t *lat_min_x1000,
                      uint8_t *lon_deg, uint16_t *lon_min_x1000);
int gps_get_data(char *out, int maxlen, uint32_t timeout_ms);

#endif /* INC_SENSOR_H_ */
