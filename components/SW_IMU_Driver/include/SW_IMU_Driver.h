/**
 * @file SW_IMU_Driver.h
 * @author Majdi Rhim
 * @brief LSM6DSO sensor
 * @version 0.1
 * @date 2023-03-16
 *
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "SW_I2c_Driver.h"
#include "lsm6dso_reg.h"

#define LSM6DSO_ADDR_7BITS 0x6B //7bit lsm6dso address



stmdev_ctx_t SW_Lsm6dso6_Init_Config(uint8_t handle);


int32_t platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
void platform_delay(uint32_t ms);
