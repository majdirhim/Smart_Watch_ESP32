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
#include "lis2mdl_reg.h"

#define LSM6DSO_ADDR_7BITS 0x6B //7bit lsm6dso address

#define LIS2MDL_ADDR_7BITS 0x1E //7bit lis2mdl address


/**
 * @brief Initializes the SW MEMS interface.
 *
 * @param handle The handle of the interface.
 * @return stmdev_ctx_t Returns the STM device context.
 *
 * @see platform_write()
 * @see platform_read()
 * @see platform_delay()
 */
stmdev_ctx_t SW_Mems_Interface_Init(uint8_t handle,uint8_t Mems_type);




/**
 * @brief Initializes and configures the LSM6DSO sensor.
 *
 * @param dev_ctx The device context for communication with the LSM6DSO sensor.
 *
 * @note Enables interrupt generation on the INT1 pin for (Activity/Inactivity)
 */
void SW_Lsm6dso6_Init_Config(stmdev_ctx_t dev_ctx);

void SW_Lis2mdl_Init_Config(stmdev_ctx_t dev_ctx);


int32_t Lsm6dso_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t Lsm6dso_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

int32_t Lis2mdl_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t Lis2mdl_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

void platform_delay(uint32_t ms);
