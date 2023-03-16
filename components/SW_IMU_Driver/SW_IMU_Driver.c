/**
 * @file SW_IMU_Driver.h
 *
 * @author Majdi Rhim
 * @brief LSM6DSO sensor
 * @version 0.1
 * @date 2023-03-16
 *
 */


#include "SW_IMU_Driver.h"


static uint8_t whoamI, rst;


/*
 * Use this Token for all I2c0 Transactions
 */
extern SemaphoreHandle_t I2c_Jeton ;

stmdev_ctx_t SW_Lsm6dso6_Init_Config(uint8_t handle){
	stmdev_ctx_t dev_ctx;
	lsm6dso_pin_int1_route_t int1_route;
	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = handle;
	/* Wait sensor boot time */
	platform_delay(10);
	/* Check device ID */
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID)
		while (1);

	/* Restore default configuration */
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	/* Set XL and Gyro Output Data Rate */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_208Hz);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_104Hz);
	/* Set 2g full XL scale and 250 dps full Gyro */
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_250dps);
	/* Set duration for Activity detection to 9.62 ms (= 2 * 1 / ODR_XL) */
	lsm6dso_wkup_dur_set(&dev_ctx, 0x02);
	/* Set duration for Inactivity detection to 4.92 s (= 2 * 512 / ODR_XL) */
	lsm6dso_act_sleep_dur_set(&dev_ctx, 0x02);
	/* Set Activity/Inactivity threshold to 62.5 mg */
	lsm6dso_wkup_threshold_set(&dev_ctx, 0x02);
	/* Inactivity configuration: XL to 12.5 in LP, gyro to Power-Down */
	lsm6dso_act_mode_set(&dev_ctx, LSM6DSO_XL_12Hz5_GY_PD);
	/* Enable interrupt generation on Inactivity INT1 pin */
	lsm6dso_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.sleep_change = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(&dev_ctx, int1_route);
	return dev_ctx;
}



int32_t platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, LSM6DSO_ADDR_7BITS, reg, bufp, len);
}



int32_t platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, LSM6DSO_ADDR_7BITS, reg, bufp, len);
}


void platform_delay(uint32_t ms){
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
