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


/**
 * @brief Initializes the SW MEMS interface.
 * 
 * @param handle The handle of the interface.
 * @param Mems_type Type of Sensor interface being initialized (0=>Lsm6dso 1=>Lis2mdl)
 * @return stmdev_ctx_t Returns the STM device context.
 * 
 * @see platform_write()
 * @see platform_read()
 * @see platform_delay()
 */

stmdev_ctx_t SW_Mems_Interface_Init(uint8_t handle,uint8_t Mems_type){
	stmdev_ctx_t dev_ctx;
	if(!Mems_type){
		/* Initialize mems driver interface */
		dev_ctx.write_reg = Lsm6dso_platform_write; // @suppress("Field cannot be resolved")
		dev_ctx.read_reg = Lsm6dso_platform_read; // @suppress("Field cannot be resolved")
		dev_ctx.handle = handle; // @suppress("Field cannot be resolved")
		/* Wait sensor boot time */
		platform_delay(10);
		return dev_ctx;
	}
	/* Initialize Lis2mdl mems driver interface */
	dev_ctx.write_reg = Lis2mdl_platform_write;// @suppress("Field cannot be resolved")
	dev_ctx.read_reg = Lis2mdl_platform_read;// @suppress("Field cannot be resolved")
	dev_ctx.handle = handle;// @suppress("Field cannot be resolved")
	/* Wait sensor boot time */
	platform_delay(10);
return dev_ctx;
}

/**
 * @brief Initializes and configures the LSM6DSO sensor.
 * 
 * @param dev_ctx The device context for communication with the LSM6DSO sensor.
 * 
 * @note Enables interrupt generation on the INT1 pin for (Activity/Inactivity)
 */
void SW_Lsm6dso6_Init_Config(stmdev_ctx_t dev_ctx){
	lsm6dso_pin_int1_route_t int1_route;
	/* Check device ID */
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID)
		while (1)
			printf("Device LSM6DSO Not found\n\r");

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
}




void SW_Lis2mdl_Init_Config(stmdev_ctx_t dev_ctx){
	  /* Wait sensor boot time */
	  platform_delay(10);
	  /* Check device ID */
	  lis2mdl_device_id_get(&dev_ctx, &whoamI);

	  if (whoamI != LIS2MDL_ID)
	    while (1)
	      printf("Device LIS2MDL Not found %d\n\r",whoamI);

	  /* Restore default configuration */
	  lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);

	  do {
	    lis2mdl_reset_get(&dev_ctx, &rst);
	  } while (rst);

	  /* Enable Block Data Update */
	  lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);
	  /* Set / Reset sensor mode */
	  lis2mdl_set_rst_mode_set(&dev_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
	  /* Enable temperature compensation */
	  lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
	  /* Set Low-pass bandwidth to ODR/4 */
	  //lis2mdl_low_pass_bandwidth_set(&dev_ctx, LIS2MDL_ODR_DIV_4);
	  /* Set device in continuous mode */
	  lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);
	  /* Enable interrupt generation on new data ready */
	  lis2mdl_drdy_on_pin_set(&dev_ctx, PROPERTY_ENABLE);

}


int32_t Lsm6dso_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, LSM6DSO_ADDR_7BITS, reg, bufp, len);
}



int32_t Lsm6dso_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, LSM6DSO_ADDR_7BITS, reg, bufp, len);
}




int32_t Lis2mdl_platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	/* Write multiple command */
	reg |= 0x80;
	return SW_Thread_Safe_I2c_Write(&I2c_Jeton, handle, LIS2MDL_ADDR_7BITS, reg, bufp, len);
}
int32_t Lis2mdl_platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	/* Read multiple command */
	reg |= 0x80;
	return  SW_Thread_Safe_I2c_Read(&I2c_Jeton, handle, LIS2MDL_ADDR_7BITS, reg, bufp, len);
}


void platform_delay(uint32_t ms){
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
