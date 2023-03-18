#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "SW_I2c_Driver.h"
#include "lsm6dso_reg.h"
#include "lis2mdl_reg.h"
#include "SW_IMU_Driver.h"


/*
 * Use SW_I2c_Driver for thread safe i2c transaction
 */
SemaphoreHandle_t I2c_Jeton =NULL;
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float magnetic_mG[3];
static float temperature_degC;
void app_main(void)
{

		  I2c_Jeton=xSemaphoreCreateBinary();
	      xSemaphoreGive(I2c_Jeton);

	      SW_I2c_Master_Init(I2C_NUM_0,19,18);

	      stmdev_ctx_t Lsm6dso_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,0); //0=>Lsm6dso
	      stmdev_ctx_t Lis2mdl_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,1);//1=>Lis2mdl
	      SW_Lsm6dso6_Init_Config(Lsm6dso_dev_ctx);
	      SW_Lis2mdl_Init_Config(Lis2mdl_dev_ctx);
		  /* Wait Events */
		  while (1) {
		    lsm6dso_all_sources_t all_source;
		    uint8_t reg;
		    /* Read output only if new value is available */
		    lis2mdl_mag_data_ready_get(&Lis2mdl_dev_ctx, &reg);

		    if (reg) {
		      /* Read magnetic field data */
		      memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
		      lis2mdl_magnetic_raw_get(&Lis2mdl_dev_ctx, data_raw_magnetic);
		      magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[0]);
		      magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[1]);
		      magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[2]);
		      printf("Mag field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);

		      /* Read temperature data */
		      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		      lis2mdl_temperature_raw_get(&Lis2mdl_dev_ctx, &data_raw_temperature);
		      temperature_degC =lis2mdl_from_lsb_to_celsius(data_raw_temperature);
		      printf("Temperature [degC]:%6.2f\r\n",temperature_degC);
		    }
		    vTaskDelay(300/portTICK_PERIOD_MS);
		    /* Check if Activity/Inactivity events */
		    lsm6dso_all_sources_get(&Lsm6dso_dev_ctx, &all_source);
		    //printf("Reading srcs %ld\n\r",dbug);
		    if (all_source.sleep_state)
		      printf("Inactivity Detected \n\r");
		    if (all_source.wake_up)
		      printf("Activity Detected \n\r");

		  }
		  vTaskDelay(300/portTICK_PERIOD_MS);
}




