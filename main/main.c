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
#include "SW_IMU_Driver.h"


/*
 * Use this Token for all I2c0 Transactions
 */
SemaphoreHandle_t I2c_Jeton =NULL;

void app_main(void)
{

		  I2c_Jeton=xSemaphoreCreateBinary();
	      xSemaphoreGive(I2c_Jeton);
	      SW_I2c_Master_Init(I2C_NUM_0,19,18);
	      stmdev_ctx_t dev_ctx = SW_Lsm6dso6_Init_Config(I2C_NUM_0);

		  /* Wait Events */
		  while (1) {
		    lsm6dso_all_sources_t all_source;
		    /* Check if Activity/Inactivity events */
		    lsm6dso_all_sources_get(&dev_ctx, &all_source);
		    //printf("Reading srcs %ld\n\r",dbug);
		    if (all_source.sleep_state)
		      printf("Inactivity Detected \n\r");
		    if (all_source.wake_up)
		      printf("Activity Detected \n\r");

		  }
}




