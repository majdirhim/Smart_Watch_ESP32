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

#include "esp_timer.h"
#include "esp_sleep.h"

#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define LSM6DSO_INT1 25 //GPIO25 => RTC_GPIO6

esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...);

/**
 * @note Use SW_SafePrint for thread safe UART communication
 */
SemaphoreHandle_t UART_Jeton =NULL;

/*
 * @note Use SW_I2c_Driver for thread safe i2c transaction
 */
SemaphoreHandle_t I2c_Jeton =NULL;

TaskHandle_t Lsm6dso_TASK_Handler ,StepCounter_Handler ;
TaskHandle_t LIS2MDL_TASK_Handler;
/************LIS2MDL Variables*********/
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float magnetic_mG[3];
static float temperature_degC;
/************LIS2MDL Variables*********/

stmdev_ctx_t Lsm6dso_dev_ctx;
stmdev_ctx_t Lis2mdl_dev_ctx;

static void IRAM_ATTR  Inactivity_Activity_IRQ(void * args){
	vTaskResume(Lsm6dso_TASK_Handler);
}


static void oneshot_timer_callback(void* arg){
	esp_light_sleep_start(); //when we reach here, it means that 20 seconds are already passed without any motion detected
}

void Lsm6dso_TASK(void * pvParameters ){
	/**************One shot timer for entering sleep mode*************/
	esp_timer_handle_t oneshot_timer;
	const esp_timer_create_args_t oneshot_timer_args = {
			.callback = &oneshot_timer_callback,
			.name = "one-shot"
	};
	esp_timer_create(&oneshot_timer_args, &oneshot_timer);
	/**************One shot timer for entering sleep mode*************/
	lsm6dso_all_sources_t all_source;
	for(;;){
		lsm6dso_all_sources_get(&Lsm6dso_dev_ctx, &all_source);
		if(all_source.sleep_state){
			esp_timer_start_once(oneshot_timer, 20000000); //20 seconds
			vTaskSuspend(StepCounter_Handler); //Stop step counting task
			SW_SafePrint(&UART_Jeton,"INACTIVITY ==> Sleep_State: %u\n\r",all_source.sleep_state);
		}else if(!all_source.sleep_state){
			esp_timer_stop(oneshot_timer);
			SW_SafePrint(&UART_Jeton,"ACTIVITY ==> Sleep_State : %u\n\r",all_source.sleep_state);
			vTaskResume(StepCounter_Handler);//Resume Step Counting
		}
		SW_SafePrint(&UART_Jeton,"LSM6dso Suicide\n\r");
		vTaskSuspend(Lsm6dso_TASK_Handler);//suspend Lsm6dso_Task (suicide)
	}
}

void StepCounter(void * pvParameters ){
	uint16_t steps;
	lsm6dso_emb_sens_t emb_sens;
	lsm6dso_steps_reset(&Lsm6dso_dev_ctx);
	/* Enable pedometer */
	lsm6dso_pedo_sens_set(&Lsm6dso_dev_ctx, LSM6DSO_FALSE_STEP_REJ_ADV_MODE);
	emb_sens.step = PROPERTY_ENABLE;
	emb_sens.step_adv = PROPERTY_ENABLE;
	lsm6dso_embedded_sens_set(&Lsm6dso_dev_ctx, &emb_sens);
	for(;;){
		lsm6dso_number_of_steps_get(&Lsm6dso_dev_ctx, &steps); //step Counting
		SW_SafePrint(&UART_Jeton,"steps :%d\r\n", steps);
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}


void LIS2MDL_TASK(void * pvParameters){
	uint8_t reg;
	double norm_x,norm_y,angle,angle_degree;
	for(;;){
		/* Read output only if new value is available */
		lis2mdl_mag_data_ready_get(&Lis2mdl_dev_ctx, &reg);
		if (reg) {
			/* Read magnetic field data */
			memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
			lis2mdl_magnetic_raw_get(&Lis2mdl_dev_ctx, data_raw_magnetic);
			magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[0]);
			magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[1]);
			magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[2]);
			//SW_SafePrint(&UART_Jeton,"Mag field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
			 norm_x=magnetic_mG[0]/sqrt(magnetic_mG[0]*magnetic_mG[0] + magnetic_mG[1]*magnetic_mG[1]+magnetic_mG[2]*magnetic_mG[2]);
			 norm_y=magnetic_mG[1]/sqrt(magnetic_mG[0]*magnetic_mG[0] + magnetic_mG[1]*magnetic_mG[1]+magnetic_mG[2]*magnetic_mG[2]);
			 angle = atan2(norm_x,norm_y);
			 angle_degree = angle*(180/M_PI)-1.1333; // 1.1333 poitiers-declination

			// print the result
			SW_SafePrint(&UART_Jeton, "The North direction relative to the sensor: %5.1f deg\r\n", angle_degree);
			/* Read temperature data */
			/*
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			lis2mdl_temperature_raw_get(&Lis2mdl_dev_ctx, &data_raw_temperature);
			temperature_degC =lis2mdl_from_lsb_to_celsius(data_raw_temperature);
			SW_SafePrint(&UART_Jeton,"Temperature [degC]:%6.2f\r\n",temperature_degC);*/
		}
		vTaskDelay(2000/portTICK_PERIOD_MS);
	}
}




void app_main(void)
{

	//UART Semaphore creation
	UART_Jeton=xSemaphoreCreateBinary();
	xSemaphoreGive(UART_Jeton);
	//I2c0 Semaphore Creation
	I2c_Jeton=xSemaphoreCreateBinary();
	xSemaphoreGive(I2c_Jeton);

	SW_I2c_Master_Init(I2C_NUM_0,I2C_MASTER_SCL_IO,I2C_MASTER_SDA_IO);
	Lsm6dso_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,0); //0=>Lsm6dso
	Lis2mdl_dev_ctx = SW_Mems_Interface_Init(I2C_NUM_0,1);//1=>Lis2mdl
	SW_Lsm6dso6_Init_Config(Lsm6dso_dev_ctx);
	SW_Lis2mdl_Init_Config(Lis2mdl_dev_ctx);

	xTaskCreate(StepCounter, "StepCounter", 10000, NULL, 1, &StepCounter_Handler);
	vTaskSuspend(StepCounter_Handler);
	xTaskCreate(Lsm6dso_TASK, "Lsm6dso_TASK", 10000, NULL, 1, &Lsm6dso_TASK_Handler);
	vTaskSuspend(Lsm6dso_TASK_Handler);

	xTaskCreate(LIS2MDL_TASK, "LIS2MDL_TASK", 10000, NULL, 1, &LIS2MDL_TASK_Handler);

	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/
	gpio_set_direction(LSM6DSO_INT1, GPIO_MODE_INPUT);
	gpio_set_intr_type(LSM6DSO_INT1,GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(LSM6DSO_INT1, Inactivity_Activity_IRQ,NULL);
	gpio_wakeup_enable(LSM6DSO_INT1, GPIO_INTR_HIGH_LEVEL);
	esp_sleep_enable_gpio_wakeup();
	/**************LSM6DSO_INT1 ISR / Wake-up Trigger *****************/

	/* Wait Events */
	while (1) {
		SW_SafePrint(&UART_Jeton,"MAIN TASK ORDO\n\r");
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

/**
 * @brief Prints a formatted string to the console using a semaphore to protect against concurrent access.
 *
 * @param Jeton Pointer to the semaphore handle used to protect access to the console
 *
 * @param ... Additional arguments to be printed according to the format string.
 *
 * @return ESP_ERR_TIMEOUT if the semaphore could not be acquired within 1000ms, otherwise returns 0.
 */
esp_err_t SW_SafePrint(SemaphoreHandle_t* Jeton,const char* fmt, ...){
	int ret_rtos= xSemaphoreTake(*Jeton,(TickType_t)1000/portTICK_PERIOD_MS);
	if(!ret_rtos)
		return ESP_ERR_TIMEOUT;
	va_list arg;
	va_start(arg, fmt);
	vprintf(fmt, arg);
	va_end(arg);
	return !(xSemaphoreGive(*Jeton)); //return 0 if ok
}


