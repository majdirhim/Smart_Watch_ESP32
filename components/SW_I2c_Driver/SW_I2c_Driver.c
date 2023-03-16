/**
 * @file SW_I2c_Driver.c
 * @author Majdi Rhim
 * @brief Thread Safe I2c transaction
 * @version 0.1
 * @date 2023-03-16
 *
 *
 */


#include "SW_I2c_Driver.h"
#include "stdio.h"

/************** Private Functions ************************/

/**
 * @brief Writes data to an I2C device.
 *
 * @param handle The handle of the I2C port.
 * @param ADD7bit The 7-bit address of the I2C device to write to.
 * @param reg The register address to write to.
 * @param bufp A pointer to the buffer that contains the data to write.
 * @param len The length of the data to write.
 * @return esp_err_t Returns ESP_OK on success or an error code on failure.
 */
static esp_err_t SW_I2c_Write(i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, const uint8_t *bufp, uint16_t len);

/**
 * @brief Read data from a register of an I2C device
 *
 * @param handle I2C port handle
 * @param ADD7bit 7-bit address of the I2C device
 * @param reg Register address to read from
 * @param bufp Pointer to the buffer to store the read data
 * @param len Length of the data to be read
 * @return esp_err_t Returns ESP_OK on success or an error code on failure.
 */
static esp_err_t SW_I2c_Read(i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, uint8_t *bufp,uint16_t len);

/************** Private Functions ************************/



/**
 * @brief Thread-safe read function for I2C communication
 * This function provides thread-safe read operation for I2C communication. It takes a semaphore handle and i2c classic arguments
 *
 * @param Jeton Pointer to the semaphore handle
 * @param handle I2C port handle
 * @param ADD7bit 7-bit address of the I2C device
 * @param reg Register address to read from
 * @param bufp Pointer to the buffer to store the read data
 * @param len Length of the data to be read
 * @return esp_err_t Returns ESP_OK==> 0  on success or an error code ==>1 on failure.
 */

esp_err_t SW_Thread_Safe_I2c_Read(SemaphoreHandle_t* Jeton, i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, uint8_t *bufp,uint16_t len){
	esp_err_t ret=0;

	int ret_rtos= xSemaphoreTake(*Jeton,(TickType_t)500/portTICK_PERIOD_MS);

	if(!ret_rtos)
		return ESP_ERR_TIMEOUT;
	else{
		ret = SW_I2c_Read(handle, ADD7bit ,reg, bufp,len);
	    ret_rtos = xSemaphoreGive(*Jeton); // Libérer le jeton
	}
	return (!(ESP_OK==ret && ret_rtos));
}



/**
 * @brief Thread-safe write function for I2C communication
 * This function provides thread-safe write operation for I2C communication. It takes a semaphore handle and i2c classic arguments
 *
 * @param Jeton Pointer to the semaphore handle
 * @param handle I2C port handle
 * @param ADD7bit 7-bit address of the I2C device
 * @param reg Register address to write to
 * @param bufp Pointer to the data to be written
 * @param len Length of the data to be written
 * @return esp_err_t Returns ESP_OK==> 0  on success or an error code ==>1 on failure.
 */

esp_err_t SW_Thread_Safe_I2c_Write(SemaphoreHandle_t* Jeton,i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, const uint8_t *bufp, uint16_t len){
	esp_err_t ret;
	int ret_rtos= xSemaphoreTake(*Jeton,(TickType_t)500/portTICK_PERIOD_MS);
	if(!ret_rtos)
		return ESP_ERR_TIMEOUT;
	else{
		ret = SW_I2c_Write(handle, ADD7bit ,reg, bufp,len);
	    ret_rtos = xSemaphoreGive(*Jeton); // Libérer le jeton
	}
	return (!(ESP_OK==ret && ret_rtos));
}



/**
 * @brief Writes data to an I2C device.
 * 
 * @param handle The handle of the I2C port. 
 * @param ADD7bit The 7-bit address of the I2C device to write to. 
 * @param reg The register address to write to. 
 * @param bufp A pointer to the buffer that contains the data to write. 
 * @param len The length of the data to write. 
 * @return esp_err_t Returns ESP_OK==> 0  on success or an error code ==>1 on failure.
 */
static esp_err_t SW_I2c_Write(i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);  // Start bit
    i2c_master_write_byte(cmd, (ADD7bit <<1 ) | WRITE_BIT, ACK_CHECK_EN); //slave_addr + wr_bit + ack
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); //  write 1 byte (register address) + ack
    i2c_master_write(cmd, bufp, len, ACK_CHECK_EN); // write n bytes + ack
    i2c_master_stop(cmd); //Stop
    esp_err_t ret = i2c_master_cmd_begin(handle, cmd, 1000 / portTICK_PERIOD_MS); //Send the Tram
    i2c_cmd_link_delete(cmd);
    return ret;
}



/**
 * @brief Read data from a register of an I2C device
 * 
 * @param handle I2C port handle 
 * @param ADD7bit 7-bit address of the I2C device 
 * @param reg Register address to read from 
 * @param bufp Pointer to the buffer to store the read data 
 * @param len Length of the data to be read 
 * @return esp_err_t Returns ESP_OK==> 0  on success or an error code ==>1 on failure.
 */
static esp_err_t SW_I2c_Read(i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, uint8_t *bufp,uint16_t len)
{
	   if (len == 0) {
	        return ESP_OK;
	    }
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, (ADD7bit <<1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, (ADD7bit <<1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
	    if (len > 1) {
	        i2c_master_read(cmd, bufp, len - 1, ACK_VAL); // read len-1 bytes + Ack
	    }
	    i2c_master_read_byte(cmd, bufp + len - 1, NACK_VAL); // read the last byte + NACK to end  transcations
	    i2c_master_stop(cmd);
	    esp_err_t ret = i2c_master_cmd_begin(handle, cmd, 1000 / portTICK_PERIOD_MS);
	    i2c_cmd_link_delete(cmd);
	    return ret;
}

/**
 * @brief Initializes the I2C master mode with the given GPIO pins
 *
 * @param handle I2C port handle
 * @param I2C_MASTER_SCL_IO The GPIO number for the SCL signal
 * @param I2C_MASTER_SDA_IO The GPIO number for the SDA signal
 */
void SW_I2c_Master_Init(i2c_port_t handle,uint8_t I2C_MASTER_SCL_IO, uint8_t I2C_MASTER_SDA_IO){
    i2c_config_t conf;
    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(handle, &conf);
    i2c_driver_install(handle, conf.mode, 0, 0, 0);
}
