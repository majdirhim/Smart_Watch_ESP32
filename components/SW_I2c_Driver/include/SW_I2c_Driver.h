/**
 * @file SW_I2c_Driver.h
 *
 * @author Majdi Rhim
 * @brief Thread Safe I2c transaction
 * @version 0.1
 *
 * @date 2023-03-16
 *
 *
 */
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "freertos/semphr.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_FREQ_HZ      100000


/**
 * @brief Initializes the I2C master mode with the given GPIO pins
 * 
 * @param handle I2C port handle 
 * @param I2C_MASTER_SCL_IO The GPIO number for the SCL signal 
 * @param I2C_MASTER_SDA_IO The GPIO number for the SDA signal
 */
void SW_I2c_Master_Init(i2c_port_t handle,uint8_t I2C_MASTER_SCL_IO, uint8_t I2C_MASTER_SDA_IO);


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
 * @return esp_err_t Returns ESP_OK on success or an error code on failure.
 */
esp_err_t SW_Thread_Safe_I2c_Read(SemaphoreHandle_t* Jeton, i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, uint8_t *bufp,uint16_t len);



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
 * @return esp_err_t Returns ESP_OK on success or an error code on failure.
 */

esp_err_t SW_Thread_Safe_I2c_Write(SemaphoreHandle_t* Jeton,i2c_port_t handle, uint8_t ADD7bit ,uint8_t reg, const uint8_t *bufp, uint16_t len);


