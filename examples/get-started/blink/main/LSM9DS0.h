/*
 * LSM9DS0.h
 *
 *  Created on: Nov 1, 2017
 *      Author: ian
 */

#ifndef MAIN_LSM9DS0_H_
#define MAIN_LSM9DS0_H_

#define XM_ADDR                            0x1D             /*!< Would be 0x1E if SDO_XM is LOW */
#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#include "LSM9DS0Ports.h"
#include "driver/i2c.h"
#include <esp_types.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static esp_err_t read_xm(uint8_t subaddr, uint8_t* data_rd, size_t size);

static esp_err_t write_xm(uint8_t subaddr, uint8_t* data_wr, size_t size);

/**
* @brief Make i2c master read bytes from slave subaddress.
*        @note
*        Only call this function in I2C master mode
*
* @param i2c_num i2c port to use
* @param slave_addr address of i2c slave to rea dfrom
* @param subaddr register to read from
* @param data_rd array to read bytes into
* @param size bytes to read
*
* @return
*     - ESP_OK Success
*     - ESP_ERR_INVALID_ARG Parameter error
*/
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t slave_addr,
		                               uint8_t subaddr, uint8_t* data_rd, size_t size);

/**
 * @brief Make i2c master write bytes to slave subaddress.
 *        @note
 *        Only call this function in I2C master mode
 *
 * @param i2c_num i2c port to use
 * @param slave_addr address of i2c slave to write to
 * @param subaddr register to write to
 * @param data_rd array to write from
 * @param size bytes to write
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_addr,
		                               uint8_t subaddr, uint8_t* data_wr, size_t size);

#endif /* MAIN_LSM9DS0_H_ */
