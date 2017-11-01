/*
 * LSM9DS0.c
 *
 *  Created on: Nov 1, 2017
 *      Author: ian
 */

#include "LSM9DS0.h"

/*
A_SCALE_2G,	// 000:  2g
A_SCALE_4G,	// 001:  4g
A_SCALE_6G,	// 010:  6g
A_SCALE_8G,	// 011:  8g
A_SCALE_16G	// 100:  16g

A_POWER_DOWN, 	// Power-down mode (0x0)
A_ODR_3125,		// 3.125 Hz	(0x1)
A_ODR_625,		// 6.25 Hz (0x2)
A_ODR_125,		// 12.5 Hz (0x3)
A_ODR_25,		// 25 Hz (0x4)
A_ODR_50,		// 50 Hz (0x5)
A_ODR_100,		// 100 Hz (0x6)
A_ODR_200,		// 200 Hz (0x7)
A_ODR_400,		// 400 Hz (0x8)
A_ODR_800,		// 800 Hz (9)
A_ODR_1600		// 1600 Hz (0xA)
*/
esp_err_t config_xm(uint8_t xm_scale, uint8_t xm_odr){
	// Set XM ODR
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:

	uint8_t one_byte = 0;
	esp_err_t ret;
	ret = read_xm(CTRL_REG1_XM, &one_byte, 1);
	if (ret != ESP_OK){
		return ret;
	}
	// Then mask out the accel ODR bits:
	one_byte &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	one_byte |= (xm_odr << 4);
	// And write the new register value back into CTRL_REG1_XM:
	write_xm(CTRL_REG1_XM, &one_byte, 1);
	return ESP_OK;

}

static esp_err_t read_xm(uint8_t subaddr, uint8_t* data_rd, size_t size){
	return i2c_master_read_slave(I2C_EXAMPLE_MASTER_NUM, XM_ADDR, subaddr, data_rd, size);
}

static esp_err_t write_xm(uint8_t subaddr, uint8_t* data_wr, size_t size){
	return i2c_master_read_slave(I2C_EXAMPLE_MASTER_NUM, XM_ADDR, subaddr, data_wr, size);
}

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t slave_addr,
		                               uint8_t subaddr, uint8_t* data_rd, size_t size)
{
	esp_err_t ret;
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    if (size > 1){
    	// MSb of SUB field has to be '1' in order for SUB to automatically
    	// increment to enable multiple data reads/writes.
    	i2c_master_write_byte(cmd, (subaddr | 0x80), ACK_CHECK_EN);
    } else {
    	i2c_master_write_byte(cmd, subaddr, ACK_CHECK_EN);
    }
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(30 / portTICK_RATE_MS);
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        // ACK_VAL = 0x0 -> receiver must pull data line LOW during HIGH period of acknowledge clock
    	// pulse in order to acknowledge.
    	i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);

    }
    i2c_master_read_byte(cmd, *(data_rd + size - 1), NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_addr,
		                               uint8_t subaddr, uint8_t* data_wr, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    if (size > 1){
    	// MSb of SUB field has to be '1' in order for SUB to automatically
    	// increment to enable multiple data reads/writes.
    	i2c_master_write_byte(cmd, (subaddr | 0x80), ACK_CHECK_EN);
    } else {
    	i2c_master_write_byte(cmd, subaddr, ACK_CHECK_EN);
    }
    if (size > 1) {
        // ACK_VAL = 0x0 -> receiver must pull data line LOW during HIGH period of acknowledge clock
    	// pulse in order to acknowledge.
    	i2c_master_write(cmd, data_wr, size - 1, ACK_VAL);

    }
    i2c_master_write_byte(cmd, *(data_wr + size - 1), NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
