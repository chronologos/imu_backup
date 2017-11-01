/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "LSM9DS0.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
	// Configure I2C master
//	int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
//	i2c_config_t conf;
//	conf.mode = I2C_MODE_MASTER;
//	conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
//	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//	conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
//	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//	conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
//	i2c_param_config(i2c_master_port, &conf);
//	i2c_driver_install(i2c_master_port, conf.mode,
//					   I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
//	                   I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
