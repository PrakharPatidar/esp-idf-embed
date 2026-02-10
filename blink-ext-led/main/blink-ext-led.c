#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>

#define BLINK_GPIO 4


void app_main(void)
{
	gpio_config_t io_conf = {
		.pin_bit_mask = 1ULL << BLINK_GPIO,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io_conf);

	while (1) {
        printf("LED ON!\n");
		gpio_set_level(BLINK_GPIO, 1);
		vTaskDelay(pdMS_TO_TICKS(500));
        printf("LED OFF!\n");
		gpio_set_level(BLINK_GPIO, 0);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
