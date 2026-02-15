#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "BLDC_ESC";

#define ESC_PWM_GPIO                4
#define ESC_PWM_FREQ_HZ             50
#define ESC_MIN_PULSE_US            1000
#define ESC_FIXED_PULSE_US          1250
#define ESC_ARM_TIME_MS             4000
#define ESC_SETTLE_TIME_MS          2000

#define ESC_LEDC_TIMER              LEDC_TIMER_0
#define ESC_LEDC_CHANNEL            LEDC_CHANNEL_0
#define ESC_LEDC_MODE               LEDC_LOW_SPEED_MODE
#define ESC_LEDC_RESOLUTION         LEDC_TIMER_14_BIT

static uint32_t pulse_us_to_duty(uint32_t pulse_width_us)
{
    const uint32_t max_duty = (1U << ESC_LEDC_RESOLUTION) - 1U;
    const uint32_t pwm_period_us = 1000000U / ESC_PWM_FREQ_HZ;

    if (pulse_width_us > pwm_period_us) {
        pulse_width_us = pwm_period_us;
    }

    return (pulse_width_us * max_duty) / pwm_period_us;
}

static esp_err_t esc_set_pulse_us(uint32_t pulse_width_us)
{
    const uint32_t duty = pulse_us_to_duty(pulse_width_us);
    esp_err_t err = ledc_set_duty(ESC_LEDC_MODE, ESC_LEDC_CHANNEL, duty);
    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(ESC_LEDC_MODE, ESC_LEDC_CHANNEL);
}

void app_main(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = ESC_LEDC_MODE,
        .duty_resolution = ESC_LEDC_RESOLUTION,
        .timer_num = ESC_LEDC_TIMER,
        .freq_hz = ESC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .gpio_num = ESC_PWM_GPIO,
        .speed_mode = ESC_LEDC_MODE,
        .channel = ESC_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = ESC_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

    ESP_LOGI(TAG, "Arming ESC on GPIO %d with %dus pulse for %dms", ESC_PWM_GPIO, ESC_MIN_PULSE_US, ESC_ARM_TIME_MS);
    ESP_ERROR_CHECK(esc_set_pulse_us(ESC_MIN_PULSE_US));
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME_MS));

    ESP_LOGI(TAG, "Starting motor at fixed pulse: %dus", ESC_FIXED_PULSE_US);
    ESP_ERROR_CHECK(esc_set_pulse_us(ESC_FIXED_PULSE_US));
    vTaskDelay(pdMS_TO_TICKS(ESC_SETTLE_TIME_MS));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
