#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include "ble_service.h"

static const char *TAG = "BLDC_ESC";

#define ESC_PWM_GPIO                4
#define ESC_PWM_FREQ_HZ             50
#define ESC_MIN_PULSE_US            1000
#define ESC_MAX_PULSE_US            2000
#define ESC_ARM_TIME_MS             4000
#define ESC_REARM_TIME_MS           1200
#define ESC_START_MIN_PULSE_US      1200
#define ESC_START_KICK_PULSE_US     1400
#define ESC_START_KICK_TIME_MS      300
#define ESC_RAMP_STEP_US            10
#define ESC_RAMP_STEP_TIME_MS       20

#define ESC_LEDC_TIMER              LEDC_TIMER_0
#define ESC_LEDC_CHANNEL            LEDC_CHANNEL_0
#define ESC_LEDC_MODE               LEDC_LOW_SPEED_MODE
#define ESC_LEDC_RESOLUTION         LEDC_TIMER_14_BIT

static SemaphoreHandle_t s_target_mutex;
static uint8_t s_target_throttle_percent;

static uint32_t throttle_percent_to_pulse_us(uint8_t throttle_percent)
{
    if (throttle_percent > 100) {
        throttle_percent = 100;
    }

    const uint32_t pulse_range = ESC_MAX_PULSE_US - ESC_MIN_PULSE_US;
    return ESC_MIN_PULSE_US + ((pulse_range * throttle_percent) / 100U);
}

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

static void esc_step_towards_pulse_us(uint32_t target_pulse_us, uint32_t *current_pulse_us)
{
    if (*current_pulse_us < target_pulse_us) {
        uint32_t next_pulse = *current_pulse_us + ESC_RAMP_STEP_US;
        if (next_pulse > target_pulse_us) {
            next_pulse = target_pulse_us;
        }
        *current_pulse_us = next_pulse;
        return;
    }

    if (*current_pulse_us > target_pulse_us) {
        uint32_t next_pulse;
        if (*current_pulse_us > (target_pulse_us + ESC_RAMP_STEP_US)) {
            next_pulse = *current_pulse_us - ESC_RAMP_STEP_US;
        } else {
            next_pulse = target_pulse_us;
        }
        *current_pulse_us = next_pulse;
    }
}

static void motor_control_task(void *arg)
{
    (void)arg;

    uint32_t current_pulse_us = ESC_MIN_PULSE_US;
    uint8_t last_target_throttle_percent = 0;
    uint32_t ramp_log_counter = 0;

    while (1) {
        uint8_t target_throttle_percent = 0;
        if (xSemaphoreTake(s_target_mutex, portMAX_DELAY) == pdTRUE) {
            target_throttle_percent = s_target_throttle_percent;
            xSemaphoreGive(s_target_mutex);
        }

        if (target_throttle_percent != last_target_throttle_percent) {
            ESP_LOGI(TAG,
                     "Motor task target update: %u%% -> %u%%",
                     last_target_throttle_percent,
                     target_throttle_percent);
            last_target_throttle_percent = target_throttle_percent;
        }

        const uint32_t target_pulse_us = throttle_percent_to_pulse_us(target_throttle_percent);
        const uint32_t pulse_before_step_us = current_pulse_us;

        if (target_throttle_percent > 0 &&
            current_pulse_us == ESC_MIN_PULSE_US &&
            target_pulse_us > ESC_MIN_PULSE_US) {
            ESP_LOGI(TAG, "Re-arm sequence before start");
            ESP_ERROR_CHECK(esc_set_pulse_us(ESC_MIN_PULSE_US));
            vTaskDelay(pdMS_TO_TICKS(ESC_REARM_TIME_MS));

            uint32_t startup_pulse_us = target_pulse_us;
            if (startup_pulse_us < ESC_START_KICK_PULSE_US) {
                startup_pulse_us = ESC_START_KICK_PULSE_US;
            }

            current_pulse_us = startup_pulse_us;
            ESP_LOGI(TAG, "Startup assist pulse: %luus", (unsigned long)current_pulse_us);
            ESP_ERROR_CHECK(esc_set_pulse_us(current_pulse_us));
            vTaskDelay(pdMS_TO_TICKS(ESC_START_KICK_TIME_MS));

            if (current_pulse_us > ESC_START_MIN_PULSE_US) {
                current_pulse_us = ESC_START_MIN_PULSE_US;
                ESP_ERROR_CHECK(esc_set_pulse_us(current_pulse_us));
                vTaskDelay(pdMS_TO_TICKS(ESC_RAMP_STEP_TIME_MS));
            }
        }

        esc_step_towards_pulse_us(target_pulse_us, &current_pulse_us);
        ESP_ERROR_CHECK(esc_set_pulse_us(current_pulse_us));

        if (current_pulse_us != pulse_before_step_us) {
            ramp_log_counter++;
            if ((ramp_log_counter % 5U) == 0U || current_pulse_us == target_pulse_us) {
                ESP_LOGI(TAG,
                         "PWM ramp: current=%luus target=%luus (%u%%)",
                         (unsigned long)current_pulse_us,
                         (unsigned long)target_pulse_us,
                         target_throttle_percent);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ESC_RAMP_STEP_TIME_MS));
    }
}

static esp_err_t esc_set_throttle_percent(uint8_t throttle_percent)
{
    if (throttle_percent > 100) {
        throttle_percent = 100;
    }

    if (s_target_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_target_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_target_throttle_percent = throttle_percent;
    xSemaphoreGive(s_target_mutex);

    ESP_LOGI(TAG, "Received throttle target: %u%%", throttle_percent);
    return ESP_OK;
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

    s_target_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_target_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    s_target_throttle_percent = 0;

    BaseType_t task_created = xTaskCreate(motor_control_task, "motor_ctrl", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_LOGI(TAG, "Motor is armed and waiting for BLE throttle command");

    const esp_err_t ble_err = ble_service_init(esc_set_throttle_percent);
    if (ble_err != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(ble_err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "BLE control ready");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
