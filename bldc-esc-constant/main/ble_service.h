#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef esp_err_t (*ble_throttle_setter_t)(uint8_t throttle_percent);

esp_err_t ble_service_init(ble_throttle_setter_t throttle_setter);
