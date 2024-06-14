// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#include "cocobus.h"


static const char* TAG = "main";


void app_main(void) {
    gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << 18),
            .pull_down_en = 0,
            .pull_up_en = 0,
    };

    gpio_config(&io_conf);
    gpio_set_level(18, 1);

    ESP_LOGW(TAG, "in app_main");

    ESP_ERROR_CHECK(cocobus_init());
}
