#include <sys/cdefs.h>
// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <driver/gpio.h>
#include <driver/dedic_gpio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#include "cocobus.h"
#include "internal.h"

static const char *TAG = "cocobus";

_Noreturn void cocobus_task(void *task_param)
{
    ESP_LOGI(TAG, "in cocobus_task");

    dedic_gpio_bundle_handle_t data_io = NULL;
    dedic_gpio_bundle_config_t data_io_config = {
        .gpio_array = (int[]){35, 36, 37, 38, 39, 40, 41, 42},
        .array_size = 8,
        .flags = {
            .in_en = 1,
            .out_en = 1,
        },
    };
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&data_io_config, &data_io));


    ESP_ERROR_CHECK(cocobus_timer_init());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t cocobus_init(void)
{
    TaskHandle_t task;
    BaseType_t result;

    ESP_LOGI(TAG, "in cocobus_init");

    result = xTaskCreatePinnedToCore(
        cocobus_task,
        "cocobus",
        4096,
        NULL,
        20,
        NULL,
        1 // pinned to app cpu
    );
    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
