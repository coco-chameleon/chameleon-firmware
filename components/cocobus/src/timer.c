// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <driver/mcpwm_prelude.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_intr_alloc.h>
#include <soc/soc.h>
#include <soc/mcpwm_periph.h>

#include "internal.h"


#define CC_MCPWM_GROUP 0

static const char *TAG = "cocobus";


struct cocobus_timer_segment {
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr_set;
    mcpwm_cmpr_handle_t cmpr_clear;
    mcpwm_gen_handle_t gen;
};

struct cocobus_timer {
    mcpwm_timer_handle_t timer;
    struct cocobus_timer_segment addr_high;
    struct cocobus_timer_segment addr_low;
    struct cocobus_timer_segment data;
    mcpwm_gen_handle_t data_gen_dir;
    intr_handle_t intr;
};

struct cocobus_timer timer;


static esp_err_t set_addr_valid(int tick)
{
    esp_err_t result;
# define CMPR_SET(comparator, value) \
    do {                             \
        result = mcpwm_comparator_set_compare_value((comparator), (value)); \
        if (result != ESP_OK) return result; \
    } while (0)

    CMPR_SET(timer.data.cmpr_clear,      2); // 16.66 ns from cycle start
    CMPR_SET(timer.addr_high.cmpr_set,   tick - 6); // interrupt latency
    CMPR_SET(timer.addr_high.cmpr_clear, tick + 1);
    CMPR_SET(timer.addr_low.cmpr_set,    tick + 1);
    CMPR_SET(timer.addr_low.cmpr_clear,  tick + 2);
    CMPR_SET(timer.data.cmpr_set,        tick + 2);
#undef CMPR_SET
    return ESP_OK;
}

esp_err_t cocobus_timer_set_slow(void)
{
    return set_addr_valid(24); // ~200 ns
}

esp_err_t cocobus_timer_set_fast(void)
{
    return set_addr_valid(12); // ~100 ns
}

static esp_err_t segment_init(
    struct cocobus_timer_segment *this,
    int gpio_num
) {
    esp_err_t result;

    mcpwm_operator_config_t operator_config = {
        .group_id = CC_MCPWM_GROUP,
        .flags = {
            .update_gen_action_on_sync = true,
            .update_dead_time_on_sync = true,
        },
    };
    result = mcpwm_new_operator(&operator_config, &this->oper);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create operator");

    result = mcpwm_operator_connect_timer(this->oper, timer.timer);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to connect operator to timer");

    mcpwm_comparator_config_t cmpr_config = {
        .flags = {
            .update_cmp_on_sync = true,
        },
    };

    result = mcpwm_new_comparator(this->oper, &cmpr_config, &this->cmpr_set);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create set comparator");

    result = mcpwm_new_comparator(this->oper, &cmpr_config, &this->cmpr_clear);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create clear comparator");

    mcpwm_generator_config_t gen_config = {
            .gen_gpio_num = gpio_num,
            .flags = {
                .invert_pwm = true,
            },
    };

    result = mcpwm_new_generator(this->oper, &gen_config, &this->gen);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create generator");

    mcpwm_gen_compare_event_action_t evt_set = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = this->cmpr_set,
        .action = MCPWM_GEN_ACTION_HIGH,
    };
    result = mcpwm_generator_set_action_on_compare_event(this->gen, evt_set);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up set event");

    mcpwm_gen_compare_event_action_t evt_clear = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = this->cmpr_clear,
            .action = MCPWM_GEN_ACTION_LOW,
    };
    result = mcpwm_generator_set_action_on_compare_event(this->gen, evt_clear);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up clear event");

    return ESP_OK;
}

esp_err_t cocobus_timer_init(void)
{
    esp_err_t result;

    mcpwm_timer_config_t timer_config = {
        .group_id = CC_MCPWM_GROUP,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .resolution_hz = 80000000, // 12.5 ns, 3 instructions
        .period_ticks = 12500, // 1 ms watchdog for host clock loss
        .flags = {
            .update_period_on_sync = true,
        },
    };

    result = mcpwm_new_timer(&timer_config, &timer.timer);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create timer");

    mcpwm_gpio_sync_src_config_t gpio_sync_config = {
        .group_id = CC_MCPWM_GROUP,
        .gpio_num = CC_GPIO_CLK,
        .flags = {
            .active_neg = 0,
        },
    };

    mcpwm_sync_handle_t gpio_sync = NULL;
    result = mcpwm_new_gpio_sync_src(&gpio_sync_config, &gpio_sync);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create sync source");

    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .sync_src = gpio_sync,
    };
    result = mcpwm_timer_set_phase_on_sync(timer.timer, &sync_phase_config);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to connect sync source");

    result = segment_init(&timer.addr_high, CC_GPIO_BUF_ADDR_HIGH);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up addr_high segment");

    result = segment_init(&timer.addr_low, CC_GPIO_BUF_ADDR_LOW);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up addr_low segment");

    result = segment_init(&timer.data, CC_GPIO_BUF_DATA_EN);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up data segment");

    result = cocobus_timer_set_slow();
    ESP_RETURN_ON_ERROR(result, TAG, "failed to configure thresholds");


    // in case the timer resets in the middle of an address phase
    // ensure the address selects are off to avoid conflicts
    mcpwm_gen_timer_event_action_t evt_sync = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .event = MCPWM_TIMER_EVENT_EMPTY,
            .action = MCPWM_GEN_ACTION_LOW,
    };
    result = mcpwm_generator_set_action_on_timer_event(timer.addr_high.gen, evt_sync);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up addr_high sync event");
    result = mcpwm_generator_set_action_on_timer_event(timer.addr_low.gen, evt_sync);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up addr_low sync event");

    mcpwm_generator_config_t data_gen_config = {
            .gen_gpio_num = CC_GPIO_BUF_DATA_OUT,
            .flags = {
                    .invert_pwm = true,
            },
    };
    result = mcpwm_new_generator(timer.data.oper, &data_gen_config, &timer.data_gen_dir);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to create generator");

    mcpwm_gen_compare_event_action_t evt_data_clear = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = timer.data.cmpr_clear,
            .action = MCPWM_GEN_ACTION_LOW,
    };
    result = mcpwm_generator_set_action_on_compare_event(timer.data_gen_dir, evt_data_clear);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to set up clear event");


    result = mcpwm_timer_enable(timer.timer);
    ESP_RETURN_ON_ERROR(result, TAG, "failed to enable timer");


    result = esp_intr_alloc(
        mcpwm_periph_signals.groups[CC_MCPWM_GROUP].irq_id,
        ESP_INTR_FLAG_NMI | ESP_INTR_FLAG_IRAM,
        NULL, // xt_nmi defined in interrupt.S
        NULL,
        &timer.intr
    );
    ESP_RETURN_ON_ERROR(result, TAG, "failed to allocate interrupt");


    ESP_LOGI(TAG, "cocobus_timer_init complete");
    return ESP_OK;
}
