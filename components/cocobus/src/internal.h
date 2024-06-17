// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef COCOBUS_INTERNAL_H
#define COCOBUS_INTERNAL_H

#include <esp_err.h>

#define CC_GPIO_CLK     2

#define CC_GPIO_BUF_ADDR_HIGH   46
#define CC_GPIO_BUF_ADDR_LOW    45
#define CC_GPIO_BUF_DATA_EN     44
#define CC_GPIO_BUF_DATA_OUT    43

#define CC_GPIO_H0  35
#define CC_GPIO_H1  36
#define CC_GPIO_H2  37
#define CC_GPIO_H3  38
#define CC_GPIO_H4  39
#define CC_GPIO_H5  40
#define CC_GPIO_H6  41
#define CC_GPIO_H7  42



esp_err_t cocobus_timer_init(void);
esp_err_t cocobus_timer_set_slow(void);
esp_err_t cocobus_timer_set_fast(void);

#endif //COCOBUS_INTERNAL_H
