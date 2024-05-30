// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(chameleon);

int main(void)
{
    LOG_INF("Hello %s\n", CONFIG_BOARD_TARGET);
    return 0;
}