// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include <xtensa/coreasm.h>
#include <xtensa/corebits.h>
#include <xtensa/config/system.h>
#include <soc/soc.h>


    .section    .iram1, "ax"
    .global     xt_highint5
    .type       xt_highint5, @function
    .align      4

xt_highint5:
    ee.set_bit_gpio_out 1
    nop
    nop
    ee.clr_bit_gpio_out 1

    rsr     a0, EXCSAVE_5
    rfi     5


    .global     ld_include_cocobus_interrupt
ld_include_cocobus_interrupt:
