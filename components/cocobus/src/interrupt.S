// SPDX-FileCopyrightText: 2024 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

/* High-priority interrupt for CoCo bus cycle.
 *
 * The NMI on core 1 is triggered by the MCPWM peripheral (see timer.c)
 * when the CoCo bus address becomes valid. It's responsible for reading
 * the Coco bus state in sync with the buffer enables managed by the MCPWM.
 *
 * We cannot call normal C code (which uses the windowed calling convention)
 * from this context because the NMI is a higher-priority exception than the
 * register window over/underflow exceptions so register spilling will not
 * occur. We can only call C functions which use the call0 convention. For
 * more information see the XISAS, particularly sections 4.4.1.11 and 6.1.
 *
 * Relevant documentation:
 *   ESP-IDF High Priority Interrupts:
 *     https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/api-guides/hlinterrupts.html
 *   ESP32-S3 Technical Reference Manual, Chapter 9 Interrupt Matrix:
 *     https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
 *   Xtensa Instruction Set Architecture Summary for all Xtensa LX Processors:
 *     https://www.cadence.com/content/dam/cadence-www/global/en_US/documents/tools/silicon-solutions/compute-ip/isa-summary.pdf
 */

#include <xtensa/coreasm.h>
#include <xtensa/corebits.h>
#include <xtensa/config/system.h>
#include <soc/soc.h>

#define NMI_FRAME_SIZE (6*4)


    .section    .iram1, "ax"

    // called by the actual vector in components/xtensa/xtensa_vectors.S
    .global     xt_nmi
    .type       xt_nmi, @function
    .align      4
xt_nmi:
    // hardware interrupt entry saves PS and PC and restores them on RFI
    // main vector saves A0 in EXCSAVE, we are responsible for restoring it
    // we are responsible for saving and restoring any other registers we use
    addi    sp, sp, -NMI_FRAME_SIZE
    s32i    a2, sp, 0*4
    s32i    a3, sp, 1*4
    s32i    a4, sp, 2*4
    s32i    a5, sp, 3*4
    s32i    a6, sp, 4*4
    s32i    a7, sp, 5*4

    // stand-in for bus handling
    ee.set_bit_gpio_out 1
    nop
    nop
    ee.clr_bit_gpio_out 1
    nop
    nop
    ee.set_bit_gpio_out 1
    nop
    nop
    ee.clr_bit_gpio_out 1

    // clear all MCPWM interrupts
    movi    a2, DR_REG_PWM0_BASE
    movi    a0, -1
    s32i    a0, a2, 0x11C // MCPWM_INT_CLR_REG

    // return from NMI
    l32i    a7, sp, 5*4
    l32i    a6, sp, 4*4
    l32i    a5, sp, 3*4
    l32i    a4, sp, 2*4
    l32i    a3, sp, 1*4
    l32i    a2, sp, 0*4
    addi    sp, sp, NMI_FRAME_SIZE
    rsr     a0, EXCSAVE + XCHAL_NMILEVEL
    rfi     XCHAL_NMILEVEL


    .global     ld_include_cocobus_interrupt
ld_include_cocobus_interrupt:
