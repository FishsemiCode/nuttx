/****************************************************************************
 * arch/csky/include/ch2201/chip.h
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_CSKY_INCLUDE_CH2201_CHIP_H
#define __ARCH_CSKY_INCLUDE_CH2201_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
.macro set_prio_cpu_ahb
    lrw     a0, 0x40000000
    movi    a1, 0x1
    stw     a1, (a0, 0x0)
    movi    a1, 0x2
    stw     a1, (a0, 0xc)
    movi    a1, 0x3
    stw     a1, (a0, 0x4)
    movi    a1, 0x4
.endm

.macro restore_eflash_state
    /* restore the eflash state when system reboot from deep sleep */
    lrw     a0, 0x40002000
    ldw     a1, (r0, 0)
    btsti   a1, 5
    bf      .EXIT
    lrw     a0, 0x4003f000
    movi    a1, 0x35
    stw     a1, (a0, 0x24)
    movi    a1, 0x16
    stw     a1, (a0, 0x28)
    movi    a1, 0x35
    stw     a1, (a0, 0x2c)
    movi    a1, 0x1b9
    stw     a1, (a0, 0x30)
    movi    a1, 0x8b10
    stw     a1, (a0, 0x34)
.EXIT:
.endm

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_CSKY_INCLUDE_CH2201_CHIP_H */

