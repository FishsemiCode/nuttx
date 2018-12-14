/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 /******************************************************************************
 * @file     vectors.S
 * @brief    define default vector handlers. Should use with
 *           GCC for CSKY Embedded Processors
 * @version  V1.0
 * @date     28. Nove 2017
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "up_arch.h"

.import trap_c

.section .bss
    .align  2
    .globl  g_trapstackalloc
    .global g_trapstackbase
    .global g_top_trapstack
g_trapstackalloc:
g_trapstackbase:
    .space 768
g_top_trapstack:

    .align 2
    .globl g_trap_sp
    .type  g_trap_sp, object
g_trap_sp:
    .long 0
    .size g_trap_sp, .-g_trap_sp

    .data
    .align 2
    .globl g_tmp_sp
    .type  g_tmp_sp, object
g_tmp_sp:
    .long 0
    .size g_tmp_sp, .-g_tmp_sp


.text

/************************************************************************************
 * Public Functions
 ************************************************************************************/
    .align 2
    .globl csky_get_vec_num
    .type  csky_get_vec_num, %function
csky_get_vec_num:
    mfcr    a0, psr
    lsri    a0, 16
    sextb   a0
    rts

    .size   csky_get_vec_num, . - csky_get_vec_num


/******************************************************************************
 * Functions:
 *     void NOVIC_IRQ_Default_Handler(void);
 * novic default irq entry
 ******************************************************************************/
    .align 2
    .global NOVIC_IRQ_Default_Handler
    .type   NOVIC_IRQ_Default_Handler, %function
NOVIC_IRQ_Default_Handler:
    psrset  ee
    subi    sp, 8
    stw     r0, (sp, 0)
    stw     r1, (sp, 4)
    lrw     r0, g_tmp_sp
    mov     r1, sp
    addi    r1, 8
    stw     r1, (r0, 0)
    ldw     r0, (sp, 0)
    ldw     r1, (sp, 4)
    addi    sp, 8

    lrw     sp, g_top_irqstack

    subi    sp, XCPTCONTEXT_SIZE
    stm     r0-r13, (sp)
    lrw     r0, g_tmp_sp
    addi    r1, sp, REG_VR0*4
    fstms   vr0-vr15, (r1)
    ldw     r0, (r0, 0)
    stw     r0, (sp, REG_SP*4)
    stw     r15, (sp, REG_LR*4)

    mfcr    r0, epsr
    stw     r0, (sp, REG_PSR*4)
    mfcr    r0, epc
    stw     r0, (sp, REG_PC*4)

    mov     r0, sp
    jbsr    up_decodeirq

    mov     r15, r0

    ldw     r0, (r15, REG_PSR*4)
    mtcr    r0, epsr
    ldw     r0, (r15, REG_PC*4)
    mtcr    r0, epc

    addi    r0, r15, REG_VR0*4
    fldms   vr0-vr15, (r0)

    ldm     r0-r14, (r15)
    ldw     r15, (r15, REG_R15*4)

    rte

/******************************************************************************
 * Functions:
 *     void trap(void);
 * default exception handler
 ******************************************************************************/
    .align 2
    .global trap
    .type   trap, %function
trap:
    psrset  ee
    subi    sp, 4
    stw     r0, (sp)

    lrw     r0, g_top_trapstack
    subi    r0, 140
    stm     r0-r15, (r0)
    stw     r28, (r0, 64)

    ldw     r1, (sp)
    addi    sp, 4
    stw     sp, (r0, 56) /* save r14 */
    mov     sp, r0
    stw     r1, (sp)     /* save r0 */

    addi    r0, 68
    fstms   vr0-vr15, (r0)
    mov     r0, sp

    mfcr    r1, epsr
    stw     r1, (sp, 132)
    mfcr    r1, epc
    stw     r1, (sp, 136)

    jbsr    trap_c

    .align  2
    .weak   Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    br      trap
    .size   Default_Handler, . - Default_Handler
