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
 * @file     startup.S
 * @brief    startup file for smartl. Should use with
 *           GCC for CSKY Embedded Processors
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/


#include <nuttx/config.h>
#include <arch/chip/chip.h>

.export __start

.section .vectors
    .align  10
    .globl  __Vectors
    .type   __Vectors, @object
__Vectors:
    .long   Reset_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler
    .long   Default_Handler

    /* External interrupts */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
    .long   NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */

    .size   __Vectors, . - __Vectors

    .text
    .long __start
    .align  2
__start:
    .text
    .align  2
    .globl  Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    lrw     r0, 0x80000200
    mtcr    r0, psr

/* Initialize the normal stack pointer from the linker definition. */
    lrw     r0, g_idle_topstack
    mov     sp, r0

/*
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
#if 1
    lrw     r1, __erodata
    lrw     r2, __data_start__
    lrw     r3, __data_end__

    subu    r3, r2
    cmpnei  r3, 0
    bf      .L_loop0_done

.L_loop0:
    ldw     r0, (r1, 0)
    stw     r0, (r2, 0)
    addi    r1, 4
    addi    r2, 4
    subi    r3, 4
    cmpnei  r3, 0
    bt      .L_loop0

.L_loop0_done:
#endif
/*
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
    lrw     r1, __bss_start__
    lrw     r2, __bss_end__

    movi    r0, 0

    subu    r2, r1
    cmpnei  r2, 0
    bf      .L_loop1_done

.L_loop1:
    stw     r0, (r1, 0)
    addi    r1, 4
    subi    r2, 4
    cmpnei  r2, 0
    bt      .L_loop1
.L_loop1_done:

#ifndef __NO_SYSTEM_INIT
    jbsr    SystemInit
#endif

#ifndef __NO_BOARD_INIT
    jbsr    board_init
#endif

    jbsr    main
    .size   Reset_Handler, . - Reset_Handler

__exit:
    br      __exit

.section .bss

    .align  2
    .global g_idle_basestack
    .global g_idle_topstack
g_idle_basestack:
    .space  CONFIG_IDLETHREAD_STACKSIZE
g_idle_topstack:

    .align  2
    .globl  g_intstackalloc
    .global g_intstackbase
    .global g_top_irqstack
g_intstackalloc:
g_intstackbase:
    .space  CONFIG_ARCH_INTERRUPTSTACK
g_top_irqstack:
