/****************************************************************************
 * arch/arm/src/song/chip.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __ARCH_ARM_SRC_SONG_CHIP_H
#define __ARCH_ARM_SRC_SONG_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/song/chip.h>

#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If the common ARMv7-M vector handling logic is used, then it expects the following
 * definition in this file that provides the number of supported external interrupts.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  (CONFIG_SONG_NR_IRQS - 16)

/* Vector Table Offset Register (VECTAB).  Redefine the mask defined in
 * arch/arm/src/armv7-m/nvic.h; The base address of the new vector table
 * must be aligned to the size of the vector table extended to the next
 * larger power of 2.
 */

#undef NVIC_VECTAB_TBLOFF_MASK
#if CONFIG_SONG_NR_IRQS > 256
#  error CONFIG_SONG_NR_IRQS can not be bigger than 256
#elif CONFIG_SONG_NR_IRQS > 128
#  define NVIC_VECTAB_TBLOFF_MASK     (0xffffc00)
#elif CONFIG_SONG_NR_IRQS > 64
#  define NVIC_VECTAB_TBLOFF_MASK     (0xffffe00)
#elif CONFIG_SONG_NR_IRQS > 32
#  define NVIC_VECTAB_TBLOFF_MASK     (0xfffff00)
#else
#  define NVIC_VECTAB_TBLOFF_MASK     (0xfffff80)
#endif

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Clock management *********************************************************/

void up_clk_initialize(void);
void up_clk_finalinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SONG_CHIP_H */
