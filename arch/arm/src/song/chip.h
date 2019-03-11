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

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <sys/types.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))
#endif

#ifdef CONFIG_SONG_GIC_VBASE
#  define CHIP_MPCORE_VBASE           CONFIG_SONG_GIC_VBASE
#endif

#ifdef CONFIG_ARCH_USE_MMU
#  ifdef __ASSEMBLY__
#    define PGTABLE_BASE_PADDR        _spgtable
#    define PGTABLE_BASE_VADDR        _spgtable
#  else
#    define PGTABLE_BASE_PADDR        ((uintptr_t)&_spgtable)
#    define PGTABLE_BASE_VADDR        ((uintptr_t)&_spgtable)
#  endif

#  ifdef CONFIG_BOOT_RUNFROMFLASH
#    define NUTTX_TEXT_PADDR          (CONFIG_FLASH_START & 0xfff00000)
#    define NUTTX_TEXT_VADDR          (CONFIG_FLASH_VSTART & 0xfff00000)
#    define NUTTX_TEXT_SIZE           CONFIG_FLASH_SIZE
#    define NUTTX_RAM_PADDR           (CONFIG_RAM_START & 0xfff00000)
#    define NUTTX_RAM_VADDR           (CONFIG_RAM_VSTART & 0xfff00000)
#    define NUTTX_RAM_SIZE            CONFIG_RAM_SIZE
#  else
#    define NUTTX_TEXT_PADDR          (CONFIG_RAM_START & 0xfff00000)
#    define NUTTX_TEXT_VADDR          (CONFIG_RAM_VSTART & 0xfff00000)
#    define NUTTX_TEXT_SIZE           CONFIG_RAM_SIZE
#  endif

#  define NUTTX_DEV_PADDR             ((CONFIG_RAM_START + CONFIG_RAM_SIZE) & 0xfff00000)
#  define NUTTX_DEV_VADDR             ((CONFIG_RAM_VSTART + CONFIG_RAM_SIZE) & 0xfff00000)
#  define NUTTX_DEV_NSECT             ((0xffffffff - NUTTX_DEV_PADDR + 1) >> 20)
#endif

/* If the common ARMv7-M vector handling logic is used, then it expects the following
 * definition in this file that provides the number of supported external interrupts.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  (CONFIG_SONG_NR_IRQS - 16)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  ifdef CONFIG_ARCH_USE_MMU
extern uint32_t _spgtable;
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* MPU **********************************************************************/

#ifdef CONFIG_ARM_MPU
void up_mpuinitialize(void);
void up_mpu_user_heap(uintptr_t start, size_t size);
void up_mpu_priv_heap(uintptr_t start, size_t size);
#else
#  define up_mpuinitialize()
#  define up_mpu_user_heap(start, size)
#  define up_mpu_priv_heap(start, size)
#endif

/* FPU **********************************************************************/

#ifdef CONFIG_ARCH_FPU
void arm_fpuconfig(void);
#else
#  define arm_fpuconfig()
#endif

/* Power ********************************************************************/

void up_cpu_doze(void);
void up_cpu_idle(void);
void up_cpu_standby(void);
void up_cpu_sleep(void);
void up_cpu_normal(void);

void up_cpu_wfi(void);
void up_cpu_save(void);
void up_cpu_restore(void);

/* Wakeup *******************************************************************/

void up_wic_disable_irq(int irq);
void up_wic_enable_irq(int irq);
void up_wic_initialize(void);

/* Clock *******************************************************************/

void up_clk_initialize(void);
void up_clk_finalinitialize(void);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_SONG_CHIP_H */
