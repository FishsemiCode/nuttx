/****************************************************************************
 * arch/arm/src/song/banks_cp_bootram.S
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Zhuang Liu <liuzhuang@pinecone.net>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_CHIP_BANKS_CP
#include <nuttx/irq.h>
#include "cp15.h"
#include "gic.h"
#include "sctlr.h"

	.file		"bootram.S"

/***************************************************************************
 * .bootram section data
 ***************************************************************************/
	.section	.bootram, "awx"

	b		__boot_start
	nop
	nop
__boot_start:
	ldr		r0, =0xFFFFFFFF
	ldr		r1, =GIC_IRQ_SPI
	ldr		r2, =GIC_ICDISR(GIC_IRQ_SPI)
1:
	str		r0, [r2]

	add		r1, r1, #32
	add		r2, r2, #4
	cmp		r1, #NR_IRQS
	blt		1b

	mov		r0, #0x80
	ldr		r1, =GIC_ICCPMR
	str		r0, [r1]

	mrc		CP15_NSACR(r0)
	orr		r0, r0, #(NSACR_NSSMP)
	mcr		CP15_NSACR(r0)
	isb

	mrc		CP15_SCR(r0)
	orr		r0, r0, #(SCR_AW | SCR_FW | SCR_NS)
	mcr		CP15_SCR(r0)
	isb

	mrc		CP15_ACTLR(r0)
	orr		r0, r0, #(ACTLR_SMP)
	mcr		CP15_ACTLR(r0)
	isb

	ldr		r0, =__start
	cpy		pc, r0
#endif
