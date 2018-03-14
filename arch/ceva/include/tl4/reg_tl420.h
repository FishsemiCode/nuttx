/****************************************************************************
 * arch/ceva/include/tl4/reg_tl420.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
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

#ifndef __ARCH_CEVA_INCLUDE_TL4_REG_TL420_H
#define __ARCH_CEVA_INCLUDE_TL4_REG_TL420_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format: */

/* The following additional registers are stored by the interrupt handling
 * logic.
 */

#define REG_TSTREAMBUFF     0
#define REG_STREAMBUFF1     1
#define REG_STREAMBUFF0     2
#define REG_X               3
#define REG_MOD2            4
#define REG_STS0            5
#define REG_OFF_LC3         6
#define REG_FIA3            7
#define REG_OFF_LC2         8
#define REG_FIA2            9
#define REG_OFF_LC1         10
#define REG_FIA1            11
#define REG_OFF_LC0         12
#define REG_FIA0            13
#define REG_MCFGJ           14
#define REG_MCFGI           15
#define REG_MIXP            16
#define REG_LC_PTR          17
#define REG_P1              18
#define REG_P1E             19
#define REG_P0              20
#define REG_P0E             21
#define REG_Y               22
#define REG_SV_SK           23
#define REG_B1              24
#define REG_B1E             25
#define REG_B0              26
#define REG_B0E             27
#define REG_A1              28
#define REG_A1E             29
#define REG_A0              30
#define REG_A0E             31
#define REG_R7              32
#define REG_R6              33
#define REG_R5              34
#define REG_R4              35
#define REG_R3              36
#define REG_R2              37
#define REG_R1              38
#define REG_R0              39
#define REG_RETREG          40

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define REG_PC              41
#define REG_REPC            42

/* The total number of registers is saved on the stack */
#define XCPTCONTEXT_REGS    43
#define XCPTCONTEXT_SIZE    (2 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_CEVA_INCLUDE_TL4_REG_TL420_H */
