/****************************************************************************
 * arch/ceva/include/xm6/reg.h
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

#ifndef __ARCH_CEVA_INCLUDE_XM6_REG_H
#define __ARCH_CEVA_INCLUDE_XM6_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format: */

/* The following registers are stored by the interrupt handling
 * logic.
 */

#define REG_SP                0
#ifdef CONFIG_ARCH_XM6_STACKCHECK
#  define REG_SB              1
#  define REG_SL              2
#  define XCPT_SP_REGS        3
#else
#  define XCPT_SP_REGS        1
#endif

#define REG_MODI              (XCPT_SP_REGS + 0)
#define REG_MODG              (XCPT_SP_REGS + 1)
#define REG_MODE              (XCPT_SP_REGS + 2)
#define REG_MODD              (XCPT_SP_REGS + 3)
#define REG_MODC              (XCPT_SP_REGS + 4)
#define REG_R56               (XCPT_SP_REGS + 5)
#define REG_R57               (XCPT_SP_REGS + 6)
#define REG_R58               (XCPT_SP_REGS + 7)
#define REG_R59               (XCPT_SP_REGS + 8)
#define REG_R60               (XCPT_SP_REGS + 9)
#define REG_R61               (XCPT_SP_REGS + 10)
#define REG_R62               (XCPT_SP_REGS + 11)
#define REG_R63               (XCPT_SP_REGS + 12)
#define REG_R48               (XCPT_SP_REGS + 13)
#define REG_R49               (XCPT_SP_REGS + 14)
#define REG_R50               (XCPT_SP_REGS + 15)
#define REG_R51               (XCPT_SP_REGS + 16)
#define REG_R52               (XCPT_SP_REGS + 17)
#define REG_R53               (XCPT_SP_REGS + 18)
#define REG_R54               (XCPT_SP_REGS + 19)
#define REG_R55               (XCPT_SP_REGS + 20)
#define REG_R40               (XCPT_SP_REGS + 21)
#define REG_R41               (XCPT_SP_REGS + 22)
#define REG_R42               (XCPT_SP_REGS + 23)
#define REG_R43               (XCPT_SP_REGS + 24)
#define REG_R44               (XCPT_SP_REGS + 25)
#define REG_R45               (XCPT_SP_REGS + 26)
#define REG_R46               (XCPT_SP_REGS + 27)
#define REG_R47               (XCPT_SP_REGS + 28)
#define REG_R32               (XCPT_SP_REGS + 29)
#define REG_R33               (XCPT_SP_REGS + 30)
#define REG_R34               (XCPT_SP_REGS + 31)
#define REG_R35               (XCPT_SP_REGS + 32)
#define REG_R36               (XCPT_SP_REGS + 33)
#define REG_R37               (XCPT_SP_REGS + 34)
#define REG_R38               (XCPT_SP_REGS + 35)
#define REG_R39               (XCPT_SP_REGS + 36)
#define REG_R24               (XCPT_SP_REGS + 37)
#define REG_R25               (XCPT_SP_REGS + 38)
#define REG_R26               (XCPT_SP_REGS + 39)
#define REG_R27               (XCPT_SP_REGS + 40)
#define REG_R28               (XCPT_SP_REGS + 41)
#define REG_R29               (XCPT_SP_REGS + 42)
#define REG_R30               (XCPT_SP_REGS + 43)
#define REG_R31               (XCPT_SP_REGS + 44)
#define REG_R16               (XCPT_SP_REGS + 45)
#define REG_R17               (XCPT_SP_REGS + 46)
#define REG_R18               (XCPT_SP_REGS + 47)
#define REG_R19               (XCPT_SP_REGS + 48)
#define REG_R20               (XCPT_SP_REGS + 49)
#define REG_R21               (XCPT_SP_REGS + 50)
#define REG_R22               (XCPT_SP_REGS + 51)
#define REG_R23               (XCPT_SP_REGS + 52)
#define REG_R8                (XCPT_SP_REGS + 53)
#define REG_R9                (XCPT_SP_REGS + 54)
#define REG_R10               (XCPT_SP_REGS + 55)
#define REG_R11               (XCPT_SP_REGS + 56)
#define REG_R12               (XCPT_SP_REGS + 57)
#define REG_R13               (XCPT_SP_REGS + 58)
#define REG_R14               (XCPT_SP_REGS + 59)
#define REG_R15               (XCPT_SP_REGS + 60)
#define REG_R0                (XCPT_SP_REGS + 61)
#define REG_R1                (XCPT_SP_REGS + 62)
#define REG_R2                (XCPT_SP_REGS + 63)
#define REG_R3                (XCPT_SP_REGS + 64)
#define REG_R4                (XCPT_SP_REGS + 65)
#define REG_R5                (XCPT_SP_REGS + 66)
#define REG_R6                (XCPT_SP_REGS + 67)
#define REG_R7                (XCPT_SP_REGS + 68)
#define REG_RETREGN           (XCPT_SP_REGS + 69)
#define REG_RETREG            (XCPT_SP_REGS + 70)
#define REG_RETREG_TEMP       (XCPT_SP_REGS + 71)
#define REG_RETREGI           (XCPT_SP_REGS + 72)
#define REG_MODVL0            (XCPT_SP_REGS + 73)
#define REG_MODVL1            (XCPT_SP_REGS + 74)
#define REG_MODVLL            (XCPT_SP_REGS + 75)
#define REG_MODVFP            (XCPT_SP_REGS + 76)
#define XCPT_GEN_REGS         77

/* The total number of registers is saved on the stack */

#define XCPTCONTEXT_REGS    (XCPT_SP_REGS + XCPT_GEN_REGS)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

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

#endif /* __ARCH_CEVA_INCLUDE_XM6_REG_H */
