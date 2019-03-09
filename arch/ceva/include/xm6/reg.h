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
#define REG_MODI              1
#define REG_MODG              2
#define REG_MODE              3
#define REG_MODD              4
#define REG_MODC              5
#define REG_R56               6
#define REG_R57               7
#define REG_R58               8
#define REG_R59               9
#define REG_R60               10
#define REG_R61               11
#define REG_R62               12
#define REG_R63               13
#define REG_R48               14
#define REG_R49               15
#define REG_R50               16
#define REG_R51               17
#define REG_R52               18
#define REG_R53               19
#define REG_R54               20
#define REG_R55               21
#define REG_R40               22
#define REG_R41               23
#define REG_R42               24
#define REG_R43               25
#define REG_R44               26
#define REG_R45               27
#define REG_R46               28
#define REG_R47               29
#define REG_R32               30
#define REG_R33               31
#define REG_R34               32
#define REG_R35               33
#define REG_R36               34
#define REG_R37               35
#define REG_R38               36
#define REG_R39               37
#define REG_R24               38
#define REG_R25               39
#define REG_R26               40
#define REG_R27               41
#define REG_R28               42
#define REG_R29               43
#define REG_R30               44
#define REG_R31               45
#define REG_R16               46
#define REG_R17               47
#define REG_R18               48
#define REG_R19               49
#define REG_R20               50
#define REG_R21               51
#define REG_R22               52
#define REG_R23               53
#define REG_R8                54
#define REG_R9                55
#define REG_R10               56
#define REG_R11               57
#define REG_R12               58
#define REG_R13               59
#define REG_R14               60
#define REG_R15               61
#define REG_R0                62
#define REG_R1                63
#define REG_R2                64
#define REG_R3                65
#define REG_R4                66
#define REG_R5                67
#define REG_R6                68
#define REG_R7                69
#define REG_RETREGN           70
#define REG_RETREG            71
#define REG_RETREG_TEMP       72
#define REG_RETREGI           73
#define REG_MODVL0            74
#define REG_MODVL1            75
#define REG_MODVLL            76
#define REG_MODVFP            77

/* The total number of registers is saved on the stack */

#define XCPTCONTEXT_REGS      78
#define XCPTCONTEXT_SIZE      (4 * XCPTCONTEXT_REGS)

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
