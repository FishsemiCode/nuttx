/****************************************************************************
 * arch/ceva/include/xc5/reg.h
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
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

#ifndef __ARCH_CEVA_INCLUDE_XC5_REG_H
#define __ARCH_CEVA_INCLUDE_XC5_REG_H

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
#define REG_BKNEST40          1
#define REG_BKNEST41          2
#define REG_BKNEST30          3
#define REG_BKNEST31          4
#define REG_BKNEST20          5
#define REG_BKNEST21          6
#define REG_MODU2D            7
#define REG_MODU3D            8
#define REG_G4D               9
#define REG_G5D              10
#define REG_G6D              11
#define REG_G7D              12
#define REG_A20              13
#define REG_A21              14
#define REG_A22              15
#define REG_A23              16
#define REG_AKLMNE           17
#define REG_RETREGN          18
#define REG_MODP             19
#define REG_MODQ             20
#define REG_A16              21
#define REG_A17              22
#define REG_A18              23
#define REG_A19              24
#define REG_AGHIJE           25
#define REG_S0               26
#define REG_S1               27
#define REG_S2               28
#define REG_G0               29
#define REG_G1               30
#define REG_G2               31
#define REG_G3               32
#define REG_R0               33
#define REG_R1               34
#define REG_R2               35
#define REG_R3               36
#define REG_RETREG           37
#define REG_RETREGB          38
#define REG_RETREGI          39
#define REG_A4               40
#define REG_A5               41
#define REG_A6               42
#define REG_A7               43
#define REG_A4567E           44
#define BKNEST10             45
#define BKNEST11             46
#define REG_S3               47
#define REG_A12              48
#define REG_A13              49
#define REG_A14              50
#define REG_A15              51
#define REG_ACDEFE           52
#define REG_MOD0             53
#define REG_MOD1             54
#define REG_MOD2             55
#define REG_MODG             56
#define REG_MOD3             57
#define REG_R4               58
#define REG_R5               59
#define REG_R6               60
#define REG_A8               61
#define REG_A9               62
#define REG_A10              63
#define REG_A11              64
#define REG_A89ABE           65
#define REG_BKNEST00         66
#define REG_BKNEST01         67
#define REG_MODU2            68
#define REG_MODU3            69
#define REG_G4               70
#define REG_G5               71
#define REG_G6               72
#define REG_G7               73
#define REG_R7               74
#define REG_A0               75
#define REG_A1               76
#define REG_A2               77
#define REG_A3               78
#define REG_A0123E           79
#define REG_MODU0            80
#define REG_MODU1            81

/* The total number of registers is saved on the stack */

#define XCPTCONTEXT_REGS      82
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

#endif /* __ARCH_CEVA_INCLUDE_XC5_REG_H */
