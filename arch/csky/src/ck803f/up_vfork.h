/****************************************************************************
 * arch/csky/src/ck803f/up_vfork.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_CSKY_SRC_CK803F_VFORK_H
#define __ARCH_CSKY_SRC_CK803F_VFORK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VFORK_R4_OFFSET  (0*4)   /* Volatile register r4 */
#define VFORK_R5_OFFSET  (1*4)   /* Volatile register r5 */
#define VFORK_R6_OFFSET  (2*4)   /* Volatile register r6 */
#define VFORK_R7_OFFSET  (3*4)   /* Volatile register r7 */
#define VFORK_R8_OFFSET  (4*4)   /* Volatile register r8 */
#define VFORK_R9_OFFSET  (5*4)   /* Volatile register r9 */
#define VFORK_R10_OFFSET (6*4)   /* Volatile register r10 */
#define VFORK_R11_OFFSET (7*4)   /* Volatile register r11 */

#define VFORK_SP_OFFSET  (8*4)   /* Stack pointer*/
#define VFORK_LR_OFFSET  (9*4)   /* Return address*/

#define VFORK_VR0_OFFSET (10*4)
#define VFORK_VR1_OFFSET (11*4)
#define VFORK_VR2_OFFSET (12*4)
#define VFORK_VR3_OFFSET (13*4)
#define VFORK_VR4_OFFSET (14*4)
#define VFORK_VR5_OFFSET (15*4)
#define VFORK_VR6_OFFSET (16*4)
#define VFORK_VR7_OFFSET (17*4)
#define VFORK_VR8_OFFSET (18*4)
#define VFORK_VR9_OFFSET (19*4)
#define VFORK_VR10_OFFSET (20*4)
#define VFORK_VR11_OFFSET (21*4)
#define VFORK_VR12_OFFSET (22*4)
#define VFORK_VR13_OFFSET (23*4)
#define VFORK_VR14_OFFSET (24*4)
#define VFORK_VR15_OFFSET (25*4)

#define VFORK_SIZEOF     (26*4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct vfork_s {
    /* CPU registers */

    uint32_t r4;   /* Volatile register r4 */
    uint32_t r5;   /* Volatile register r5 */
    uint32_t r6;   /* Volatile register r6 */
    uint32_t r7;   /* Volatile register r7 */
    uint32_t r8;   /* Volatile register r8 */
    uint32_t r9;   /* Volatile register r9 */
    uint32_t r10;  /* Volatile register r10 */
    uint32_t r11;  /* Volatile register r11 */

    uint32_t sp;   /* Stack pointer*/
    uint32_t lr;   /* Return address*/

    uint32_t vr0;   /* floatpiont register */
    uint32_t vr1;   /* floatpiont register */
    uint32_t vr2;   /* floatpiont register */
    uint32_t vr3;   /* floatpiont register */
    uint32_t vr4;   /* floatpiont register */
    uint32_t vr5;   /* floatpiont register */
    uint32_t vr6;   /* floatpiont register */
    uint32_t vr7;   /* floatpiont register */
    uint32_t vr8;   /* floatpiont register */
    uint32_t vr9;   /* floatpiont register */
    uint32_t vr10;   /* floatpiont register */
    uint32_t vr11;   /* floatpiont register */
    uint32_t vr12;   /* floatpiont register */
    uint32_t vr13;   /* floatpiont register */
    uint32_t vr14;   /* floatpiont register */
    uint32_t vr15;   /* floatpiont register */
};
#endif

#endif /* __ARCH_CSKY_SRC_CK803F_VFORK_H */
