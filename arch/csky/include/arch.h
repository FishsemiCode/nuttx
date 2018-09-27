/****************************************************************************
 * arch/csky/include/arch.h
 *
 *   Copyright (C) 2007-2009, 2014 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_CSKY_INCLUDE_ARCH_H
#define __ARCH_CSKY_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/pgalloc.h>
#  include <nuttx/addrenv.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPU Exceptions
 */

#define VEC_RESET       0
#define VEC_ALIGN       1
#define VEC_ACCESS      2
#define VEC_ZERODIV     3
#define VEC_ILLEGAL     4
#define VEC_PRIV        5
#define VEC_TRACE       6
#define VEC_BREAKPOINT  7
#define VEC_UNRECOVER   8
#define VEC_SOFTRESET   9
#define VEC_AUTOVEC     10
#define VEC_FAUTOVEC    11
#define VEC_HWACCEL     12

#define VEC_TLBFATAL    13
#define VEC_TLBMISS     14
#define VEC_TLBMODIFIED 15

#define VEC_SYS         16
#define VEC_TRAP0       16
#define VEC_TRAP1       17
#define VEC_TRAP2       18
#define VEC_TRAP3       19

#define VEC_TLBINVALIDL 20
#define VEC_TLBINVALIDS 21

#define VEC_PRFL        29
#define VEC_FPE         30


/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_CSKY_INCLUDE_ARCH_H */
