/****************************************************************************
 * arch/ceva/include/syscall.h
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

/* This file should never be included directed but, rather, only indirectly
 * through include/syscall.h or include/sys/sycall.h
 */

#ifndef __ARCH_CEVA_INCLUDE_SYSCALL_H
#define __ARCH_CEVA_INCLUDE_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* TRAP call with SYS_ call number and no parameters */

uintptr_t sys_call0(unsigned int nbr);

/* TRAP call with SYS_ call number and one parameter */

uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1);

/* TRAP call with SYS_ call number and two parameters */

uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2);

/* TRAP call with SYS_ call number and three parameters */

uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3);

/* TRAP call with SYS_ call number and four parameters */

uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3,
                    uintptr_t parm4);

/* TRAP call with SYS_ call number and five parameters */

uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3,
                    uintptr_t parm4, uintptr_t parm5);

/* TRAP call with SYS_ call number and six parameters */

uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                    uintptr_t parm2, uintptr_t parm3,
                    uintptr_t parm4, uintptr_t parm5,
                    uintptr_t parm6);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_CEVA_INCLUDE_SYSCALL_H */
