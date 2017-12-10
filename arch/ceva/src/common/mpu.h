/*****************************************************************************
 * arch/ceva/src/common/mpu.h
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

#ifndef __ARCH_CEVA_SRC_COMMON_MPU_H
#define __ARCH_CEVA_SRC_COMMON_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ARCH_MPU

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 ****************************************************************************/

void mpu_control(bool enable);

/****************************************************************************
 * Name: mpu_user_code
 *
 * Description:
 *   Configure a region for user code
 *
 ****************************************************************************/

void mpu_user_code(const void *base, size_t size);

/****************************************************************************
 * Name: mpu_priv_code
 *
 * Description:
 *   Configure a region for privileged code
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void mpu_priv_code(const void *base, size_t size);

/****************************************************************************
 * Name: mpu_user_data
 *
 * Description:
 *   Configure a region as user data
 *
 ****************************************************************************/

void mpu_user_data(void *base, size_t size);

/****************************************************************************
 * Name: mpu_priv_data
 *
 * Description:
 *   Configure a region as privileged data
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void mpu_priv_data(void *base, size_t size);

/****************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged peripheral address space
 *
 ****************************************************************************/

void mpu_peripheral(void *base, size_t size);

/****************************************************************************
 * Name: mpu_stronglyordered
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

void mpu_stronglyordered(void *base, size_t size);

#else

#define mpu_control(enable)
#define mpu_user_code(base, size)
#define mpu_priv_code(base, size)
#define mpu_user_data(base, size)
#define mpu_priv_data(base, size)
#define mpu_peripheral(base, size)
#define mpu_stronglyordered(base, size)

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __ASSEMBLY__ */
#endif  /* __ARCH_CEVA_SRC_COMMON_MPU_H */
