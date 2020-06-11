/****************************************************************************
 * crypto/secutils.h
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Author:  Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __CRYPTO_SECUTILS_H
#define __CRYPTO_SECUTILS_H

#include <stdint.h>

/* Generate some quality randomness for internal crypto library use.
 * The default is to use /dev/random.
 */

uint8_t secrandbyte(void);
void    secrand(FAR uint8_t *dest, uint32_t len);

/* Security failure - Call this when the current routine has to strictly fail
 * because of a  inconsistency that could have been caused by a
 * security attack on the CPU.
 */

void secfail(const char *msg);

/* Execution time of these routines do not depend on addresses nor data */

/* Copy of arbitrary non overlapping blocks. */

void secmemcpy(FAR uint8_t *dest, FAR uint8_t *src, uint32_t len);
void secmemset(FAR uint8_t *dest, uint8_t val, uint32_t len);
void secmemclr(FAR uint8_t *dest, uint32_t len);

/* Copy of blocks of fixed size. Avoid attacker controlled arbitrary buffer
 * copy.
 */

void seccpy8(FAR uint8_t *dest, FAR uint8_t *src);
void seccpy16(FAR uint8_t *dest, FAR uint8_t *src);

/* XOR of blocks of fixed size. Useful for operation modes. */

void secxor8(FAR uint8_t *dest, FAR uint8_t *srca, FAR uint8_t *srcb);
void secxor16(FAR uint8_t *dest, FAR uint8_t *srca, FAR uint8_t *srcb);

/* Buffer reductions */

/* OR of all bytes */


uint8_t secmemor(FAR uint8_t *buf, uint32_t len);

/* AND of all bytes */

uint8_t secmemand(FAR uint8_t *buf, uint32_t len);

/* OR of all XOR between both buffers */

uint8_t secorofxor(FAR uint8_t *bufa, FAR uint8_t *bufb, uint32_t len);

#endif /* __CRYPTO_SECUTILS_H */


