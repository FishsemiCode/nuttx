/****************************************************************************
 * arch/ceva/include/tl4/math.h
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

#ifndef __ARCH_CEVA_INCLUDE_TL4_MATH_H
#define __ARCH_CEVA_INCLUDE_TL4_MATH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include_next <math.h>

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

float       ceilf (float x);
long double ceill (long double x);

float       floorf(float x);
long double floorl(long double x);

float       roundf(float x);
double      round (double x);
long double roundl(long double x);

float       fabsf (float x);
long double fabsl (long double x);

float       modff (float x, float *iptr);
long double modfl (long double x, long double *iptr);

float       fmodf (float x, float div);
long double fmodl (long double x, long double div);

float       powf  (float b, float e);
long double powl  (long double b, long double e);

float       expf  (float x);
long double expl  (long double x);

double      gamma(double x);
double      lgamma(double x);

float       logf  (float x);
long double logl  (long double x);

float       log10f(float x);
long double log10l(long double x);

float       log2f (float x);
double      log2  (double x);
long double log2l (long double x);

float       sqrtf (float x);
long double sqrtl (long double x);

float       ldexpf(float x, int n);
long double ldexpl(long double x, int n);

float       frexpf(float x, int *exp);
long double frexpl(long double x, int *exp);

float       sinf  (float x);
long double sinl  (long double x);

float       cosf  (float x);
long double cosl  (long double x);

float       tanf  (float x);
long double tanl  (long double x);

float       asinf (float x);
long double asinl (long double x);

float       acosf (float x);
long double acosl (long double x);

float       atanf (float x);
long double atanl (long double x);

float       atan2f(float y, float x);
long double atan2l(long double y, long double x);

float       sinhf (float x);
long double sinhl (long double x);

float       coshf (float x);
long double coshl (long double x);

float       tanhf (float x);
long double tanhl (long double x);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_CEVA_INCLUDE_TL4_MATH_H */
