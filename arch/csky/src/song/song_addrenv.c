/****************************************************************************
 * arch/csky/src/song/song_addrenv.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "song_addrenv.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct song_addrenv_s g_addrenv_dummy;
static const struct song_addrenv_s *g_addrenv = &g_addrenv_dummy;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_addrenv_initialize(const struct song_addrenv_s *addrenv)
{
  g_addrenv = addrenv;
}

void *up_addrenv_pa_to_va(uintptr_t pa)
{
  uint32_t i;

  for (i = 0; g_addrenv[i].size; i++)
    {
      if (pa - g_addrenv[i].pa < g_addrenv[i].size)
        {
          pa = g_addrenv[i].va + (pa - g_addrenv[i].pa);
          break;
        }
    }

  return (void *)pa;
}

uintptr_t up_addrenv_va_to_pa(void *va)
{
  uint32_t i;

  for (i = 0; g_addrenv[i].size; i++)
    {
      if (va - (void *)g_addrenv[i].va < g_addrenv[i].size)
        {
          va = g_addrenv[i].pa + (va - g_addrenv[i].va);
          break;
        }
    }

  return (uintptr_t)va;
}
