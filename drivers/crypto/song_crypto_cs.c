/****************************************************************************
 * drivers/crypto/song_crypto_cs.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li <liguiding@pinecone.net>
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

#include <nuttx/arch.h>
#include <nuttx/crypto/manager.h>
#include <nuttx/kmalloc.h>

#include <semaphore.h>
#include <string.h>

#include "song_crypto.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_crypto_cs_s
{
  struct alg_head_s head;

  sem_t     sem;
  int       result;

  uint32_t  checksum;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *song_crypto_cs_init(struct alg_head_s *head)
{
  struct song_crypto_cs_s *alg;
  uint32_t val;

  alg = kmm_zalloc(sizeof(struct song_crypto_cs_s));
  if (!alg)
    {
      return NULL;
    }

  memcpy(&alg->head, head, sizeof(struct alg_head_s));

  nxsem_init(&alg->sem, 0, 0);
  nxsem_setprotocol(&alg->sem, SEM_PRIO_NONE);

  val  = song_crypto_read(alg, CIPHER_INT_EN);
  val |= CIPHER_INT_EN_CS_FINISH | CIPHER_INT_EN_FIFO_OVF;
  song_crypto_write(alg, CIPHER_INT_EN, val);

  return alg;
}

int song_crypto_cs_uninit(FAR void *alg_)
{
  struct song_crypto_cs_s *alg = alg_;

  nxsem_destroy(&alg->sem);
  free(alg);

  return 0;
}

int song_crypto_cs_ioctl(FAR void *alg_, uint32_t param,
                         uint32_t length, FAR uint8_t *value)
{
  return 0;
}

int song_crypto_cs_isr(FAR void *alg_)
{
  struct song_crypto_cs_s *alg = alg_;

  alg->result = song_crypto_interrupt(alg);
  nxsem_post(&alg->sem);

  return 0;
}

int song_crypto_cs_exe(FAR void *alg_, bool first, bool last,
                       uint32_t inlen, FAR uint8_t *indata,
                       uint32_t outlen, FAR uint8_t *outdata,
                       uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_cs_s *alg = alg_;
  uintptr_t dma_addr;

  if (!indata)
    {
      return -EINVAL;
    }

  song_crypto_write(alg, CIPHER_INT_STATUS, CIPHER_INT_STATUS_CS_FINISH);

  song_crypto_write(alg, CIPHER_DMA_SH_RDAT_ADDR, 0);
  song_crypto_write(alg, CIPHER_DMA_SH_RDAT_LEN, 0);

  dma_addr = up_addrenv_va_to_pa(indata);
  song_crypto_write(alg, CIPHER_DMA_SC_RDAT_ADDR, dma_addr);
  song_crypto_write(alg, CIPHER_DMA_SC_RDAT_LEN, inlen);

  if (first)
    {
      song_crypto_write(alg, CIPHER_CHECKSUM_DATA, 0);
    }
  else
    {
      song_crypto_write(alg, CIPHER_CHECKSUM_DATA, alg->checksum);
    }

  song_crypto_write(alg, CIPHER_EN_HC_EN, 0x05);

  nxsem_wait_uninterruptible(&alg->sem);
  if (alg->result)
    {
      return alg->result;
    }

  alg->checksum = song_crypto_read(alg, CIPHER_CHECKSUM_DATA);

  if (last)
    {
      uint16_t *out = (uint16_t *)outdata;

      if (!out || outlen < 2)
        {
          return -ENOBUFS;
        }

      *out = (uint16_t)alg->checksum;

      return 2;
    }

  return 0;
}

