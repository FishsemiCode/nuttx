/****************************************************************************
 * drivers/crypto/song_crypto_sha.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_CRYPTO_SHA_BLKLEN      64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_crypto_sha_s
{
  struct alg_head_s head;

  sem_t     sem;
  int       result;

  int       nhash;
  uint32_t  hash_mac[8];

  uint32_t  total;
  uint8_t   buffer[SONG_CRYPTO_SHA_BLKLEN * 2];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_hash_sha1[] =
{
  0x01234567, 0x89abcdef, 0xfedcba98, 0x76543210, 0xf0e1d2c3,
};

static const uint32_t g_hash_sha256[] =
{
  0x67e6096a, 0x85ae67bb, 0x72f36e3c, 0x3af54fa5,
  0x7f520e51, 0x8c68059b, 0xabd9831f, 0x19cde05b,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int song_crypto_sha_padding(struct song_crypto_sha_s *alg,
                               uint32_t inlen, uint8_t *indata)
{
  uint64_t tbits = alg->total * 8;
  int i, off, padlen;
  uint8_t *ptr;

  off = inlen & 0x3f;

  /* Padding one bit "1", followed by zero, last 8bytes is total bits */

  memset(alg->buffer, 0, sizeof(alg->buffer));
  memcpy(alg->buffer, indata + (inlen - off), off);

  ptr = alg->buffer + off;
  ptr[0] = 0x80;

  if (SONG_CRYPTO_SHA_BLKLEN - off > 9)
    {
      padlen = SONG_CRYPTO_SHA_BLKLEN;
    }
  else
    {
      padlen = SONG_CRYPTO_SHA_BLKLEN * 2;
    }

  ptr = alg->buffer + padlen - 1;

  for (i = 0; i < sizeof(tbits); i++)
    {
      *ptr-- = (uint8_t)(tbits >> (i*8));
    }

  return padlen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *song_crypto_sha_init(struct alg_head_s *head)
{
  struct song_crypto_sha_s *alg;
  uint32_t val;

  alg = kmm_zalloc(sizeof(struct song_crypto_sha_s));
  if (!alg)
    {
      return NULL;
    }

  memcpy(&alg->head, head, sizeof(struct alg_head_s));

  nxsem_init(&alg->sem, 0, 0);
  nxsem_setprotocol(&alg->sem, SEM_PRIO_NONE);
  val  = song_crypto_read(alg, CIPHER_INT_EN);
  val |= CIPHER_INT_EN_SHA_FINISH | CIPHER_INT_EN_FIFO_OVF;
  song_crypto_write(alg, CIPHER_INT_EN, val);

  if (alg->head.algid == ALG_SHA1)
    {
      alg->nhash = 5;
    }
  else
    {
      alg->nhash = 8;
    }

  return alg;
}

int song_crypto_sha_uninit(FAR void *alg_)
{
  struct song_crypto_sha_s *alg = alg_;

  nxsem_destroy(&alg->sem);
  free(alg);

  return 0;
}

int song_crypto_sha_ioctl(FAR void *alg_, uint32_t param,
                          uint32_t length, FAR uint8_t *value)
{
  return 0;
}

int song_crypto_sha_isr(FAR void *alg_)
{
  struct song_crypto_sha_s *alg = alg_;

  alg->result = song_crypto_interrupt(alg);
  nxsem_post(&alg->sem);

  return 0;
}

int song_crypto_sha_exe(FAR void *alg_, bool first, bool last,
                        uint32_t inlen, FAR uint8_t *indata,
                        uint32_t outlen, FAR uint8_t *outdata,
                        uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_sha_s *alg = alg_;
  uintptr_t dma_addr;
  uint32_t *hash_tb;
  int i;

  if (!indata || (!last && inlen & 0x3F))
    {
      return -EINVAL;
    }

  alg->total = first ? inlen : alg->total + inlen;

  song_crypto_write(alg, CIPHER_INT_STATUS, CIPHER_INT_STATUS_SHA_FINISH);

  if (alg->head.algid == ALG_SHA1)
    {
      hash_tb = (uint32_t *)g_hash_sha1;
      song_crypto_write(alg, CIPHER_SHA_INFO, CIPHER_SHA_INFO_SHA1);
    }
  else if (alg->head.algid == ALG_SHA256)
    {
      hash_tb = (uint32_t *)g_hash_sha256;
      song_crypto_write(alg, CIPHER_SHA_INFO, CIPHER_SHA_INFO_SHA256);
    }
  else
    {
      return -1;
    }

  if (!first)
    {
      hash_tb = alg->hash_mac;
    }

  for (i = 0; i < alg->nhash; i++)
    {
      song_crypto_write(alg, CIPHER_HASH_MAC0 + i*4, hash_tb[i]);
    }

  if (last)
    {
      int padlen;

      padlen = song_crypto_sha_padding(alg, inlen, indata);

      dma_addr = up_addrenv_va_to_pa(indata);
      song_crypto_write(alg, CIPHER_DMA_SH_RDAT_ADDR, dma_addr);
      song_crypto_write(alg, CIPHER_DMA_SH_RDAT_LEN, inlen & ~0x3f);

      dma_addr = up_addrenv_va_to_pa(alg->buffer);
      song_crypto_write(alg, CIPHER_DMA_SC_RDAT_ADDR, dma_addr);
      song_crypto_write(alg, CIPHER_DMA_SC_RDAT_LEN, padlen);
    }
  else
    {
      dma_addr = up_addrenv_va_to_pa(indata);
      song_crypto_write(alg, CIPHER_DMA_SH_RDAT_ADDR, dma_addr);
      song_crypto_write(alg, CIPHER_DMA_SH_RDAT_LEN, inlen);

      song_crypto_write(alg, CIPHER_DMA_SC_RDAT_ADDR, 0);
      song_crypto_write(alg, CIPHER_DMA_SC_RDAT_LEN, 0);
    }

  song_crypto_write(alg, CIPHER_EN_HC_EN, 0x01);

  nxsem_wait_uninterruptible(&alg->sem);
  if (alg->result)
    {
      return alg->result;
    }

  for (i = 0; i < alg->nhash; i++)
    {
      alg->hash_mac[i] = song_crypto_read(alg, CIPHER_HASH_MAC0 + i*4);
    }

  if (last)
    {
      uint32_t *out = (uint32_t *)outdata;

      if (!out || outlen < alg->nhash * 4)
        {
          return -ENOBUFS;
        }

      for (i = 0; i < alg->nhash; i++)
        {
          out[i] = alg->hash_mac[i];
        }

      return alg->nhash * 4;
    }

  return 0;
}

