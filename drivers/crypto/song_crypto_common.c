/****************************************************************************
 * drivers/crypto/song_crypto_common.c
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

#define SONG_CRYPTO_COMMON_IV       16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_crypto_common_s
{
  struct alg_head_s head;

  sem_t     sem;
  int       result;
  bool      keyupdate;

  uint8_t   iv[SONG_CRYPTO_COMMON_IV];
  uint32_t  ivlen;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *song_crypto_common_init(struct alg_head_s *head)
{
  struct song_crypto_common_s *alg;
  uint32_t val;

  alg = kmm_zalloc(sizeof(struct song_crypto_common_s));
  if (!alg)
    {
      return NULL;
    }

  memcpy(&alg->head, head, sizeof(struct alg_head_s));

  nxsem_init(&alg->sem, 0, 0);
  nxsem_setprotocol(&alg->sem, SEM_PRIO_NONE);

  val = song_crypto_read(alg, CIPHER_INT_EN);
  if (alg->head.algclass == ALG_CLASS_AES)
    {
      val |= CIPHER_INT_EN_AES_FINISH | CIPHER_INT_EN_FIFO_OVF;
    }
  else
    {
      val |= CIPHER_INT_EN_SM4_FINISH | CIPHER_INT_EN_FIFO_OVF;
    }
  song_crypto_write(alg, CIPHER_INT_EN, val);

  return alg;
}

int song_crypto_common_uninit(FAR void *alg_)
{
  struct song_crypto_common_s *alg = alg_;

  nxsem_destroy(&alg->sem);
  free(alg);

  return 0;
}

int song_crypto_common_ioctl(FAR void *alg_, uint32_t param,
                             uint32_t length, FAR uint8_t *value)
{
  struct song_crypto_common_s *alg = alg_;

  switch (param)
    {
      case ALGPARAM_IV:
        if (alg->head.algid == ALG_AES_ECB || alg->head.algid == ALG_SM4_ECB)
          {
            return -ENOTSUP;
          }

        if (length > SONG_CRYPTO_COMMON_IV)
          {
            return -EINVAL;
          }

        memcpy(alg->iv, value, length);
        alg->ivlen = length;
        break;
      case ALGPARAM_KEYUPDATE:
        alg->keyupdate = true;
        break;
      default:
        break;
    }

  return 0;
}

int song_crypto_common_isr(FAR void *alg_)
{
  struct song_crypto_common_s *alg = alg_;

  alg->result = song_crypto_interrupt(alg);
  nxsem_post(&alg->sem);

  return 0;
}

int song_crypto_common_exe(FAR void *alg_, bool first, bool last,
                           uint32_t inlen, FAR uint8_t *indata,
                           uint32_t outlen, FAR uint8_t *outdata,
                           uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_common_s *alg = alg_;
  uint32_t *pkey = (uint32_t *)keydata;
  uint32_t *piv = (uint32_t *)alg->iv;
  uintptr_t dma_addr;
  int i, ret = 0;

  if (!indata || inlen == 0 || !outdata || outlen < inlen)
    {
      return -EINVAL;
    }

  if ((alg->head.algclass == ALG_CLASS_AES && keylen > 32) ||
          (alg->head.algclass == ALG_CLASS_SM4 && keylen != 16))
    {
      return -EINVAL;
    }

  if (alg->head.algclass == ALG_CLASS_AES)
    {
      song_crypto_write(alg, CIPHER_INT_STATUS, CIPHER_INT_STATUS_AES_FINISH);
    }
  else if (alg->head.algclass == ALG_CLASS_SM4)
    {
      song_crypto_write(alg, CIPHER_INT_STATUS, CIPHER_INT_STATUS_SM4_FINISH);
    }
  else
    {
      return -EINVAL;
    }

  for (i = 0; i < keylen/4; i++)
    {
      song_crypto_write(alg, CIPHER_BC_KEY_DAT0 + i*4, *pkey++);
    }

  if (first || alg->keyupdate)
    {
      ret = song_crypto_bcinfo(alg, ALG_MODE_CAL, ALGOP_CIPHER, keylen);
      if (ret)
        {
          return ret;
        }

      song_crypto_write(alg, CIPHER_BC_REKEY_CTRL, 0x01);
      while(song_crypto_read(alg, CIPHER_BC_REKEY_CTRL));

      alg->keyupdate = false;
    }

  ret = song_crypto_bcinfo(alg, alg->head.algid, alg->head.algmode, keylen);
  if (ret)
    {
      return ret;
    }

  if (alg->ivlen)
    {
      for (i = 0; i < alg->ivlen/4; i++)
        {
          song_crypto_write(alg, CIPHER_BC_IV_DAT0 + i*4, *piv++);
        }
    }

  dma_addr = up_addrenv_va_to_pa(indata);
  song_crypto_write(alg, CIPHER_DMA_BC_RDAT_ADDR, dma_addr);
  song_crypto_write(alg, CIPHER_DMA_BC_RDAT_LEN, inlen);

  dma_addr = up_addrenv_va_to_pa(outdata);
  song_crypto_write(alg, CIPHER_DMA_BC_WDAT_ADDR, dma_addr);
  song_crypto_write(alg, CIPHER_DMA_BC_WDAT_LEN, inlen);

  song_crypto_write(alg, CIPHER_EN_BC_EN, 0x01);

  nxsem_wait_uninterruptible(&alg->sem);
  if (alg->result)
    {
      return alg->result;
    }

  if (!last && alg->ivlen)
    {
      piv = (uint32_t *)alg->iv;
      for (i = 0; i < alg->ivlen/4; i++)
        {
          *piv++ = song_crypto_read(alg, CIPHER_BC_IV_DAT0 + i*4);
        }
    }
  else
    {
      alg->ivlen = 0;
    }

  return inlen;
}

