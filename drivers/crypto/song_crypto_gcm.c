/****************************************************************************
 * drivers/crypto/song_cipher_gcm.c
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

#define SONG_CRYPTO_GCM_IV          256
#define SONG_CRYPTO_GCM_AAD         256
#define SONG_CRYPTO_GCM_BLK         128

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_crypto_gcm_s
{
  struct alg_head_s head;

  sem_t     sem;
  int       result;

  uint8_t   iv[SONG_CRYPTO_GCM_IV];
  uint32_t  ivlen;

  uint8_t   aad[SONG_CRYPTO_GCM_AAD];
  uint32_t  aadlen;

  uint32_t  intotal;
  uint32_t  gcm_mac[4];
  uint32_t  ghash_ey[4];
  uint32_t  ghash_prod[4];

  bool      keyupdate;
};

static int song_crypto_gcm_setup(void *alg_, bool first,
                                 uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_gcm_s *alg = alg_;
  uint32_t *pkey = (uint32_t *)keydata;
  int i, ret = 0;

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

      song_crypto_write(alg, CIPHER_BC_TEXT_IN0, 0);
      song_crypto_write(alg, CIPHER_BC_TEXT_IN1, 0);
      song_crypto_write(alg, CIPHER_BC_TEXT_IN2, 0);
      song_crypto_write(alg, CIPHER_BC_TEXT_IN3, 0);

      song_crypto_write(alg, CIPHER_BC_HKEY_CTRL, 0x01);
      while(song_crypto_read(alg, CIPHER_BC_HKEY_CTRL));

      song_crypto_write(alg, CIPHER_GHASH_MTBL_CTRL, 0x01);
      while(song_crypto_read(alg, CIPHER_GHASH_MTBL_CTRL));

      alg->keyupdate = false;
    }

  ret = song_crypto_bcinfo(alg, alg->head.algid, alg->head.algmode, keylen);
  if (ret)
    {
      return ret;
    }

  return 0;
}

static int song_crypto_gcm_iv_blk(void *alg_, bool first, bool last,
                                  uint32_t ivlen, FAR uint8_t *ivdata,
                                  uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_gcm_s *alg = alg_;
  uintptr_t dma_addr;
  int ret;

  ret = song_crypto_gcm_setup(alg_, first, keylen, keydata);
  if (ret)
    {
      return ret;
    }

  if (first)
    {
      song_crypto_write(alg, CIPHER_GHASH_EY0_VLD, 0x0);

      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT0, 0);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT1, 0);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT2, 0);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT3, 0);
    }
  else
    {
      song_crypto_write(alg, CIPHER_GHASH_EY0_VLD, 0x1);

      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT0, alg->ghash_ey[0]);
      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT1, alg->ghash_ey[1]);
      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT2, alg->ghash_ey[2]);
      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT3, alg->ghash_ey[3]);

      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT0, alg->ghash_prod[0]);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT1, alg->ghash_prod[1]);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT2, alg->ghash_prod[2]);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT3, alg->ghash_prod[3]);
    }

  if (!last)
    {
      song_crypto_write(alg, CIPHER_GHASH_LEN_BLK_VLD, 0x0);
      song_crypto_write(alg, CIPHER_GHASH_TEXT_LEN, ivlen);
    }
  else
    {
      song_crypto_write(alg, CIPHER_GHASH_LEN_BLK_VLD, 0x1);
      song_crypto_write(alg, CIPHER_GHASH_TEXT_LEN, alg->ivlen);
    }

  song_crypto_write(alg, CIPHER_GHASH_AAD_LEN, 0);

  dma_addr = up_addrenv_va_to_pa(ivdata);
  song_crypto_write(alg, CIPHER_DMA_AAD_RDAT_ADDR, dma_addr);
  song_crypto_write(alg, CIPHER_DMA_AAD_RDAT_LEN, ivlen);

  song_crypto_write(alg, CIPHER_DMA_BC_RDAT_LEN, 0);
  song_crypto_write(alg, CIPHER_DMA_BC_WDAT_LEN, 0);

  song_crypto_write(alg, CIPHER_EN_BC_EN, 0x01);

  nxsem_wait_uninterruptible(&alg->sem);
  if (alg->result)
    {
      return alg->result;
    }

  if (!last)
    {
      alg->ghash_ey[0] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT0);
      alg->ghash_ey[1] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT1);
      alg->ghash_ey[2] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT2);
      alg->ghash_ey[3] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT3);

      alg->ghash_prod[0] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT0);
      alg->ghash_prod[1] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT1);
      alg->ghash_prod[2] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT2);
      alg->ghash_prod[3] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT3);

      alg->ivlen   = 0;
      alg->aadlen  = 0;
      alg->intotal = 0;
    }

  return 0;
}

static int song_crypto_gcm_blk(void *alg_, bool first, bool last,
                               uint32_t inlen, FAR uint8_t *indata,
                               uint32_t outlen, FAR uint8_t *outdata,
                               uint32_t keylen, FAR uint8_t *keydata,
                               uint32_t aadlen, FAR uint8_t *aaddata)
{
  struct song_crypto_gcm_s *alg = alg_;
  uintptr_t dma_addr;
  int ret;

  ret = song_crypto_gcm_setup(alg_, first, keylen, keydata);
  if (ret)
    {
      return ret;
    }

  if (indata)
    {
      song_crypto_write(alg, CIPHER_GHASH_TEXT_LEN, inlen);

      dma_addr = up_addrenv_va_to_pa(indata);
      song_crypto_write(alg, CIPHER_DMA_BC_RDAT_ADDR, dma_addr);
      song_crypto_write(alg, CIPHER_DMA_BC_RDAT_LEN, inlen);

      dma_addr = up_addrenv_va_to_pa(outdata);
      song_crypto_write(alg, CIPHER_DMA_BC_WDAT_ADDR, dma_addr);
      song_crypto_write(alg, CIPHER_DMA_BC_WDAT_LEN, inlen);
    }
  else
    {
      song_crypto_write(alg, CIPHER_GHASH_TEXT_LEN, 0);
      song_crypto_write(alg, CIPHER_DMA_BC_RDAT_LEN, 0);
      song_crypto_write(alg, CIPHER_DMA_BC_WDAT_LEN, 0);
    }

  if (aaddata)
    {
      song_crypto_write(alg, CIPHER_GHASH_AAD_LEN, aadlen);

      dma_addr = up_addrenv_va_to_pa(aaddata);
      song_crypto_write(alg, CIPHER_DMA_AAD_RDAT_ADDR, dma_addr);
      song_crypto_write(alg, CIPHER_DMA_AAD_RDAT_LEN, aadlen);
    }
  else
    {
      song_crypto_write(alg, CIPHER_GHASH_AAD_LEN, 0);
      song_crypto_write(alg, CIPHER_DMA_AAD_RDAT_LEN, 0);
    }

  if (first)
    {
      song_crypto_write(alg, CIPHER_GHASH_EY0_VLD, 0x0);

      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT0, 0);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT1, 0);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT2, 0);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT3, 0);

      alg->intotal = inlen;
    }
  else
    {
      song_crypto_write(alg, CIPHER_GHASH_EY0_VLD, 0x1);

      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT0, alg->ghash_ey[0]);
      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT1, alg->ghash_ey[1]);
      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT2, alg->ghash_ey[2]);
      song_crypto_write(alg, CIPHER_GHASH_EY0_DAT3, alg->ghash_ey[3]);

      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT0, alg->ghash_prod[0]);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT1, alg->ghash_prod[1]);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT2, alg->ghash_prod[2]);
      song_crypto_write(alg, CIPHER_GHASH_PROD_DAT3, alg->ghash_prod[3]);

      alg->intotal += inlen;
    }

  if (!last)
    {
      song_crypto_write(alg, CIPHER_GHASH_LEN_BLK_VLD, 0x0);
    }
  else
    {
      song_crypto_write(alg, CIPHER_GHASH_LEN_BLK_VLD, 0x1);
      song_crypto_write(alg, CIPHER_GHASH_AAD_LEN, alg->aadlen);
      song_crypto_write(alg, CIPHER_GHASH_TEXT_LEN, alg->intotal);
    }

  song_crypto_write(alg, CIPHER_EN_BC_EN, 0x01);

  nxsem_wait_uninterruptible(&alg->sem);
  if (alg->result)
    {
      return alg->result;
    }

  if (!last)
    {
      alg->ghash_ey[0] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT0);
      alg->ghash_ey[1] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT1);
      alg->ghash_ey[2] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT2);
      alg->ghash_ey[3] = song_crypto_read(alg, CIPHER_GHASH_EY0_DAT3);

      alg->ghash_prod[0] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT0);
      alg->ghash_prod[1] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT1);
      alg->ghash_prod[2] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT2);
      alg->ghash_prod[3] = song_crypto_read(alg, CIPHER_GHASH_PROD_DAT3);
    }
  else
    {
      alg->gcm_mac[0] = song_crypto_read(alg, CIPHER_BC_GCM_MAC0);
      alg->gcm_mac[1] = song_crypto_read(alg, CIPHER_BC_GCM_MAC1);
      alg->gcm_mac[2] = song_crypto_read(alg, CIPHER_BC_GCM_MAC2);
      alg->gcm_mac[3] = song_crypto_read(alg, CIPHER_BC_GCM_MAC3);
    }

  return inlen;
}

static int song_crypto_gcm_iv(void *alg_, uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_gcm_s *alg = alg_;
  uint32_t *piv32 = (uint32_t *)alg->iv;
  uint8_t *piv = alg->iv;
  int ret, once, blk, len;
  bool first = true, last = false;

  if (alg->ivlen == 0)
    {
      return -EINVAL;
    }

  if (alg->ivlen == 12)
    {
      song_crypto_write(alg, CIPHER_BC_IV_DAT0, *piv32++);
      song_crypto_write(alg, CIPHER_BC_IV_DAT1, *piv32++);
      song_crypto_write(alg, CIPHER_BC_IV_DAT2, *piv32++);
      song_crypto_write(alg, CIPHER_BC_IV_DAT3, 0x1000000);
      return 0;
    }

  len = alg->ivlen;
  blk = SONG_CRYPTO_GCM_BLK;
  while (len)
    {
      once = len > blk ? blk : len;
      last = len > blk ? false : true;

      ret = song_crypto_gcm_iv_blk(alg_, first, last, once, piv, keylen, keydata);
      if (ret)
        {
          return ret;
        }

      len  -= once;
      piv  += once;
      first = false;
    }

  song_crypto_write(alg, CIPHER_BC_IV_DAT0, song_crypto_read(alg, CIPHER_GHASH_PROD_DAT0));
  song_crypto_write(alg, CIPHER_BC_IV_DAT1, song_crypto_read(alg, CIPHER_GHASH_PROD_DAT1));
  song_crypto_write(alg, CIPHER_BC_IV_DAT2, song_crypto_read(alg, CIPHER_GHASH_PROD_DAT2));
  song_crypto_write(alg, CIPHER_BC_IV_DAT3, song_crypto_read(alg, CIPHER_GHASH_PROD_DAT3));

  return 0;
}

static int song_crypto_gcm_aad(void *alg_, uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_gcm_s *alg = alg_;
  uint8_t *paad = alg->aad;
  int len = alg->aadlen;
  int ret, once, blk;
  bool first = true;

  blk = SONG_CRYPTO_GCM_BLK;

  while (len)
    {
      once = len > blk ? blk : len;

      ret = song_crypto_gcm_blk(alg_, first, false,
                                0, NULL,
                                0, NULL,
                                keylen, keydata,
                                once, paad);
      if (ret < 0)
        {
          return ret;
        }

      len  -= once;
      paad += once;
      first = false;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *song_crypto_gcm_init(struct alg_head_s *head)
{
  struct song_crypto_gcm_s *alg;
  uint32_t val;

  alg = kmm_zalloc(sizeof(struct song_crypto_gcm_s));
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

int song_crypto_gcm_uninit(FAR void *alg_)
{
  struct song_crypto_gcm_s *alg = alg_;

  nxsem_destroy(&alg->sem);
  free(alg);

  return 0;
}

int song_crypto_gcm_ioctl(FAR void *alg_, uint32_t param,
                          uint32_t length, FAR uint8_t *value)
{
  struct song_crypto_gcm_s *alg = alg_;

  switch (param)
    {
      case ALGPARAM_IV:
        if (length > SONG_CRYPTO_GCM_IV)
          {
            return -EINVAL;
          }

        memcpy(alg->iv, value, length);
        alg->ivlen = length;
        break;
      case ALGPARAM_AESGCM_AAD:
        if (length > SONG_CRYPTO_GCM_AAD)
          {
            return -EINVAL;
          }

        memcpy(alg->aad, value, length);
        alg->aadlen = length;
        break;
      case ALGPARAM_AESGCM_TAG:
        if (length < sizeof(alg->gcm_mac))
          {
            return -EINVAL;
          }

        memcpy(value, alg->gcm_mac, sizeof(alg->gcm_mac));
        break;
      case ALGPARAM_KEYUPDATE:
        alg->keyupdate = true;
        break;
      default:
        break;
    }

  return 0;
}

int song_crypto_gcm_isr(FAR void *alg_)
{
  struct song_crypto_gcm_s *alg = alg_;

  alg->result = song_crypto_interrupt(alg);
  nxsem_post(&alg->sem);

  return 0;
}

int song_crypto_gcm_exe(FAR void *alg_, bool first, bool last,
                        uint32_t inlen, FAR uint8_t *indata,
                        uint32_t outlen, FAR uint8_t *outdata,
                        uint32_t keylen, FAR uint8_t *keydata)
{
  struct song_crypto_gcm_s *alg = alg_;
  int ret;

  if (!outdata || outlen < inlen)
    {
      return -EINVAL;
    }

  if ((alg->head.algclass == ALG_CLASS_AES && keylen > 32) ||
          (alg->head.algclass == ALG_CLASS_SM4 && keylen != 16))
    {
      return -EINVAL;
    }

  if (first)
    {
      ret = song_crypto_gcm_iv(alg_, keylen, keydata);
      if (ret)
        {
          return ret;
        }

      if (alg->aadlen)
        {
          ret = song_crypto_gcm_aad(alg_, keylen, keydata);
          if (ret)
            {
              return ret;
            }

          first = false;
        }
    }

  return song_crypto_gcm_blk(alg_, first, last,
                             inlen, indata,
                             outlen, outdata,
                             keylen, keydata,
                             0, NULL);
}

