/****************************************************************************
 * drivers/crypto/song_crypto_utils.c
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

#include <nuttx/crypto/manager.h>
#include <nuttx/semaphore.h>

#include <semaphore.h>
#include <string.h>

#include "song_crypto.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t song_crypto_read(void *alg_, uint32_t offset)
{
  struct alg_head_s *head = alg_;
  return (*(volatile uint32_t *)(head->base + B2C(offset)));
}

void song_crypto_write(void *alg_, uint32_t offset, uint32_t val)
{
  struct alg_head_s *head = alg_;
  *(volatile uint32_t *)(head->base + B2C(offset)) = val;
}

int song_crypto_bcinfo(void *alg_, uint32_t algid, uint32_t algmode,
                       uint32_t keylen)
{
  struct alg_head_s *head = alg_;
  uint32_t info = 0;

  switch (algid)
    {
      case ALG_AES_ECB:
      case ALG_SM4_ECB:
        info |= CIPHER_BC_INFO_MODE_ECB;
        break;
      case ALG_AES_CBC:
      case ALG_SM4_CBC:
        info |= CIPHER_BC_INFO_MODE_CBC;
        break;
      case ALG_AES_CTR:
      case ALG_SM4_CTR:
        info |= CIPHER_BC_INFO_MODE_CTR;
        break;
      case ALG_AES_GCM:
      case ALG_SM4_GCM:
        info |= CIPHER_BC_INFO_MODE_GCM;
        break;
      case ALG_MODE_CAL:
        info |= CIPHER_BC_INFO_MODE_CAL;
        break;
      default:
        return -1;
    }

  if (algmode == ALGOP_CIPHER)
    {
      info |= CIPHER_BC_INFO_DIR_ENC;
    }
  else if (algmode == ALGOP_DECIPHER)
    {
      info |= CIPHER_BC_INFO_DIR_DEC;
    }
  else
    {
      return -1;
    }

  if (keylen == 16)
    {
      info |= CIPHER_BC_INFO_KEY_SIZE_BIT128;
    }
  else if (keylen == 24)
    {
      info |= CIPHER_BC_INFO_KEY_SIZE_BIT192;
    }
  else if (keylen == 32)
    {
      info |= CIPHER_BC_INFO_KEY_SIZE_BIT256;
    }
  else
    {
      return -1;
    }

  switch (head->algclass)
    {
      case ALG_CLASS_AES:
        info |= CIPHER_BC_INFO_ALG_AES;
        break;
      case ALG_CLASS_SM4:
        info |= CIPHER_BC_INFO_ALG_SM4;
        break;
      default:
        return -1;
    }

  song_crypto_write(head, CIPHER_BC_INFO, info);

  return 0;
}

int song_crypto_interrupt(void *alg_)
{
  struct alg_head_s *head = alg_;
  int status, bit, ret = -1;

  status = song_crypto_read(head, CIPHER_INT_STATUS);

  if (status & CIPHER_INT_EN_FIFO_OVF)
    {
      goto out;
    }

  switch (head->algclass)
    {
      case ALG_CLASS_AES:
        bit = CIPHER_INT_STATUS_AES_FINISH;
        break;
      case ALG_CLASS_CS:
        bit = CIPHER_INT_STATUS_CS_FINISH;
        break;
      case ALG_CLASS_SHA:
        bit = CIPHER_INT_STATUS_SHA_FINISH;
        break;
      case ALG_CLASS_SM3:
        bit = CIPHER_INT_STATUS_SM3_FINISH;
        break;
      case ALG_CLASS_SM4:
        bit = CIPHER_INT_STATUS_SM4_FINISH;
        break;
      default:
        bit = 0;
        break;
    }

  if (status & bit)
    {
      ret = 0;
    }

out:
  song_crypto_write(head, CIPHER_INT_STATUS, status);
  return ret;
}

