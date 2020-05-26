/****************************************************************************
 * drivers/crypto/song_crypto.c
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
 *    used to endorse or promote products derived from this song_ware
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

#include <stdint.h>
#include <stdbool.h>

#include <errno.h>

#include <nuttx/clk/clk.h>
#include <nuttx/crypto/module.h>
#include <nuttx/crypto/manager.h>
#include <nuttx/crypto/api.h>
#include <nuttx/crypto/song_crypto.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include "song_crypto.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_CRYPTO_KEY_MAXCNT              8
#define SONG_CRYPTO_KEY_MAXLEN              256
#define SONG_CRYPTO_KEY_ID_BASE             0xE0

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void *(*song_crypto_init_t)(struct alg_head_s *head);
typedef int (*song_crypto_uninit_t)(FAR void *alg);
typedef int (*song_crypto_ioctl_t)(FAR void *alg, uint32_t param,
                                   uint32_t length, FAR uint8_t *value);
typedef int (*song_crypto_isr_t)(FAR void *alg);
typedef int (*song_crypto_exe_t)(FAR void *alg, bool first, bool last,
                                 uint32_t inlen, FAR uint8_t *indata,
                                 uint32_t outlen, FAR uint8_t *outdata,
                                 uint32_t keylen, FAR uint8_t *keydata);

struct song_crypto_alg_s
{
  uint32_t             algclass;
  uint32_t             algid;
  uint32_t             blocklen;
  song_crypto_init_t   init;
  song_crypto_uninit_t uninit;
  song_crypto_ioctl_t  ioctl;
  song_crypto_isr_t    isr;
  song_crypto_exe_t    exe;
};

struct song_crypto_key_s
{
  uint8_t  key[SONG_CRYPTO_KEY_MAXLEN];
  bool     keyon;
  uint32_t keylen;
  uint32_t keyflags;
  uint32_t keyid;
};

struct song_crypto_sess_s
{
  struct song_crypto_s           *crypto;
  struct song_crypto_key_s        keys[SONG_CRYPTO_KEY_MAXCNT];
  const struct song_crypto_alg_s *algop;
  void                           *alg;
  int                             algkey;
  bool                            algupdate;
};

struct song_crypto_s
{
  struct cryptomodule_s              module;
  struct song_crypto_sess_s         *cur_sess;
  const struct song_crypto_config_s *config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR void* song_crypto_sessioncreate(FAR struct cryptomodule_s *module);
static int song_crypto_sessionfree(FAR void* session);
static int song_crypto_sessionauth(FAR void *session, uint32_t step,
                                   uint32_t cmdlen, FAR uint8_t *cmd,
                                   uint32_t rsplen, uint8_t *rsp);
static int song_crypto_keycount(FAR void *session);
static int song_crypto_keyinfo(FAR void *session, int index, FAR uint32_t *id,
                               FAR uint32_t *flags, FAR uint32_t *length);
static int song_crypto_keycreate(FAR void *session, uint32_t id, uint32_t flags,
                                 uint32_t length, uint8_t *value);
static int song_crypto_keydelete(FAR void *session, uint32_t id);
static int song_crypto_keygen(FAR void *session, uint32_t id, uint32_t rngalg);
static int song_crypto_keyupdate(FAR void *session, uint32_t id,
                                 uint32_t component, uint32_t unwrapalg,
                                 uint32_t len, FAR uint8_t *value);
static int song_crypto_keyread(FAR void *session, uint32_t id,
                               uint32_t component, uint32_t buflen,
                               FAR uint8_t *buffer);
static int song_crypto_keycopy(FAR void *session, uint32_t dest, uint32_t src);
static int song_crypto_algsupported(FAR void *session, uint32_t algid);
static int song_crypto_alginit(FAR void *session, uint32_t algid, uint32_t mode,
                               uint32_t keyid);
static int song_crypto_algioctl(FAR void *session, uint32_t param,
                                uint32_t length, FAR uint8_t *value);
static int song_crypto_algupdate(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata);
static int song_crypto_algfinish(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata);
static const FAR struct song_crypto_alg_s *song_crypto_algfindop(uint32_t algid);
static int song_crypto_keyfind(void *session, uint32_t keyid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct song_crypto_alg_s song_crypto_algs[] =
{
  /* CLASS AES */

  {
    ALG_CLASS_AES,
    ALG_AES_ECB,
    16,
    song_crypto_common_init,
    song_crypto_common_uninit,
    song_crypto_common_ioctl,
    song_crypto_common_isr,
    song_crypto_common_exe,
  },
  {
    ALG_CLASS_AES,
    ALG_AES_CBC,
    16,
    song_crypto_common_init,
    song_crypto_common_uninit,
    song_crypto_common_ioctl,
    song_crypto_common_isr,
    song_crypto_common_exe,
  },
  {
    ALG_CLASS_AES,
    ALG_AES_CTR,
    16,
    song_crypto_common_init,
    song_crypto_common_uninit,
    song_crypto_common_ioctl,
    song_crypto_common_isr,
    song_crypto_common_exe,
  },
  {
    ALG_CLASS_AES,
    ALG_AES_GCM,
    16,
    song_crypto_gcm_init,
    song_crypto_gcm_uninit,
    song_crypto_gcm_ioctl,
    song_crypto_gcm_isr,
    song_crypto_gcm_exe,
  },

  /* CLASS SM4 */

  {
    ALG_CLASS_SM4,
    ALG_SM4_ECB,
    16,
    song_crypto_common_init,
    song_crypto_common_uninit,
    song_crypto_common_ioctl,
    song_crypto_common_isr,
    song_crypto_common_exe,
  },
  {
    ALG_CLASS_SM4,
    ALG_SM4_CBC,
    16,
    song_crypto_common_init,
    song_crypto_common_uninit,
    song_crypto_common_ioctl,
    song_crypto_common_isr,
    song_crypto_common_exe,
  },
  {
    ALG_CLASS_SM4,
    ALG_SM4_CTR,
    16,
    song_crypto_common_init,
    song_crypto_common_uninit,
    song_crypto_common_ioctl,
    song_crypto_common_isr,
    song_crypto_common_exe,
  },
  {
    ALG_CLASS_SM4,
    ALG_SM4_GCM,
    16,
    song_crypto_gcm_init,
    song_crypto_gcm_uninit,
    song_crypto_gcm_ioctl,
    song_crypto_gcm_isr,
    song_crypto_gcm_exe,
  },

  /* CLASS SHA */

  {
    ALG_CLASS_SHA,
    ALG_SHA1,
    64,
    song_crypto_sha_init,
    song_crypto_sha_uninit,
    song_crypto_sha_ioctl,
    song_crypto_sha_isr,
    song_crypto_sha_exe,
  },
  {
    ALG_CLASS_SHA,
    ALG_SHA256,
    64,
    song_crypto_sha_init,
    song_crypto_sha_uninit,
    song_crypto_sha_ioctl,
    song_crypto_sha_isr,
    song_crypto_sha_exe,
  },

  /* CLASS SM3 */

  {
    ALG_CLASS_SM3,
    ALG_SM3,
    64,
    song_crypto_sm3_init,
    song_crypto_sm3_uninit,
    song_crypto_sm3_ioctl,
    song_crypto_sm3_isr,
    song_crypto_sm3_exe,
  },

  /* CLASS CHECKSUM */

  {
    ALG_CLASS_CS,
    ALG_IPV4HDR_CHECKSUM,
    16,
    song_crypto_cs_init,
    song_crypto_cs_uninit,
    song_crypto_cs_ioctl,
    song_crypto_cs_isr,
    song_crypto_cs_exe,
  },
};

static const struct cryptomodule_ops_s g_song_crypto_ops =
{
  song_crypto_sessioncreate,
  song_crypto_sessionfree,
  song_crypto_sessionauth,
  song_crypto_keycount,
  song_crypto_keyinfo,
  song_crypto_keycreate,
  song_crypto_keydelete,
  song_crypto_keygen,
  song_crypto_keyupdate,
  song_crypto_keyread,
  song_crypto_keycopy,
  song_crypto_algsupported,
  song_crypto_alginit,
  song_crypto_algioctl,
  song_crypto_algupdate,
  song_crypto_algfinish,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool song_crypto_is_dma_addr(FAR const struct song_crypto_config_s *config,
                                    FAR void *addr)
{
  extern uintptr_t _srodata;
  extern uintptr_t _erodata;
  uintptr_t addrr = (uintptr_t)addr;

  if (config->rodata_dma)
    {
      return true;
    }

  if ((addrr >= (uintptr_t)&_srodata) && (addrr <= (uintptr_t)&_erodata))
    {
      return false;
    }
  else
    {
      return true;
    }
}

static FAR void* song_crypto_sessioncreate(FAR struct cryptomodule_s *module)
{
  struct song_crypto_s *crypto = (struct song_crypto_s *)module;
  struct song_crypto_sess_s *sess;

  sess = kmm_zalloc(sizeof(struct song_crypto_sess_s));
  if (!sess)
    {
      return NULL;
    }

  crypto->cur_sess = sess;
  sess->crypto     = crypto;

  return sess;
}

static int song_crypto_sessionfree(FAR void* session)
{
  struct song_crypto_sess_s *sess = session;

  if (sess)
    {
      if (sess->alg)
      {
        sess->algop->uninit(sess->alg);
      }
      free(sess);
    }

  return 0;
}

static int song_crypto_sessionauth(FAR void *session, uint32_t step,
                                   uint32_t cmdlen, FAR uint8_t *cmd,
                                   uint32_t rsplen, uint8_t *rsp)
{
  return -ENOTSUP;
}

static int song_crypto_keycount(FAR void *session)
{
  struct song_crypto_sess_s *sess = session;
  int i, cnt = 0;

  if (!sess)
    {
      return -EINVAL;
    }

  for (i = 0; i < SONG_CRYPTO_KEY_MAXCNT; i++)
    {
      if (sess->keys[i].keyon)
        {
          cnt++;
        }
    }

  return cnt;
}

static int song_crypto_keyinfo(FAR void *session, int index, FAR uint32_t *id,
                               FAR uint32_t *flags, FAR uint32_t *length)
{
  struct song_crypto_sess_s *sess = session;

  if (!sess || index < 0 || index > SONG_CRYPTO_KEY_MAXCNT)
    {
      return -EINVAL;
    }

  if (!sess->keys[index].keyon)
    {
      return -ENODEV;
    }

  *id     = sess->keys[index].keyid;
  *flags  = sess->keys[index].keyflags;
  *length = sess->keys[index].keylen;

  return 0;
}

static int song_crypto_keycreate(FAR void *session, uint32_t id, uint32_t flags,
                                 uint32_t length, uint8_t *value)
{
  struct song_crypto_sess_s *sess = session;
  int index;

  if (!sess || (value && length > SONG_CRYPTO_KEY_MAXLEN))
    {
      return -EINVAL;
    }

  /* Check ID exist */

  if (id)
    {
      index = song_crypto_keyfind(session, id);
      if (index >= 0)
        {
          return -EEXIST;
        }
    }

  /* Find empty one */

  for (index = 0; index < SONG_CRYPTO_KEY_MAXCNT; index++)
    {
      if (!sess->keys[index].keyon)
        {
          break;
        }
    }

  if (index >= SONG_CRYPTO_KEY_MAXCNT)
    {
      return -ENOMEM;
    }

  /* Found */

  sess->keys[index].keyon    = true;
  sess->keys[index].keyflags = flags;
  sess->keys[index].keyid    = id ? id : SONG_CRYPTO_KEY_ID_BASE + index;

  if (value)
    {
      memcpy(sess->keys[index].key, value, length);
      sess->keys[index].keylen = length;
    }

  return sess->keys[index].keyid;
}

static int song_crypto_keydelete(FAR void *session, uint32_t id)
{
  struct song_crypto_sess_s *sess = session;
  int index;

  if (!sess)
    {
      return -EINVAL;
    }

  index = song_crypto_keyfind(session, id);
  if (index < 0)
    {
      return -ENOENT;
    }

  memset(&sess->keys[index], 0, sizeof(struct song_crypto_key_s));

  return 0;
}

static int song_crypto_keygen(FAR void *session, uint32_t id, uint32_t rngalg)
{
  return -ENOTSUP;
}

static int song_crypto_keyupdate(FAR void *session, uint32_t id,
                                 uint32_t component, uint32_t unwrapalg,
                                 uint32_t len, FAR uint8_t *value)
{
  struct song_crypto_sess_s *sess = session;
  int index;

  if (!sess || (value && len > SONG_CRYPTO_KEY_MAXLEN))
    {
      return -EINVAL;
    }

  index = song_crypto_keyfind(session, id);
  if (index < 0)
    {
      return -ENOENT;
    }

  if (value)
    {
      memcpy(sess->keys[index].key, value, len);
      sess->keys[index].keylen = len;
    }

  if (sess->algop && sess->alg)
    {
      sess->algop->ioctl(sess->alg, ALGPARAM_KEYUPDATE, 0, NULL);
    }

  return 0;
}

static int song_crypto_keyread(FAR void *session, uint32_t id,
                               uint32_t component, uint32_t buflen,
                               FAR uint8_t *buffer)
{
  struct song_crypto_sess_s *sess = session;
  int index;

  if (!sess || !buffer)
    {
      return -EINVAL;
    }

  index = song_crypto_keyfind(session, id);
  if (index < 0)
    {
      return -ENOENT;
    }

  if (buflen < sess->keys[index].keylen)
    {
      return -EINVAL;
    }

  if (sess->keys[index].keyflags & KEYFLAG_READABLE)
    {
      memcpy(buffer, sess->keys[index].key, sess->keys[index].keylen);
    }

  return 0;
}

static int song_crypto_keycopy(FAR void *session, uint32_t dest, uint32_t src)
{
  struct song_crypto_sess_s *sess = session;
  int id, is;

  if (!sess)
    {
      return -EINVAL;
    }

  id = song_crypto_keyfind(session, dest);
  is = song_crypto_keyfind(session, src);
  if (id < 0 || is < 0)
    {
      return -ENOENT;
    }

  memcpy(sess->keys[id].key, sess->keys[is].key, sess->keys[is].keylen);
  sess->keys[id].keyon    = sess->keys[is].keyon;
  sess->keys[id].keylen   = sess->keys[is].keylen;
  sess->keys[id].keyflags = sess->keys[is].keyflags;

  return 0;
}

static int song_crypto_algsupported(FAR void *session, uint32_t algid)
{
  const struct song_crypto_alg_s *alg;

  alg = song_crypto_algfindop(algid);

  return alg ? alg->blocklen : -ENOTSUP;
}

static int song_crypto_alginit(FAR void *session, uint32_t algid, uint32_t mode,
                               uint32_t keyid)
{
  struct song_crypto_sess_s *sess = session;
  struct song_crypto_s *crypto = sess->crypto;
  struct alg_head_s head;
  int keyidx = 0;

  if (!sess)
    {
      return -EINVAL;
    }

  if (sess->alg && sess->algop)
    {
      sess->algop->uninit(sess->alg);
    }

  sess->algop = song_crypto_algfindop(algid);
  if (!sess->algop)
    {
      return -ENOENT;
    }

  head.base     = crypto->config->base;
  head.algclass = sess->algop->algclass;
  head.algid    = sess->algop->algid;
  head.algmode  = mode;
  sess->alg = sess->algop->init(&head);
  if (!sess->alg)
    {
      sess->algop = NULL;
      return -EINVAL;
    }

  if (keyid)
    {
      keyidx = song_crypto_keyfind(session, keyid);
      if (keyidx < 0)
        {
          return -ENOENT;
        }
    }

  sess->algkey    = keyidx;
  sess->algupdate = false;

  return 0;
}

static int song_crypto_algioctl(FAR void *session, uint32_t param,
                                uint32_t length, FAR uint8_t *value)
{
  struct song_crypto_sess_s *sess = session;

  if (!sess)
    {
      return -EINVAL;
    }

  if (!sess->alg || !sess->algop)
    {
      return -EPERM;
    }

  return sess->algop->ioctl(sess->alg, param, length, value);
}

static int song_crypto_algupdate(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata)
{
  struct song_crypto_sess_s *sess = session;
  uint8_t *in = indata;
  int ret;

  if (!sess)
    {
      return -EINVAL;
    }

  if (!sess->alg || !sess->algop)
    {
      return -EPERM;
    }

  if (inlen > 0 && !song_crypto_is_dma_addr(sess->crypto->config, indata))
    {
      in = kmm_malloc(inlen);
      if (!in)
        {
          return -ENOMEM;
        }
      memcpy(in, indata, inlen);
    }

  if (sess->algupdate)
    {
      ret = sess->algop->exe(sess->alg, false, false,
                            inlen, in,
                            outlen, outdata,
                            sess->keys[sess->algkey].keylen,
                            sess->keys[sess->algkey].key);
    }
  else
    {
      ret = sess->algop->exe(sess->alg, true, false,
                            inlen, in,
                            outlen, outdata,
                            sess->keys[sess->algkey].keylen,
                            sess->keys[sess->algkey].key);
    }

  sess->algupdate = true;

  if (in != indata)
    {
      free(in);
    }

  return ret;
}

static int song_crypto_algfinish(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata)
{
  struct song_crypto_sess_s *sess = session;
  uint8_t *in = indata;
  int ret;

  if (!sess)
    {
      return -EINVAL;
    }

  if (!sess->alg || !sess->algop)
    {
      return -EPERM;
    }

  if (inlen > 0 && !song_crypto_is_dma_addr(sess->crypto->config, indata))
    {
      in = kmm_malloc(inlen);
      if (!in)
        {
          return -ENOMEM;
        }
      memcpy(in, indata, inlen);
    }

  if (sess->algupdate)
    {
      ret = sess->algop->exe(sess->alg, false, true,
                            inlen, in,
                            outlen, outdata,
                            sess->keys[sess->algkey].keylen,
                            sess->keys[sess->algkey].key);
    }
  else
    {
      ret = sess->algop->exe(sess->alg, true, true,
                            inlen, in,
                            outlen, outdata,
                            sess->keys[sess->algkey].keylen,
                            sess->keys[sess->algkey].key);
    }

  if (in != indata)
    {
      free(in);
    }

  return ret;
}

static const struct song_crypto_alg_s *song_crypto_algfindop(uint32_t algid)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(song_crypto_algs); i++)
    {
      if (algid == song_crypto_algs[i].algid)
        {
          return &song_crypto_algs[i];
        }
    }

  return NULL;
}

static int song_crypto_keyfind(void *session, uint32_t keyid)
{
  struct song_crypto_sess_s *sess = session;
  int i;

  for (i = 0; i < SONG_CRYPTO_KEY_MAXCNT; i++)
    {
      if (sess->keys[i].keyon && sess->keys[i].keyid == keyid)
        {
          return i;
        }
    }

  return -1;
}

static int song_crypto_irq(int irq, FAR void *context, FAR void *arg)
{
  struct song_crypto_s *crypto = arg;
  struct song_crypto_sess_s *sess = crypto->cur_sess;

  if (irq != crypto->config->irq)
    {
      return -EINVAL;
    }

  if (!sess || !sess->alg || !sess->algop)
    {
      return -EPERM;
    }

  return sess->algop->isr(sess->alg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int song_crypto_initialize(const struct song_crypto_config_s *config)
{
  struct song_crypto_s *crypto;
  int ret;

  if (!config)
    {
      return -EINVAL;
    }

  crypto = kmm_zalloc(sizeof(struct song_crypto_s));
  if (!crypto)
    {
      return -ENOMEM;
    }

  crypto->config = config;
  strncpy(crypto->module.name, "SONGCRYPTO", 32);
  crypto->module.flags = CRYPTOMOD_FLAG_INSECURE;
  crypto->module.ops = &g_song_crypto_ops;

  ret = cryptoman_register(&crypto->module);
  if (ret < 0)
    {
      kmm_free(crypto);
      return ret;
    }

  if (config->clk)
    {
      clk_enable(clk_get(config->clk));
    }

  irq_attach(config->irq, song_crypto_irq, crypto);
  up_enable_irq(config->irq);

  return ret;
}
