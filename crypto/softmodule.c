/****************************************************************************
 * crypto/softmodule.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <stdint.h>
#include <stdbool.h>

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/crypto/module.h>
#include <nuttx/crypto/manager.h>
#include <nuttx/crypto/api.h>

#include "secutils.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct softcrypto_session_s;

struct softcrypto_key_s
{
  uint32_t flags;
  uint8_t id;
  uint32_t length;
  uint8_t value[16];
};

struct softcrypto_session_s
{
  struct softcrypto_key_s keys[CONFIG_CRYPTO_SOFTMODULE_RAMKEYS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR void* softcrypto_sessioncreate(void);
static int softcrypto_sessionfree(FAR void* session);
static int softcrypto_sessionauth(FAR void *session, uint32_t step,
                                  uint32_t cmdlen, FAR uint8_t *cmd,
                                  uint32_t rsplen, uint8_t *rsp);
static int softcrypto_keycount(FAR void *session);
static int softcrypto_keyinfo(FAR void *session, int index, FAR uint32_t *id,
                              FAR uint32_t *flags, FAR uint32_t *length);
static int softcrypto_keycreate(FAR void *session, uint32_t id, uint32_t flags,
                                uint32_t length, uint8_t *value);
static int softcrypto_keydelete(FAR void *session, uint32_t id);
static int softcrypto_keygen(FAR void *session, uint32_t id, uint32_t rngalg);
static int softcrypto_keyupdate(FAR void *session, uint32_t id,
                                uint32_t component, uint32_t unwrapalg,
                                uint32_t len, FAR uint8_t *value);
static int softcrypto_keyread(FAR void *session, uint32_t id,
                              uint32_t component, uint32_t buflen,
                              FAR uint8_t *buffer);
static int softcrypto_keycopy(FAR void *session, uint32_t dest, uint32_t src);
static int softcrypto_algsupported(FAR void *session, uint32_t algid);
static int softcrypto_alginit(FAR void *session, uint32_t algid, uint32_t mode,
                              uint32_t keyid);
static int softcrypto_algioctl(FAR void *session, uint32_t param,
                               uint32_t length, FAR uint8_t *value);
static int softcrypto_algupdate(FAR void *session, uint32_t inlen,
                                FAR uint8_t *indata, uint32_t outlen,
                                FAR uint8_t *outdata);
static int softcrypto_algfinish(FAR void *session, uint32_t inlen,
                                FAR uint8_t *indata, uint32_t outlen,
                                FAR uint8_t *outdata);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct cryptomodule_ops_s softmodule_ops =
{
  softcrypto_sessioncreate,
  softcrypto_sessionfree,
  softcrypto_sessionauth,
  softcrypto_keycount,
  softcrypto_keyinfo,
  softcrypto_keycreate,
  softcrypto_keydelete,
  softcrypto_keygen,
  softcrypto_keyupdate,
  softcrypto_keyread,
  softcrypto_keycopy,
  softcrypto_algsupported,
  softcrypto_alginit,
  softcrypto_algioctl,
  softcrypto_algupdate,
  softcrypto_algfinish,
};

struct cryptomodule_s softmodule =
{
    "SOFTCRYPTO",
    0,
    &softmodule_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Crypto Module Functions
 ****************************************************************************/

/****************************************************************************
 * Name: softcrypto_sessioncreate
 ****************************************************************************/

static FAR void* softcrypto_sessioncreate(void)
{
  FAR struct softcrypto_session_s *sess;
  int i;

  sess = kmm_zalloc(sizeof(struct softcrypto_session_s));
  if(!sess)
    {
      return NULL;
    }

  /* Erase all the keys - this destroys any potential uninitialized value they
   * may contain.
   */

  for(i=0;i<CONFIG_CRYPTO_SOFTMODULE_RAMKEYS; i++)
    {
      secmemclr((uint8_t*)&sess->keys[i], sizeof(sess->keys[i]) );
    }

  return sess;
}

/****************************************************************************
 * Name: softcrypto_sessionfree
 ****************************************************************************/

static int softcrypto_sessionfree(FAR void* session)
{
  FAR struct softcrypto_session_s *sess;
  int i;

  sess = (FAR struct softcrypto_session_s*)session;

  /* Erase all the keys, avoiding to leave some critical values around */

  for(i=0;i<CONFIG_CRYPTO_SOFTMODULE_RAMKEYS; i++)
    {
      secmemclr((uint8_t*)&sess->keys[i], sizeof(sess->keys[i]) );
    }

  free(session);
  return 0;
}

/****************************************************************************
 * Name: softcrypto_sessionauth
 ****************************************************************************/

static int softcrypto_sessionauth(FAR void *session, uint32_t step,
                                  uint32_t cmdlen, FAR uint8_t *cmd,
                                  uint32_t rsplen, uint8_t *rsp)
{
  cryptinfo("Called\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keycount
 ****************************************************************************/

static int softcrypto_keycount(FAR void *session)
{
  FAR struct softcrypto_session_s *sess;
  int i;
  int count = 0;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  for(i=0;i<CONFIG_CRYPTO_SOFTMODULE_RAMKEYS; i++)
    {
      if(sess->keys[i].length)
        {
          count += 1;
        }
    }

  return count;
}

/****************************************************************************
 * Name: softcrypto_keyinfo
 ****************************************************************************/

static int softcrypto_keyinfo(FAR void *session, int index, FAR uint32_t *id,
                              FAR uint32_t *flags, FAR uint32_t *length)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keycreate
 * Free keys are identified by their zero 'length' field.
 ****************************************************************************/

static int softcrypto_keycreate(FAR void *session, uint32_t id, uint32_t flags,
                                uint32_t length, uint8_t *value)
{
  FAR struct softcrypto_session_s *sess;
  int i;
  int slot;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  /* Find a slot */
  slot = -1;
  for(i=0;i<CONFIG_CRYPTO_SOFTMODULE_RAMKEYS; i++)
    {
      if(!sess->keys[i].length)
        {
          /* Found a slot */
          slot = i;
        }
    }

  if(slot < 0)
    {
      crypterr("no free key slot\n");
      return -ENOMEM;
    }

  if(length > sizeof(sess->keys[slot].value))
    {
      crypterr("length requested is too big\n");
      return -EINVAL;
    }

  sess->keys[slot].length = length;
  sess->keys[slot].flags  = flags;
  secmemcpy(sess->keys[slot].value, value, length);

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keydelete
 ****************************************************************************/

static int softcrypto_keydelete(FAR void *session, uint32_t id)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keygen
 ****************************************************************************/

static int softcrypto_keygen(FAR void *session, uint32_t id, uint32_t rngalg)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("Called\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keyupdate
 ****************************************************************************/

static int softcrypto_keyupdate(FAR void *session, uint32_t id,
                                uint32_t component, uint32_t unwrapalg,
                                uint32_t len, FAR uint8_t *value)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keyread
 ****************************************************************************/

static int softcrypto_keyread(FAR void *session, uint32_t id,
                              uint32_t component, uint32_t buflen,
                              FAR uint8_t *buffer)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_keycopy
 ****************************************************************************/

static int softcrypto_keycopy(FAR void *session, uint32_t dest, uint32_t src)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_algsupported
 ****************************************************************************/

static int softcrypto_algsupported(FAR void *session, uint32_t algid)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_alginit
 ****************************************************************************/

static int softcrypto_alginit(FAR void *session, uint32_t algid, uint32_t mode,
                               uint32_t keyid)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_algioctl
 ****************************************************************************/

static int softcrypto_algioctl(FAR void *session, uint32_t param,
                                uint32_t length, FAR uint8_t *value)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_algupdate
 ****************************************************************************/

static int softcrypto_algupdate(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Name: softcrypto_algfinish
 ****************************************************************************/

static int softcrypto_algfinish(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata)
{
  FAR struct softcrypto_session_s *sess;

  sess = (FAR struct softcrypto_session_s*)session;

  if(!session)
    {
      crypterr("no session!\n");
      return -ENODEV;
    }

  cryptinfo("TODO\n");
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: softcrypto_register
 ****************************************************************************/

int softcrypto_register(void)
{
  cryptinfo("Registering Module\n");
  return cryptoman_register(&softmodule);
}

