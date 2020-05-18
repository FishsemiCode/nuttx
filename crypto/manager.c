/****************************************************************************
 * crypto/manager.c
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

#include <stdint.h>
#include <stdbool.h>

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/crypto/module.h>
#include <nuttx/crypto/manager.h>
#include <nuttx/crypto/api.h>
#include <nuttx/crypto/ioctl.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     cryptoman_open (FAR struct file *filep);
static int     cryptoman_close(FAR struct file *filep);
static ssize_t cryptoman_read(FAR struct file *filep, FAR char *buffer,
                              size_t len);
static ssize_t cryptoman_write(FAR struct file *filep, FAR const char *buffer,
                               size_t len);
static int     cryptoman_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations cryptoman_fops =
{
  cryptoman_open,  /* open */
  cryptoman_close, /* close */
  cryptoman_read,  /* read */
  cryptoman_write, /* write */
  NULL,            /* seek */
  cryptoman_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions: Char driver callbacks
 ****************************************************************************/

/****************************************************************************
 * Name: cryptoman_open
 * Description: Allocate a cryptographic management session to the
 *   user/task calling this function.
 ****************************************************************************/

static int cryptoman_open(FAR struct file *filep)
{
  FAR struct cryptoman_context_s *ctx;

  ctx = cryptoman_sessalloc();
  if(!ctx)
    {
      return -ENOMEM;
    }

  filep->f_priv = ctx;

  return 0;
}

/****************************************************************************
 * Name: cryptoman_open
 ****************************************************************************/

static int cryptoman_close(FAR struct file *filep)
{
  FAR struct cryptoman_context_s *ctx = (FAR struct cryptoman_context_s*) filep->f_priv;

  return cryptoman_sessfree(ctx);
}

/****************************************************************************
 * Name: cryptoman_read
 * Description: Does nothing but required for O_RDWR opening
 ****************************************************************************/

static ssize_t cryptoman_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: cryptoman_write
 * Description: Does nothing but required for O_RDWR opening
 ****************************************************************************/

static ssize_t cryptoman_write(FAR struct file *filep, FAR const char *buffer,
                               size_t len)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: cryptoman_ioctl
 ****************************************************************************/

static int cryptoman_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct cryptoman_context_s *ctx = (FAR struct cryptoman_context_s*) filep->f_priv;

  switch(cmd)
    {
      case CRYPTOIOC_MODINFO    :
        {
          FAR struct cryptoioc_modinfo_s *modinfo = (FAR struct cryptoioc_modinfo_s*)arg;
          return cryptoman_modinfo(ctx,
                                   modinfo->first,
                                   modinfo->name,
                                   modinfo->alg,
                                   &modinfo->flags);
        }

      case CRYPTOIOC_MODSELECT  :
        {
          FAR struct cryptoioc_modselect_s *modselect = (FAR struct cryptoioc_modselect_s*)arg;
          return cryptoman_modselect(ctx,
                                     modselect->name);
        }

      case CRYPTOIOC_MODAUTH    :
        {
          FAR struct cryptoioc_modauth_s *modauth = (FAR struct cryptoioc_modauth_s*)arg;
          return cryptoman_modauth(ctx,
                                   modauth->step,
                                   modauth->len,
                                   modauth->data,
                                   modauth->rsplen,
                                   modauth->response);
        }

      case CRYPTOIOC_KEYINFO    :
        {
          FAR struct cryptoioc_keyinfo_s *keyinfo = (FAR struct cryptoioc_keyinfo_s*)arg;
          return cryptoman_keyinfo(ctx,
                                   keyinfo->first,
                                   &keyinfo->id,
                                   &keyinfo->length,
                                   &keyinfo->flags);
        }

      case CRYPTOIOC_KEYCREATE  :
        {
          FAR struct cryptoioc_keycreate_s *keycreate = (FAR struct cryptoioc_keycreate_s*)arg;
          return cryptoman_keycreate(ctx,
                                    keycreate->id,
                                    keycreate->flags,
                                    keycreate->length,
                                    keycreate->value);
        }

      case CRYPTOIOC_KEYDELETE  :
        {
          FAR struct cryptoioc_keydelete_s *keydelete = (FAR struct cryptoioc_keydelete_s*)arg;
          return cryptoman_keydelete(ctx,
                                     keydelete->keyid);
        }

      case CRYPTOIOC_KEYGENERATE:
        {
          FAR struct cryptoioc_keygen_s *keygen = (FAR struct cryptoioc_keygen_s*)arg;
          return cryptoman_keygen(ctx,
                                  keygen->keyid,
                                  keygen->rngalgid);
        }

      case CRYPTOIOC_KEYUPDATE  :
        {
          FAR struct cryptoioc_keyupdate_s *keyupdate = (FAR struct cryptoioc_keyupdate_s*)arg;
          return cryptoman_keyupdate(ctx,
                                     keyupdate->id,
                                     keyupdate->component,
                                     keyupdate->unwrapalg,
                                     keyupdate->buflen,
                                     keyupdate->buffer);
        }

      case CRYPTOIOC_KEYREAD    :
        {
          FAR struct cryptoioc_keyread_s *keyread = (FAR struct cryptoioc_keyread_s*)arg;
          return cryptoman_keyread(ctx,
                                   keyread->id,
                                   keyread->component,
                                   keyread->buflen,
                                   keyread->buffer);
        }

      case CRYPTOIOC_KEYCOPY    :
        {
          FAR struct cryptoioc_keycopy_s *keycopy = (FAR struct cryptoioc_keycopy_s*)arg;
          return cryptoman_keycopy(ctx,
                                   keycopy->destid,
                                   keycopy->flags,
                                   keycopy->srcid);
        }

      case CRYPTOIOC_ALGINFO    :
        {
          FAR struct cryptoioc_alginfo_s *alginfo = (FAR struct cryptoioc_alginfo_s*)arg;
          return cryptoman_alginfo(ctx,
                                   alginfo->id,
                                   &alginfo->blocklen);
        }

      case CRYPTOIOC_ALGINIT    :
        {
          FAR struct cryptoioc_alginit_s *alginit = (FAR struct cryptoioc_alginit_s*)arg;
          return cryptoman_alginit(ctx,
                                   alginit->algid,
                                   alginit->algop,
                                   alginit->keyid);
        }

      case CRYPTOIOC_ALGSETUP   :
        {
          FAR struct cryptoioc_algsetup_s *algsetup = (FAR struct cryptoioc_algsetup_s*)arg;
          return cryptoman_algsetup(ctx,
                                    algsetup->id,
                                    algsetup->length,
                                    algsetup->value);
        }

      case CRYPTOIOC_ALGSTATUS  :
        {
          FAR struct cryptoioc_algstatus_s *algstatus = (FAR struct cryptoioc_algstatus_s*)arg;
          return cryptoman_algstatus(ctx,
                                     algstatus->id,
                                     algstatus->buflen,
                                     algstatus->buffer);
        }

      case CRYPTOIOC_ALGUPDATE  :
        {
          FAR struct cryptoioc_algupdatefinish_s *algupdate = (FAR struct cryptoioc_algupdatefinish_s*)arg;
          int ret = cryptoman_algupdate(ctx,
                                        algupdate->len_in,
                                        algupdate->data_in,
                                        algupdate->len_out,
                                        algupdate->data_out);
          if (ret < 0)
            {
              return ret;
            }

          algupdate->len_out = ret;
          return 0;
        }

      case CRYPTOIOC_ALGFINISH  :
        {
          FAR struct cryptoioc_algupdatefinish_s *algfinish = (FAR struct cryptoioc_algupdatefinish_s*)arg;
          int ret = cryptoman_algfinish(ctx,
                                        algfinish->len_in,
                                        algfinish->data_in,
                                        algfinish->len_out,
                                        algfinish->data_out);
          if (ret < 0)
            {
              return ret;
            }

          algfinish->len_out = ret;
          return 0;
        }

    }
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devcrypto_register
 *
 * Description:
 *   Register the manager version of /dev/crypto
 *
 ****************************************************************************/

int devcrypto_register(void)
{
  cryptinfo("Initializing cryptography manager\n");
  if(cryptoman_libinit() != 0)
    {
      crypterr("Failed to initialize crypto manager\n");
      return -EINVAL;
    }
  return register_driver("/dev/crypto", &cryptoman_fops, 0666, NULL);
}

