/****************************************************************************
 * crypto/libmanager.c
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/crypto/module.h>
#include <nuttx/crypto/manager.h>
#include <nuttx/crypto/ioctl.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cryptoman_context_s
{
  FAR struct cryptomodule_s *module;        /* Pointer to the current module */
  FAR void                  *session;       /* Session handle within the module */

  int                       current_module; /* Used for module enumeration */
  int                       current_key;    /* Used for key enumeration */

  uint32_t                  blocksize;      /* Selected ALG block length,
                                             * Zero means not BLOCK
                                             */
  FAR uint8_t               *buffer;        /* Buffer for ALG */
  uint32_t                  buflen;         /* Number of bytes used in the buffer */
  uint32_t                  opmode;         /* Opmode for the selected alg */
  uint8_t                   padding;        /* Padding method used with selected alg.
                                             * Padding must be ALG_PADDING_NONE when
                                             * blocksize == 0;
                                             */
};

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* Access protection to the list of modules */

static sem_t g_crypto_semaphore;

static struct cryptomodule_s *g_crypto_modules[CONFIG_CRYPTO_MANAGER_NMODULES];

/*****************************************************************************
 * Private functions
 *****************************************************************************/

/****************************************************************************
 * Name: cryptoman_padzero
 * Description: Pad with zero bytes. CANNOT BE REMOVED BY DECRYPTION.
 ****************************************************************************/

static int cryptoman_padzero(FAR uint8_t *buf, uint32_t len, uint32_t opmode)
{
  if (ALGOP_CIPHER == opmode)
    {
      memset(buf, 0, len);
    }

  return 0;
}

/****************************************************************************
 * Name: cryptoman_padx923
 * Descriptin: ALGOP_CIPHER:zeros, with explicit length in last byte
 *             ALGOP_DECIPHER:Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padx923(FAR uint8_t *buf, uint32_t len, uint32_t opmode)
{
  if (ALGOP_CIPHER == opmode)
    {
      buf[len-1] = len;
      memset(buf, 0, len - 1);
    }

  return 0;
}

/****************************************************************************
 * Name: cryptoman_padiso78164
 * Description: ALGOP_CIPHER:0x80, followed by zeros.
 *              ALGOP_DECIPHER:Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padiso78164(FAR uint8_t *buf, uint32_t len, uint32_t opmode)
{
  if (ALGOP_CIPHER == opmode)
    {
      /* Need at least one byte */

      if (len < 1)
        {
          return -EINVAL; /* Should not happen */
        }

      buf[0] = 0x80;
      memset(buf + 1, 0, len - 1);
    }

  return 0;
}

/****************************************************************************
 * Name: cryptoman_padpkcs5
 * Description: ALGOP_CIPHER:Use a byte that represents the length, repeated as required.
 *              Name copied for javacard for ease of comparison.
 *              ALGOP_DECIPHER:Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padpkcs5(FAR uint8_t *buf, uint32_t len, uint32_t opmode)
{
  if (ALGOP_CIPHER == opmode)
    {
      memset(buf, len, len);
    }

  return 0;
}

/****************************************************************************
 * Name: cryptoman_padiso10126
 ****************************************************************************/

static int cryptoman_padiso10126(FAR uint8_t *buf, uint32_t len, uint32_t opmode)
{
  return cryptoman_padx923(buf, len, opmode);
}

/****************************************************************************
 * Name: cryptoman_processfullblocks
 * Description: Process as many full blocks of data as possible, put the
 *              remain which not enough for one full block to ctx->buffer
 * Parameters :
 * ctx        : The crypto context
 * len_in     : Number of bytes available in the input buffer
 * data_in    : Pointer to bytes that need to be managed
 * len_out    : Number of bytes available in the output buffer
 * data_out   : Pointer to the output buffer, where generated data is stored
 * finishing  : The last frame data and no padding will be set true,
 *                else will be set false.
 * Returns    : The number of bytes WRITTEN in buf_out (obviously a
 *              multiple of the block size), or a negative error
 ****************************************************************************/

static int cryptoman_processfullblocks(FAR struct cryptoman_context_s *ctx,
                                       uint32_t len_in, FAR uint8_t *data_in,
                                       uint32_t len_out, FAR uint8_t *data_out,
                                       bool finishing)
{
  uint32_t blocklen;
  uint32_t done = 0; /* How many bytes have been written to output (total)*/
  int ret = 0;

  /* First, look at the accumulation buffer */

  if (ctx->buflen > 0)
    {
      /* Some bytes in the temp buffer require processing. First compute how
       * many bytes are required to fill the block buffer.
       */

      uint32_t processlen = ctx->blocksize - ctx->buflen;

      if (len_in >= processlen)
        {
          /* There is enough incoming data to fill the temp buf.
           * Complete the buffer up to a block and process it,
           */

          memcpy(ctx->buffer + ctx->buflen, data_in, processlen);

          /* We just call the algorithm update routine EXCEPT IF
           * Padding is not used (or finishing would be false)
           * We are called from alg_finish, AND
           * The incoming data EXACTLY completes the temp block */

          if (finishing && (len_in == processlen))
            {
              /* We are in the last steps, there was no padding, and the
               * incoming data is just big enough to complete the pending block
               */

              ret = ctx->module->ops->alg_finish(ctx->session,
                                                 ctx->blocksize, ctx->buffer,
                                                 len_out, data_out);
            }
          else
            {
              /* Normal case: This is not the last crypto operation on this
               * stream because there is padding, too many data to fit in this
               * block, or called from alg_update
               */
              ret = ctx->module->ops->alg_update(ctx->session,
                                                 ctx->blocksize, ctx->buffer,
                                                 len_out, data_out);
            }

          if (ret < 0)
            {
              return ret; /* Something failed */
            }

          /* Update input variables to indicate how many have been consumed */

          data_in += processlen;
          len_in  -= processlen;

          /* Update output variables to indicate what has been consumed */

          data_out += ret; /* Update output position */
          len_out  -= ret; /* Update output room */
          done     += ret; /* Remember how many bytes we've done */

          /* Now the temp buffer has been processed */

          ctx->buflen = 0;
        }
      else
        {
          /* The incoming data is not long enough to full fill the temp buffer.
           * So a full block cannot be processed, then save to temp buffer.
           */

          goto save_to_buffer;
        }
    }

  /* The temp buffer has been potentially processed. Now process full blocks
   * from incoming buffer.
   */

  blocklen = (len_in / ctx->blocksize) * ctx->blocksize;

  if (blocklen)
    {
      /* Some full blocks are available */

      if (finishing && blocklen == len_in)
        {
          /* We were called from alg_finish with no padding, this is the last
           * call on this data stream
           */

          ret = ctx->module->ops->alg_finish(ctx->session,
                                             blocklen, data_in,
                                             len_out, data_out);
        }
      else
        {
          /* Not called from finish and/or padding is applied to this data stream */

          ret = ctx->module->ops->alg_update(ctx->session,
                                             blocklen, data_in,
                                             len_out, data_out);
        }

      if (ret < 0)
        {
          return ret; /* Something failed */
        }

      /* Update variables so the caller knows what has been managed */

      data_in += blocklen;
      len_in  -= blocklen;
      done    += ret;
    }

save_to_buffer:
  if (len_in)
    {
      /* Save whatever bytes were not processed to the temp buffer */

      memcpy(ctx->buffer + ctx->buflen, data_in, len_in);
      ctx->buflen += len_in;
    }

  return done;
}

/****************************************************************************
 * Public Functions : Kernel crypto API usable by other parts of the system
 ****************************************************************************/

/****************************************************************************
 * Name: cryptoman_sessalloc
 ****************************************************************************/

FAR struct cryptoman_context_s *cryptoman_sessalloc(void)
{
  FAR struct cryptoman_context_s *context;

  context = kmm_zalloc(sizeof(struct cryptoman_context_s));

  return context;
}

/****************************************************************************
 * Name: cryptoman_sessfree
 ****************************************************************************/

int cryptoman_sessfree(FAR struct cryptoman_context_s *ctx)
{
  if (ctx)
    {
      if (ctx->buffer)
        {
          kmm_free(ctx->buffer); /* Free any allocated block buffer */
        }

      /* Start by closing the module's session if a module has been selected */

      if (ctx->module)
        {
          ctx->module->ops->session_free(ctx->session);
        }

      kmm_free(ctx);
    }

  return 0;
}


/****************************************************************************
 * Name: cryptoman_modinfo
 ****************************************************************************/

int cryptoman_modinfo(FAR struct cryptoman_context_s *ctx,
                      bool first,
                      char name[32],
                      uint32_t alg,
                      FAR uint32_t *flags)
{
  if (first)
    {
      ctx->current_module = -1;
    }

  /* Find next module */

  while (true)
    {
      FAR struct cryptomodule_s *curmod;

      ctx->current_module += 1;

      if (ctx->current_module >= CONFIG_CRYPTO_MANAGER_NMODULES)
        {
          /* Reached end of list. In the future, browse the dynamic
           * list, too..
           */
          break;
        }

      curmod = g_crypto_modules[ctx->current_module];

      if (curmod == NULL)
        {
          continue; /* No module in this slot */
        }

      /* Here curmod is not null */

      /* If name was passed, check that the module name matches */

      if (name[0] && strncmp(name, curmod->name, 32))
        {
          continue;
        }

      /* Check if alg supported by module. */

      if (alg != ALG_NONE)
        {
          if (curmod->ops->alg_supported(ctx->session, alg) < 0)
            {
              continue; /* Requested alg not supported by module */
            }
        }

      /* Found a module, copy info */

      strncpy(name, curmod->name, 32);
      *flags = curmod->flags;

      return 0;
    }

  /* All modules checked and none can be returned */

  return -ENODEV;
}

/****************************************************************************
 * Name: cryptoman_modselect
 * Description: see include/nuttx/crypto/api.h
 ****************************************************************************/

int cryptoman_modselect(FAR struct cryptoman_context_s *ctx, char name[32])
{
  FAR struct cryptomodule_s *tempmodule = NULL;
  FAR void *tempsession;
  int i;

  /* First of all check that the user is not reselecting the same module */

  if (ctx->module && !strncmp(ctx->module->name, name, 32))
    {
      return 0;
    }

  /* Search for the required module. We iterate on ALL modules entries */

  for (i = 0; i < CONFIG_CRYPTO_MANAGER_NMODULES; i++)
    {
      if (!g_crypto_modules[i])
        {
          continue;
        }

      if (g_crypto_modules[i]->name &&
              strncmp(g_crypto_modules[i]->name, name, 32) == 0)
        {
          tempmodule = g_crypto_modules[i];
        }
    }

  /* Required module not found: Exit without changing the current module */

  if (!tempmodule)
    {
      return -ENODEV;
    }

  /* Try to make a session in the new module */

  tempsession = tempmodule->ops->session_create(tempmodule);
  if (!tempsession)
    {
      return -ENOMEM; /* Session creation failed */
    }

  /* All ok. If the previous module had a session, free it first */

  if (ctx->module && ctx->session)
    {
      ctx->module->ops->session_free(ctx->session);
    }

  /* Now update the context with the new info and create a session */

  ctx->module  = tempmodule;
  ctx->session = tempsession;

  return 0;
}

/****************************************************************************
 * Name: cryptoman_modauth
 ****************************************************************************/

int cryptoman_modauth(FAR struct cryptoman_context_s *ctx,
                      uint32_t step,
                      uint32_t len,
                      FAR uint8_t *data,
                      uint32_t rsplen,
                      FAR uint8_t *response)
{
  if (!ctx || !data || !response)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->session_auth(ctx->session, step, len, data,
                                        rsplen, response);
}

/****************************************************************************
 * Name: cryptoman_keyinfo
 ****************************************************************************/

int cryptoman_keyinfo(FAR struct cryptoman_context_s *ctx,
                      bool first,
                      FAR uint32_t *id,
                      FAR uint32_t *length,
                      FAR uint32_t *flags)
{
  int ret, key_cnt;

  if (!ctx || !id || !length || !flags)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  ret = ctx->module->ops->key_count(ctx->session);
  if (ret < 0)
    {
      return ret;
    }

  key_cnt = ret;

  if (first)
    {
      ctx->current_key = -1;
    }

  while (true)
    {
      uint32_t keyid;

      ctx->current_key += 1;

      if (ctx->current_key >= key_cnt)
        {
          break;
        }

      ret = ctx->module->ops->key_info(ctx->session, ctx->current_key,
                                       &keyid, flags, length);
      if (ret == -ENODEV)
        {
          continue;
        }
      else if (ret)
        {
          return ret;
        }

      if (*id && *id != keyid)
        {
          continue;
        }

      *id = keyid;
      return 0;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: cryptoman_keycreate
 ****************************************************************************/

int cryptoman_keycreate(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t flags,
                        uint32_t length,
                        FAR uint8_t *value)
{
  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->key_create(ctx->session, id, flags, length, value);
}

/****************************************************************************
 * Name: cryptoman_keydelete
 ****************************************************************************/

int cryptoman_keydelete(FAR struct cryptoman_context_s *ctx,
                        uint32_t id)
{
  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->key_delete(ctx->session, id);
}

/****************************************************************************
 * Name: cryptoman_keygen
 ****************************************************************************/

int cryptoman_keygen(FAR struct cryptoman_context_s *ctx,
                     uint32_t keyid,
                     uint32_t rngalgid)
{
  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->key_gen(ctx->session, keyid, rngalgid);
}

/****************************************************************************
 * Name: cryptoman_keyupdate
 ****************************************************************************/

int cryptoman_keyupdate(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t component,
                        uint32_t unwrapalg,
                        uint32_t buflen,
                        FAR uint8_t *buffer)
{
  if (!ctx || !buffer)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->key_update(ctx->session, id, component, unwrapalg,
                                      buflen, buffer);
}

/****************************************************************************
 * Name: cryptoman_keyread
 ****************************************************************************/

int cryptoman_keyread(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t component,
                        uint32_t buflen,
                        FAR uint8_t *buffer)
{
  if (!ctx || !buffer)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->key_read(ctx->session, id, component, buflen,
                                    buffer);
}

/****************************************************************************
 * Name: cryptoman_keycopy
 ****************************************************************************/

int cryptoman_keycopy(FAR struct cryptoman_context_s *ctx,
                      uint32_t destid,
                      uint32_t flags,
                      uint32_t srcid)
{
  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  //TODO, check flags or set to lower ?

  return ctx->module->ops->key_copy(ctx->session, destid, srcid);
}

/****************************************************************************
 * Name: cryptoman_alginfo
 ****************************************************************************/

int cryptoman_alginfo(FAR struct cryptoman_context_s *ctx,
                      uint32_t id,
                      FAR uint32_t *blocklen)
{
  int ret;

  if (!ctx || !blocklen)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  ret = ctx->module->ops->alg_supported(ctx->session, id);
  if (ret < 0)
    {
      return ret;
    }

  *blocklen = ret;

  return 0;
}

/****************************************************************************
 * Name: cryptoman_alginit
 ****************************************************************************/

int cryptoman_alginit(FAR struct cryptoman_context_s *ctx,
                      uint32_t algid,
                      uint32_t opmode,
                      uint32_t keyid)
{
  int ret;

  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  /* Get the algorithm block size and allocate a block buffer. This is used to
   * handle arbitrary buffer sizes the user may pass.
   */

  ret = ctx->module->ops->alg_supported(ctx->session, algid);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate new block buffer if size has changed */

  if (ctx->blocksize != ret)
    {
      /* Maybe we have a previous buffer? */

      if (ctx->buffer)
        {
          kmm_free(ctx->buffer);
        }

      ctx->blocksize = ret;
      if (ctx->blocksize > 0)
        {
          ctx->buffer = kmm_malloc(ctx->blocksize);
          if (!ctx->buffer)
            {
              return -ENOMEM;
            }
        }
    }

  ctx->buflen  = 0; /* Mark the buffer as empty */
  ctx->padding = ALG_PADDING_NONE; /* Default to no padding */
  ctx->opmode  = opmode;

  return ctx->module->ops->alg_init(ctx->session, algid, opmode, keyid);
}

/****************************************************************************
 * Name: cryptoman_algsetup
 ****************************************************************************/

int cryptoman_algsetup(FAR struct cryptoman_context_s *ctx,
                       uint32_t id,
                       uint32_t length,
                       FAR uint8_t *value)
{
  int ret = -EINVAL;

  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  /* Some parameters are managed at the library level */

  switch(id)
    {
      case ALGPARAM_PADDING:
        {
          if (length != 1)
            {
              break;
            }

          if (ctx->blocksize == 0 && value[0] != ALG_PADDING_NONE)
            {
              /* NON block mode, padding must be zero */

              break;
            }
          else
            {
              ctx->padding = value[0];
              ret = 0;
            }
          break;
        }
      default:
        {
          ret = ctx->module->ops->alg_ioctl(ctx->session, id, length, value);
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cryptoman_algstatus
 ****************************************************************************/

int cryptoman_algstatus(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t buflen,
                        FAR uint8_t *buffer)
{
  if (!ctx)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  return ctx->module->ops->alg_ioctl(ctx->session, id, buflen, buffer);
}

/****************************************************************************
 * Name: cryptoman_algupdate
 * Description: see include/nuttx/crypto/api.h
 ****************************************************************************/

int cryptoman_algupdate(FAR struct cryptoman_context_s *ctx,
                        uint32_t len_in, FAR uint8_t *data_in,
                        uint32_t len_out, FAR uint8_t *data_out)
{
  if (!ctx || !data_in || !data_out)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  if (ctx->blocksize == 0 || len_in == 0)
    {
      /* NON-blcok mode, just call lower ops */

      return ctx->module->ops->alg_update(ctx->session,
                                          len_in, data_in,
                                          len_out, data_out);
    }
  else
    {

      /* Block mode, prcess blocks and save the remain to buffer */

      return cryptoman_processfullblocks(ctx,
                                         len_in, data_in,
                                         len_out, data_out, false);
    }
}

/****************************************************************************
 * Name: cryptoman_algfinish
 * Description: see include/nuttx/crypto/api.h
 ****************************************************************************/

int cryptoman_algfinish(FAR struct cryptoman_context_s *ctx,
                        uint32_t len_in, FAR uint8_t *data_in,
                        uint32_t len_out, FAR uint8_t *data_out)
{
  int (*padfunc)(FAR uint8_t *buf, uint32_t len, uint32_t opmode);
  int done = 0; /* Total number of bytes written to output */
  int ret;

  if (!ctx || !data_in || !data_out)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  /* NON-blcok mode, just call lower ops */

  if (ctx->blocksize == 0 || (len_in == 0 && ctx->buflen == 0))
    {
      return ctx->module->ops->alg_finish(ctx->session,
                                          len_in, data_in,
                                          len_out, data_out);
    }

  /* Following are blcok mode */

  switch(ctx->padding)
    {
      case ALG_PADDING_NONE:
        padfunc = NULL;
        break;
      case ALG_PADDING_ZERO:
        padfunc = cryptoman_padzero;
        break;
      case ALG_PADDING_X923:
        padfunc = cryptoman_padx923;
        break;
      case ALG_PADDING_ISO78164:
        padfunc = cryptoman_padiso78164;
        break;
      case ALG_PADDING_PKCS5:
        padfunc = cryptoman_padpkcs5;
        break;
      case ALG_PADDING_ISO10126:
        padfunc = cryptoman_padiso10126;
        break;
      default:
        return -EINVAL;
    }

  /* Apply common processing to the temp buffer and full sized data blocks.
   * This is NEVER the last step if some padding has to be applied.
   */

  ret = cryptoman_processfullblocks(ctx, len_in, data_in,
                                    len_out, data_out,
                                    (ctx->padding == ALG_PADDING_NONE));
  if (ret < 0)
    {
      return ret; /* Something failed */
    }

  data_out += ret;
  len_out  -= ret;
  done     += ret;

  /* We are finishing the ciphering session. There are multiple situations:
   * Algorithm has padding: It MUST be applied even if no data remains
   * Algorithm has NO padding: if no data remains, then return
   */

  if (!padfunc && ctx->buflen == 0)
    {
      return done; /* Nothing more to do */
    }

  if (padfunc)
    {
      /* Do the padding in the temp buffer. At this point we are sure that at
       * least one byte is available in the buffer (len<blocksize) otherwise it
       * would have been processed before !
       * NOTE: IT CURRENTLY ONLY WORKS WITH PADDING WITH MINIMAL LENGTH 1, which
       * is always the case with currently supported paddings.
       * If a padding with a minimal length of 2 was to be supported, then some
       * changes would be necessary. SHA type algorithms manage their padding in
       * the crypto modules, it is not managed here since it's very special and
       * is usually generated inside the hardware modules themselves.
       */

      ret = padfunc(ctx->buffer + ctx->buflen, ctx->blocksize - ctx->buflen, ctx->opmode);
      if (ret < 0)
        {
          return ret; /* Something failed */
        }

      ctx->buflen = ctx->blocksize;
    }

  /* Last crypto round on full padded block */

  ret = ctx->module->ops->alg_finish(ctx->session,
                                     ctx->buflen, ctx->buffer,
                                     len_out, data_out);
  if (ret < 0)
    {
      return ret;
    }

  ctx->buflen = 0;

  done += ret;
  return done;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cryptomod_register
 *
 * Description:
 *   Register a cryptographic module with the manager.
 *
 ****************************************************************************/

int cryptoman_register(FAR struct cryptomodule_s *module)
{
  int ret = -EACCES;
  int i;

  if (!module)
    {
      return -EINVAL;
    }

  nxsem_wait(&g_crypto_semaphore);

  /* Find a slot in the list of module pointers */

  for (i = 0; i < CONFIG_CRYPTO_MANAGER_NMODULES; i++)
    {
      if (g_crypto_modules[i] &&
          strncmp(g_crypto_modules[i]->name, module->name, 32) == 0)
        {
          ret = -EADDRINUSE;
          break;
        }
      else if (g_crypto_modules[i] == NULL)
        {
          g_crypto_modules[i] = module;
          ret = 0;
          break;
        }
    }

  nxsem_post(&g_crypto_semaphore);
  return ret;
}

/****************************************************************************
 * Name: cryptoman_libinit
 *
 * Description:
 *   Init cryptographic module.
 *
 ****************************************************************************/

int cryptoman_libinit(void)
{
  int i;

  nxsem_init(&g_crypto_semaphore, 0, 1);

  for (i = 0; i < CONFIG_CRYPTO_MANAGER_NMODULES; i++)
    {
      g_crypto_modules[i] = NULL;
    }

  return 0;
}

