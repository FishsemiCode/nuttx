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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

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
  FAR struct cryptomodule_s *module; /* Pointer to the current module */
  FAR void    *session;                 /* Session handle within the module */

  int          current_module;       /* Used for module enumeration */
  int          current_key;          /* Used for key enumeration */

  uint32_t     blocksize;            /* Selected ALG block length */
  FAR uint8_t *buffer;               /* Buffer for ALG */
  uint32_t     buflen;               /* Number of bytes used in the buffer */
  uint32_t     opmode;               /* Opmode for the selected alg */
  uint8_t      padding;              /* Padding method used with selected alg */
};

static sem_t gUserSemaphore; /* Access protection to the list of modules */

FAR struct cryptomodule_s * gCryptoModules[CONFIG_CRYPTO_MANAGER_NMODULES];

#ifdef CONFIG_CRYPTO_MANAGER_DYNMODULES
#error dynamically allocated modules not supported yet (TODO)!
#endif

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
  context->blocksize = 0;
  return context;
}

/****************************************************************************
 * Name: cryptoman_sessfree
 ****************************************************************************/

int cryptoman_sessfree(FAR struct cryptoman_context_s *ctx)
{
  if(ctx)
    {
      if(ctx->buffer)
        {
          kmm_free(ctx->buffer); /* Free any allocated block buffer */
        }
      /* Start by closing the module's session if a module has been selected */
      if(ctx->module)
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
  if(first)
    {
      ctx->current_module = -1;
    }

  /* Find next module */

  while(true)
    {
      FAR struct cryptomodule_s *curmod;

      ctx->current_module += 1;

      if(ctx->current_module >= CONFIG_CRYPTO_MANAGER_NMODULES)
        {
          /* Reached end of list. In the future, browse the dynamic
           * list, too..
           */
          break;
        }

      curmod = gCryptoModules[ctx->current_module];

      if(curmod == NULL)
        {
          continue; /* No module in this slot */
        }

      /* Here curmod is not null */

      /* If name was passed, check that the module name matches */

      if(name[0] && strncmp(name, curmod->name, 32))
        {
          continue;
        }

      /* Check if alg supported by module. */

      if(alg != ALG_NONE)
        {
          if(curmod->ops->alg_supported(ctx->session, alg) < 0)
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

int cryptoman_modselect(FAR struct cryptoman_context_s *ctx,
                        char name[32])
{
  int i;
  FAR struct cryptomodule_s *tempModule = NULL;
  FAR void *tempSession;

  /* First of all check that the user is not reselecting the same module */

  if(ctx->module && !strncmp(ctx->module->name, name, 32))
    {
      return 0;
    }

  /* Search for the required module. We iterate on ALL modules entries */
  for (i = 0; i < CONFIG_CRYPTO_MANAGER_NMODULES; i++)
    {
      if(!gCryptoModules[i])
        {
          continue;
        }

      if (gCryptoModules[i]->name &&
              strncmp(gCryptoModules[i]->name, name, 32) == 0)
        {
          tempModule = gCryptoModules[i];
        }
    }

  /* Required module not found: Exit without changing the current module */

  if (!tempModule)
    {
      return -ENODEV;
    }

  /* Try to make a session in the new module */

  tempSession = tempModule->ops->session_create();
  if (!tempSession)
    {
      return -ENOMEM; /* Session creation failed */
    }

  /* All ok. If the previous module had a session, free it first */

  if (ctx->module && ctx->session)
    {
      ctx->module->ops->session_free(ctx->session);
    }

  /* Now update the context with the new info and create a session */

  ctx->module  = tempModule;
  ctx->session = tempSession;

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
                                       &keyid, length, flags);
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
      return -ENOTSUP;
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
  if (ret <= 0)
    {
      return -ENOTSUP;
    }

  /* Allocate new block buffer if size has changed */
  if(ctx->blocksize != ret)
    {
      /* Maybe we have a previous buffer? */
      if(ctx->buffer)
        {
          kmm_free(ctx->buffer);
        }
      ctx->blocksize = ret;
      ctx->buffer = kmm_malloc(ctx->blocksize);
      if (!ctx->buffer)
        {
          return -ENOMEM;
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
          if(length == 1)
            {
              ctx->padding = value[0];
            }
          else
            {
              return -EINVAL;
            }
          break;
        }
      default:
        return ctx->module->ops->alg_ioctl(ctx->session, id, length, value);
    }

  return 0;
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
 * Name: cryptoman_processfullblocks
 * Description: Process as many full blocks of data as possible.
 * Parameters:
 * ctx          : The crypto context
 * len_in_ptr   : Pointer to how many bytes have to be managed, updated here
 *                  to indicate the remaining number of bytes that were not
 *                  managed by this call. After the call this variable shall
 *                  be less than the block size.
 * data_in      : Pointer to bytes that need to be managed
 * len_outbuf_av: Number of bytes available in the output buffer
 * buf_out      : pointer to the output buffer, where generated data is stored
 * finishing    : TRUE if we know that this will be the LAST call to this func
 * Returns      : The number of bytes WRITTEN in buf_out (obviously a
 *                  multiple of the block size), or a negative error
 ****************************************************************************/

int cryptoman_processfullblocks(FAR struct cryptoman_context_s *ctx,
                                FAR uint32_t *len_in_ptr, FAR uint8_t *data_in,
                                uint32_t len_outbuf_available,
                                FAR uint8_t *buf_out, bool finishing)
{
  int ret;
  uint32_t blocklen;
  uint32_t done = 0; /* How many bytes have been written to output (total)*/

  /* First, look at the accumulation buffer */

  if(ctx->buflen > 0)
    {
      /* Some bytes in the temp buffer require processing. First compute how
       * many bytes are required to fill the block buffer.
       */

      uint32_t processlen = ctx->blocksize - ctx->buflen;

      if(*len_in_ptr >= processlen)
        {

          /* There is enough incoming data to fill the temp buf.
           * Complete the buffer up to a block and process it,
           */

          memcpy(ctx->buffer + ctx->buflen, data_in, processlen);

          /* We just call the algorithm update routine EXCEPT IF
           * - Padding is not used (or finishing would be false)
           * - We are called from alg_finish, AND
           *   The incoming data EXACTLY completes the temp block */

          if(finishing && (*len_in_ptr == processlen))
            {
              /* We are in the last steps, there was no padding, and the
               * incoming data is just big enough to complete the pending block
               */

              ret = ctx->module->ops->alg_finish(ctx->session,
                                                 ctx->blocksize, ctx->buffer,
                                                 len_outbuf_available, buf_out);
            }
          else
            {
              /* Normal case: This is not the last crypto operation on this
               * stream because there is padding, too many data to fit in this
               * block, or called from alg_update
               */

              ret = ctx->module->ops->alg_update(ctx->session,
                                                 ctx->blocksize, ctx->buffer,
                                                 len_outbuf_available, buf_out);
            }

          if(ret < 0)
            {
              return ret; /* Something failed while calling the crypto module */
            }

          /* Now the temp buffer has been processed */

          ctx->buflen = 0;

          /* Update input variables to indicate how many have been consumed */

          data_in              += processlen;
          *len_in_ptr          -= processlen;

          /* Update output variables to indicate what has been consumed */

          done                 += ret; /* Remember how many bytes we've done */
          buf_out              += ret; /* Update output position */
          len_outbuf_available -= ret; /* Update output room */

          /* We are now ready to process the normal case */

          ctx->buflen = 0;
        }
      else
        {
          /* The incoming data is not long enough to fill the temp buffer.
           * A full block cannot be processed.
           */
          return 0; /* No bytes produced here */
        }
    }

  /* The temp buffer has been potentially processed. Now process full blocks
   * from incoming buffer.
   */

  blocklen = ((*len_in_ptr) / ctx->blocksize) * ctx->blocksize;

  if(blocklen)
    {
      /* Some full blocks are available */
      if(finishing)
        {
          /* We were called from alg_finish with no padding, this is the last
           * call on this data stream
           */
    
          ret = ctx->module->ops->alg_finish(ctx->session,
                                             blocklen, data_in,
                                             len_outbuf_available, buf_out);
        }
      else
        {
          /* Not called from finish and/or padding is applied to this data
           * stream
           */
    
          ret = ctx->module->ops->alg_update(ctx->session,
                                             blocklen, data_in,
                                             len_outbuf_available, buf_out);
        }
    
      if(ret < 0)
        {
          return ret; /* Something failed */
        }
  
      /* Update variables so the caller knows what has been managed*/
      *len_in_ptr -= blocklen;
      done += ret;
    }

  return done;
}

/****************************************************************************
 * Name: cryptoman_algupdate
 * Description: see include/nuttx/crypto/api.h
 ****************************************************************************/

int cryptoman_algupdate(FAR struct cryptoman_context_s *ctx,
                        uint32_t len_in,
                        FAR uint8_t *data_in,
                        uint32_t len_outbuf,
                        FAR uint8_t *buf_out)
{
  int ret;
  uint32_t remain = len_in;

  if (!ctx || !data_in || !buf_out)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  /* Apply common processing to the temp buffer and full sized data blocks */

  ret = cryptoman_processfullblocks(ctx, &remain, data_in,
                                    len_outbuf, buf_out, false);

  if(ret < 0)
    {
      return ret; /* Something failed */
    }

  /* Everything could not be managed by processfllblocks. Accumulate the
   * remaining bytes, that will not produce any output NOW, but will be included
   * in the next calculations.
  */

  if(remain > 0)
    {
      assert(remain <= ctx->blocksize); /* Just make sure of that */

      /* The process with full blocks processed some data bytes. Store whatever
       * bytes were not processed in the temp buffer until next call. Guaranteed
       * not to overflow since processfullblocks() will always leave less than
       * one block unprocessed, and ctx->buflen will be zero at this point.
       */
      memcpy(ctx->buffer + ctx->buflen, data_in + len_in - remain, remain);
      ctx->buflen += remain;
    }

  return ret;
}

/****************************************************************************
 * Name: cryptoman_padzero
 * Description: Pad with zero bytes. CANNOT BE REMOVED BY DECRYPTION.
 ****************************************************************************/

static int cryptoman_padzero(FAR uint8_t *buf, uint32_t len)
{
  memset(buf, 0, len);
  return 0;
}

/****************************************************************************
 * Name: cryptoman_padx923
 * Descriptin: zeros, with explicit length in last byte
 ****************************************************************************/

static int cryptoman_padx923(FAR uint8_t *buf, uint32_t len)
{
  buf[len-1] = len;
  memset(buf, 0, len - 1);
  return 0;
}

/****************************************************************************
 * Name: cryptoman_padlenx923
 * Description: Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padlenx923(FAR uint8_t *buf, uint32_t len)
{
  return buf[len-1];
}

/****************************************************************************
 * Name: cryptoman_padiso78164
 * Description: 0x80, followed by zeros.
 ****************************************************************************/

static int cryptoman_padiso78164(FAR uint8_t *buf, uint32_t len)
{
  buf[0] = 0x80;
  memset(buf + 1, 0, len - 1);
  return 0;
}

/****************************************************************************
 * Name: cryptoman_padleniso78164
 * Description: Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padleniso78164(FAR uint8_t *buf, uint32_t len)
{
#warning todo
  return 0;
}

/****************************************************************************
 * Name: cryptoman_padpkcs5
 * Description: Use a byte that represents the length, repeated as required.
 * Name copied for javacard for ease of comparison.
 ****************************************************************************/

static int cryptoman_padpkcs5(FAR uint8_t *buf, uint32_t len)
{
  memset(buf, len, len);
  return 0;
}

/****************************************************************************
 * Name: cryptoman_padlenpkcs5
 * Description: Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padlenpkcs5(FAR uint8_t *buf, uint32_t len)
{
#warning todo check all bytes
  return buf[len-1];
  return 0;
}

/****************************************************************************
 * Name: cryptoman_padiso10126
 ****************************************************************************/

static int cryptoman_padiso10126(FAR uint8_t *buf, uint32_t len)
{
#warning TODO
  return cryptoman_padx923(buf, len);
}

/****************************************************************************
 * Name: cryptoman_padleniso10126
 * Description: Helper to check padding validity
 ****************************************************************************/

static int cryptoman_padleniso10126(FAR uint8_t *buf, uint32_t len)
{
#warning TODO
  return cryptoman_padlenx923(buf, len);
}

/****************************************************************************
 * Name: cryptoman_algfinish
 * Description: see include/nuttx/crypto/api.h
 ****************************************************************************/

int cryptoman_algfinish(FAR struct cryptoman_context_s *ctx,
                        uint32_t len_in,
                        FAR uint8_t *data_in,
                        uint32_t len_outbuf,
                        FAR uint8_t *buf_out)
{
  int ret;
  int done=0; /* Total number of bytes written */
  uint32_t remain;

  if (!ctx || !data_in || !buf_out)
    {
      return -EINVAL;
    }

  if (!ctx->session || !ctx->module)
    {
      return -EPERM;
    }

  /* Apply common processing to the temp buffer and full sized data blocks.
   * This is NEVER the last step if some padding has to be applied.
   */

  remain = len_in;
  ret = cryptoman_processfullblocks(ctx, &remain, data_in,
                                    len_outbuf, buf_out,
                                    (ctx->padding == ALG_PADDING_NONE) );
  if(ret < 0)
    {
      return ret; /* Something failed */
    }

  done = ret;

  /* When ciphering, manage the padding addition */

  if(ctx->opmode == ALGOP_CIPHER)
    {
      int (*padfunc)(FAR uint8_t *buf, uint32_t len);

      switch(ctx->padding)
        {
          case ALG_PADDING_NONE    : padfunc = NULL;                  break;
          case ALG_PADDING_ZERO    : padfunc = cryptoman_padzero;     break;
          case ALG_PADDING_X923    : padfunc = cryptoman_padx923;     break;
          case ALG_PADDING_ISO78164: padfunc = cryptoman_padiso78164; break;
          case ALG_PADDING_PKCS5   : padfunc = cryptoman_padpkcs5;    break;
          case ALG_PADDING_ISO10126: padfunc = cryptoman_padiso10126; break;
          default: return -EINVAL;
        }

      /* We are finishing the ciphering session. There are multiple situations:
       * Algorithm has padding: It MUST be applied even if no data remains
       * Algorithm has NO padding: if data remains this is an error, else
       * we're done.
       */

      if(!padfunc)
        {
          if(remain)
            {
              /* No padding but data could not be process in exact
               * full blocks
               */
              return -EINVAL;
            }
          else
            {
              return done; /* Nothing more to do, processfullblock did
                            * everything.
                            */
            }
        }

      /* We have some padding to do. Copy the last user bytes to the temp
       * buffer.
       */

      assert(remain <= ctx->blocksize); /* Just make sure of that */
      assert(ctx->buflen == 0);
      memcpy(ctx->buffer, data_in + len_in - remain, remain);
      ctx->buflen = remain;

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

      ret = padfunc(ctx->buffer + ctx->buflen, ctx->blocksize - ctx->buflen);
      if(ret < 0)
        {
          return ret; /* Something failed */
        }

      ret = ctx->module->ops->alg_finish(ctx->session,
                                         ctx->blocksize, ctx->buffer,
                                         len_outbuf - done, buf_out + done);
      if(ret < 0)
        {
          return ret;
        }
    }


  /* Last crypto round on full padded block */

  if(ctx->opmode == ALGOP_DECIPHER)
    {
      int (*padfunc)(FAR uint8_t *buf, uint32_t len);

      /* If we have padding, it must be verified after the last decryption */
      switch(ctx->padding)
        {
          case ALG_PADDING_ZERO    : /* Zero padding is not removable */
          case ALG_PADDING_NONE    : padfunc = NULL;                     break;
          case ALG_PADDING_X923    : padfunc = cryptoman_padlenx923;     break;
          case ALG_PADDING_ISO78164: padfunc = cryptoman_padleniso78164; break;
          case ALG_PADDING_PKCS5   : padfunc = cryptoman_padlenpkcs5;    break;
          case ALG_PADDING_ISO10126: padfunc = cryptoman_padleniso10126; break;
          default: return -EINVAL;
        }

      /* Check padding */

      ret = padfunc(buf_out + done, ctx->blocksize); /* Returns pad length */

      if(ret < 0)
        {
          return ret; /* Incorrect padding */
        }

      done -= ret ;
    }


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
  int i;
  cryptinfo("Registering module -> %s\n", module->name);

  nxsem_wait(&gUserSemaphore);
  /* Find a slot in the list of module pointers */
  for(i=0; i<CONFIG_CRYPTO_MANAGER_NMODULES; i++)
    {
      if(gCryptoModules[i] == NULL)
        {
#warning TODO check if module with this name already exists, and then reject.
          /* Found a free slot */
          gCryptoModules[i] = module;
          nxsem_post(&gUserSemaphore);
          cryptinfo("Registered at index %d\n", i);
          return 0;
        }
    }
  crypterr("Module registration failed\n");
  nxsem_post(&gUserSemaphore);
  return -EACCES;
}

int cryptoman_libinit(void)
{
  int i;
  cryptinfo("Called\n");
  nxsem_init(&gUserSemaphore, 0, 1);
  for(i=0; i<CONFIG_CRYPTO_MANAGER_NMODULES; i++)
    {
      gCryptoModules[i] = NULL;
    }
  return 0;
}

