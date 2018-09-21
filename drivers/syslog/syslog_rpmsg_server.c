/****************************************************************************
 * drivers/syslog/syslog_rpmsg_server.c
 * Syslog driver for rpmsg syslog server
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

#include <nuttx/kmalloc.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include <openamp/open_amp.h>

#include "syslog.h"
#include "syslog_rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYSLOG_RPMSG_MAXLEN             256

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct syslog_rpmsg_server_s
{
  char         *tmpbuf;
  unsigned int nextpos;
  unsigned int alloced;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void syslog_rpmsg_write(const char *buf1, size_t len1,
                               const char *buf2, size_t len2);
static void syslog_rpmsg_channel_created(struct rpmsg_channel *channel);
static void syslog_rpmsg_channel_destroyed(struct rpmsg_channel *channel);
static void syslog_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void syslog_rpmsg_write(const char *buf1, size_t len1,
                               const char *buf2, size_t len2)
{
  const char *nl;
  size_t len;

  nl = memchr(buf2, '\n', len2);
  DEBUGASSERT(nl != NULL);
  len = nl + 1 - buf2;

  if (len1 + len <= SYSLOG_RPMSG_MAXLEN)
    {
      char tmpbuf[SYSLOG_RPMSG_MAXLEN];

      /* Ensure each syslog_write's buffer end with '\n' */

      memcpy(tmpbuf, buf1, len1);
      memcpy(tmpbuf + len1, buf2, len);
      syslog_write(tmpbuf, len1 + len);

      if (len < len2)
        {
          syslog_write(nl + 1, len2 - len);
        }
    }
  else
    {
      /* Give up, the merge buffer is too big */

      syslog_write(buf1, len1);
      syslog_write(buf2, len2);
    }
}

static void syslog_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct syslog_rpmsg_server_s *priv;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv)
    {
      rpmsg_set_privdata(channel, priv);
    }
}

static void syslog_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
  struct syslog_rpmsg_server_s *priv = rpmsg_get_privdata(channel);

  if (priv)
    {
      if (priv->nextpos)
        {
          syslog_rpmsg_write(priv->tmpbuf, priv->nextpos, "\n", 1);
        }
      kmm_free(priv->tmpbuf);
      kmm_free(priv);
    }
}

static void syslog_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct syslog_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct syslog_rpmsg_header_s *header = data;

  if (header->command == SYSLOG_RPMSG_TRANSFER)
    {
      struct syslog_rpmsg_transfer_s *msg = data;
      struct syslog_rpmsg_header_s done;
      unsigned int copied = msg->count;
      unsigned int printed = 0;
      const char *nl;

      nl = memrchr(msg->data, '\n', msg->count);
      if (nl != NULL)
        {
          printed = nl + 1 - msg->data;
          copied = msg->count - printed;

          if (priv->nextpos)
            {
              syslog_rpmsg_write(priv->tmpbuf, priv->nextpos, msg->data, printed);
              priv->nextpos = 0;
            }
          else
            {
              syslog_write(msg->data, printed);
            }
        }

      if (copied != 0)
        {
          unsigned int newsize = priv->nextpos + copied;
          if (newsize > priv->alloced)
            {
              char *newbuf = kmm_realloc(priv->tmpbuf, newsize);
              if (newbuf != NULL)
                {
                  priv->tmpbuf  = newbuf;
                  priv->alloced = newsize;
                }
              else
                {
                  copied = priv->alloced - priv->nextpos;
                }
            }
          memcpy(priv->tmpbuf + priv->nextpos, msg->data + printed, copied);
          priv->nextpos += copied;
        }

      done.command = SYSLOG_RPMSG_TRANSFER_DONE;
      done.result  = printed + copied;
      rpmsg_send(channel, &done, sizeof(done));
    }
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

int syslog_rpmsg_server_init(void)
{
  return rpmsg_register_callback(
                SYSLOG_RPMSG_CHANNEL_NAME,
                NULL,
                NULL,
                NULL,
                syslog_rpmsg_channel_created,
                syslog_rpmsg_channel_destroyed,
                syslog_rpmsg_channel_received);
}
