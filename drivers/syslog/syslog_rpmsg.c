/****************************************************************************
 * drivers/syslog/syslog_rpmsg.c
 * Syslog driver for rpmsg syslog
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

#include <ctype.h>
#include <errno.h>
#include <string.h>

#include <openamp/open_amp.h>

#include <nuttx/irq.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/wqueue.h>

#include "syslog.h"
#include "syslog_rpmsg.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef CONFIG_SYSLOG_RPMSG_WORK_DELAY
#define CONFIG_SYSLOG_RPMSG_WORK_DELAY 0
#endif

#define SYSLOG_RPMSG_WORK_DELAY         MSEC2TICK(CONFIG_SYSLOG_RPMSG_WORK_DELAY)

#define SYSLOG_RPMSG_COUNT(h, t, size)  ((B2C_OFF(h)>=(t)) ? \
                                          B2C_OFF(h)-(t) : \
                                          (size)-((t)-B2C_OFF(h)))
#define SYSLOG_RPMSG_SPACE(h, t, size)  ((size) - 1 - SYSLOG_RPMSG_COUNT(h, t, size))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct syslog_rpmsg_s
{
  volatile size_t      head;         /* The head index (where data is added) */
  volatile size_t      tail;         /* The tail index (where data is removed) */
  size_t               size;         /* Size of the RAM buffer */
  FAR char             *buffer;      /* Circular RAM buffer */
  struct work_s        work;         /* Used for deferred callback work */

  struct rpmsg_channel *channel;
  const char           *cpu_name;
  bool                 suspend;
  bool                 transfer;     /* The transfer flag */
  ssize_t              trans_len;    /* The data length when transfer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void syslog_rpmsg_work(void *priv_);
static int  syslog_rpmsg_putc(int ch);
static int  syslog_rpmsg_flush(void);
static void syslog_rpmsg_device_created(struct remote_device *rdev, void *priv_);
static void syslog_rpmsg_channel_created(struct rpmsg_channel *channel);
static void syslog_rpmsg_channel_destroyed(struct rpmsg_channel *channel);
static void syslog_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct syslog_rpmsg_s g_syslog_rpmsg;

static const struct syslog_channel_s g_syslog_rpmsg_channel =
{
#ifdef CONFIG_SYSLOG_WRITE
  syslog_default_write,
#endif
  syslog_rpmsg_putc,
  syslog_rpmsg_putc,
  syslog_rpmsg_flush
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void syslog_rpmsg_work(void *priv_)
{
  struct syslog_rpmsg_s *priv = priv_;
  struct syslog_rpmsg_transfer_s *msg;
  irqstate_t flags;
  uint32_t space;
  size_t len, len_end;

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, false);
  if (!msg)
    {
      return;
    }

  memset(msg, 0, sizeof(*msg));

  flags = enter_critical_section();

  space  -= sizeof(*msg);
  len     = SYSLOG_RPMSG_COUNT(priv->head, priv->tail, priv->size);
  len_end = priv->size - priv->tail;

  if (len > space)
    {
      len = space;
    }
  else
    {
      if (B2C_REM(priv->head))
        {
          priv->head += C2B(1) - B2C_REM(priv->head);
        }
    }

  if (len > len_end)
    {
      memcpy(msg->data, &priv->buffer[priv->tail], len_end);
      memcpy(msg->data + len_end, priv->buffer, len - len_end);
    }
  else
    {
      memcpy(msg->data, &priv->buffer[priv->tail], len);
    }

  priv->trans_len = len;
  priv->transfer  = true;

  leave_critical_section(flags);

  msg->header.command = SYSLOG_RPMSG_TRANSFER;
  msg->count          = C2B(len);
  rpmsg_send_nocopy(priv->channel, msg, sizeof(*msg) + len);
}

static int syslog_rpmsg_putc(int ch)
{
  struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  irqstate_t flags;

  flags = enter_critical_section();

  if (B2C_REM(priv->head) == 0)
    {
      priv->buffer[B2C_OFF(priv->head)] = 0;
    }

  priv->buffer[B2C_OFF(priv->head)] |= (ch & 0xff) << (8 * B2C_REM(priv->head));

  priv->head += 1;
  if (priv->head >= C2B(priv->size))
    {
      priv->head = 0;
    }

  /* Allow overwrite */

  if (priv->head == C2B(priv->tail))
    {
      priv->buffer[priv->tail] = 0;

      priv->tail += 1;
      if (priv->tail >= priv->size)
        {
          priv->tail = 0;
        }

      if (priv->transfer)
        {
          priv->trans_len--;
        }
    }

  if (priv->channel && !priv->suspend && !priv->transfer)
    {
      systime_t delay = SYSLOG_RPMSG_WORK_DELAY;
      size_t space = SYSLOG_RPMSG_SPACE(priv->head, priv->tail, priv->size);

      /* Start work immediately when data more then 75% and meet '\n' */

      if (space < priv->size / 4 && ch == '\n')
        {
          delay = 0;
        }

      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, delay);
    }

  leave_critical_section(flags);

  return ch;
}

static int syslog_rpmsg_flush(void)
{
  return OK;
}

static void syslog_rpmsg_device_created(struct remote_device *rdev, void *priv_)
{
  struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;

  if (priv->buffer && strcmp(priv->cpu_name, rdev->proc->cpu_name) == 0)
    {
      rpmsg_create_channel(rdev, SYSLOG_RPMSG_CHANNEL_NAME);
    }
}

static void syslog_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  struct remote_device *rdev = channel->rdev;

  if (priv->buffer && strcmp(priv->cpu_name, rdev->proc->cpu_name) == 0)
    {
      priv->channel = channel;
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, SYSLOG_RPMSG_WORK_DELAY);
    }
}

static void syslog_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
  struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  struct remote_device *rdev = channel->rdev;

  if (priv->buffer && strcmp(priv->cpu_name, rdev->proc->cpu_name) == 0)
    {
      work_cancel(HPWORK, &priv->work);
      priv->channel = NULL;
    }
}

static void syslog_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  struct syslog_rpmsg_header_s *header = data;

  if (header->command == SYSLOG_RPMSG_SUSPEND)
    {
      work_cancel(HPWORK, &priv->work);
      priv->suspend = true;
    }
  else if (header->command == SYSLOG_RPMSG_RESUME)
    {
      priv->suspend = false;
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, SYSLOG_RPMSG_WORK_DELAY);
    }
  else if (header->command == SYSLOG_RPMSG_TRANSFER)
    {
      struct syslog_rpmsg_transfer_s *msg = data;
      struct syslog_rpmsg_header_s done;

      syslog_write(msg->data, msg->count);

      memset(&done, 0, sizeof(done));
      done.command = SYSLOG_RPMSG_TRANSFER_DONE;
      done.result  = msg->count;
      rpmsg_send(channel, &done, sizeof(done));
    }
  else if (header->command == SYSLOG_RPMSG_TRANSFER_DONE)
    {
      irqstate_t flags;
      ssize_t len_end;

      flags = enter_critical_section();

      if (priv->trans_len > 0)
        {
          len_end = priv->size - priv->tail;

          if (priv->trans_len > len_end)
            {
              memset(&priv->buffer[priv->tail], 0, len_end);
              memset(priv->buffer, 0, priv->trans_len - len_end);
            }
          else
            {
              memset(&priv->buffer[priv->tail], 0, priv->trans_len);
            }

          priv->tail += priv->trans_len;
          if (priv->tail >= priv->size)
            {
              priv->tail -= priv->size;
            }
        }

      priv->transfer = false;

      if (SYSLOG_RPMSG_COUNT(priv->head, priv->tail, priv->size))
        {
          work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

int up_putc(int ch)
{
  return syslog_rpmsg_putc(ch);
}

int syslog_rpmsg_init_early(const char *cpu_name, void *buffer, size_t size)
{
  struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  char prev, cur;
  size_t i, j;

  priv->cpu_name = cpu_name;
  priv->buffer   = buffer;
  priv->size     = size;

  prev = (priv->buffer[size - 1] >> (CHAR_BIT - 8)) & 0xff;

  for (i = 0; i < size; i++)
    {
      for (j = 0; j * 8 < CHAR_BIT; j++)
        {
          cur = (priv->buffer[i] >> j * 8) & 0xff;

          if (!isascii(cur))
            {
              goto out;
            }

          if (prev && !cur)
            {
              priv->head = C2B(i) + j;
            }

          if (!prev && cur)
            {
              priv->tail = i;
            }

          prev = cur;
        }
    }

out:
  if (i != size)
    {
      priv->head = priv->tail = 0;
      memset(priv->buffer, 0, size);
    }

  return syslog_channel(&g_syslog_rpmsg_channel);
}

int syslog_rpmsg_init(void)
{
  return rpmsg_register_callback(
                SYSLOG_RPMSG_CHANNEL_NAME,
                NULL,
                syslog_rpmsg_device_created,
                NULL,
                syslog_rpmsg_channel_created,
                syslog_rpmsg_channel_destroyed,
                syslog_rpmsg_channel_received);
}
