/****************************************************************************
 * drivers/clk/clk-rpmsg.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin<zhuyanlin@pinecone.net>
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

#include <string.h>

#include <openamp/open_amp.h>

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/semaphore.h>

#include "clk-rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLK_RPMSG_MSGLEN(msg, clk)     (sizeof(*msg) + B2C(strlen(clk->name) + 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct clk_rpmsg_cookie_s
{
  sem_t                         sem;
  int64_t                       result;
};

struct clk_rpmsg_priv_s
{
  struct rpmsg_channel          *channel;
  const char                    *cpu_name;
};

/***********************************************************************************
 * Private Datas
 ***********************************************************************************/
struct clk_rpmsg_priv_s         g_clk_rpmsg;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int64_t clk_rpmsg_sendrecv(struct rpmsg_channel *chnl, uint32_t command,
            struct clk_rpmsg_header_s *msg, int32_t len)
{
  struct clk_rpmsg_cookie_s cookie = {0};
  int ret;

  msg->command = command;
  msg->cookie  = (uintptr_t)&cookie;

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_setprotocol(&cookie.sem, SEM_PRIO_NONE);
  cookie.result  = -EIO;

  ret = rpmsg_send_nocopy(chnl, msg, len);
  if (ret < 0)
    return ret;

  ret = nxsem_wait_uninterruptible(&cookie.sem);
  if (ret < 0)
    return ret;

  return cookie.result;
}

static void clk_rpmsg_device_created(struct remote_device *rdev, void *priv_)
{
  struct clk_rpmsg_priv_s *priv = priv_;
  struct rpmsg_channel *channel;

  if (strcmp(priv->cpu_name, rdev->proc->cpu_name) == 0)
    {
      channel = rpmsg_create_channel(rdev, CLK_RPMSG_NAME);
      if (channel != NULL)
        {
          rpmsg_set_privdata(channel, priv);
        }
    }
}

static void clk_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct clk_rpmsg_priv_s *priv = rpmsg_get_privdata(channel);

  if (priv != NULL)
    {
      priv->channel = channel;
    }
}

static void clk_rpmsg_channel_destoryed(struct rpmsg_channel *channel)
{
  struct clk_rpmsg_priv_s *priv = rpmsg_get_privdata(channel);

  if (priv != NULL)
    {
      priv->channel = NULL;
    }
}

static void clk_rpmsg_received(struct rpmsg_channel *channel, void *data,
                                  int len, void *priv, unsigned long src)
{
  struct clk_rpmsg_header_s *hdr = data;
  struct clk_rpmsg_cookie_s *cookie =
    (struct clk_rpmsg_cookie_s *)(uintptr_t)hdr->cookie;

  if (cookie)
    {
      cookie->result = hdr->result;
      if (hdr->command == CLK_RPMSG_GETRATE && !hdr->result)
        {
          struct clk_rpmsg_getrate_s *msg = data;
          cookie->result = msg->rate;
        }
      nxsem_post(&cookie->sem);
    }
}

static int clk_rpmsg_enable(struct clk *clk)
{
  struct rpmsg_channel *chnl = g_clk_rpmsg.channel;
  struct clk_rpmsg_enable_s *msg;
  uint32_t size, len = CLK_RPMSG_MSGLEN(msg, clk);

  if (!chnl)
    return -ENODEV;

  size = rpmsg_get_buffer_size(chnl);
  if (len > size)
    return -ENOMEM;

  msg = rpmsg_get_tx_payload_buffer(chnl, &size, true);
  if (!msg)
      return -ENOMEM;

  cstr2bstr(msg->name, clk->name);

  return clk_rpmsg_sendrecv(chnl, CLK_RPMSG_ENABLE,
          (struct clk_rpmsg_header_s *)msg, len);
}

static void clk_rpmsg_disable(struct clk *clk)
{
  struct rpmsg_channel *chnl = g_clk_rpmsg.channel;
  struct clk_rpmsg_disable_s *msg;
  uint32_t size, len = CLK_RPMSG_MSGLEN(msg, clk);

  if (!chnl)
    return;

  size = rpmsg_get_buffer_size(chnl);
  if (len > size)
    return;

  msg = rpmsg_get_tx_payload_buffer(chnl, &size, true);
  if (!msg)
      return;

  cstr2bstr(msg->name, clk->name);

  clk_rpmsg_sendrecv(chnl, CLK_RPMSG_DISABLE,
      (struct clk_rpmsg_header_s *)msg, len);
}

static int64_t clk_rpmsg_round_rate(struct clk *clk, uint64_t rate, uint64_t *parent_rate)
{
  /* directly return the input rate to clk-core */
  return rate;
}

static int clk_rpmsg_set_rate(struct clk *clk, uint64_t rate, uint64_t parent_rate)
{
  struct rpmsg_channel *chnl = g_clk_rpmsg.channel;
  struct clk_rpmsg_setrate_s *msg;
  uint32_t size, len = CLK_RPMSG_MSGLEN(msg, clk);

  if (!chnl)
    return -ENODEV;

  size = rpmsg_get_buffer_size(chnl);
  if (len > size)
    return -ENOMEM;

  msg = rpmsg_get_tx_payload_buffer(chnl, &size, true);
  if (!msg)
      return -ENOMEM;

  msg->rate = rate;
  cstr2bstr(msg->name, clk->name);

  return clk_rpmsg_sendrecv(chnl, CLK_RPMSG_SETRATE,
            (struct clk_rpmsg_header_s *)msg, len);
}

static uint64_t clk_rpmsg_recalc_rate(struct clk *clk, uint64_t parent_rate)
{
  struct rpmsg_channel *chnl = g_clk_rpmsg.channel;
  struct clk_rpmsg_getrate_s *msg;
  uint32_t size, len = CLK_RPMSG_MSGLEN(msg, clk);

  if (!chnl)
    return -ENODEV;

  size = rpmsg_get_buffer_size(chnl);
  if (len > size)
    return -ENOMEM;

  msg = rpmsg_get_tx_payload_buffer(chnl, &size, true);
  if (!msg)
      return -ENOMEM;

  cstr2bstr(msg->name, clk->name);

  return clk_rpmsg_sendrecv(chnl, CLK_RPMSG_GETRATE,
            (struct clk_rpmsg_header_s *)msg, len);
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_rpmsg_ops =
{
  .enable = clk_rpmsg_enable,
  .disable = clk_rpmsg_disable,
  .recalc_rate = clk_rpmsg_recalc_rate,
  .round_rate = clk_rpmsg_round_rate,
  .set_rate = clk_rpmsg_set_rate,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_rpmsg(const char *name, uint64_t flags)
{
  /* rpmsg clk is consider as orphan clk (no parents) in remoteproc client */
  return clk_register(name, 0, NULL, flags, &clk_rpmsg_ops, NULL);
}

int clk_rpmsg_initialize(const char *cpu_name)
{
  g_clk_rpmsg.cpu_name = cpu_name;
  rpmsg_register_callback(CLK_RPMSG_NAME,
                 &g_clk_rpmsg,
                 clk_rpmsg_device_created,
                 NULL,
                 clk_rpmsg_channel_created,
                 clk_rpmsg_channel_destoryed,
                 clk_rpmsg_received);
  return 0;
}
