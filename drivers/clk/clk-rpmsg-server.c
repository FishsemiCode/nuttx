/****************************************************************************
 * drivers/clk/clk-rpmsg-server.c
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

#include <openamp/open_amp.h>

#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>

#include "clk-rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct clk_rpmsg_s
{
  struct clk*               clk;
  char*                     name;
  uint32_t                  count;
  struct list_node          node;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct clk_rpmsg_s *clk_rpmsg_get(struct rpmsg_channel *channel, const char *name);
static void clk_rpmsg_enable_handler(struct rpmsg_channel *channel,
            void *data, int len, void *priv, unsigned long src);
static void clk_rpmsg_disable_handler(struct rpmsg_channel *channel,
             void *data, int len, void *priv, unsigned long src);

static void clk_rpmsg_getrate_handler(struct rpmsg_channel *channel,
             void *data, int len, void *priv, unsigned long src);
static void clk_rpmsg_setrate_handler(struct rpmsg_channel *channel,
             void *data, int len, void *priv, unsigned long src);
static void clk_rpmsg_received(struct rpmsg_channel *channel,
            void *data, int len, void *priv, unsigned long src);
static void clk_rpmsg_channel_created(struct rpmsg_channel *channel);
static void clk_rpmsg_channel_destroyed(struct rpmsg_channel *channel);

/***********************************************************************************
 * Private Datas
 ***********************************************************************************/

static const rpmsg_rx_cb_t clk_rpmsg_handler[] =
{
  [CLK_RPMSG_ENABLE]  = clk_rpmsg_enable_handler,
  [CLK_RPMSG_DISABLE] = clk_rpmsg_disable_handler,
  [CLK_RPMSG_SETRATE] = clk_rpmsg_setrate_handler,
  [CLK_RPMSG_GETRATE] = clk_rpmsg_getrate_handler,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static struct clk_rpmsg_s *clk_rpmsg_get(struct rpmsg_channel *channel, const char *name)
{
  struct list_node   *clk_rpmsg_list = rpmsg_get_privdata(channel);
  struct clk_rpmsg_s *clkrp;

  list_for_every_entry(clk_rpmsg_list, clkrp, struct clk_rpmsg_s, node)
    {
      if (!strcmp(clkrp->name, name))
        {
          return clkrp;
        }
    }

  clkrp = kmm_zalloc(sizeof(struct clk_rpmsg_s));
  if (!clkrp)
    {
      goto fail_clkrp;
    }

  clkrp->name = kmm_zalloc(strlen(name) + 1);
  if (!clkrp->name)
    {
      goto fail_clkrp_name;
    }

  clkrp->clk = clk_get(name);
  if (clkrp->clk)
    {
      strcpy(clkrp->name, name);
      list_add_head(clk_rpmsg_list, &clkrp->node);
      return clkrp;
    }

  kmm_free(clkrp->name);
fail_clkrp_name:
  kmm_free(clkrp);
fail_clkrp:
  return NULL;
}

static void clk_rpmsg_enable_handler(struct rpmsg_channel *channel,
            void *data, int len, void *priv, unsigned long src)
{
  struct clk_rpmsg_enable_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get(channel, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_enable(clkrp->clk);
      if (!msg->header.result)
        {
          clkrp->count++;
        }
    }
  else
    msg->header.result = -ENOENT;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void clk_rpmsg_disable_handler(struct rpmsg_channel *channel,
             void *data, int len, void *priv, unsigned long src)
{
  struct clk_rpmsg_disable_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get(channel, msg->name);

  if (clkrp)
    {
      clk_disable(clkrp->clk);
      clkrp->count--;
      msg->header.result = 0;
    }
  else
    msg->header.result = -ENOENT;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void clk_rpmsg_getrate_handler(struct rpmsg_channel *channel,
             void *data, int len, void *priv, unsigned long src)
{
  struct clk_rpmsg_getrate_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get(channel, msg->name);

  if (clkrp)
    {
      msg->rate = clk_get_rate(clkrp->clk);
      msg->header.result = 0;
    }
  else
    msg->header.result = -ENOENT;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void clk_rpmsg_setrate_handler(struct rpmsg_channel *channel,
             void *data, int len, void *priv, unsigned long src)
{
  struct clk_rpmsg_setrate_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get(channel, msg->name);

  if (clkrp)
    msg->header.result = clk_set_rate(clkrp->clk, msg->rate);
  else
    msg->header.result = -ENOENT;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void clk_rpmsg_received(struct rpmsg_channel *channel,
            void *data, int len, void *priv, unsigned long src)
{
  struct clk_rpmsg_header_s *hdr = data;
  uint32_t cmd = hdr->command;

  if ((cmd < ARRAY_SIZE(clk_rpmsg_handler)) && clk_rpmsg_handler[cmd])
    clk_rpmsg_handler[cmd](channel, data, len, priv, src);

}

static void clk_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct list_node *clk_rpmsg_list = kmm_zalloc(sizeof(struct list_node));

  if (clk_rpmsg_list)
    {
      list_initialize(clk_rpmsg_list);
      rpmsg_set_privdata(channel, clk_rpmsg_list);
    }
}

static void clk_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
  struct list_node *clk_rpmsg_list = rpmsg_get_privdata(channel);
  struct clk_rpmsg_s *clkrp, *clkrp_tmp;
  uint32_t count;

  list_for_every_entry_safe(clk_rpmsg_list, clkrp, clkrp_tmp,
                              struct clk_rpmsg_s, node)
    {
      count = clkrp->count;

      while (count--)
        {
          clk_disable(clkrp->clk);
        }

      list_delete(&clkrp->node);

      kmm_free(clkrp->name);
      kmm_free(clkrp);
    }

  kmm_free(clk_rpmsg_list);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int clk_rpmsg_server_initialize(void)
{
  return rpmsg_register_callback(CLK_RPMSG_NAME,
                 NULL,
                 NULL,
                 NULL,
                 clk_rpmsg_channel_created,
                 clk_rpmsg_channel_destroyed,
                 clk_rpmsg_received);
}
