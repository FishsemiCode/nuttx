/****************************************************************************
 * drivers/rptun/song_rptun.c
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

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/rptun/song_rptun.h>

#include <errno.h>
#include <string.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_rptun_dev_s
{
  struct rptun_dev_s               rptun;
  const struct song_rptun_config_s *config;
  struct mbox_dev_s                *mbox_tx;
  struct mbox_dev_s                *mbox_rx;
  rptun_callback_t                 callback;
  void                             *arg;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static const char *song_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *song_rptun_get_firmware(struct rptun_dev_s *dev);
static void *song_rptun_get_resource(struct rptun_dev_s *dev);
static bool song_rptun_is_autostart(struct rptun_dev_s *dev);
static bool song_rptun_is_master(struct rptun_dev_s *dev);
static int song_rptun_start(struct rptun_dev_s *dev);
static int song_rptun_stop(struct rptun_dev_s *dev);
static int song_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int song_rptun_registercallback(struct rptun_dev_s *dev,
                                    rptun_callback_t callback, void *arg);
static int song_rptun_vring_isr(void *arg, uintptr_t msg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct rptun_ops_s g_song_rptun_ops =
{
  .get_cpuname       = song_rptun_get_cpuname,
  .get_firmware      = song_rptun_get_firmware,
  .get_resource      = song_rptun_get_resource,
  .is_autostart      = song_rptun_is_autostart,
  .is_master         = song_rptun_is_master,
  .start             = song_rptun_start,
  .stop              = song_rptun_stop,
  .notify            = song_rptun_notify,
  .register_callback = song_rptun_registercallback,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static const char *song_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return config->cpuname;
}

static const char *song_rptun_get_firmware(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return config->firmware;
}

static void *song_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return config->rsc;
}

static bool song_rptun_is_autostart(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return !config->nautostart;
}

static bool song_rptun_is_master(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return config->master;
}

static int song_rptun_start(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  if (config->start)
    {
      return config->start(config);
    }

  return 0;
}

static int song_rptun_stop(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  if (config->stop)
    {
      return config->stop(config);
    }

  return 0;
}

static int song_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return  MBOX_SEND(priv->mbox_tx, config->vringtx, 0);
}

static int song_rptun_registercallback(struct rptun_dev_s *dev,
                                    rptun_callback_t callback, void *arg)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  priv->callback = callback;
  priv->arg      = arg;

  return MBOX_REGISTER_CALLBACK(priv->mbox_rx, config->vringrx,
                                callback ? song_rptun_vring_isr : NULL, priv);

}

static int song_rptun_vring_isr(void *arg, uintptr_t msg)
{
  struct song_rptun_dev_s *priv = arg;

  if (priv->callback)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  return 0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct rptun_dev_s *song_rptun_initialize(
                const struct song_rptun_config_s *config,
                struct mbox_dev_s *mbox_tx,
                struct mbox_dev_s *mbox_rx)
{
  struct song_rptun_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct song_rptun_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->rptun.ops = &g_song_rptun_ops;
  priv->config    = config;
  priv->mbox_tx   = mbox_tx;
  priv->mbox_rx   = mbox_rx;

  ret = rptun_initialize((struct rptun_dev_s *)priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return (struct rptun_dev_s *)priv;
}
