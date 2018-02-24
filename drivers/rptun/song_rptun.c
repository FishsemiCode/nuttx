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
  struct mbox_dev_s                *mbox_rx;
  struct mbox_dev_s                *mbox_tx;
  uint32_t                         count_start;
  rptun_callback_t                 callback;
  void                             *arg;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static const char *song_rptun_get_cpuname(struct rptun_dev_s *dev);
static int song_rptun_get_resource(struct rptun_dev_s *dev,
                                    struct rsc_table_info *rsc, uint32_t *role);
static int song_rptun_boot(struct rptun_dev_s *dev);
static int song_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int song_rptun_registercallback(struct rptun_dev_s *dev,
                                    rptun_callback_t callback, void *arg);
static int song_rptun_ioctl(struct rptun_dev_s *dev, int cmd, unsigned long arg);
static int song_rptun_start_isr(void *arg, uintptr_t msg);
static int song_rptun_vring_isr(void *arg, uintptr_t msg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct rptun_ops_s g_song_rptun_ops =
{
  .get_cpuname       = song_rptun_get_cpuname,
  .get_resource      = song_rptun_get_resource,
  .boot              = song_rptun_boot,
  .notify            = song_rptun_notify,
  .register_callback = song_rptun_registercallback,
  .ioctl             = song_rptun_ioctl,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static const char *song_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  return config->cpu_name;
}

static int song_rptun_get_resource(struct rptun_dev_s *dev,
                                    struct rsc_table_info *rsc, uint32_t *role)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  memcpy(rsc, &config->rsc, sizeof(config->rsc));
  *role = config->role;

  return 0;
}

static int song_rptun_boot(struct rptun_dev_s *dev)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;

  if (config->boot)
    {
      return config->boot(config);
    }

  return 0;
}

static int song_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;
  int ret;

  if (vqid == RPTUN_NOTIFY_START && config->ch_start_tx >= 0)
    {
      ret = MBOX_SEND(priv->mbox_tx, config->ch_start_tx, 0);
    }
  else
    {
      ret = MBOX_SEND(priv->mbox_tx, config->ch_vring_tx, 0);
    }

  return ret;
}

static int song_rptun_registercallback(struct rptun_dev_s *dev,
                                    rptun_callback_t callback, void *arg)
{
  struct song_rptun_dev_s *priv = (struct song_rptun_dev_s *)dev;
  const struct song_rptun_config_s *config = priv->config;
  int ret = 0;

  priv->callback = callback;
  priv->arg      = arg;

  if (config->ch_start_rx >= 0)
    {
      ret |= MBOX_REGISTER_CALLBACK(priv->mbox_rx, config->ch_start_rx,
                        callback ? song_rptun_start_isr : NULL, priv);
    }

  ret |= MBOX_REGISTER_CALLBACK(priv->mbox_rx, config->ch_vring_rx,
                    callback ? song_rptun_vring_isr : NULL, priv);

  return ret;
}

static int song_rptun_ioctl(struct rptun_dev_s *dev, int cmd, unsigned long arg)
{
  if (cmd == RPTUN_USER_IOCBASE)
    {
      *((void **)(uintptr_t)arg) = dev;
      return 0;
    }

  return -ENOTTY;
}

static int song_rptun_start_isr(void *arg, uintptr_t msg)
{
  struct song_rptun_dev_s *priv = arg;

  if (priv->count_start != 0)
    {
      if (priv->callback)
        {
          priv->callback(priv->arg, RPTUN_NOTIFY_START);
        }
    }

  priv->count_start++;

  return 0;
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
                struct mbox_dev_s *mbox_rx,
                struct mbox_dev_s *mbox_tx,
                int minor)
{
  struct song_rptun_dev_s *priv;
  int ret;

  if (config->rsc_flash)
    {
      memcpy(config->rsc.rsc_tab, (void *)config->rsc_flash, config->rsc.size);
    }

  priv = kmm_zalloc(sizeof(struct song_rptun_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->rptun.ops = &g_song_rptun_ops;
  priv->config    = config;
  priv->mbox_rx   = mbox_rx;
  priv->mbox_tx   = mbox_tx;

  ret = rptun_register((struct rptun_dev_s *)priv, minor);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  return (struct rptun_dev_s *)priv;
}
