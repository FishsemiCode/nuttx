/****************************************************************************
 * drivers/mbox/song_mbox.c
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
#include <nuttx/clk/clk.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mbox/song_mbox.h>

#include <errno.h>
#include <string.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_mbox_cb_s
{
  mbox_receive_t callback;
  void           *arg;
};

struct song_mbox_dev_s
{
  struct mbox_dev_s               mbox;
  const struct song_mbox_config_s *config;
  struct song_mbox_cb_s           *cb;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline uint32_t song_mbox_read(const struct song_mbox_config_s *config, uint32_t offset);
static inline void song_mbox_write(const struct song_mbox_config_s *config,
                                    uint32_t offset, uint32_t val);
static inline void song_mbox_modify(const struct song_mbox_config_s *config,
                                    uint32_t offset, uint32_t bit, uint32_t val);

static int song_mbox_send(struct mbox_dev_s *dev, uint32_t ch, uintptr_t msg);
static int song_mbox_registercallback(struct mbox_dev_s *dev, uint32_t ch,
                                        mbox_receive_t callback, void *arg);
static int song_mbox_isr(int irq, void *context, void *arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct mbox_ops_s g_song_mbox_ops =
{
  .send             = song_mbox_send,
  .registercallback = song_mbox_registercallback,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static inline uint32_t song_mbox_read(const struct song_mbox_config_s *config, uint32_t offset)
{
  return (*(volatile uint32_t *)(config->base + B2C(offset)));
}

static inline void song_mbox_write(const struct song_mbox_config_s *config,
                                    uint32_t offset, uint32_t val)
{
  *(volatile uint32_t *)(config->base + B2C(offset)) = val;
}

static inline void song_mbox_modify(const struct song_mbox_config_s *config,
                                    uint32_t offset, uint32_t bit, uint32_t val)
{
  irqstate_t flags;
  uint32_t regval;

  offset += 4 * (bit/32);
  bit %= 32;

  flags   = enter_critical_section();
  regval  = song_mbox_read(config, offset);
  regval &= ~(1 << bit);
  regval |= val << bit;
  song_mbox_write(config, offset, regval);
  leave_critical_section(flags);
}

static int song_mbox_send(struct mbox_dev_s *dev, uint32_t ch, uintptr_t msg)
{
  struct song_mbox_dev_s *priv = (struct song_mbox_dev_s *)dev;
  const struct song_mbox_config_s *config = priv->config;

  if (msg)
    {
      return -ENOTSUP;
    }

  song_mbox_write(config, config->set_off, ch);

  return 0;
}

static int song_mbox_registercallback(struct mbox_dev_s *dev, uint32_t ch,
                                        mbox_receive_t callback, void *arg)
{
  struct song_mbox_dev_s *priv = (struct song_mbox_dev_s *)dev;
  const struct song_mbox_config_s *config = priv->config;

  if (config->irq < 0 || ch >= config->chnl_count)
    {
      return -EINVAL;
    }

  priv->cb[ch].callback = callback;
  priv->cb[ch].arg      = arg;

  /* Enable ch interrupt */

  song_mbox_modify(config, config->src_en_off, ch, !!callback);
  return 0;
}

static int song_mbox_isr(int irq, void *context, void *arg)
{
  struct song_mbox_dev_s *priv = arg;
  const struct song_mbox_config_s *config = priv->config;
  int32_t i, j, idx;
  uint32_t stat;

  for (i = 0; i < config->chnl_count; i += 32)
    {
      stat = song_mbox_read(config, config->sta_off + i/8);

      for (j = 0; stat; j++, stat >>= 1)
        {
          if (stat & 0x1)
            {
              song_mbox_write(config, config->sta_off + i/8, 1 << j);

              idx = i + j;
              if (idx < config->chnl_count && priv->cb[idx].callback)
                {
                  priv->cb[idx].callback(priv->cb[idx].arg, 0);
                }
            }
        }
    }

  return 0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct mbox_dev_s *song_mbox_initialize(const struct song_mbox_config_s *config)
{
  struct song_mbox_dev_s *priv;
  struct clk *hclk = NULL;
  int i, ret;

  priv = kmm_zalloc(sizeof(struct song_mbox_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->mbox.ops = &g_song_mbox_ops;
  priv->config   = config;

  if (config->clk)
    {
      hclk = clk_get(config->clk);
      if (!hclk)
        {
          goto fail;
        }
      if (clk_enable(hclk) < 0)
        {
          goto fail;
        }
    }

  if (config->irq >= 0)
    {
      priv->cb = kmm_zalloc(config->chnl_count * sizeof(struct song_mbox_cb_s));
      if (priv->cb == NULL)
        {
          goto fail1;
        }

      /* Disable all the ch interrupt */

      for (i = 0; i < config->chnl_count; i += 32)
        {
          song_mbox_write(config, config->src_en_off + i/8, 0);
        }

      ret = irq_attach(config->irq, song_mbox_isr, priv);
      if (ret < 0)
        {
          goto fail2;
        }

      up_enable_irq(config->irq);

      /* Enable total interrupt */

      song_mbox_modify(config, config->en_off, config->en_bit, 1);
    }

  return (struct mbox_dev_s *)priv;

fail2:
  kmm_free(priv->cb);
fail1:
  if (hclk)
    {
      clk_disable(hclk);
    }
fail:
  kmm_free(priv);
  return NULL;
}
