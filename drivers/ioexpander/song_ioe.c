/****************************************************************************
 * drivers/ioexpander/song_ioe.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "song_ioe.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_ioe_cb_s
{
  ioe_pinset_t   pinset;
  ioe_callback_t func;
  FAR void*      arg;
};

struct song_ioe_dev_s
{
  FAR struct ioexpander_dev_s ioe;
  FAR struct song_ioe_cb_s    cb[CONFIG_SONG_IOE_INTCALLBACKS];
  mutex_t                     lock;
  uint32_t                    cpu;
  uint32_t                    base;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t readreg(struct song_ioe_dev_s *priv, uint32_t offset);
static inline void writereg(struct song_ioe_dev_s *priv, uint32_t offset, uint32_t val);

static int song_ioe_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      int dir);
static int song_ioe_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      int opt, void *regval);
static int song_ioe_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      bool value);
static int song_ioe_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      FAR bool *value);
static int song_ioe_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      FAR bool *value);

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int song_ioe_multiwritepin(FAR struct ioexpander_dev_s *dev,
      FAR uint8_t *pins, FAR bool *values, int count);
static int song_ioe_multireadpin(FAR struct ioexpander_dev_s *dev,
      FAR uint8_t *pins, FAR bool *values, int count);
static int song_ioe_multireadbuf(FAR struct ioexpander_dev_s *dev,
      FAR uint8_t *pins, FAR bool *values, int count);
#endif

static FAR void *song_ioe_attach(FAR struct ioexpander_dev_s *dev,
      ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int song_ioe_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle);

static int song_ioe_handler(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Datas
 ****************************************************************************/

/* Song ioexpander interface operations */
static const struct ioexpander_ops_s g_song_ioe_ops =
{
  song_ioe_direction,
  song_ioe_option,
  song_ioe_writepin,
  song_ioe_readpin,
  song_ioe_readbuf,
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  song_ioe_multiwritepin,
  song_ioe_multireadpin,
  song_ioe_multireadbuf,
#endif
  song_ioe_attach,
  song_ioe_detach,
};

static const uint32_t g_song_ioe_inttype[] =
{
  [IOEXPANDER_VAL_DISABLE] = SONG_IOE_INT_DISABLE,
  [IOEXPANDER_VAL_HIGH]    = SONG_IOE_INT_HIGHLEVEL,
  [IOEXPANDER_VAL_LOW]     = SONG_IOE_INT_LOWLEVEL,
  [IOEXPANDER_VAL_RISING]  = SONG_IOE_INT_GPIOCLK_RISING,
  [IOEXPANDER_VAL_FALLING] = SONG_IOE_INT_GPIOCLK_FALLING,
  [IOEXPANDER_VAL_BOTH]    = SONG_IOE_INT_GPIOCLK_BOTHEDGES,
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

static inline uint32_t readreg(struct song_ioe_dev_s *priv,
      uint32_t offset)
{
  return (*(volatile uint32_t *)(priv->base + B2C(offset)));
}

static inline void writereg(struct song_ioe_dev_s *priv,
      uint32_t offset, uint32_t val)
{
  *(volatile uint32_t *)(priv->base + B2C(offset)) = val;
}

#if CONFIG_IOEXPANDER_NPINS <= 64
static int song_ioe_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct song_ioe_dev_s *priv = arg;
  ioe_pinset_t status = 0;
  uint32_t i;

#if CONFIG_IOEXPANDER_NPINS > 32
  status = readreg(priv, SONG_IOE_INTR_STATUS(priv->cpu, 32));
  status <<= 32;
#endif
  status |= readreg(priv, SONG_IOE_INTR_STATUS(priv->cpu, 0));

  for (i = 0; i < CONFIG_SONG_IOE_INTCALLBACKS; i++)
    {
      if (priv->cb[i].func)
        {
          ioe_pinset_t match = priv->cb[i].pinset & status;

          if (match)
            {
              priv->cb[i].func(&priv->ioe, match, priv->cb[i].arg);
            }
        }
    }

  writereg(priv, SONG_IOE_INTR_CLR(0), status);
#if CONFIG_IOEXPANDER_NPINS > 32
  writereg(priv, SONG_IOE_INTR_CLR(32), status >> 32);
#endif

  return 0;
}
#else
static int song_ioe_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct song_ioe_dev_s *priv = arg;
  uint32_t status, i;

  for (i = 0; i < CONFIG_SONG_IOE_INTCALLBACKS; i++)
    {
      if (priv->cb[i].func)
        {
          ioe_pinset_t pinset = priv->cb[i].pinset;

          status = readreg(priv, SONG_IOE_INTR_STATUS(priv->cpu, pinset));
          if (status & (1 << (pinset % 32)))
            {
              priv->cb[i].func(&priv->ioe, pinset, priv->cb[i].arg);

              writereg(priv, SONG_IOE_INTR_CLR(pinset),
                  1 << (pinset % 32));
            }
        }
    }

  for (i = 0; i < CONFIG_IOEXPANDER_NPINS; i += 32)
    {
      status = readreg(priv, SONG_IOE_INTR_STATUS(priv->cpu, i));
      if (status)
        {
          writereg(priv, SONG_IOE_INTR_CLR(i), status);
        }
    }

  return 0;
}
#endif

static int song_ioe_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      int direction)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;
  uint32_t bit, val;

  bit = pin % 16;
  val = 1 << (bit + 16);

  if (direction == IOEXPANDER_DIRECTION_OUT)
    val |= 1 << bit;

  writereg(priv, SONG_IOE_PORT_DDR(pin), val);
  return 0;
}

static int song_ioe_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      int opt, FAR void *val)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;
  uint32_t para = (uint32_t)(uintptr_t)val;

  if (opt == IOEXPANDER_OPTION_INVERT && para == IOEXPANDER_VAL_INVERT)
    {
      return -EINVAL;
    }

  else if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      uint32_t inttype = g_song_ioe_inttype[para];
      uint32_t reg, bit;

      bit = pin % 4;
      reg = (inttype << (bit * 4)) | (1 << (bit + 16));
      writereg(priv, SONG_IOE_INTR_CTRL(pin), reg);

      /* unmask it */
      bit = pin % 16;
      reg = 1 << (bit + 16);
      writereg(priv, SONG_IOE_INTR_MASK(priv->cpu, pin), reg);
    }

    return 0;
}

static int song_ioe_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      FAR bool *value)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;

  uint32_t val = readreg(priv, SONG_IOE_EXT_PORT(pin));

  *value = ((val & (1 << (pin % 32))) != 0);
  return 0;
}

static int song_ioe_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      FAR bool *value)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;

  uint32_t val = readreg(priv, SONG_IOE_PORT_DR(pin));

  *value = ((val & (1 << (pin % 16))) != 0);
  return 0;
}

static int song_ioe_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
      bool value)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;
  uint32_t bit, val;

  bit = pin % 16;
  val = (1 << (bit + 16));
  if (value)
    val |= 1 << bit;

  writereg(priv, SONG_IOE_PORT_DR(pin), val);
  return 0;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int song_ioe_multiwritepin(FAR struct ioexpander_dev_s *dev,
      FAR uint8_t *pins, FAR bool *values, int count)
{
  uint32_t i;

  for (i = 0; i < count; i++)
    {
      song_ioe_writepin(dev, pins[i], values[i]);
    }
  return 0;
}

static int song_ioe_multireadpin(FAR struct ioexpander_dev_s *dev,
      FAR uint8_t *pins, FAR bool *values, int count)
{
  uint32_t i;

  for (i = 0; i < count; i++)
    {
      song_ioe_readpin(dev, pins[i], &values[i]);
    }
  return 0;
}

static int song_ioe_multireadbuf(FAR struct ioexpander_dev_s *dev,
      FAR uint8_t *pins, FAR bool *values, int count)
{
  uint32_t i;

  for (i = 0; i < count; i++)
    {
      song_ioe_readbuf(dev, pins[i], &values[i]);
    }
  return 0;
}
#endif

static FAR void *song_ioe_attach(FAR struct ioexpander_dev_s *dev,
      ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;
  uint32_t i;
  FAR void *cb = NULL;

  nxmutex_lock(&priv->lock);

  for (i = 0; i < CONFIG_SONG_IOE_INTCALLBACKS; i++)
    {
      if (!priv->cb[i].func)
        {
          priv->cb[i].pinset  = pinset;
          priv->cb[i].func    = callback;
          priv->cb[i].arg     = arg;
          cb                  = &priv->cb[i];
          break;
        }
    }

  nxmutex_unlock(&priv->lock);
  return cb;
}

static int song_ioe_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct song_ioe_dev_s *priv = (FAR struct song_ioe_dev_s *)dev;
  FAR struct song_ioe_cb_s *cb = handle;

  nxmutex_lock(&priv->lock);

  cb->pinset = 0;
  cb->func   = NULL;
  cb->arg    = NULL;

  nxmutex_unlock(&priv->lock);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:song_ioe_initialize
 *
 * Description:
 *   Create ioe device driver instances for song platform.
 *
 * Input Parameters:
 *   cfg - Pointer to struct song_ioe_config_s
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *song_ioe_initialize(FAR const struct song_ioe_config_s *cfg)
{
  FAR struct song_ioe_dev_s *priv;
  struct clk *mclk;
  int ret = 0;

  priv = kmm_zalloc(sizeof(struct song_ioe_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  mclk = clk_get(cfg->mclk);
  if (mclk)
    {
      ret = clk_enable(mclk);
      if (ret < 0)
        return NULL;
    }

  priv->cpu  = cfg->cpu;
  priv->base = cfg->base;

  ret = irq_attach(cfg->irq, song_ioe_handler, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }
  up_enable_irq(cfg->irq);

  priv->ioe.ops = &g_song_ioe_ops;
  nxmutex_init(&priv->lock);

  return &priv->ioe;
}
