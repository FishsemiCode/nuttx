/****************************************************************************
 * drivers/pinctrl/song_pinctrl.c
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

#include <nuttx/pinctrl/pinctrl.h>
#include <nuttx/pinctrl/song_pinctrl.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_PINCTRL_MUX_MASK      0x3
#define SONG_PINCTRL_DT_MASK       0xc
#define SONG_PINCTRL_DS_MASK       0x30
#define SONG_PINCTRL_SLEW_MASK     0x100

#define SONG_PINCTRL_MUX_GPIO_FUNC 0x2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_pinctrl_dev_s
{
  FAR struct pinctrl_dev_s pinctrl;
  uint32_t                 base;
  uint32_t                 num;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t readreg(struct song_pinctrl_dev_s *priv, uint32_t offset);
static inline void writereg(struct song_pinctrl_dev_s *priv, uint32_t offset, uint32_t val);
static inline void updatereg(struct song_pinctrl_dev_s *priv,
      uint32_t offset, uint32_t mask, uint32_t val);

static int song_set_mux(FAR struct pinctrl_dev_s *dev, uint32_t pin, uint32_t selector);
static int song_set_ds(FAR struct pinctrl_dev_s *dev,  uint32_t pin, uint32_t level);
static int song_set_dt(FAR struct pinctrl_dev_s *dev, uint32_t pin, enum pinctrl_drivertype_e type);
static int song_set_slew(FAR struct pinctrl_dev_s *dev, uint32_t pin, uint32_t state);
static int song_sel_gpio(FAR struct pinctrl_dev_s *dev, uint32_t pin);

/****************************************************************************
 * Private Datas
 ****************************************************************************/

/* Song pinctrl interface operations */
static const struct pinctrl_ops_s g_song_pinctrl_ops =
{
  song_set_mux,
  song_set_ds,
  song_set_dt,
  song_set_slew,
  song_sel_gpio,
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

static inline uint32_t readreg(struct song_pinctrl_dev_s *priv,
      uint32_t offset)
{
  return (*(volatile uint32_t *)(priv->base + B2C(offset)));
}

static inline void writereg(struct song_pinctrl_dev_s *priv,
      uint32_t offset, uint32_t val)
{
  *(volatile uint32_t *)(priv->base + B2C(offset)) = val;
}

static inline void updatereg(struct song_pinctrl_dev_s *priv,
      uint32_t offset, uint32_t mask, uint32_t val)
{
  writereg(priv, offset, (val & mask) |
       (readreg(priv, offset) & ~mask));
}

static int song_set_mux(FAR struct pinctrl_dev_s *dev, uint32_t pin, uint32_t selector)
{
  FAR struct song_pinctrl_dev_s *priv = (FAR struct song_pinctrl_dev_s *)dev;

  if (pin < priv->num)
    {
      updatereg(priv, pin * 4, SONG_PINCTRL_MUX_MASK, selector);
      return 0;
    }
  return -EINVAL;
}

static int song_set_ds(FAR struct pinctrl_dev_s *dev,  uint32_t pin, uint32_t level)
{
  FAR struct song_pinctrl_dev_s *priv = (FAR struct song_pinctrl_dev_s *)dev;

  if (pin < priv->num)
    {
      updatereg(priv, pin * 4, SONG_PINCTRL_DS_MASK, level);
      return 0;
    }
  return -EINVAL;
}

static int song_set_dt(FAR struct pinctrl_dev_s *dev, uint32_t pin, enum pinctrl_drivertype_e type)
{
  FAR struct song_pinctrl_dev_s *priv = (FAR struct song_pinctrl_dev_s *)dev;

  if (pin < priv->num)
    {
      updatereg(priv, pin * 4, SONG_PINCTRL_DT_MASK, type);
      return 0;
    }
  return -EINVAL;
}

static int song_set_slew(FAR struct pinctrl_dev_s *dev, uint32_t pin, uint32_t state)
{
  FAR struct song_pinctrl_dev_s *priv = (FAR struct song_pinctrl_dev_s *)dev;

  if (pin < priv->num)
    {
      updatereg(priv, pin * 4, SONG_PINCTRL_SLEW_MASK, state);
      return 0;
    }
  return -EINVAL;
}

static int song_sel_gpio(FAR struct pinctrl_dev_s *dev, uint32_t pin)
{
  FAR struct song_pinctrl_dev_s *priv = (FAR struct song_pinctrl_dev_s *)dev;

  if (pin < priv->num)
    {
      updatereg(priv, pin * 4, SONG_PINCTRL_MUX_MASK, SONG_PINCTRL_MUX_GPIO_FUNC);
      return 0;
    }
  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:song_pinctrl_initialize
 *
 * Description:
 *   Create pinctrl device driver instances for song platform.
 *
 * Input Parameters:
 *   base  - The base address of song pinctrl driver.
 *   num   - Number of pins contain in pinctrl controller.
 *
 * Returned Value:
 *   an pinctrl_dev_s instance on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct pinctrl_dev_s *song_pinctrl_initialize(uint32_t base, uint32_t num)
{
  FAR struct song_pinctrl_dev_s *priv;

  priv = kmm_zalloc(sizeof(struct song_pinctrl_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->base = base;
  priv->num  = num;

  priv->pinctrl.ops = &g_song_pinctrl_ops;

  return &priv->pinctrl;
}
