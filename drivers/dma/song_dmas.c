/****************************************************************************
 * drivers/dma/song_dmas.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: ZhongAn <zhongan@pinecone.net>
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
#include <nuttx/dma/song_dmas.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_DMAS_REG_EN                        0x000
#define SONG_DMAS_REG_CLR                       0x004
#define SONG_DMAS_REG_STA                       0x008
#define SONG_DMAS_REG_INT_RAW0                  0x00c
#define SONG_DMAS_REG_INT_CLR0                  0x028
#define SONG_DMAS_REG_INTV_UNIT                 0x02c
#define SONG_DMAS_REG_PAUSE                     0x038
#define SONG_DMAS_REG_RESUME                    0x03c
#define SONG_DMAS_REG_INT_RAW1                  0x30c
#define SONG_DMAS_REG_INT_CLR1                  0x328
#define SONG_DMAS_REG_LP_CTL                    0x3fc

#define SONG_DMAS_REG_INT_EN0(x)                (((x) < 3 ? 0x010 : 0x324) + (x) * 0x04)
#define SONG_DMAS_REG_INT0(x)                   (((x) < 3 ? 0x01c : 0x32c) + (x) * 0x04)
#define SONG_DMAS_REG_INT_EN1(x)                (((x) < 3 ? 0x310 : 0x334) + (x) * 0x04)
#define SONG_DMAS_REG_INT1(x)                   (((x) < 3 ? 0x31c : 0x33c) + (x) * 0x04)

#define SONG_DMAS_REG_SAR(x)                    (0x040 + (x) * 0x20)
#define SONG_DMAS_REG_DAR(x)                    (0x044 + (x) * 0x20)
#define SONG_DMAS_REG_CTL0(x)                   (0x048 + (x) * 0x20)
#define SONG_DMAS_REG_CTL1(x)                   (0x04c + (x) * 0x20)
#define SONG_DMAS_REG_CA(x)                     (0x050 + (x) * 0x20)
#define SONG_DMAS_REG_INTA(x)                   (0x054 + (x) * 0x20)
#define SONG_DMAS_REG_WD(x)                     (0x240 + (x) * 0x04)

#define SONG_DMAS_INTV_UNIT_MIN                 1
#define SONG_DMAS_INTV_UNIT_MAX                 255


#define SONG_DMAS_CTL1_WIDTH_8BITS              0x00000000
#define SONG_DMAS_CTL1_WIDTH_16BITS             0x00000001
#define SONG_DMAS_CTL1_WIDTH_32BITS             0x00000002
#define SONG_DMAS_CTL1_WIDTH_MASK               0x00000003

#define SONG_DMAS_CTL1_WRAP_EN                  0x00000004

#define SONG_DMAS_CTL1_CHMOD_BLOCK              0x00000000
#define SONG_DMAS_CTL1_CHMOD_VARIABLE           0x00000008
#define SONG_DMAS_CTL1_CHMOD_MASK               0x00000008

#define SONG_DMAS_CTL1_POR_MIN                  2
#define SONG_DMAS_CTL1_POR_MAX                  15
#define SONG_DMAS_CTL1_POR_SHIFT                8
#define SONG_DMAS_CTL1_POR_MASK                 0x00000f00

#define SONG_DMAS_CTL1_INTVTIM_MIN              0
#define SONG_DMAS_CTL1_INTVTIM_MAX              4095
#define SONG_DMAS_CTL1_INTVTIM_SHIFT            16
#define SONG_DMAS_CTL1_INTVTIM_MASK             0x0fff0000

#define SONG_DMAS_CTL1_BLKMOD_SIGNLE            0x00000000
#define SONG_DMAS_CTL1_BLKMOD_MULTIPLE          0x00010000
#define SONG_DMAS_CTL1_BLKMOD_MASK              0x00010000

#define SONG_DMAS_CTL1_FLUSH                    0x20000000

#define MIN(x, y)                               ((x) < (y) ? (x) : (y))
#define MAX(x, y)                               ((x) > (y) ? (x) : (y))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_dmas_dev_s;

struct song_dmas_chan_s
{
  struct dma_chan_s chan;
  struct song_dmas_dev_s *dev;
  unsigned int index;
  dma_callback_t callback;
  void *arg;
  uintptr_t dst_addr;
  uintptr_t src_addr;
  size_t period_num;
  size_t period_len;
  size_t next_period;
};

struct song_dmas_dev_s
{
  struct dma_dev_s dev;
  uintptr_t base;
  int cpu;
  const char *clkname;
  bool clkinit;
  struct song_dmas_chan_s channels[16];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void song_dmas_write(struct song_dmas_dev_s *dev,
                            uint32_t offset, uint32_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = regval;
}

static uint32_t song_dmas_read(struct song_dmas_dev_s *dev,
                               uint32_t offset)
{
  return *(volatile uint32_t *)(dev->base + B2C(offset));
}

static void song_dmas_update_bits(struct song_dmas_dev_s *dev,
                                  uint32_t offset, uint32_t mask,
                                  uint32_t regval)
{
  song_dmas_write(dev, offset, (regval & mask) |
                 (song_dmas_read(dev, offset) & ~mask));
}

static bool song_dmas_is_busy(struct song_dmas_dev_s *dev,
                              unsigned int index)
{
  return (song_dmas_read(dev, SONG_DMAS_REG_STA) >> index) & 1;
}

static void song_dmas_set_priority(struct song_dmas_dev_s *dev,
                                   unsigned int index,
                                   unsigned int priority)
{
  if (!priority)
    return;

  priority = MAX(priority, SONG_DMAS_CTL1_POR_MIN);
  priority = MIN(priority, SONG_DMAS_CTL1_POR_MAX);
  song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(index),
                        SONG_DMAS_CTL1_POR_MASK,
                        priority << SONG_DMAS_CTL1_POR_SHIFT);
}

static int song_dmas_set_timeout(struct song_dmas_dev_s *dev,
                                 unsigned index,
                                 unsigned int timeout)
{
  uint32_t intvtim;

  if (!timeout)
    return OK;

  if (dev->clkname && !dev->clkinit)
    return -EPERM;

  if (index < 8)
    return -EINVAL; /* only the receive channel support timeout */

  intvtim = MAX(timeout, SONG_DMAS_CTL1_INTVTIM_MIN);
  intvtim = MIN(intvtim, SONG_DMAS_CTL1_INTVTIM_MAX);
  song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(index),
                        SONG_DMAS_CTL1_INTVTIM_MASK,
                        intvtim << SONG_DMAS_CTL1_INTVTIM_SHIFT);

  return OK;
}

static inline uint32_t song_dmas_get_width(unsigned int width)
{
  switch (width)
    {
      case 1:
        return SONG_DMAS_CTL1_WIDTH_8BITS;
      case 2:
        return SONG_DMAS_CTL1_WIDTH_16BITS;
      case 4:
      default:
        return SONG_DMAS_CTL1_WIDTH_32BITS;
    }
}

static int song_dmas_chan_config(struct dma_chan_s *chan_, const struct dma_config_s *cfg)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;
  struct song_dmas_dev_s *dev = chan->dev;

  if (!cfg)
    return -EINVAL;

  switch (cfg->direction)
    {
      case DMA_DEV_TO_MEM:
        if (chan->index < 8)
          return -EINVAL;
        break;
      case DMA_MEM_TO_DEV:
        if (chan->index > 7)
          return -EINVAL;
        break;
      default:
        return -EINVAL;
    }

  if (song_dmas_is_busy(dev, chan->index))
    return -EBUSY;

  if (song_dmas_set_timeout(dev, chan->index, cfg->timeout))
    return -EINVAL;

  song_dmas_set_priority(dev, chan->index, cfg->priority);

  if (cfg->dst_width && chan->index < 8)
    song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(chan->index),
                          SONG_DMAS_CTL1_WIDTH_MASK,
                          song_dmas_get_width(cfg->dst_width));

  if (cfg->src_width && chan->index > 7)
    song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(chan->index),
                          SONG_DMAS_CTL1_WIDTH_MASK,
                          song_dmas_get_width(cfg->src_width));

  return OK;
}

static int song_dmas_start(struct dma_chan_s *chan_, dma_callback_t callback,
                           void *arg, uintptr_t dst, uintptr_t src, size_t len)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;
  struct song_dmas_dev_s *dev = chan->dev;
  unsigned int index = chan->index;

  if (song_dmas_is_busy(dev, index))
    return -EBUSY;

  chan->callback = callback;
  chan->arg = arg;
  chan->dst_addr = dst;
  chan->src_addr = src;
  chan->period_num = 1;
  chan->period_len = len;

  if (index < 8)
    song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(index),
                          SONG_DMAS_CTL1_BLKMOD_MASK,
                          SONG_DMAS_CTL1_BLKMOD_SIGNLE);
  else
    {
      song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(index),
                            SONG_DMAS_CTL1_CHMOD_MASK |
                            SONG_DMAS_CTL1_WRAP_EN,
                            SONG_DMAS_CTL1_CHMOD_BLOCK);
      song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN0(dev->cpu),
                            1 << (index + 16), 1 << (index + 16));
    }

  song_dmas_write(dev, SONG_DMAS_REG_DAR(index),
                  chan->dst_addr);
  song_dmas_write(dev, SONG_DMAS_REG_SAR(index),
                  chan->src_addr);
  song_dmas_write(dev, SONG_DMAS_REG_CTL0(index), len);
  song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN0(dev->cpu),
                        1 << index, 1 << index);
  song_dmas_write(dev, SONG_DMAS_REG_EN, index);

  return OK;
}

static int song_dmas_start_cyclic(struct dma_chan_s *chan_,
                                  dma_callback_t callback, void *arg,
                                  uintptr_t dst, uintptr_t src,
                                  size_t len, size_t period_len)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;
  struct song_dmas_dev_s *dev = chan->dev;
  unsigned int index = chan->index;
  uintptr_t phy_addr;

  if (song_dmas_is_busy(dev, index))
    return -EBUSY;
  if (!len || !period_len)
    return -EINVAL;
  if (len % 64)
    return -EINVAL;
  if (len % period_len || len <= period_len)
    return -EINVAL;

  chan->callback = callback;
  chan->arg = arg;
  chan->dst_addr = dst;
  chan->src_addr = src;
  chan->period_num = len / period_len;
  chan->period_len = period_len;
  chan->next_period = 1;

  if (index < 8)
    {
      phy_addr = chan->src_addr;
      song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(index),
                            SONG_DMAS_CTL1_BLKMOD_MASK,
                            SONG_DMAS_CTL1_BLKMOD_MULTIPLE);
    }
  else
    {
      phy_addr = chan->dst_addr;
      song_dmas_update_bits(dev, SONG_DMAS_REG_CTL1(index),
                            SONG_DMAS_CTL1_CHMOD_MASK |
                            SONG_DMAS_CTL1_WRAP_EN,
                            SONG_DMAS_CTL1_CHMOD_VARIABLE |
                            SONG_DMAS_CTL1_WRAP_EN);
      song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN0(dev->cpu),
                            1 << (index + 16), 1 << (index + 16));
    }

  song_dmas_write(dev, SONG_DMAS_REG_DAR(index),
                  chan->dst_addr);
  song_dmas_write(dev, SONG_DMAS_REG_SAR(index),
                  chan->src_addr);
  song_dmas_write(dev, SONG_DMAS_REG_CTL0(index), len);
  song_dmas_write(dev, SONG_DMAS_REG_INTA(index), phy_addr + period_len);
  song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN1(dev->cpu),
                        1 << index, 1 << index);
  song_dmas_write(dev, SONG_DMAS_REG_EN, index);
  return OK;
}

static int song_dmas_pause(struct dma_chan_s *chan_)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;

  song_dmas_write(chan->dev, SONG_DMAS_REG_PAUSE, chan->index);

  return OK;
}

static int song_dmas_stop(struct dma_chan_s *chan_)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;
  struct song_dmas_dev_s *dev = chan->dev;
  unsigned int index = chan->index;

  /* disable finish/match/flush interrupt */
  song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN0(dev->cpu),
                        1 << index, 0);
  song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN1(dev->cpu),
                        1 << index, 0);
  if (index > 7)
    song_dmas_update_bits(dev, SONG_DMAS_REG_INT_EN0(dev->cpu),
                          1 << (index + 16), 0);
  song_dmas_write(dev, SONG_DMAS_REG_CLR, index);
  while(song_dmas_is_busy(dev, index));
  /* clear finish/match/flush status */
  song_dmas_write(dev, SONG_DMAS_REG_INT_CLR0, index);
  song_dmas_write(dev, SONG_DMAS_REG_INT_CLR1, index);
  if (index > 7)
    song_dmas_write(dev, SONG_DMAS_REG_INT_CLR0,
                    index + 16);

  return OK;
}

static int song_dmas_resume(struct dma_chan_s *chan_)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;

  song_dmas_write(chan->dev, SONG_DMAS_REG_RESUME, chan->index);

  return OK;
}

static size_t song_dmas_delivered(struct song_dmas_chan_s *chan)
{
  uintptr_t start = chan->index < 8 ? chan->src_addr : chan->dst_addr;
  uint32_t ca = song_dmas_read(chan->dev, SONG_DMAS_REG_CA(chan->index));
  size_t delivered = ca - start;

  if (delivered > chan->period_len * chan->period_num)
    {
      delivered = chan->period_len * chan->period_num;
    }

  return delivered;
}

static size_t song_dmas_residual(struct dma_chan_s *chan_)
{
  struct song_dmas_chan_s *chan = (struct song_dmas_chan_s *)chan_;
  return chan->period_len * chan->period_num - song_dmas_delivered(chan);
}

static void song_dmas_next_transfer(struct song_dmas_chan_s *chan)
{
  struct song_dmas_dev_s *dev = chan->dev;
  uintptr_t phy_addr;

  chan->next_period = (chan->next_period + 1) % chan->period_num;
  if (chan->index < 8)
    phy_addr = chan->src_addr + chan->next_period * chan->period_len;
  else
    phy_addr = chan->dst_addr + chan->next_period * chan->period_len;

  song_dmas_write(dev, SONG_DMAS_REG_INTA(chan->index), phy_addr);
}

static void song_dmas_chan_irq(struct song_dmas_chan_s* chan, bool match, bool flush)
{
  if (chan->period_num > 1)
    {
      if (match)
        {
          song_dmas_next_transfer(chan);
        }
    }
  else if (flush)
    {
      song_dmas_stop(&chan->chan);
    }

  if (chan->callback)
    {
      chan->callback(&chan->chan, chan->arg, song_dmas_delivered(chan));
    }
}

static int song_dmas_irq_handler(int irq, FAR void *context, void *args)
{
  struct song_dmas_dev_s *dev = args;
  uint32_t int0, int1 , i;

  int0 = song_dmas_read(dev, SONG_DMAS_REG_INT0(dev->cpu));
  int1 = song_dmas_read(dev, SONG_DMAS_REG_INT1(dev->cpu));

  for (i = 0; i < 16; i++)
    {
      bool finish = (int0 >> i) & 1;
      bool flush  = (int0 >> (i + 16)) & 1;
      bool match  = (int1 >> i) & 1;

      if (finish || flush || match)
        {
          if (finish)
            song_dmas_write(dev, SONG_DMAS_REG_INT_CLR0, i);
          if (flush)
            song_dmas_write(dev, SONG_DMAS_REG_INT_CLR0, i + 16);

          /* clear FINISH/FLUSH before to avoid clear the upcoming request accidentally
           * clear MATCH after to let the handler update the match address first
           */
          song_dmas_chan_irq(&dev->channels[i], match, flush);

          if (match)
            song_dmas_write(dev, SONG_DMAS_REG_INT_CLR1, i);
        }
     }
  return OK;
}

static struct dma_chan_s *song_dmas_get_chan(struct dma_dev_s *dev_, unsigned int ident)
{
  struct song_dmas_dev_s *dev = (struct song_dmas_dev_s *)dev_;

  if (ident > 15)
    return NULL;

  song_dmas_stop(&dev->channels[ident].chan);

  return &dev->channels[ident].chan;
}

static void song_dmas_put_chan(struct dma_dev_s *dev, struct dma_chan_s *chan)
{
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dma_ops_s g_song_dmas_ops =
{
  song_dmas_chan_config,
  song_dmas_start,
  song_dmas_start_cyclic,
  song_dmas_stop,
  song_dmas_pause,
  song_dmas_resume,
  song_dmas_residual,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct dma_dev_s *song_dmas_initialize(int cpu, uintptr_t base, int irq, const char *clkname)
{
  struct song_dmas_dev_s *dev;
  unsigned int i;

  dev = kmm_zalloc(sizeof(struct song_dmas_dev_s));
  if (!dev)
    return NULL;

  for (i = 0; i < 16; ++i)
    {
      dev->channels[i].dev = dev;
      dev->channels[i].index = i;
      dev->channels[i].chan.ops = &g_song_dmas_ops;
    }

  dev->base = base;
  dev->cpu = cpu;
  dev->clkname = clkname;

  if (dev->clkname && !dev->clkinit)
    {
      struct clk *dma_clk;
      uint32_t intv_unit;

      dma_clk = clk_get(dev->clkname);
      if (dma_clk == NULL)
        {
          kmm_free(dev);
          return NULL;
        }

      if (clk_enable(dma_clk) < 0)
        {
          kmm_free(dev);
          return NULL;
        }

      intv_unit = clk_get_rate(dma_clk) / 1000000; /* microsecond */
      intv_unit = MAX(intv_unit, SONG_DMAS_INTV_UNIT_MIN);
      intv_unit = MIN(intv_unit, SONG_DMAS_INTV_UNIT_MAX);
      song_dmas_write(dev, SONG_DMAS_REG_INTV_UNIT, intv_unit);

      dev->clkinit = true;
    }

  /* enable the low power control */
  song_dmas_write(dev, SONG_DMAS_REG_LP_CTL, 0x01ffff);

  dev->dev.get_chan = song_dmas_get_chan;
  dev->dev.put_chan = song_dmas_put_chan;

  irq_attach(irq, song_dmas_irq_handler, dev);
  up_enable_irq(irq);

  return &dev->dev;
}
