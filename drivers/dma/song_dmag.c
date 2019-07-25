/****************************************************************************
 * drivers/dma/song_dmag.c
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
#include <nuttx/dma/song_dmag.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_DMAG_REG_STATUS                    0x0000
#define SONG_DMAG_REG_PRIOR                     0x0020
#define SONG_DMAG_REG_LP_EN                     0x0080
#define SONG_DMAG_REG_BUS_LP_EN                 0x0088

#define SONG_DMAG_REG_INTR_EN(x)                (0x0040 + (x) * 0x10)
#define SONG_DMAG_REG_INTR_MASK(x)              (0x0044 + (x) * 0x10)
#define SONG_DMAG_REG_INTR_STATUS(x)            (0x0048 + (x) * 0x10)
#define SONG_DMAG_REG_RAM_BOUNDARY(x)           (0x00a0 + (x) * 0x04)
#define SONG_DMAG_REG_CTRL(x)                   (0x0100 + (x) * 0x40)
#define SONG_DMAG_REG_CONFIG(x)                 (0x0104 + (x) * 0x40)
#define SONG_DMAG_REG_SRC_ADDR(x)               (0x0108 + (x) * 0x40)
#define SONG_DMAG_REG_DST_ADDR(x)               (0x0110 + (x) * 0x40)
#define SONG_DMAG_REG_SIZE(x)                   (0x0118 + (x) * 0x40)
#define SONG_DMAG_REG_LINK_ADDR(x)              (0x0124 + (x) * 0x40)
#define SONG_DMAG_REG_LINK_NUM(x)               (0x0128 + (x) * 0x40)
#define SONG_DMAG_REG_CH_INTR_EN(x)             (0x012c + (x) * 0x40)
#define SONG_DMAG_REG_CH_INTR_STATUS(x)         (0x0130 + (x) * 0x40)
#define SONG_DMAG_REG_CH_INTR_RAW(x)            (0x0134 + (x) * 0x40)
#define SONG_DMAG_REG_MONITOR_CTRL(x)           (0x0138 + (x) * 0x40)
#define SONG_DMAG_REG_MONITOR_OUT(x)            (0x013c + (x) * 0x40)

#define SONG_DMAG_CTRL_ENABLE                   0x01
#define SONG_DMAG_CTRL_CLOSE                    0x02
#define SONG_DMAG_CTRL_PAUSE                    0x04
#define SONG_DMAG_CTRL_RESUME                   0x00
#define SONG_DMAG_CTRL_FIFO_RESET               0x08

#define SONG_DMAG_CONFIG_SCATTER_NO             0x0000
#define SONG_DMAG_CONFIG_SCATTER_SRC            0x0001
#define SONG_DMAG_CONFIG_SCATTER_DST            0x0002
#define SONG_DMAG_CONFIG_SCATTER_BOTH           0x0003
#define SONG_DMAG_CONFIG_SCATTER_MASK           0x0003

#define SONG_DMAG_CONFIG_SRC_FIX                0x0000
#define SONG_DMAG_CONFIG_SRC_INC                0x0004
#define SONG_DMAG_CONFIG_SRC_MASK               0x000c

#define SONG_DMAG_CONFIG_BLOCK_SHORT            0x0000
#define SONG_DMAG_CONFIG_BLOCK_LONG             0x0400
#define SONG_DMAG_CONFIG_BLOCK_MASK             0x0400

#define SONG_DMAG_INTR_TRANS_END                0x0001
#define SONG_DMAG_INTR_DST_SLAERR               0x0004
#define SONG_DMAG_INTR_DST_DECERR               0x0008
#define SONG_DMAG_INTR_SRC_SLAERR               0x0010
#define SONG_DMAG_INTR_SRC_DECERR               0x0020
#define SONG_DMAG_INTR_FIFO_ERR                 0x0040
#define SONG_DMAG_INTR_FIFO_NOT_EMPTY           0x0080
#define SONG_DMAG_INTR_FIFO_HF                  0x0200
#define SONG_DMAG_INTR_FIFO_FULL                0x0800
#define SONG_DMAG_INTR_W_TIMEOUT                0x1000
#define SONG_DMAG_INTR_AW_TIMEOUT               0x2000
#define SONG_DMAG_INTR_R_TIMEOUT                0x4000
#define SONG_DMAG_INTR_AR_TIMEOUT               0x8000
#define SONG_DMAG_INTR_ERROR                    0xf07c

#define SONG_DMAG_MONITOR_SRC_ADDR              0x00
#define SONG_DMAG_MONITOR_DST_ADDR              0x01
#define SONG_DMAG_MONITOR_SRC_NUM               0x02
#define SONG_DMAG_MONITOR_DST_NUM               0x03
#define SONG_DMAG_MONITOR_MASK                  0x03

#ifdef CONFIG_SONG_DMAG_LINK
# define SONG_DMAG_RAM_DATA_OFFSET              CONFIG_SONG_DMAG_RAM_OFFSET
# define SONG_DMAG_CH_LINK_SIZE                 CONFIG_SONG_DMAG_CH_RAM_SIZE
# define SONG_DMAG_CH_DST_LINK_SIZE             CONFIG_SONG_DMAG_DST_CH_RAM_SIZE
# define SONG_DMAG_CH_SRC_LINK_SIZE             (SONG_DMAG_CH_LINK_SIZE - SONG_DMAG_CH_DST_LINK_SIZE)
# define SONG_DMAG_CH_LINK_ITEM_SIZE            8
# define SONG_DMAG_CH_DST_LINK_NUM              (SONG_DMAG_CH_DST_LINK_SIZE / SONG_DMAG_CH_LINK_ITEM_SIZE)
# define SONG_DMAG_CH_SRC_LINK_NUM              (SONG_DMAG_CH_SRC_LINK_SIZE / SONG_DMAG_CH_LINK_ITEM_SIZE)
#endif

#define MIN(x, y)                               ((x) < (y) ? (x) : (y))
#define MAX(x, y)                               ((x) > (y) ? (x) : (y))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_dmag_dev_s;

struct song_dmag_chan_s
{
  struct dma_chan_s chan;
  struct song_dmag_dev_s *dev;
  unsigned int index;
  dma_callback_t callback;
  void *arg;
  uintptr_t dst_addr;
  uintptr_t src_addr;
  size_t len;
#ifdef CONFIG_SONG_DMAG_LINK
  unsigned int work_mode;
  struct dma_link_config_s *link_cfg;
#endif
};

struct song_dmag_dev_s
{
  struct dma_dev_s dev;
  uintptr_t base;
  int cpu;
  const char *clkname;
  bool clkinit;
  struct song_dmag_chan_s channels[16];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void song_dmag_write(struct song_dmag_dev_s *dev, uint32_t offset,
                            uint32_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = (regval);
}

static uint32_t song_dmag_read(struct song_dmag_dev_s *dev,
                               uint32_t offset)
{
  uint32_t regaddr = dev->base + B2C(offset);
  return *(volatile uint32_t *)(regaddr);
}

static void song_dmag_update_bits(struct song_dmag_dev_s *dev,
                                  uint32_t offset, uint32_t mask, uint32_t regval)
{
  song_dmag_write(dev, offset, (regval & mask) |
                  (song_dmag_read(dev, offset) & ~mask));
}

static bool song_dmag_is_busy(struct song_dmag_dev_s *dev,
                              unsigned int index)
{
  return (song_dmag_read(dev, SONG_DMAG_REG_STATUS) >> index) & 1;
}

static int song_dmag_pause(struct dma_chan_s *chan_)
{
  struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
  struct song_dmag_dev_s *dev = chan->dev;

  song_dmag_write(dev, SONG_DMAG_REG_CTRL(chan->index),
                  SONG_DMAG_CTRL_PAUSE);

  return OK;
}

static int song_dmag_resume(struct dma_chan_s *chan_)
{
  struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
  struct song_dmag_dev_s *dev = chan->dev;

  song_dmag_write(dev, SONG_DMAG_REG_CTRL(chan->index),
                  SONG_DMAG_CTRL_RESUME);

  return OK;
}

static size_t song_dmag_residual(struct dma_chan_s *chan_)
{
  struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
  struct song_dmag_dev_s *dev = chan->dev;
  uint32_t ca;

#ifdef CONFIG_SONG_DMAG_LINK
  if (chan->work_mode != DMA_BLOCK_MODE)
    {
      song_dmag_write(dev, SONG_DMAG_REG_MONITOR_CTRL(chan->index),
                      SONG_DMAG_MONITOR_SRC_NUM);
      ca = song_dmag_read(dev, SONG_DMAG_REG_MONITOR_OUT(chan->index));

      return chan->link_cfg->src_link_num - ca;
    }
#endif
  song_dmag_write(dev, SONG_DMAG_REG_MONITOR_CTRL(chan->index),
                  SONG_DMAG_MONITOR_DST_ADDR);
  ca = song_dmag_read(dev, SONG_DMAG_REG_MONITOR_OUT(chan->index));

  return chan->len - (ca - chan->dst_addr);
}

static int song_dmag_chan_config(struct dma_chan_s *chan_,
                                 const struct dma_config_s *cfg)
{
 struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
 struct song_dmag_dev_s *dev = chan->dev;

  if (!cfg || cfg->direction != DMA_MEM_TO_MEM)
    return -EINVAL;

  if (song_dmag_is_busy(dev, chan->index))
    return -EBUSY;

  return OK;
}

static void song_dmag_chan_irq(struct song_dmag_chan_s *chan)
{
  struct song_dmag_dev_s *dev = chan->dev;
  unsigned int index = chan->index;
  uint32_t status = song_dmag_read(dev, SONG_DMAG_REG_CH_INTR_STATUS(index));
  ssize_t len;

#ifdef CONFIG_SONG_DMAG_LINK
  if (chan->work_mode != DMA_BLOCK_MODE)
    len = chan->link_cfg->src_link_num - song_dmag_residual(&chan->chan);
  else
#endif
    len = chan->len - song_dmag_residual(&chan->chan);

  song_dmag_write(dev, SONG_DMAG_REG_CH_INTR_STATUS(index), status);
  if (status & SONG_DMAG_INTR_ERROR)
    len = -ENXIO;
  if (chan->callback)
    chan->callback(&chan->chan, chan->arg, len);
}

static int song_dmag_irq_handler(int irq, void *context, void *args)
{
  struct song_dmag_dev_s *dev = args;
  uint32_t status;
  unsigned int i;

  status = song_dmag_read(dev, SONG_DMAG_REG_INTR_STATUS(dev->cpu));
  for (i = 0; i < 16; i++)
   {
     if (status & (1 << i))
       song_dmag_chan_irq(&dev->channels[i]);
   }
  return OK;
}

static int song_dmag_start(struct dma_chan_s *chan_,
                           dma_callback_t callback,
                           void *arg, uintptr_t dst,
                           uintptr_t src, size_t len)
{
  struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
  struct song_dmag_dev_s *dev = chan->dev;
  unsigned int index = chan->index;

  if (song_dmag_is_busy(dev, index))
    return -EBUSY;

  chan->dst_addr = dst;
  chan->src_addr = src;
  chan->len = len;
  chan->callback = callback;
  chan->arg = arg;
#ifdef CONFIG_SONG_DMAG_LINK
  chan->work_mode = DMA_BLOCK_MODE;
#endif

  song_dmag_update_bits(dev, SONG_DMAG_REG_CONFIG(index),
                        SONG_DMAG_CONFIG_BLOCK_MASK |
                        SONG_DMAG_CONFIG_SCATTER_MASK,
                        SONG_DMAG_CONFIG_BLOCK_LONG |
                        SONG_DMAG_CONFIG_SCATTER_NO);
  song_dmag_write(dev, SONG_DMAG_REG_DST_ADDR(index),
                  chan->dst_addr);
  song_dmag_write(dev, SONG_DMAG_REG_SRC_ADDR(index),
                  chan->src_addr);
  song_dmag_write(dev, SONG_DMAG_REG_SIZE(index), len);
  song_dmag_write(dev, SONG_DMAG_REG_CH_INTR_EN(index),
                  SONG_DMAG_INTR_ERROR |
                  SONG_DMAG_INTR_TRANS_END);
  song_dmag_update_bits(dev, SONG_DMAG_REG_INTR_EN(dev->cpu),
                        1 << index, 1 << index);
  song_dmag_write(dev, SONG_DMAG_REG_CTRL(index),
                  SONG_DMAG_CTRL_ENABLE);

  return OK;
}

static int song_dmag_start_cyclic(struct dma_chan_s *chan,
                                  dma_callback_t callback,
                                  void *arg, uintptr_t dst,
                                  uintptr_t src, size_t len,
                                  size_t period_len)
{
  return -ENOTSUP;
}

static int song_dmag_stop(struct dma_chan_s *chan_)
{
  struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
  struct song_dmag_dev_s *dev = chan->dev;
  unsigned int index = chan->index;

  song_dmag_write(dev, SONG_DMAG_REG_CH_INTR_EN(index), 0);
  song_dmag_update_bits(dev, SONG_DMAG_REG_INTR_EN(dev->cpu),
                        1 << index, 0);
  song_dmag_write(dev, SONG_DMAG_REG_CTRL(index),
                  SONG_DMAG_CTRL_CLOSE);
  while(song_dmag_is_busy(dev, index));
  song_dmag_write(dev, SONG_DMAG_REG_CH_INTR_STATUS(index), ~0);

  return OK;
}

static struct dma_chan_s *song_dmag_get_chan(struct dma_dev_s *dev_,
                                             unsigned int ident)
{
  struct song_dmag_dev_s *dev = (struct song_dmag_dev_s *)dev_;

  if (ident > 15)
    return NULL;

  if (dev->clkname && !dev->clkinit)
    {
      struct clk *dma_clk;
      dma_clk = clk_get(dev->clkname);
      if (dma_clk == NULL)
        {
          return NULL;
        }
      if (clk_enable(dma_clk) < 0)
        {
          return NULL;
        }
      dev->clkinit = true;
    }

  song_dmag_stop(&dev->channels[ident].chan);

  return &dev->channels[ident].chan;
}

static void song_dmag_put_chan(struct dma_dev_s *dev,
                               struct dma_chan_s *chan)
{
}

#ifdef CONFIG_SONG_DMAG_LINK
static int song_dmag_start_link(struct dma_chan_s *chan_,
                                dma_callback_t callback, void *arg,
                                unsigned int work_mode, struct dma_link_config_s *cfg)
{
    struct song_dmag_chan_s *chan = (struct song_dmag_chan_s *)chan_;
    struct song_dmag_dev_s *dev = chan->dev;
    unsigned int index = chan->index;
    unsigned int src_link_offset, dst_link_offset;
    unsigned int cnt;

    if (song_dmag_is_busy(dev, index))
      return -EBUSY;

    if (chan->work_mode > DMA_DUAL_LINK_MODE ||
        cfg->src_link_num > SONG_DMAG_CH_SRC_LINK_NUM ||
        cfg->dst_link_num > SONG_DMAG_CH_DST_LINK_NUM)
      return -EINVAL;

    chan->callback = callback;
    chan->arg = arg;
    chan->link_cfg = cfg;
    chan->work_mode = work_mode;

    song_dmag_update_bits(dev, SONG_DMAG_REG_CONFIG(index),
                          SONG_DMAG_CONFIG_SRC_MASK |
                          SONG_DMAG_CONFIG_SCATTER_MASK,
                          SONG_DMAG_CONFIG_SRC_INC |
                          work_mode);

    src_link_offset = SONG_DMAG_RAM_DATA_OFFSET + index * SONG_DMAG_CH_LINK_SIZE;
    dst_link_offset = src_link_offset + SONG_DMAG_CH_SRC_LINK_SIZE;

    song_dmag_write(dev, SONG_DMAG_REG_LINK_ADDR(index), (dst_link_offset << 16) + src_link_offset);
    song_dmag_write(dev, SONG_DMAG_REG_LINK_NUM(index), (cfg->dst_link_num << 16) + cfg->src_link_num);

    for (cnt = 0; cnt < cfg->src_link_num; cnt++)
      {
        song_dmag_write(dev, src_link_offset, cfg->src_link[cnt].addr);
        song_dmag_write(dev, src_link_offset + 0x04, cfg->src_link[cnt].link_size);

        src_link_offset += SONG_DMAG_CH_LINK_ITEM_SIZE;
      }

    for (cnt = 0; cnt < cfg->dst_link_num; cnt++)
      {
        song_dmag_write(dev, dst_link_offset, cfg->dst_link[cnt].addr);
        song_dmag_write(dev, dst_link_offset + 0x04, cfg->dst_link[cnt].link_size);

        dst_link_offset += SONG_DMAG_CH_LINK_ITEM_SIZE;
      }

    song_dmag_write(dev, SONG_DMAG_REG_CH_INTR_EN(index),
                    SONG_DMAG_INTR_ERROR |
                    SONG_DMAG_INTR_TRANS_END);
    song_dmag_update_bits(dev, SONG_DMAG_REG_INTR_EN(dev->cpu),
                          1 << index, 1 << index);
    song_dmag_write(dev, SONG_DMAG_REG_CTRL(index),
                    SONG_DMAG_CTRL_ENABLE);

    return OK;
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct dma_ops_s g_song_dmag_ops =
{
  song_dmag_chan_config,
  song_dmag_start,
  song_dmag_start_cyclic,
#ifdef CONFIG_SONG_DMAG_LINK
  song_dmag_start_link,
#endif
  song_dmag_stop,
  song_dmag_pause,
  song_dmag_resume,
  song_dmag_residual,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct dma_dev_s *song_dmag_initialize(int cpu, uintptr_t base, int irq, const char *clkname)
{
  struct song_dmag_dev_s *dev;
  int i;

  dev = kmm_zalloc(sizeof(struct song_dmag_dev_s));
  if (!dev)
    return NULL;

  for (i = 0; i < 16; ++i)
   {
     dev->channels[i].index = i;
     dev->channels[i].dev = dev;
     dev->channels[i].chan.ops = &g_song_dmag_ops;
   }

  dev->base = base;
  dev->cpu = cpu;
  dev->clkname = clkname;

  /* enable the low power control */
  song_dmag_write(dev, SONG_DMAG_REG_LP_EN, 0xffffffff);
  song_dmag_write(dev, SONG_DMAG_REG_BUS_LP_EN, 0x030003);

  dev->dev.get_chan = song_dmag_get_chan;
  dev->dev.put_chan = song_dmag_put_chan;

  irq_attach(irq, song_dmag_irq_handler, dev);
  up_enable_irq(irq);

  return &dev->dev;
}
