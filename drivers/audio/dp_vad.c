/****************************************************************************
 * drivers/audio/dp_vad.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Zhong An <zhongan@pinecone.net>
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

#include <nuttx/audio/dp_vad.h>
#include <nuttx/clk/clk.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DP_VAD_CR0        0
#define DP_VAD_CR1        1
#define DP_VAD_CR2        2
#define DP_VAD_CR3        3
#define DP_VAD_CR4        4

#define VTRACK_MASK       0xc0
#define VTRACK_SHIFT      6
#define NTRACK_MASK       0x38
#define NTRACK_SHIFT      3
#define PWR_MASK          0x7

#define MINDELAY_MASK     0x18
#define MINDELAY_SHIFT    3
#define MINEVENT_MASK     0x7

#define NFI_DET_MASK      0x3f

#define SB                0x80
#define SLEEP             0x40
#define MCLK_DIV_MASK     0X38
#define MCLK_DIV_SHIFT    3

#define IRQ_MODE_MASK     0x04
#define IRQ_MODE_HIGH     0x00
#define IRQ_MODE_PLUSE    0x04
#define IRQ_FLAG          0x02

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dp_vad_s
{
  struct audio_lowerhalf_s dev;
  struct clk *mclk;
  uint32_t base;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_vad_start(struct audio_lowerhalf_s *dev_,
                        void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int dp_vad_stop(struct audio_lowerhalf_s *dev_,
                       void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int dp_vad_pause(struct audio_lowerhalf_s *dev_,
                        void *session);
static int dp_vad_resume(struct audio_lowerhalf_s *dev_,
                         void *session);
#endif
#else
static int dp_vad_start(struct audio_lowerhalf_s *dev_);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int dp_vad_stop(struct audio_lowerhalf_s *dev_);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int dp_vad_pause(struct audio_lowerhalf_s *dev_);
static int dp_vad_resume(struct audio_lowerhalf_s *dev_);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_dp_vad_ops =
{
  .start = dp_vad_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop = dp_vad_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause = dp_vad_pause,
  .resume = dp_vad_resume,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void dp_vad_putreg(struct dp_vad_s *dev,
                                 uint8_t offset, uint8_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint8_t *)(regaddr) = regval;
}

static inline uint8_t dp_vad_getreg(struct dp_vad_s *dev,
                                     uint8_t offset)
{
  return *(volatile uint8_t *)(dev->base + B2C(offset));
}

static inline void dp_vad_updatereg(struct dp_vad_s *dev,
                                    uint8_t offset, uint8_t mask,
                                    uint8_t regval)
{
  dp_vad_putreg(dev, offset, (regval & mask) |
                ((dp_vad_getreg(dev, offset) & ~mask)));
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_vad_start(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_vad_start(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_vad_s *dev = (struct dp_vad_s *)dev_;

  clk_enable(clk_get("vad_bus_clk"));
  clk_enable(clk_get("vad_mclk_pll0"));
  clk_enable(dev->mclk);

  dp_vad_updatereg(dev, DP_VAD_CR3, SB, 0);
  dp_vad_updatereg(dev, DP_VAD_CR3, SLEEP, 0);

  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_vad_stop(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_vad_stop(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_vad_s *dev = (struct dp_vad_s *)dev_;

  dp_vad_updatereg(dev, DP_VAD_CR3, SLEEP, SLEEP);
  dp_vad_updatereg(dev, DP_VAD_CR3, SB, SB);

  clk_disable(dev->mclk);
  clk_disable(clk_get("vad_bus_clk"));
  clk_disable(clk_get("vad_mclk_pll0"));

  return OK;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_vad_pause(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_vad_pause(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_vad_s *dev = (struct dp_vad_s *)dev_;

  dp_vad_updatereg(dev, DP_VAD_CR3, SLEEP, SLEEP);
  clk_disable(dev->mclk);

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_vad_resume(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_vad_resume(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_vad_s *dev = (struct dp_vad_s *)dev_;

  clk_enable(dev->mclk);
  dp_vad_updatereg(dev, DP_VAD_CR3, SLEEP, 0);

  return OK;
}
#endif

static int dp_vad_irq_handler(int irq, FAR void *context, void *args)
{
  struct dp_vad_s *dev = args;
  struct audio_msg_s msg;
  uint32_t status;

  up_udelay(100);

  status = dp_vad_getreg(dev, DP_VAD_CR4) & IRQ_FLAG;
  if (status)
    {
      msg.msgId = AUDIO_MSG_WAKEUP;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      dev->dev.upper(dev->dev.priv, AUDIO_CALLBACK_MESSAGE, (struct ap_buffer_s *)&msg, OK, NULL);
#else
      dev->dev.upper(dev->dev.priv, AUDIO_CALLBACK_MESSAGE, (struct ap_buffer_s *)&msg, OK);
#endif

      dp_vad_updatereg(dev, DP_VAD_CR4, IRQ_FLAG, IRQ_FLAG);
      up_udelay(100);
      dp_vad_getreg(dev, DP_VAD_CR4);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *dp_vad_initialize(const char *mclk, uint32_t base, int irq)
{
  struct dp_vad_s *dev;

  dev = kmm_zalloc(sizeof(struct dp_vad_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_dp_vad_ops;
  dev->base = base;
  dev->mclk = clk_get(mclk);
  if (!dev->mclk)
    {
      kmm_free(dev);
      return NULL;
    }

  clk_enable(clk_get("vad_bus_clk"));
  clk_enable(clk_get("vad_mclk_pll0"));

  dp_vad_updatereg(dev, DP_VAD_CR4,
                   IRQ_MODE_MASK, IRQ_MODE_HIGH);

  irq_attach(irq, dp_vad_irq_handler, dev);
  up_enable_irq(irq);

  clk_disable(clk_get("vad_bus_clk"));
  clk_disable(clk_get("vad_mclk_pll0"));

  return &dev->dev;
}
