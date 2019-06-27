/****************************************************************************
 * drivers/audio/song_audio_path_vt.c
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

#include <nuttx/audio/audio.h>
#include <nuttx/audio/song_audio_path.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>

#include "song_audio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_AUDIO_PATH_ADC_CTL0                    0x400
#define SONG_AUDIO_PATH_ADC_CTL1                    0x404
#define SONG_AUDIO_PATH_ADC_CFG0                    0x414
#define SONG_AUDIO_PATH_ADC_CFG1                    0x418
#define SONG_AUDIO_PATH_ADC_CFG2                    0x41c
#define SONG_AUDIO_PATH_CTL0                        0x480
#define SONG_AUDIO_PATH_CFG                         0x488

#define SONG_AUDIO_PATH_VOICE_ADCx_ENABLE_MASK      0x0000000f
#define SONG_AUDIO_PATH_VOICE_ADCx_ENABLE(x)        (0x01 << (x))

#define SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET         0x00020000
#define SONG_AUDIO_PATH_VOICE_ADC_FIFO_RESET        0x00010000
#define SONG_AUDIO_PATH_VOICE_VT_ENABLE             0x00000002
#define SONG_AUDIO_PATH_VOICE_ADC_ENABLE            0x00000001

#define SONG_AUDIO_PATH_VOICE_VT_SRC_MASK           0x00000001
#define SONG_AUDIO_PATH_VOICE_VT_SRC_VOICE_ADC3     0
#define SONG_AUDIO_PATH_VOICE_VT_SRC_DMAS           1

#define SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(x)   (0x70 << (x * 8))
#define SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(x)  (4 + x * 8)
#define SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(x)      (0x07 << (x * 8))
#define SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(x)     (x * 8)
#define SONG_AUDIO_PATH_VOICE_ADC_768K_16K          1
#define SONG_AUDIO_PATH_VOICE_ADC_3072K_16K         3

#define SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC        0x00000100
#define SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC_MK     0x00000600
#define SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC3       0x00000600
#define SONG_AUDIO_PATH_VOICE_DMA_SRC_EXT_ADC       0x00000001
#define SONG_AUDIO_PATH_VOICE_DMA_SLOT0_MK          0x0000000c
#define SONG_AUDIO_PATH_VOICE_DMA_SLOT12_MK         0x000000f0
#define SONG_AUDIO_PATH_VOICE_DMA_SLOT0_ADC3        0x0000000c

#define SONG_AUDIO_PATH_D_ADC3_EN                   0x00000080

#define SONG_AUDIO_PATH_DMIC_FREQ_MASK              0x00000007
#define SONG_AUDIO_PATH_EXT_ADC_FREQ_MASK           0x00070000
#define SONG_AUDIO_PATH_EXT_ADC_FREQ_16K            0x00010000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_audio_path_s
{
  struct audio_lowerhalf_s dev;
  uintptr_t base;
  int vt_src;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int song_audio_path_shutdown(struct audio_lowerhalf_s *dev_);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_start(struct audio_lowerhalf_s *dev_,
                                 void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int song_audio_path_stop(struct audio_lowerhalf_s *dev_,
                                void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int song_audio_path_pause(struct audio_lowerhalf_s *dev_,
                                 void *session);
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_,
                                  void *session);
#endif
#else
static int song_audio_path_start(struct audio_lowerhalf_s *dev_);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int song_audio_path_stop(struct audio_lowerhalf_s *dev_);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int song_audio_path_pause(struct audio_lowerhalf_s *dev_);
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_song_audio_path_ops =
{
  .shutdown = song_audio_path_shutdown,
  .start = song_audio_path_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop = song_audio_path_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause = song_audio_path_pause,
  .resume = song_audio_path_resume,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void audio_path_putreg(struct song_audio_path_s *dev,
                                     uint32_t offset, uint32_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = regval;
}

static inline uint32_t audio_path_getreg(struct song_audio_path_s *dev,
                                         uint32_t offset)
{

  return *(volatile uint32_t *)(dev->base + B2C(offset));
}

static inline void audio_path_updatereg(struct song_audio_path_s *dev,
                                        uint32_t offset, uint32_t mask,
                                        uint32_t regval)
{
  audio_path_putreg(dev, offset, (regval & mask) |
                    ((audio_path_getreg(dev, offset) & ~mask)));
}

static int song_audio_path_shutdown(struct audio_lowerhalf_s *dev_)
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;

  audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                       SONG_AUDIO_PATH_VOICE_ADCx_ENABLE(3), 0);
  audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                       SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET,
                       SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET);
  audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                       SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET, 0);
  audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                       SONG_AUDIO_PATH_VOICE_VT_ENABLE, 0);
  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_start(struct audio_lowerhalf_s *dev_, void *session)
#else
static int song_audio_path_start(struct audio_lowerhalf_s *dev_)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;

  clk_enable(clk_get(AUDIO_SYS_CLK3072K));
  clk_enable(clk_get(AUDIO_SYS_CLK49152K));

  if (dev->vt_src == AUDIO_PATH_VT_SRC_EXTERN_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC3_EN,
                           SONG_AUDIO_PATH_D_ADC3_EN);
    }
  else if (dev->vt_src == AUDIO_PATH_VT_SRC_VOICE_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_ADC_ENABLE,
                           SONG_AUDIO_PATH_VOICE_ADC_ENABLE);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE(3),
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE(3));
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_VT_ENABLE,
                           SONG_AUDIO_PATH_VOICE_VT_ENABLE);
    }

  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_stop(struct audio_lowerhalf_s *dev_, void *session)
#else
static int song_audio_path_stop(struct audio_lowerhalf_s *dev_)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;

  if (dev->vt_src == AUDIO_PATH_VT_SRC_EXTERN_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC3_EN, 0);
    }
  else if (dev->vt_src == AUDIO_PATH_VT_SRC_VOICE_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_ADC_ENABLE, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE(3), 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET,
                           SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_VT_FIFO_RESET, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_VT_ENABLE, 0);
    }

  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));

  return OK;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_pause(struct audio_lowerhalf_s *dev_, void *session)
#else
static int song_audio_path_pause(struct audio_lowerhalf_s *dev_)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;

  if (dev->vt_src == AUDIO_PATH_VT_SRC_EXTERN_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC3_EN, 0);
    }
  else if (dev->vt_src == AUDIO_PATH_VT_SRC_VOICE_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_VT_ENABLE, 0);
    }

  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_, void *session)
#else
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;

  clk_enable(clk_get(AUDIO_SYS_CLK3072K));
  clk_enable(clk_get(AUDIO_SYS_CLK49152K));

  if (dev->vt_src == AUDIO_PATH_VT_SRC_EXTERN_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC3_EN,
                           SONG_AUDIO_PATH_D_ADC3_EN);
    }
  else if (dev->vt_src == AUDIO_PATH_VT_SRC_VOICE_ADC3)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_VT_ENABLE,
                           SONG_AUDIO_PATH_VOICE_VT_ENABLE);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *song_audio_path_vt_initialize(uintptr_t base, int vt_src, bool dma_out)
{
  struct song_audio_path_s *dev;

  dev = kmm_zalloc(sizeof(struct song_audio_path_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_song_audio_path_ops;
  dev->base = base;
  dev->vt_src = vt_src;

  clk_enable(clk_get("audio_mclk"));
  clk_enable(clk_get("thinkers_pclk"));
  clk_enable(clk_get("thinkers_mclk"));
  clk_set_rate(clk_get("thinkers_mclk"), 12288000);

  switch (vt_src)
    {
      case AUDIO_PATH_VT_SRC_DMA0:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG0,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_MASK,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_DMAS);
        break;
      case AUDIO_PATH_VT_SRC_VOICE_ADC3:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG0,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_MASK,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_VOICE_ADC3);

        if (audio_path_getreg(dev, SONG_AUDIO_PATH_CFG) & SONG_AUDIO_PATH_DMIC_FREQ_MASK
                              & (1 << ((audio_path_getreg(dev, SONG_AUDIO_PATH_ADC_CFG1)
                              & SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(3)) >>
                              (SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(3) + 1))))
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                 SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(3),
                                 SONG_AUDIO_PATH_VOICE_ADC_768K_16K <<
                                 SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(3));
          }
        else
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                 SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(3),
                                 SONG_AUDIO_PATH_VOICE_ADC_3072K_16K <<
                                 SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(3));
          }

        break;
      default:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC_MK,
                             SONG_AUDIO_PATH_VOICE_VT_SRC_EXT_ADC3);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                             SONG_AUDIO_PATH_EXT_ADC_FREQ_MASK,
                             SONG_AUDIO_PATH_EXT_ADC_FREQ_16K);
        break;
    }

  if (vt_src != AUDIO_PATH_VT_SRC_DMA0 && dma_out)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                           SONG_AUDIO_PATH_VOICE_DMA_SRC_EXT_ADC,
                           SONG_AUDIO_PATH_VOICE_DMA_SRC_EXT_ADC);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                           SONG_AUDIO_PATH_VOICE_DMA_SLOT0_MK,
                           SONG_AUDIO_PATH_VOICE_DMA_SLOT0_ADC3);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                           SONG_AUDIO_PATH_VOICE_DMA_SLOT12_MK,
                           0);
    }

  return &dev->dev;
}
