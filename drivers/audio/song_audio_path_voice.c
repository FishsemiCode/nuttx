/****************************************************************************
 * drivers/audio/song_audio_path_voice.c
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

#define SONG_AUDIO_PATH_VOICE_ADCx_REST_MASK        0x000f0000
#define SONG_AUDIO_PATH_VOICE_ADCx_ENABLE_MASK      0x0000000f

#define SONG_AUDIO_PATH_VOICE_ADCx_GAIN_MASK(x)     (0x7 << (x * 3 + 11))
#define SONG_AUDIO_PATH_VOICE_ADCx_GAIN_SHIFT(x)    (x * 3 + 11)

#define SONG_AUDIO_PATH_VOICE_ADC_FIFO_RESET        0x00010000
#define SONG_AUDIO_PATH_VOICE_ADC_ENABLE            0x00000001

#define SONG_AUDIO_PATH_VOICE_VT_SRC_MASK           0x00000001
#define SONG_AUDIO_PATH_VOICE_VT_SRC_VOICE_ADC3     0
#define SONG_AUDIO_PATH_VOICE_VT_SRC_DMAS           1

#define SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(x)   (0x70 << (x * 8))
#define SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(x)  (4 + x * 8)
#define SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(x)      (0x07 << (x * 8))
#define SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(x)     (x * 8)
#define SONG_AUDIO_PATH_VOICE_ADC_UX_SWAP_MASK      (0x88888888)
#define SONG_AUDIO_PATH_VOICE_ADC_768K_48K          0
#define SONG_AUDIO_PATH_VOICE_ADC_768K_16K          1
#define SONG_AUDIO_PATH_VOICE_ADC_3072K_48K         2
#define SONG_AUDIO_PATH_VOICE_ADC_3072K_16K         3
#define SONG_AUDIO_PATH_VOICE_ADC_768K_8K           4
#define SONG_AUDIO_PATH_VOICE_ADC_3072K_8K          6

#define SONG_AUDIO_PATH_DUAL_MIC_MODE_MASK          0x0000f000
#define SONG_AUDIO_PATH_DUAL_MIC_96K_48K            0x00000000
#define SONG_AUDIO_PATH_DUAL_MIC_96K_16K            0x00001000
#define SONG_AUDIO_PATH_DUAL_MIC_384K_48K           0x00002000
#define SONG_AUDIO_PATH_DUAL_MIC_384K_16K           0x00003000
#define SONG_AUDIO_PATH_DUAL_MIC_768K_48K           0x00004000
#define SONG_AUDIO_PATH_DUAL_MIC_768K_16K           0x00005000
#define SONG_AUDIO_PATH_DUAL_MIC_96K_8K             0x00009000
#define SONG_AUDIO_PATH_DUAL_MIC_384K_8K            0x0000b000
#define SONG_AUDIO_PATH_DUAL_MIC_768K_8K            0x0000d000
#define SONG_AUDIO_PATH_VOICE_DMA_SRC_EXT_ADC       0x00000001
#define SONG_AUDIO_PATH_VOICE_DMA_SLOT_MK(x)        (0x3 << (x * 2 + 2))
#define SONG_AUDIO_PATH_VOICE_DMA_SLOT_SHIFT(x)     (x * 2 + 2)

#define SONG_AUDIO_PATH_D_ADC12_EN                  0x00000040
#define SONG_AUDIO_PATH_D_ADC3_EN                   0x00000080

#define SONG_AUDIO_PATH_D_ADC12_FS_MASK             0x00007000
#define SONG_AUDIO_PATH_D_ADC12_FS_16K              0x00001000
#define SONG_AUDIO_PATH_D_ADC12_FS_48K              0x00002000
#define SONG_AUDIO_PATH_D_ADC12_FS_96K              0x00003000
#define SONG_AUDIO_PATH_D_ADC12_FS_192K             0x00004000
#define SONG_AUDIO_PATH_D_ADC12_FS_384K             0x00005000
#define SONG_AUDIO_PATH_D_ADC12_FS_768K             0x00006000
#define SONG_AUDIO_PATH_D_ADC3_FS_MASK              0x00070000
#define SONG_AUDIO_PATH_D_ADC3_FS_16K               0x00010000
#define SONG_AUDIO_PATH_D_ADC3_FS_48K               0x00020000
#define SONG_AUDIO_PATH_D_ADC3_FS_96K               0x00030000
#define SONG_AUDIO_PATH_D_ADC3_FS_192K              0x00040000
#define SONG_AUDIO_PATH_D_ADC3_FS_384K              0x00050000
#define SONG_AUDIO_PATH_D_ADC3_FS_768K              0x00060000
#define SONG_AUDIO_PATH_DMIC_FREQ_MASK              0x00000007

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_audio_path_s
{
  struct audio_lowerhalf_s dev;
  uint32_t base;
  uint32_t voice_adc_enable_bitsmap;
  uint8_t channels;
  bool extern_adc;
  int adcs;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_configure(struct audio_lowerhalf_s *dev_,
                                     void *session,
                                     const struct audio_caps_s *caps);
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
static int song_audio_path_configure(struct audio_lowerhalf_s *dev_,
                                     const struct audio_caps_s *caps);
static int song_audio_path_start(struct audio_lowerhalf_s *dev_);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int song_audio_path_stop(struct audio_lowerhalf_s *dev_);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int song_audio_path_pause(struct audio_lowerhalf_s *dev_);
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_);
#endif
#endif
static int song_audio_path_channels(struct song_audio_path_s *dev_,
                                    uint8_t channels);
static uint32_t song_audio_path_samplerate(struct song_audio_path_s *dev_,
                                           uint32_t rate);
static uint32_t song_audio_path_datawidth(struct song_audio_path_s *dev_,
                                          int bits);
static int song_audio_path_set_fmt(struct song_audio_path_s *dev_,
                                   uint16_t fmt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_song_audio_path_ops =
{
  .configure = song_audio_path_configure,
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

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_configure(struct audio_lowerhalf_s *dev_,
                                     void *session,
                                     const struct audio_caps_s *caps)
#else
static int song_audio_path_configure(struct audio_lowerhalf_s *dev_,
                                     const struct audio_caps_s *caps)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;
  int samprate, nchannels, bpsamp, ret;

  DEBUGASSERT(song_audio_path && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_INPUT:

        /* Save the current stream configuration */

        samprate  = caps->ac_controls.hw[0] |
                    (caps->ac_controls.b[3] << 16);
        nchannels = caps->ac_channels;
        bpsamp    = caps->ac_controls.b[2];

        ret = song_audio_path_channels(dev, nchannels);
        if (ret < 0)
          return ret;
        ret = song_audio_path_datawidth(dev, bpsamp);
        if (ret < 0)
          return ret;
        return song_audio_path_samplerate(dev, samprate);
      case AUDIO_TYPE_EXTENSION:
        switch(caps->ac_format.hw)
          {
            case AUDIO_EU_HW_FORMAT:
              return song_audio_path_set_fmt(dev, caps->ac_controls.hw[0]);
          }
      default:
        return -ENOTTY;
    }
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

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN | SONG_AUDIO_PATH_D_ADC3_EN,
                           SONG_AUDIO_PATH_D_ADC12_EN | SONG_AUDIO_PATH_D_ADC3_EN);
    }
  else
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                         SONG_AUDIO_PATH_VOICE_ADC_ENABLE,
                         SONG_AUDIO_PATH_VOICE_ADC_ENABLE);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE_MASK,
                           dev->voice_adc_enable_bitsmap);
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

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN | SONG_AUDIO_PATH_D_ADC3_EN,
                           0);
    }
  else
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_ADC_ENABLE, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_ADC_FIFO_RESET,
                           SONG_AUDIO_PATH_VOICE_ADC_FIFO_RESET);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL1,
                           SONG_AUDIO_PATH_VOICE_ADC_FIFO_RESET, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_REST_MASK,
                           dev->voice_adc_enable_bitsmap << 16);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_REST_MASK, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE_MASK, 0);
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

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN | SONG_AUDIO_PATH_D_ADC3_EN,
                           0);
    }
  else
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE_MASK, 0);
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

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN | SONG_AUDIO_PATH_D_ADC3_EN,
                           SONG_AUDIO_PATH_D_ADC12_EN | SONG_AUDIO_PATH_D_ADC3_EN);
    }
  else
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CTL0,
                           SONG_AUDIO_PATH_VOICE_ADCx_ENABLE_MASK,
                           dev->voice_adc_enable_bitsmap);
    }

  return OK;
}
#endif

static int song_audio_path_channels(struct song_audio_path_s *dev,
                                    uint8_t channels)
{
  int i;

  /* Set MIC data source */

  if (dev->extern_adc)
    {
      if (!channels || channels > 2)
        return -EINVAL;

      for (i = 0; i < 3; i++)
        {
          uint8_t adc = (dev->adcs >> i * 8) & 0xff;

          if (adc != 0xff)
            {
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                                   SONG_AUDIO_PATH_VOICE_DMA_SLOT_MK(i),
                                   (adc + 1) << SONG_AUDIO_PATH_VOICE_DMA_SLOT_SHIFT(i));
            }
          else
            {
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                                   SONG_AUDIO_PATH_VOICE_DMA_SLOT_MK(i),
                                   0);
              break;
            }
        }
    }
  else
    {
      if (!channels || channels > 4)
        return -EINVAL;

      for (i = 0; i < channels; i++)
        {
          audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                               SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(i),
                               i << SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(i));

          audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG0,
                               SONG_AUDIO_PATH_VOICE_ADCx_GAIN_MASK(i),
                               0x3 << SONG_AUDIO_PATH_VOICE_ADCx_GAIN_SHIFT(i));
        }
    }

  dev->voice_adc_enable_bitsmap = (1 << channels) - 1;
  dev->channels = channels;
  return OK;
}

static uint32_t song_audio_path_datawidth(struct song_audio_path_s *dev, int bits)
{
  switch (bits)
    {
      case 16:
        return OK;
      default:
        return -EINVAL;
    }
}

static uint32_t song_audio_path_samplerate(struct song_audio_path_s *dev,
                                           uint32_t rate)
{
  int i;

  if (dev->extern_adc)
    {
      switch (rate)
        {
        case 16000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_16K);
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC3_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC3_FS_16K);
          break;
        case 48000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_48K);
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC3_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC3_FS_48K);
          break;
        case 96000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_96K);
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC3_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC3_FS_96K);
          break;
        case 192000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_192K);
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC3_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC3_FS_192K);
          break;
        case 384000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_384K);
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC3_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC3_FS_384K);
          break;
        case 768000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_768K);
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC3_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC3_FS_768K);
          break;
        default:
          return -EINVAL;
        }

      return 0;
    }

  switch (rate)
    {
      case 8000:
        for (i = 0; i < dev->channels; ++i)
          {
            if (audio_path_getreg(dev, SONG_AUDIO_PATH_CFG) & SONG_AUDIO_PATH_DMIC_FREQ_MASK
                & (1 << ((audio_path_getreg(dev, SONG_AUDIO_PATH_ADC_CFG1)
                & SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(i)) >>
                (SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(i) + 1))))
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(i),
                                   SONG_AUDIO_PATH_VOICE_ADC_768K_8K <<
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(i));
            else
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(i),
                                   SONG_AUDIO_PATH_VOICE_ADC_3072K_8K <<
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(i));
           }
        break;
      case 16000:
        for (i = 0; i < dev->channels; ++i)
          {
            if (audio_path_getreg(dev, SONG_AUDIO_PATH_CFG) & SONG_AUDIO_PATH_DMIC_FREQ_MASK
                & (1 << ((audio_path_getreg(dev, SONG_AUDIO_PATH_ADC_CFG1)
                & SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(i)) >>
                (SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(i) + 1))))
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(i),
                                   SONG_AUDIO_PATH_VOICE_ADC_768K_16K <<
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(i));
            else
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(i),
                                   SONG_AUDIO_PATH_VOICE_ADC_3072K_16K <<
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(i));
          }
        break;
      case 48000:
        for (i = 0; i < dev->channels; ++i)
          {
            if (audio_path_getreg(dev, SONG_AUDIO_PATH_CFG) & SONG_AUDIO_PATH_DMIC_FREQ_MASK
                & (1 << ((audio_path_getreg(dev, SONG_AUDIO_PATH_ADC_CFG1)
                & SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_MASK(i)) >>
                (SONG_AUDIO_PATH_VOICE_ADC_MIC_SRC_SHIFT(i) + 1))))
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(i),
                                   SONG_AUDIO_PATH_VOICE_ADC_768K_48K <<
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(i));
            else
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_MASK(i),
                                   SONG_AUDIO_PATH_VOICE_ADC_3072K_48K <<
                                   SONG_AUDIO_PATH_VOICE_ADC_MODE_SHIFT(i));
          }
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int song_audio_path_set_fmt(struct song_audio_path_s *dev,
                                   uint16_t fmt)
{
  switch (fmt & AUDIO_HWFMT_FORMAT_MASK)
    {
      case AUDIO_HWFMT_DSP_A:
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_INV_MASK)
    {
      case AUDIO_HWFMT_NB_NF:
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_MASTER_MASK)
    {
      case AUDIO_HWFMT_CBS_CFS:
        break;
      default:
        return -EINVAL;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *song_audio_path_voice_initialize(uintptr_t base,
                                                           bool extern_adc,
                                                           uint32_t adcs)
{
  struct song_audio_path_s *dev;

  dev = kmm_zalloc(sizeof(struct song_audio_path_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_song_audio_path_ops;
  dev->base = base;
  dev->extern_adc = extern_adc;
  dev->adcs = adcs;

  clk_enable(clk_get("audio_mclk"));

  audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG1,
                            SONG_AUDIO_PATH_VOICE_ADC_UX_SWAP_MASK,
                            0);
  if (extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ADC_CFG2,
                           SONG_AUDIO_PATH_VOICE_DMA_SRC_EXT_ADC,
                           SONG_AUDIO_PATH_VOICE_DMA_SRC_EXT_ADC);
    }

  return &dev->dev;
}
