/****************************************************************************
 * drivers/audio/song_audio_path_in.c
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
#include <nuttx/kmalloc.h>

#include "song_audio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_AUDIO_PATH_ANC_CTL(x)                  (0x000 + (x) * 0x200)
#define SONG_AUDIO_PATH_ANC_CFG(x)                  (0x004 + (x) * 0x200)
#define SONG_AUDIO_PATH_ANC_CFG2(x)                 (0x00c + (x) * 0x200)

#define SONG_AUDIO_PATH_CTL0                        0x480
#define SONG_AUDIO_PATH_CFG                         0x488

#define SONG_AUDIO_PATH_ANC_ENABLE                  0x001

#define SONG_AUDIO_PATH_ANC_SDM_BYPASS              0x00200000
#define SONG_AUDIO_PATH_ANC_HW_ANC_EN               0x00080000
#define SONG_AUDIO_PATH_ANC_FB_MIC_EN               0x00000010
#define SONG_AUDIO_PATH_ANC_FF_MIC_EN               0x00000001
#define SONG_AUDIO_PATH_ANC_CIC2_MODE               0x00020000

#define SONG_AUDIO_PATH_FF_MIC_SEL_MASK             0x00000030
#define SONG_AUDIO_PATH_FF_MIC_DOLPHIN_ADC1         0x00000010
#define SONG_AUDIO_PATH_FB_MIC_SEL_MASK             0x000000c0
#define SONG_AUDIO_PATH_FB_MIC_DOLPHIN_ADC2         0x00000080

#define SONG_AUDIO_PATH_I2S_EN                      0x00000020
#define SONG_AUDIO_PATH_AKM_EN                      0x00000010
#define SONG_AUDIO_PATH_PDM_EN                      0x00000008
#define SONG_AUDIO_PATH_D_ADC12_EN                  0x00000040

#define SONG_AUDIO_PATH_PDM3_CLK_DIR                0x00000008

#define SONG_AUDIO_PATH_D_ADC12_FS_MASK             0x00007000
#define SONG_AUDIO_PATH_D_ADC12_FS_16K              0x00001000
#define SONG_AUDIO_PATH_D_ADC12_FS_48K              0x00002000
#define SONG_AUDIO_PATH_D_ADC12_FS_96K              0x00003000
#define SONG_AUDIO_PATH_D_ADC12_FS_192K             0x00004000
#define SONG_AUDIO_PATH_D_ADC12_FS_384K             0x00005000
#define SONG_AUDIO_PATH_D_ADC12_FS_768K             0x00006000

#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC              0x0000000e
#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC              0x000000e0

#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC_0            0x00000000
#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC_1            0x00000002
#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC_2            0x00000004
#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC_3            0x00000006
#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC_4            0x00000008
#define SONG_AUDIO_PATH_ANC_FF_MIC_SRC_5            0x00000010

#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC_0            0x00000000
#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC_1            0x00000020
#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC_2            0x00000040
#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC_3            0x00000060
#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC_4            0x00000080
#define SONG_AUDIO_PATH_ANC_FB_MIC_SRC_5            0x000000a0

#define SONG_AUDIO_PATH_ANC_FF_MIC_MODE             0x00000300
#define SONG_AUDIO_PATH_ANC_FB_MIC_MODE             0x00000c00

#define SONG_AUDIO_PATH_ANC_FF_MIC_MODE_0           0x00000000
#define SONG_AUDIO_PATH_ANC_FF_MIC_MODE_1           0x00000100
#define SONG_AUDIO_PATH_ANC_FF_MIC_MODE_2           0x00000200
#define SONG_AUDIO_PATH_ANC_FF_MIC_MODE_3           0x00000300

#define SONG_AUDIO_PATH_ANC_FB_MIC_MODE_0           0x00000000
#define SONG_AUDIO_PATH_ANC_FB_MIC_MODE_1           0x00000400
#define SONG_AUDIO_PATH_ANC_FB_MIC_MODE_2           0x00000800
#define SONG_AUDIO_PATH_ANC_FB_MIC_MODE_3           0x00000c00

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_audio_path_s
{
  struct audio_lowerhalf_s dev;
  uint32_t base;
  uint8_t channels;
  bool extern_adc;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_configure(struct audio_lowerhalf_s *dev_,
                                     void *session,
                                     const struct audio_caps_s *caps);
static int song_audio_path_start(struct audio_lowerhalf_s *dev,
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
static int song_audio_path_channels(struct song_audio_path_s *dev,
                                    uint8_t channels);
static int song_audio_path_samplerate(struct song_audio_path_s *dev,
                                      uint32_t rate);
static int song_audio_path_set_fmt(struct song_audio_path_s *dev,
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
  int nchannels, samprate;
  int ret = OK;

  DEBUGASSERT(song_audio_path && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_INPUT:

        samprate  = caps->ac_controls.hw[0] |
                    (caps->ac_controls.b[3] << 16);

        if (song_audio_path_samplerate(dev, samprate) < 0)
          return -EINVAL;

        break;
      case AUDIO_TYPE_OUTPUT:

        nchannels = caps->ac_channels;

        if (song_audio_path_channels(dev, nchannels) < 0)
          return -EINVAL;

        break;
      case AUDIO_TYPE_EXTENSION:
        switch(caps->ac_format.hw)
          {
            case AUDIO_EU_HW_FORMAT:
              return song_audio_path_set_fmt(dev, caps->ac_controls.hw[0]);
            default:
              return -ENOTTY;
           }
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_start(struct audio_lowerhalf_s *dev_, void *session)
#else
static int song_audio_path_start(struct audio_lowerhalf_s *dev_)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;
  int i;

  clk_enable(clk_get(AUDIO_SYS_CLK3072K));
  clk_enable(clk_get(AUDIO_SYS_CLK49152K));
  clk_enable(clk_get("out0"));
  clk_enable(clk_get("audio_sys_in_clk"));

  for (i = 0; i < dev->channels; ++i)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_SDM_BYPASS, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                           SONG_AUDIO_PATH_ANC_ENABLE,
                           SONG_AUDIO_PATH_ANC_ENABLE);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN,
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN,
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_FF_MIC_MODE |
                           SONG_AUDIO_PATH_ANC_FB_MIC_MODE,
                           SONG_AUDIO_PATH_ANC_FF_MIC_MODE_3 |
                           SONG_AUDIO_PATH_ANC_FB_MIC_MODE_3);
    }

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN,
                           SONG_AUDIO_PATH_D_ADC12_EN);
    }
  else
    {
      for (i = 0; i < dev->channels; ++i)
        {
          audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                               SONG_AUDIO_PATH_ANC_FF_MIC_SRC |
                               SONG_AUDIO_PATH_ANC_FB_MIC_SRC,
                               SONG_AUDIO_PATH_ANC_FF_MIC_SRC_0 |
                               SONG_AUDIO_PATH_ANC_FB_MIC_SRC_1);
        }
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
  int i;

  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));
  clk_disable(clk_get("out0"));
  clk_disable(clk_get("audio_sys_in_clk"));

  for (i = 0; i < dev->channels; ++i)
     {
      if (!clk_is_enabled(clk_get(AUDIO_SYS_CLK3072K)))
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                             SONG_AUDIO_PATH_ANC_ENABLE, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN |
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN, 0);
     }

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN, 0);
    }

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
  int i;

  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));
  clk_disable(clk_get("out0"));
  clk_disable(clk_get("audio_sys_in_clk"));

  for (i = 0; i < dev->channels; ++i)
    {
      if (!clk_is_enabled(clk_get(AUDIO_SYS_CLK3072K)))
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                             SONG_AUDIO_PATH_ANC_ENABLE, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN |
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN, 0);
    }

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN, 0);
    }

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_, void *session)
#else
static int song_audio_path_resume(struct audio_lowerhalf_s *dev_)
#endif
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;
  int i;

  clk_enable(clk_get(AUDIO_SYS_CLK3072K));
  clk_enable(clk_get(AUDIO_SYS_CLK49152K));
  clk_enable(clk_get("out0"));
  clk_enable(clk_get("audio_sys_in_clk"));

  for (i = 0; i < dev->channels; ++i)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                           SONG_AUDIO_PATH_ANC_ENABLE,
                           SONG_AUDIO_PATH_ANC_ENABLE);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN,
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN,
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN);

    }

  if (dev->extern_adc)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                           SONG_AUDIO_PATH_D_ADC12_EN,
                           SONG_AUDIO_PATH_D_ADC12_EN);
    }

  return OK;
}
#endif

static int song_audio_path_channels(struct song_audio_path_s *dev,
                                    uint8_t channels)
{
  int i;

  if (!channels || channels > 2)
    return -EINVAL;

  dev->channels = channels;

  for (i = 0; i < channels; ++i)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_SDM_BYPASS, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_CIC2_MODE,
                           SONG_AUDIO_PATH_ANC_CIC2_MODE);

      if (dev->extern_adc)
        {
          audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG2(i),
                               SONG_AUDIO_PATH_FF_MIC_SEL_MASK |
                               SONG_AUDIO_PATH_FB_MIC_SEL_MASK,
                               SONG_AUDIO_PATH_FF_MIC_DOLPHIN_ADC1 |
                               SONG_AUDIO_PATH_FB_MIC_DOLPHIN_ADC2);
        }
    }

  return OK;
}

static int song_audio_path_samplerate(struct song_audio_path_s *dev,
                                      uint32_t rate)
{
  if (dev->extern_adc)
    {
      switch (rate)
        {
        case 16000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_16K);
          break;
        case 48000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_48K);
          break;
        case 96000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_96K);
          break;
        case 192000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_192K);
          break;
        case 384000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_384K);
          break;
        case 768000:
          audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                               SONG_AUDIO_PATH_D_ADC12_FS_MASK,
                               SONG_AUDIO_PATH_D_ADC12_FS_768K);
          break;
        default:
          return -EINVAL;
        }
    }

  return 0;
}

static int song_audio_path_set_fmt(struct song_audio_path_s *dev,
                                   uint16_t fmt)
{
  audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                       SONG_AUDIO_PATH_I2S_EN, 0);
  audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                       SONG_AUDIO_PATH_PDM_EN,
                       SONG_AUDIO_PATH_PDM_EN);
  audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                       SONG_AUDIO_PATH_AKM_EN,
                       SONG_AUDIO_PATH_AKM_EN);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *song_audio_path_anc_initialize(uintptr_t base, bool extern_adc)
{
  struct song_audio_path_s *dev;

  dev = kmm_zalloc(sizeof(struct song_audio_path_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_song_audio_path_ops;
  dev->base = base;
  dev->extern_adc = extern_adc;

  song_audio_path_set_fmt(dev, AUDIO_HWFMT_CBS_CFS);
  song_audio_path_channels(dev, 2);

  return &dev->dev;
}
