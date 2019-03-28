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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_AUDIO_PATH_ANC_CTL(x)                  (0x000 + (x) * 0x200)
#define SONG_AUDIO_PATH_ANC_CFG(x)                  (0x004 + (x) * 0x200)

#define SONG_AUDIO_PATH_CTL0                        0x480
#define SONG_AUDIO_PATH_CFG                         0x488

#define SONG_AUDIO_PATH_ANC_ENABLE                  0x001

#define SONG_AUDIO_PATH_ANC_SDM_BYPASS              0x00200000
#define SONG_AUDIO_PATH_ANC_HW_ANC_EN               0x00080000
#define SONG_AUDIO_PATH_ANC_FB_MIC_EN               0x00000010
#define SONG_AUDIO_PATH_ANC_FF_MIC_EN               0x00000001

#define SONG_AUDIO_PATH_I2S_EN                      0x00000020
#define SONG_AUDIO_PATH_AKM_EN                      0x00000010
#define SONG_AUDIO_PATH_PDM_EN                      0x00000008

#define SONG_AUDIO_PATH_PDM3_CLK_DIR                0x00000008

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
  int nchannels;
  int ret = OK;

  DEBUGASSERT(song_audio_path && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:

        /* Save the current stream configuration */

        nchannels = caps->ac_channels;

        if (!nchannels)
          return -EINVAL;
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

  clk_enable(clk_get("audio_clk_3072k"));
  clk_enable(clk_get("audio_sys_clk_30720k"));
  clk_enable(clk_get("out0"));
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
      if (!dev->extern_adc)
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                             SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                             SONG_AUDIO_PATH_ANC_FB_MIC_EN,
                             SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                             SONG_AUDIO_PATH_ANC_FB_MIC_EN);
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

  clk_disable(clk_get("audio_clk_3072k"));
  clk_disable(clk_get("audio_sys_clk_30720k"));
  clk_disable(clk_get("out0"));

  for (i = 0; i < dev->channels; ++i)
     {
      if (!clk_is_enabled(clk_get("audio_clk_3072k")))
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                             SONG_AUDIO_PATH_ANC_ENABLE, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN |
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN, 0);
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

  clk_disable(clk_get("audio_clk_3072k"));
  clk_disable(clk_get("audio_sys_clk_30720k"));
  clk_disable(clk_get("out0"));

  for (i = 0; i < dev->channels; ++i)
    {
      if (!clk_is_enabled(clk_get("audio_clk_3072k")))
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                             SONG_AUDIO_PATH_ANC_ENABLE, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN |
                           SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                           SONG_AUDIO_PATH_ANC_FB_MIC_EN, 0);
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

  clk_enable(clk_get("audio_clk_3072k"));
  clk_enable(clk_get("audio_sys_clk_30720k"));
  clk_enable(clk_get("out0"));
  for (i = 0; i < dev->channels; ++i)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                           SONG_AUDIO_PATH_ANC_ENABLE,
                           SONG_AUDIO_PATH_ANC_ENABLE);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN,
                           SONG_AUDIO_PATH_ANC_HW_ANC_EN);
      if (!dev->extern_adc)
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                             SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                             SONG_AUDIO_PATH_ANC_FB_MIC_EN,
                             SONG_AUDIO_PATH_ANC_FF_MIC_EN |
                             SONG_AUDIO_PATH_ANC_FB_MIC_EN);

    }

  return OK;
}
#endif

static int song_audio_path_channels(struct song_audio_path_s *dev,
                                    uint8_t channels)
{
  int i;

  if (!channels)
    return -EINVAL;

  dev->channels = channels;
  for (i = 0; i < channels; ++i)
    audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                         SONG_AUDIO_PATH_ANC_SDM_BYPASS, 0);

  return OK;
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
