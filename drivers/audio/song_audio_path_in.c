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
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>
#include <nuttx/regmap/regmap.h>

#include "song_audio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_AUDIO_PATH_ANC_CTL(x)                  (0x000 + (x) * 0x200)
#define SONG_AUDIO_PATH_ANC_CFG(x)                  (0x004 + (x) * 0x200)

#define SONG_AUDIO_PATH_I2S_SCLK_CFG                0x440
#define SONG_AUDIO_PATH_I2S_FSYNC_CFG               0x444
#define SONG_AUDIO_PATH_I2S_PCM_EN                  0x44c
#define SONG_AUDIO_PATH_I2S_MODE                    0x450
#define SONG_AUDIO_PATH_CTL0                        0x480
#define SONG_AUDIO_PATH_CTL1                        0x484
#define SONG_AUDIO_PATH_CFG                         0x488
#define SONG_AUDIO_PATH_LP_EN                       0x48c
#define SONG_AUDIO_PATH_INTR_EN                     0x4a4
#define SONG_AUDIO_PATH_INTR_STATUS                 0x4a8

#define SONG_AUDIO_PATH_ANC_OUT_FIFO_RESET          0x008
#define SONG_AUDIO_PATH_ANC_IN_FIFO_RESET           0x004
#define SONG_AUDIO_PATH_ANC_SYSTEM_RESET            0x002
#define SONG_AUDIO_PATH_ANC_ENABLE                  0x001

#define SONG_AUDIO_PATH_ANC_PCM_OUT_SEL_IN_96K      0x0000000
#define SONG_AUDIO_PATH_ANC_PCM_OUT_SEL_ANC         0x8000000
#define SONG_AUDIO_PATH_ANC_SDM_BYPASS              0x00200000
#define SONG_AUDIO_PATH_ANC_LPF2_BYPASS             0x00100000
#define SONG_AUDIO_PATH_ANC_SOFT_ANC_EN             0x00040000
#define SONG_AUDIO_PATH_ANC_HBF2_BYPASS             0x00008000
#define SONG_AUDIO_PATH_ANC_SRC_MODE                0x00004000
#define SONG_AUDIO_PATH_ANC_SRC_MODE_88K            0x00000000
#define SONG_AUDIO_PATH_ANC_SRC_MODE_32K            0x00004000
#define SONG_AUDIO_PATH_ANC_SRC_BYPASS              0x00002000
#define SONG_AUDIO_PATH_ANC_HBF0_BYPASS             0x00001000

#define SONG_AUDIO_PATH_I2S_SCLK_DIV_MASK           0x00000fff

#define SONG_AUDIO_PATH_I2S_CHAN_NUM_MASK           0x00070000
#define SONG_AUDIO_PATH_I2S_CHAN_NUM_SHIFT          16
#define SONG_AUDIO_PATH_I2S_FSYNC_BIT_MASK          0x00003000
#define SONG_AUDIO_PATH_I2S_FSYNC_BIT_SHIFT         12
#define SONG_AUDIO_PATH_I2S_FSYNC_DIV_MASK          0x00000fff

#define SONG_AUDIO_PATH_I2S_PCM_ENABLE              0x00000001

#define SONG_AUDIO_PATH_I2S_BYTE_REV                0x00200000
#define SONG_AUDIO_PATH_I2S_DOP_EN                  0x00080000
#define SONG_AUDIO_PATH_I2S_SLOT_NUM_MASK           0x00030000
#define SONG_AUDIO_PATH_I2S_SLOT_NUM_SHIFT          16
#define SONG_AUDIO_PATH_I2S_DATA_FORMAT             0x00001000
#define SONG_AUDIO_PATH_I2S_TRAN_POL_RAISE          0x00000000
#define SONG_AUDIO_PATH_I2S_TRAN_POL_FALL           0x00000800
#define SONG_AUDIO_PATH_I2S_TRAN_POL_MASK           0x00000800
#define SONG_AUDIO_PATH_I2S_REC_POL_RAISE           0x00000400
#define SONG_AUDIO_PATH_I2S_REC_POL_FALL            0x00000000
#define SONG_AUDIO_PATH_I2S_REC_POL_MASK            0x00000400
#define SONG_AUDIO_PATH_I2S_SYNC_INVERT             0x00000200
#define SONG_AUDIO_PATH_I2S_TDM_MODE                0x00000100
#define SONG_AUDIO_PATH_I2S_SCLK_INVERT             0x00000080
#define SONG_AUDIO_PATH_I2S_DLY_MODE                0x00000040
#define SONG_AUDIO_PATH_I2S_FLUSH_TBUF              0x00000020
#define SONG_AUDIO_PATH_I2S_FLUSH_RBUF              0x00000010
#define SONG_AUDIO_PATH_I2S_I2S_EN                  0x00000008
#define SONG_AUDIO_PATH_I2S_REC_EN                  0x00000004
#define SONG_AUDIO_PATH_I2S_TRAN_EN                 0x00000002
#define SONG_AUDIO_PATH_I2S_SLAVE_EN                0x00000001

#define SONG_AUDIO_PATH_I2S_LP_TRAN                 0x00000002
#define SONG_AUDIO_PATH_I2S_LP_REC                  0x00000001

#define SONG_AUDIO_PATH_I2S_EN                      0x00000020
#define SONG_AUDIO_PATH_AKM_EN                      0x00000010
#define SONG_AUDIO_PATH_PDM_EN                      0x00000008

#define SONG_AUDIO_PATH_AKM_FIFO_RESET              0x00000002
#define SONG_AUDIO_PATH_AUDIO_IN_RESET              0x00000001

#define SONG_AUDIO_PATH_AUDIO_IN_SLOT_MASK          0x00000400
#define SONG_AUDIO_PATH_AUDIO_IN_SLOT_SHIT          10
#define SONG_AUDIO_PATH_AUDIO_IN_WIDTH_MASK         0x00000200
#define SONG_AUDIO_PATH_AUDIO_IN_WIDTH_16BITS       0x00000000
#define SONG_AUDIO_PATH_AUDIO_IN_WIDTH_24BITS       0x00000200
#define SONG_AUDIO_PATH_PCM_OUT_FSYNC_MASK          0x00000180
#define SONG_AUDIO_PATH_PCM_OUT_FSYNC_SHIFT         7
#define SONG_AUDIO_PATH_PCM_OUT_FSYNC_96K           0
#define SONG_AUDIO_PATH_PCM_OUT_FSYNC_384K          1
#define SONG_AUDIO_PATH_PCM_OUT_FSYNC_768K          2
#define SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_MASK        0x00000040
#define SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_16BITS      0x00000000
#define SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_24BITS      0x00000040
#define SONG_AUDIO_PATH_PDM3_CLK_DIR                0x00000008
#define SONG_AUDIO_PATH_LP_AHB                      0x00000001

#define SONG_AUDIO_PATH_INTR_ANC(x)                 (1 << (x))
#define SONG_AUDIO_PATH_MAX_REG                     0x4c4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_audio_path_s
{
  struct audio_lowerhalf_s dev;
  uint32_t base;
  uint8_t channels;
  uint8_t bpsamp;
  uint8_t bclk_ratio;
  struct clk *sys_in_clk;
  struct clk *i2s_sclk;
  bool i2s_en;
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
static int song_audio_path_ioctl(struct audio_lowerhalf_s *dev_, int cmd,
                                 unsigned long arg);
static int song_audio_path_channels(struct song_audio_path_s *dev,
                                    uint8_t channels);
static uint32_t song_audio_path_datawidth(struct song_audio_path_s *dev,
                                     int bits);
static uint32_t song_audio_path_samplerate(struct song_audio_path_s *dev,
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
  .ioctl = song_audio_path_ioctl,
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
  int samprate, nchannels, bpsamp;
  int ret = OK;

  DEBUGASSERT(song_audio_path && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:

        /* Save the current stream configuration */

        samprate  = caps->ac_controls.hw[0] |
                    (caps->ac_controls.b[3] << 16);
        nchannels = caps->ac_channels;
        bpsamp    = caps->ac_controls.b[2];

        ret = song_audio_path_channels(dev, nchannels);
        if (ret < 0)
          return -EINVAL;
        ret = song_audio_path_datawidth(dev, bpsamp);
        if (ret < 0)
          return -EINVAL;
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
  int i;

  clk_enable(dev->sys_in_clk);
  clk_enable(clk_get(AUDIO_SYS_CLK3072K));
  clk_enable(clk_get(AUDIO_SYS_CLK49152K));
  if (!dev->i2s_en)
    clk_enable(clk_get("out0"));
  if (dev->i2s_en)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_TRAN_EN,
                           SONG_AUDIO_PATH_I2S_TRAN_EN);
      clk_enable(dev->i2s_sclk);
    }

  for (i = 0; i < dev->channels; ++i)
    audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                         SONG_AUDIO_PATH_ANC_ENABLE,
                         SONG_AUDIO_PATH_ANC_ENABLE);

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

  for (i = 0; i < dev->channels; i++)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                           SONG_AUDIO_PATH_ANC_ENABLE, 0);
    }

  audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL1,
                       SONG_AUDIO_PATH_AKM_FIFO_RESET |
                       SONG_AUDIO_PATH_AUDIO_IN_RESET,
                       SONG_AUDIO_PATH_AKM_FIFO_RESET |
                       SONG_AUDIO_PATH_AUDIO_IN_RESET);
  audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL1,
                       SONG_AUDIO_PATH_AKM_FIFO_RESET |
                       SONG_AUDIO_PATH_AUDIO_IN_RESET, 0);

  for (i = 0; i < dev->channels; ++i)
     {
       audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                            SONG_AUDIO_PATH_ANC_IN_FIFO_RESET,
                            SONG_AUDIO_PATH_ANC_IN_FIFO_RESET);
       audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                            SONG_AUDIO_PATH_ANC_IN_FIFO_RESET, 0);
    }

  if (dev->i2s_en)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_TRAN_EN, 0);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_FLUSH_TBUF,
                           SONG_AUDIO_PATH_I2S_FLUSH_TBUF);
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_FLUSH_TBUF, 0);

      clk_disable(dev->i2s_sclk);
    }

  clk_disable(dev->sys_in_clk);
  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));

  if (!dev->i2s_en)
    clk_disable(clk_get("out0"));

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

  if (dev->i2s_en)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_TRAN_EN, 0);
      clk_disable(dev->i2s_sclk);
    }

  clk_disable(dev->sys_in_clk);
  clk_disable(clk_get(AUDIO_SYS_CLK3072K));
  clk_disable(clk_get(AUDIO_SYS_CLK49152K));
  if (!dev->i2s_en)
    clk_disable(clk_get("out0"));
  if (!clk_is_enabled(clk_get(AUDIO_SYS_CLK3072K)))
    for (i = 0; i < dev->channels; ++i)
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                           SONG_AUDIO_PATH_ANC_ENABLE, 0);

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

  clk_enable(dev->sys_in_clk);
  clk_enable(clk_get(AUDIO_SYS_CLK3072K));
  clk_enable(clk_get(AUDIO_SYS_CLK49152K));
  if (!dev->i2s_en)
    clk_enable(clk_get("out0"));
  if (dev->i2s_en)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_TRAN_EN,
                           SONG_AUDIO_PATH_I2S_TRAN_EN);
      clk_enable(dev->i2s_sclk);
    }

  for (i = 0; i < dev->channels; ++i)
    audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CTL(i),
                         SONG_AUDIO_PATH_ANC_ENABLE,
                         SONG_AUDIO_PATH_ANC_ENABLE);
  return OK;
}
#endif

static int song_audio_path_ioctl(struct audio_lowerhalf_s *dev_, int cmd,
                                 unsigned long arg)
{
  struct song_audio_path_s *dev = (struct song_audio_path_s *)dev_;
  struct regmap_arg_s *regmap_arg = (struct regmap_arg_s *)arg;
  int i;

  switch (cmd)
    {
      case REGMAPIOC_GETREGVALUE:
        if (!regmap_arg || !regmap_arg->offsets ||
            !regmap_arg->values || !regmap_arg->reg_num ||
            regmap_arg->reg_num > SONG_AUDIO_PATH_MAX_REG)
          return -EINVAL;
        for (i = 0; i < regmap_arg->reg_num; ++i)
          {
            if (regmap_arg->offsets[i] > SONG_AUDIO_PATH_MAX_REG)
              continue;
            regmap_arg->values[i] = audio_path_getreg(dev, regmap_arg->offsets[i]);
          }
        break;
      case REGMAPIOC_SETREGVALUE:
        if (!regmap_arg || !regmap_arg->offsets ||
            !regmap_arg->values || !regmap_arg->reg_num ||
            regmap_arg->reg_num > SONG_AUDIO_PATH_MAX_REG)
          return -EINVAL;
        for (i = 0; i < regmap_arg->reg_num; ++i)
          {
            if (regmap_arg->offsets[i] > SONG_AUDIO_PATH_MAX_REG)
              continue;
            audio_path_putreg(dev, regmap_arg->offsets[i], regmap_arg->values[i]);
          }
        break;
      default:
        return -ENOTTY;
    }

  return OK;
}

static int song_audio_path_channels(struct song_audio_path_s *dev,
                                    uint8_t channels)
{
  int i;

  if (!channels)
    return -EINVAL;

  dev->channels = channels;
  audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                       SONG_AUDIO_PATH_AUDIO_IN_SLOT_MASK,
                       (channels - 1) <<
                       SONG_AUDIO_PATH_AUDIO_IN_SLOT_SHIT);
  if (dev->i2s_en)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                           SONG_AUDIO_PATH_I2S_SLOT_NUM_MASK,
                           (channels - 1) << SONG_AUDIO_PATH_I2S_SLOT_NUM_SHIFT);
      for (i = 0; i < channels; ++i)
        audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                             SONG_AUDIO_PATH_ANC_SDM_BYPASS,
                             SONG_AUDIO_PATH_ANC_SDM_BYPASS);
    }
  else
    for (i = 0; i < channels; ++i)
      audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                           SONG_AUDIO_PATH_ANC_SDM_BYPASS, 0);
  return OK;
}

static uint32_t song_audio_path_datawidth(struct song_audio_path_s *dev,
                                          int bits)
{
  uint32_t out_bits = audio_path_getreg(dev, SONG_AUDIO_PATH_CFG) &
                        SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_MASK;
  if (!out_bits)
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                           SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_MASK,
                           SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_16BITS);
      dev->bpsamp = 16;
      dev->bclk_ratio = 16;
    }
  else
    {
      audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                           SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_MASK,
                           SONG_AUDIO_PATH_AUDIO_OUT_WIDTH_24BITS);
      dev->bpsamp = 24;
      dev->bclk_ratio = 32;
    }

  switch (bits)
    {
      case 16:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                             SONG_AUDIO_PATH_AUDIO_IN_WIDTH_MASK,
                             SONG_AUDIO_PATH_AUDIO_IN_WIDTH_16BITS);
        break;
      case 24:
      case 32:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                             SONG_AUDIO_PATH_AUDIO_IN_WIDTH_MASK,
                             SONG_AUDIO_PATH_AUDIO_IN_WIDTH_24BITS);
        break;
      default:
        return -EINVAL;
    }
  return OK;
}

static uint32_t song_audio_path_samplerate(struct song_audio_path_s *dev,
                                           uint32_t rate)
{
  int i;

  for (i = 0; i < dev->channels; ++i)
    audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                         SONG_AUDIO_PATH_ANC_LPF2_BYPASS |
                         SONG_AUDIO_PATH_ANC_PCM_OUT_SEL_ANC,
                         SONG_AUDIO_PATH_ANC_PCM_OUT_SEL_ANC);

  switch (rate)
    {
      case 8000:
        for (i = 0; i < dev->channels; ++i)
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                                 SONG_AUDIO_PATH_ANC_HBF0_BYPASS |
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_MODE,
                                 SONG_AUDIO_PATH_ANC_SRC_MODE_32K);
          }
        clk_set_rate(dev->sys_in_clk, 32000);
        break;
      case 16000:
        for (i = 0; i < dev->channels; ++i)
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                                 SONG_AUDIO_PATH_ANC_HBF0_BYPASS |
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_MODE,
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_MODE_32K);
          }
        clk_set_rate(dev->sys_in_clk, 32000);
        break;
      case 44100:
        for (i = 0; i < dev->channels; ++i)
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                                 SONG_AUDIO_PATH_ANC_HBF0_BYPASS |
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_MODE,
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_MODE_88K);
          }
        clk_set_rate(dev->sys_in_clk, 88200);
        break;
      case 48000:
        for (i = 0; i < dev->channels; ++i)
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                                 SONG_AUDIO_PATH_ANC_HBF0_BYPASS |
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS,
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS);
          }
        clk_set_rate(dev->sys_in_clk, 96000);
        break;
      case 96000:
        for (i = 0; i < dev->channels; ++i)
          {
            audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                                 SONG_AUDIO_PATH_ANC_HBF0_BYPASS |
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS,
                                 SONG_AUDIO_PATH_ANC_HBF0_BYPASS |
                                 SONG_AUDIO_PATH_ANC_HBF2_BYPASS |
                                 SONG_AUDIO_PATH_ANC_SRC_BYPASS);
          }
        clk_set_rate(dev->sys_in_clk, 96000);
        break;
      default:
        return -EINVAL;
    }

  if (dev->i2s_en)
    {
      audio_path_putreg(dev, SONG_AUDIO_PATH_I2S_FSYNC_CFG, dev->channels * dev->bpsamp);
      switch((audio_path_getreg(dev, SONG_AUDIO_PATH_CFG) &
              SONG_AUDIO_PATH_PCM_OUT_FSYNC_MASK) >>
              SONG_AUDIO_PATH_PCM_OUT_FSYNC_SHIFT)
        {
          case 0:
            for (i = 0; i < dev->channels; ++i)
              audio_path_updatereg(dev, SONG_AUDIO_PATH_ANC_CFG(i),
                                   SONG_AUDIO_PATH_ANC_LPF2_BYPASS |
                                   SONG_AUDIO_PATH_ANC_PCM_OUT_SEL_ANC,
                                   SONG_AUDIO_PATH_ANC_LPF2_BYPASS);
            i = 96000;
            audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_MASK,
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_96K <<
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_SHIFT);
            break;
          case 1:
            i = 384000;
            audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_MASK,
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_384K <<
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_SHIFT);
            break;
          case 2:
            i = 768000;
            audio_path_updatereg(dev, SONG_AUDIO_PATH_CFG,
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_MASK,
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_768K <<
                                 SONG_AUDIO_PATH_PCM_OUT_FSYNC_SHIFT);
            break;
          default:
            return -EINVAL;
        }

      clk_set_rate(dev->i2s_sclk, dev->channels * i * dev->bclk_ratio);
    }

  return OK;
}

static int song_audio_path_set_fmt(struct song_audio_path_s *dev,
                                   uint16_t fmt)
{
  audio_path_putreg(dev, SONG_AUDIO_PATH_CTL0,
                    SONG_AUDIO_PATH_I2S_EN);
  dev->i2s_en = true;

  switch (fmt & AUDIO_HWFMT_FORMAT_MASK)
    {
      case AUDIO_HWFMT_I2S:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_PCM_EN,
                             SONG_AUDIO_PATH_I2S_PCM_ENABLE, 0);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_DLY_MODE |
                             SONG_AUDIO_PATH_I2S_I2S_EN,
                             SONG_AUDIO_PATH_I2S_I2S_EN);
        break;
      case AUDIO_HWFMT_LEFT_J:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_PCM_EN,
                             SONG_AUDIO_PATH_I2S_PCM_ENABLE, 0);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_DLY_MODE |
                             SONG_AUDIO_PATH_I2S_I2S_EN,
                             SONG_AUDIO_PATH_I2S_DLY_MODE |
                             SONG_AUDIO_PATH_I2S_I2S_EN);
        break;
      case AUDIO_HWFMT_DSP_A:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_DLY_MODE |
                             SONG_AUDIO_PATH_I2S_I2S_EN, 0);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_PCM_EN,
                             SONG_AUDIO_PATH_I2S_PCM_ENABLE,
                             SONG_AUDIO_PATH_I2S_PCM_ENABLE);
        break;
      case AUDIO_HWFMT_DSP_B:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_DLY_MODE |
                             SONG_AUDIO_PATH_I2S_I2S_EN,
                             SONG_AUDIO_PATH_I2S_DLY_MODE);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_PCM_EN,
                             SONG_AUDIO_PATH_I2S_PCM_ENABLE,
                             SONG_AUDIO_PATH_I2S_PCM_ENABLE);
        break;
      case AUDIO_HWFMT_PDM:
        dev->i2s_en = false;
        audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                             SONG_AUDIO_PATH_I2S_EN, 0);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                             SONG_AUDIO_PATH_PDM_EN,
                             SONG_AUDIO_PATH_PDM_EN);
        audio_path_updatereg(dev, SONG_AUDIO_PATH_CTL0,
                             SONG_AUDIO_PATH_AKM_EN,
                             SONG_AUDIO_PATH_AKM_EN);
        break;
      default:
        return -EINVAL;
    }

  switch (fmt & AUDIO_HWFMT_INV_MASK)
    {
      case AUDIO_HWFMT_NB_NF:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_SYNC_INVERT |
                             SONG_AUDIO_PATH_I2S_SCLK_INVERT, 0);
        break;
      case AUDIO_HWFMT_NB_IF:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_SYNC_INVERT |
                             SONG_AUDIO_PATH_I2S_SCLK_INVERT,
                             SONG_AUDIO_PATH_I2S_SYNC_INVERT);
        break;
      case AUDIO_HWFMT_IB_NF:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_SYNC_INVERT |
                             SONG_AUDIO_PATH_I2S_SCLK_INVERT,
                             SONG_AUDIO_PATH_I2S_SCLK_INVERT);
        break;
      case AUDIO_HWFMT_IB_IF:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_SYNC_INVERT |
                             SONG_AUDIO_PATH_I2S_SCLK_INVERT,
                             SONG_AUDIO_PATH_I2S_SYNC_INVERT |
                             SONG_AUDIO_PATH_I2S_SCLK_INVERT);
        break;
      default:
        return -EINVAL;
    }

  switch (fmt & AUDIO_HWFMT_MASTER_MASK)
    {
      case AUDIO_HWFMT_CBM_CFM:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_TRAN_POL_MASK |
                             SONG_AUDIO_PATH_I2S_REC_POL_MASK |
                             SONG_AUDIO_PATH_I2S_SLAVE_EN,
                             SONG_AUDIO_PATH_I2S_TRAN_POL_RAISE |
                             SONG_AUDIO_PATH_I2S_REC_POL_RAISE |
                             SONG_AUDIO_PATH_I2S_SLAVE_EN);
        break;
      case AUDIO_HWFMT_CBS_CFS:
        audio_path_updatereg(dev, SONG_AUDIO_PATH_I2S_MODE,
                             SONG_AUDIO_PATH_I2S_TRAN_POL_MASK |
                             SONG_AUDIO_PATH_I2S_REC_POL_MASK |
                             SONG_AUDIO_PATH_I2S_SLAVE_EN,
                             SONG_AUDIO_PATH_I2S_TRAN_POL_FALL |
                             SONG_AUDIO_PATH_I2S_REC_POL_FALL);
        break;
      default:
        return -EINVAL;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *song_audio_path_in_initialize(uintptr_t base,
                                                        const char *sys_in_clk,
                                                        const char *i2s_mclk)
{
  struct song_audio_path_s *dev;

  dev = kmm_zalloc(sizeof(struct song_audio_path_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_song_audio_path_ops;
  dev->base = base;
  dev->sys_in_clk = clk_get(sys_in_clk);

  if (!dev->sys_in_clk)
    {
      kmm_free(dev);
      return NULL;
    }

  dev->i2s_sclk = clk_register_divider("i2s.sclk", i2s_mclk, CLK_SET_RATE_PARENT,
                                       dev->base + SONG_AUDIO_PATH_I2S_SCLK_CFG, 0, 12,
                                       (8 << CLK_DIVIDER_MINDIV_OFF) | CLK_DIVIDER_ROUND_CLOSEST |
                                       CLK_DIVIDER_ONE_BASED);
  if (!dev->i2s_sclk)
    {
      kmm_free(dev);
      return NULL;
    }

  return &dev->dev;
}
