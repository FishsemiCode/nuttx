/****************************************************************************
 * drivers/audio/song_i2s.c
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
#include <nuttx/audio/song_i2s.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_I2S_SCLK_CFG             0x00
#define SONG_I2S_FSYNC_CFG            0x04
#define SONG_I2S_FIFO_STA             0x08
#define SONG_I2S_PCM_EN               0x0c
#define SONG_I2S_MODE                 0x10
#define SONG_I2S_REC_FIFO             0x14
#define SONG_I2S_TRAN_FIFO            0x18
#define SONG_I2S_PCM_DOP_CFG          0x2c
#define SONG_I2S_LP_EN                0x30

#define SONG_I2S_FSYNC_BIT_16         0x000000
#define SONG_I2S_FSYNC_BIT_24         0x001000
#define SONG_I2S_FSYNC_BIT_32         0x002000
#define SONG_I2S_FSYNC_BIT_MASK       0x003000

#define SONG_I2S_FSYNC_DIV_MAX        0x000fff
#define SONG_I2S_FSYNC_DIV_OFF        0
#define SONG_I2S_FSYNC_DIV_MASK       0x000fff

#define SONG_I2S_IBUF_AF              0x000080
#define SONG_I2S_IBUF_HF              0x000040
#define SONG_I2S_IBUF_NE              0x000020
#define SONG_I2S_IBUF_FL              0x000010
#define SONG_I2S_OBUF_NF              0x000008
#define SONG_I2S_OBUF_HE              0x000004
#define SONG_I2S_OBUF_AE              0x000002
#define SONG_I2S_OBUF_EM              0x000001

#define SONG_I2S_BYTE_REVERSE         0x200000
#define SONG_I2S_DOP_EN               0x080000
#define SONG_I2S_ENDIAN_BE            0x040000

#define SONG_I2S_SLOT_NUM_OFF         16
#define SONG_I2S_SLOT_NUM_MASK        0x030000

#define SONG_I2S_PACK_24              0x001000

#define SONG_I2S_TRAN_POL_RISE        0x000000
#define SONG_I2S_TRAN_POL_FALL        0x000800
#define SONG_I2S_TRAN_POL_MASK        0x000800

#define SONG_I2S_REC_POL_FALL         0x000000
#define SONG_I2S_REC_POL_RISE         0x000400
#define SONG_I2S_REC_POL_MASK         0x000400

#define SONG_I2S_SYNC_INVERT          0x000200

#define SONG_I2S_MODE_I2S             0x000000
#define SONG_I2S_MODE_TDM             0x000100
#define SONG_I2S_MODE_MASK            0x000100

#define SONG_I2S_SCLK_INVERT          0x000080
#define SONG_I2S_NO_DELAY             0x000040
#define SONG_I2S_FLUSH_TRAN_BUF       0x000020
#define SONG_I2S_FLUSH_REC_BUF        0x000010
#define SONG_I2S_I2S_EN               0x000008
#define SONG_I2S_REC_EN               0x000004
#define SONG_I2S_TRAN_EN              0x000002
#define SONG_I2S_SLAVE_EN             0x000001

#define SONG_PCM_DOP_MARKER           0xfa05fa05

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_i2s_s
{
  struct i2s_dev_s dev;
  struct clk *sclk;
  uint32_t base;
  uint8_t data_width;
  uint8_t channels;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      song_i2s_channels(struct i2s_dev_s *dev_, uint8_t channels);
static uint32_t song_i2s_samplerate(struct i2s_dev_s *dev_, uint32_t rate);
static uint32_t song_i2s_datawidth(struct i2s_dev_s *dev_, int bits);
static int      song_i2s_set_fmt(struct song_i2s_s *dev, uint16_t fmt);
static int      song_i2s_ioctl(struct i2s_dev_s *dev_, int cmd, unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void song_i2s_putreg(struct song_i2s_s *dev,
                                   uint32_t offset, uint32_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = regval;
}

static inline uint32_t song_i2s_getreg(struct song_i2s_s *dev,
                                       uint32_t offset)
{
  return *(volatile uint32_t *)(dev->base + B2C(offset));
}

static inline void song_i2s_updatereg(struct song_i2s_s *dev,
                                      uint32_t offset, uint32_t mask,
                                      uint32_t regval)
{
  song_i2s_putreg(dev, offset, (regval & mask) |
                  (song_i2s_getreg(dev, offset) & ~mask));
}

static int song_i2s_channels(struct i2s_dev_s *dev_, uint8_t channels)
{
  struct song_i2s_s *dev = (struct song_i2s_s *)dev_;

  if (!channels || channels > 4)
    return -EINVAL;
  dev->channels = channels;
  song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_SLOT_NUM_MASK,
                     ((channels - 1) << SONG_I2S_SLOT_NUM_OFF));

  return OK;
}

static uint32_t song_i2s_samplerate(struct i2s_dev_s *dev_, uint32_t rate)
{
  struct song_i2s_s *dev = (struct song_i2s_s *)dev_;
  unsigned long bclk_rate;

  bclk_rate = dev->channels * dev->data_width * rate;

#ifndef CONFIG_U31_AP
  song_i2s_updatereg(dev, SONG_I2S_FSYNC_CFG, SONG_I2S_FSYNC_DIV_MASK,
                     (dev->channels * dev->data_width) << SONG_I2S_FSYNC_DIV_OFF);

  clk_set_rate(dev->sclk, bclk_rate);
#else
  song_i2s_updatereg(dev, SONG_I2S_FSYNC_CFG, SONG_I2S_FSYNC_DIV_MASK,
                     (dev->channels * dev->data_width * 2 + 2) << SONG_I2S_FSYNC_DIV_OFF);
#endif

  return rate;
}

static uint32_t song_i2s_datawidth(struct i2s_dev_s *dev_, int bits)
{
  struct song_i2s_s *dev = (struct song_i2s_s *) dev_;
  uint32_t fsync_bit;

  switch (bits)
    {
      case 16:
        fsync_bit = SONG_I2S_FSYNC_BIT_16;
        break;
      case 24:
        fsync_bit = SONG_I2S_FSYNC_BIT_24;
        break;
      case 32:
        fsync_bit = SONG_I2S_FSYNC_BIT_32;
        break;
      default:
        return -EINVAL;
    }
  dev->data_width = bits;
  song_i2s_updatereg(dev, SONG_I2S_FSYNC_CFG,
                     SONG_I2S_FSYNC_BIT_MASK,
                     fsync_bit);

  return bits;
}

static int song_i2s_set_fmt(struct song_i2s_s *dev, uint16_t fmt)
{
  switch (fmt & AUDIO_HWFMT_FORMAT_MASK)
    {
      case AUDIO_HWFMT_I2S:
        song_i2s_putreg(dev, SONG_I2S_PCM_EN, 0);
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_NO_DELAY |
                           SONG_I2S_I2S_EN, SONG_I2S_I2S_EN);
        break;
      case AUDIO_HWFMT_LEFT_J:
        song_i2s_putreg(dev, SONG_I2S_PCM_EN, 0);
        song_i2s_updatereg(dev, SONG_I2S_MODE,
                           SONG_I2S_NO_DELAY | SONG_I2S_I2S_EN,
                           SONG_I2S_NO_DELAY | SONG_I2S_I2S_EN);
        break;
      case AUDIO_HWFMT_DSP_A:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_NO_DELAY |
                           SONG_I2S_I2S_EN, 0);
        song_i2s_putreg(dev, SONG_I2S_PCM_EN, 1);
        break;
      case AUDIO_HWFMT_DSP_B:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_NO_DELAY |
                           SONG_I2S_I2S_EN, SONG_I2S_NO_DELAY);
        song_i2s_putreg(dev, SONG_I2S_PCM_EN, 1);
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_INV_MASK)
    {
      case AUDIO_HWFMT_NB_NF:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_SYNC_INVERT |
                           SONG_I2S_SCLK_INVERT, 0);
        break;
      case AUDIO_HWFMT_NB_IF:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_SYNC_INVERT |
                           SONG_I2S_SCLK_INVERT, SONG_I2S_SYNC_INVERT);
        break;
      case AUDIO_HWFMT_IB_NF:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_SYNC_INVERT |
                           SONG_I2S_SCLK_INVERT, SONG_I2S_SCLK_INVERT);
        break;
      case AUDIO_HWFMT_IB_IF:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_SYNC_INVERT |
                           SONG_I2S_SCLK_INVERT, SONG_I2S_SYNC_INVERT |
                           SONG_I2S_SCLK_INVERT);
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_MASTER_MASK)
    {
      case AUDIO_HWFMT_CBM_CFM:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_TRAN_POL_MASK |
                           SONG_I2S_REC_POL_MASK | SONG_I2S_SLAVE_EN,
                           SONG_I2S_TRAN_POL_RISE | SONG_I2S_REC_POL_RISE |
                           SONG_I2S_SLAVE_EN);
        break;
      case AUDIO_HWFMT_CBS_CFS:
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_TRAN_POL_MASK |
                           SONG_I2S_REC_POL_MASK | SONG_I2S_SLAVE_EN,
                           SONG_I2S_TRAN_POL_FALL | SONG_I2S_REC_POL_FALL |
                           0);

        if (strstr(clk_get_name(dev->sclk), "pcm") != NULL)
          {
            song_i2s_updatereg(dev, SONG_I2S_MODE,
                               SONG_I2S_SLAVE_EN, SONG_I2S_SLAVE_EN);
          }
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int song_i2s_ioctl(struct i2s_dev_s *dev_, int cmd, unsigned long arg)
{
  struct song_i2s_s *dev = (struct song_i2s_s *) dev_;
  struct audio_caps_s *caps;
  bool playback = !!arg;
  int ret;

  switch (cmd)
    {
      case AUDIOIOC_START:
      case AUDIOIOC_RESUME:
        ret = clk_enable(dev->sclk);
        if (ret < 0)
          return ret;

        /* FIFO RESET */

        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_FLUSH_TRAN_BUF | SONG_I2S_FLUSH_REC_BUF,
                           SONG_I2S_FLUSH_REC_BUF | SONG_I2S_FLUSH_TRAN_BUF);
        song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_FLUSH_TRAN_BUF | SONG_I2S_FLUSH_REC_BUF, 0);

        if (playback)
          song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_TRAN_EN,
                             SONG_I2S_TRAN_EN);
        else
          song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_REC_EN,
                             SONG_I2S_REC_EN);
        break;
      case AUDIOIOC_STOP:
        if (playback)
          {
            song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_FLUSH_TRAN_BUF,
                               SONG_I2S_FLUSH_TRAN_BUF);
            song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_FLUSH_TRAN_BUF, 0);
          }
        else
          {
            song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_FLUSH_REC_BUF,
                               SONG_I2S_FLUSH_REC_BUF);
            song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_FLUSH_REC_BUF, 0);
          }
          /* fall through */
      case AUDIOIOC_PAUSE:
        if (playback)
          song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_TRAN_EN, 0);
        else
          song_i2s_updatereg(dev, SONG_I2S_MODE, SONG_I2S_REC_EN, 0);
        clk_disable(dev->sclk);
        break;
      case AUDIOIOC_CONFIGURE:
        caps = (struct audio_caps_s *) arg;
        DEBUGASSERT(caps);
        switch (caps->ac_type & AUDIO_TYPE_EXTENSION)
          {
            case AUDIO_TYPE_EXTENSION:
              switch(caps->ac_format.hw)
                {
                  case AUDIO_EU_HW_FORMAT:
                    return song_i2s_set_fmt(dev, caps->ac_controls.hw[0]);
                  default:
                    return -ENOTTY;
                }
              break;
            default:
              return -ENOTTY;
          }
      default:
        return -ENOTTY;
    }

    return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_song_i2s_ops =
{
  .i2s_rxchannels    = song_i2s_channels,
  .i2s_rxsamplerate  = song_i2s_samplerate,
  .i2s_rxdatawidth   = song_i2s_datawidth,
  .i2s_txchannels    = song_i2s_channels,
  .i2s_txsamplerate  = song_i2s_samplerate,
  .i2s_txdatawidth   = song_i2s_datawidth,
  .i2s_ioctl         = song_i2s_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2s_dev_s *song_i2s_initialize(uintptr_t base, const char *mclk)
{
  struct song_i2s_s *dev;
  char sclk_name[32];

  if (!mclk)
    return NULL;

  dev = kmm_zalloc(sizeof(struct song_i2s_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_song_i2s_ops;
  dev->base = base;

  sprintf(sclk_name, "%s.sclk", mclk);
  dev->sclk = clk_register_divider(sclk_name, mclk, CLK_SET_RATE_PARENT,
                                   dev->base + SONG_I2S_SCLK_CFG, 0, 12,
                                   (6 << CLK_DIVIDER_MINDIV_OFF) | CLK_DIVIDER_ROUND_CLOSEST |
                                   CLK_DIVIDER_ONE_BASED);
  if (!dev->sclk)
    {
      kmm_free(dev);
      return NULL;
    }

  song_i2s_putreg(dev, SONG_I2S_PCM_DOP_CFG, SONG_PCM_DOP_MARKER);
  song_i2s_putreg(dev, SONG_I2S_LP_EN, 0x3f);

  return &dev->dev;
}
