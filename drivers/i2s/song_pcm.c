/****************************************************************************
 * drivers/audio/song_pcm.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
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
#include <nuttx/audio/song_pcm.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>

#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_PCM_MODE                  0x00
#define SONG_PCM_SCLK_CFG              0x04
#define SONG_PCM_SYNC_CFG              0x08
#define SONG_PCM_EN                    0x0c
#define SONG_PCM_FIFO_STA              0x10
#define SONG_PCM_REC_FIFO              0x14
#define SONG_PCM_TRAN_FIFO             0x18
#define SONG_PCM_INTR_STA_RAW          0x20
#define SONG_PCM_INTR_STA              0x24
#define SONG_PCM_INTR_EN               0x28
#define SONG_PCM_DOP_CFG               0x2c

#define SONG_PCM_BYTE_REVERSE          0x800000
#define SONG_PCM_DOP_EN                0x200000
#define SONG_PCM_ENDIAN_BE             0x100000

#define SONG_PCM_SLOT_LENGTH_16        0x000000
#define SONG_PCM_SLOT_LENGTH_24        0x040000
#define SONG_PCM_SLOT_LENGTH_32        0x080000
#define SONG_PCM_SLOT_LENGTH_MASK      0x0c0000

#define SONG_PCM_SLOT_NUM_OFF          15
#define SONG_PCM_SLOT_NUM_MASK         0x038000

#define SONG_PCM_PACK_24               0x002000

#define SONG_PCM_TRAN_POL_RISE         0x000000
#define SONG_PCM_TRAN_POL_FALL         0x000800
#define SONG_PCM_TRAN_POL_MASK         0x000800

#define SONG_PCM_REC_POL_FALL          0x000000
#define SONG_PCM_REC_POL_RISE          0x000400
#define SONG_PCM_REC_POL_MASK          0x000400

#define SONG_PCM_SYNC_FORMAT_SHORT     0x000000
#define SONG_PCM_SYNC_FORMAT_LONG      0x000200
#define SONG_PCM_SYNC_FORMAT_MASK      0x000200

#define SONG_PCM_SCLK_INVERT           0x000080
#define SONG_PCM_NO_DELAY              0x000040
#define SONG_PCM_FLUSH_TRAN_BUF        0x000020
#define SONG_PCM_FLUSH_REC_BUF         0x000010
#define SONG_PCM_REC_EN                0x000004
#define SONG_PCM_TRAN_EN               0x000002
#define SONG_PCM_SLAVE_EN              0x000001

#define SONG_PCM_IBUF_AF               0x000080
#define SONG_PCM_IBUF_HF               0x000040
#define SONG_PCM_IBUF_NE               0x000020
#define SONG_PCM_IBUF_FL               0x000010
#define SONG_PCM_OBUF_NF               0x000008
#define SONG_PCM_OBUF_HE               0x000004
#define SONG_PCM_OBUF_AE               0x000002
#define SONG_PCM_OBUF_EM               0x000001

#define SONG_PCM_IBUF_OV               0x000002
#define SONG_PCM_OBUF_UF               0x000001

#define SONG_PCM_DOP_MARKER            0xfa05fa05

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_pcm_s
{
  struct i2s_dev_s  dev;
  uint32_t          base;
  struct clk        *sclk;
  uint32_t          samplerate;
  uint8_t           data_width;
  uint8_t           channels;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      song_pcm_channels(struct i2s_dev_s *dev_, uint8_t channels);
static uint32_t song_pcm_samplerate(struct i2s_dev_s *dev_, uint32_t rate);
static uint32_t song_pcm_datawidth(struct i2s_dev_s *dev_, int bits);
static int      song_pcm_ioctl(struct i2s_dev_s *dev_, int cmd, unsigned long arg);
static int      song_pcm_set_fmt(struct song_pcm_s *dev, uint16_t fmt);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void song_pcm_putreg(struct song_pcm_s *dev,
                              uint32_t offset, uint32_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = regval;
}

static inline uint32_t song_pcm_getreg(struct song_pcm_s *dev,
                                  uint32_t offset)
{
  return *(volatile uint32_t *)(dev->base + B2C(offset));
}

static inline void song_pcm_updatereg(struct song_pcm_s *dev,
                                 uint32_t offset, uint32_t mask,
                                 uint32_t regval)
{
  song_pcm_putreg(dev, offset, (regval & mask) |
             (song_pcm_getreg(dev, offset) & ~mask));
}

static int song_pcm_hw_params(struct song_pcm_s *dev)
{
  unsigned int slot_length;
  unsigned int frame_bits;
  int ret;

  if (!dev->channels || !dev->samplerate || !dev->data_width)
    return OK;

  frame_bits = dev->channels * dev->data_width;

  ret = clk_set_rate(dev->sclk, dev->samplerate * frame_bits);
  if (ret < 0)
    return ret;

  if (dev->data_width == 16)
    slot_length = SONG_PCM_SLOT_LENGTH_16;
  else if (dev->data_width == 24)
    slot_length = SONG_PCM_SLOT_LENGTH_24;
  else
    slot_length = SONG_PCM_SLOT_LENGTH_32;

  song_pcm_updatereg(dev, SONG_PCM_MODE,
          SONG_PCM_SLOT_LENGTH_MASK | SONG_PCM_SLOT_NUM_MASK,
          slot_length | ((dev->channels - 1) << SONG_PCM_SLOT_NUM_OFF));

  song_pcm_putreg(dev, SONG_PCM_SYNC_CFG, frame_bits);

  return OK;
}

static int song_pcm_channels(struct i2s_dev_s *dev_, uint8_t channels)
{
  struct song_pcm_s *dev = (struct song_pcm_s *)dev_;
  int ret;

  if (!channels)
    return -EINVAL;

  dev->channels = channels;

  ret = song_pcm_hw_params(dev);
  if (ret < 0)
    return ret;

  return channels;
}

static uint32_t song_pcm_samplerate(struct i2s_dev_s *dev_, uint32_t rate)
{
  struct song_pcm_s *dev = (struct song_pcm_s *)dev_;
  int ret;

  if (!rate)
    return -EINVAL;

  dev->samplerate = rate;

  ret = song_pcm_hw_params(dev);
  if (ret < 0)
    return ret;

  return rate;
}

static uint32_t song_pcm_datawidth(struct i2s_dev_s *dev_, int bits)
{
  struct song_pcm_s *dev = (struct song_pcm_s *) dev_;
  int ret;

  if (bits % 8 || bits > 32)
    return -EINVAL;

  dev->data_width = bits;

  ret = song_pcm_hw_params(dev);
  if (ret < 0)
    return ret;

  return bits;
}

static int song_pcm_set_fmt(struct song_pcm_s *dev, uint16_t fmt)
{
  switch (fmt & AUDIO_HWFMT_FORMAT_MASK)
    {
      case AUDIO_HWFMT_DSP_A:
        song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_NO_DELAY, 0);
        break;
      case AUDIO_HWFMT_DSP_B:
        song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_NO_DELAY, SONG_PCM_NO_DELAY);
        break;
      default:
        return -EINVAL;
    }

  switch (fmt & AUDIO_HWFMT_INV_MASK)
    {
      case AUDIO_HWFMT_NB_NF:
        song_pcm_updatereg(dev, SONG_PCM_MODE,
                SONG_PCM_SYNC_FORMAT_MASK | SONG_PCM_SCLK_INVERT, SONG_PCM_SYNC_FORMAT_SHORT);
        break;
      case AUDIO_HWFMT_NB_IF:
        song_pcm_updatereg(dev, SONG_PCM_MODE,
            SONG_PCM_SYNC_FORMAT_MASK | SONG_PCM_SCLK_INVERT, SONG_PCM_SYNC_FORMAT_LONG);

        break;
      case AUDIO_HWFMT_IB_NF:
        song_pcm_updatereg(dev, SONG_PCM_MODE,
            SONG_PCM_SYNC_FORMAT_MASK | SONG_PCM_SCLK_INVERT,
                SONG_PCM_SYNC_FORMAT_SHORT | SONG_PCM_SCLK_INVERT);
        break;
      case AUDIO_HWFMT_IB_IF:
        song_pcm_updatereg(dev, SONG_PCM_MODE,
                SONG_PCM_SYNC_FORMAT_MASK | SONG_PCM_SCLK_INVERT,
                SONG_PCM_SYNC_FORMAT_LONG | SONG_PCM_SCLK_INVERT);
        break;
      default:
        return -EINVAL;
    }
  /* ideally transmit at the falling edge and receive at the rising edge */
  switch (fmt & AUDIO_HWFMT_MASTER_MASK)
    {
      case AUDIO_HWFMT_CBM_CFM:
        /* but invert the transmit polarity to compensate the pad delay for slave */
        song_pcm_updatereg(dev, SONG_PCM_MODE,
                SONG_PCM_TRAN_POL_MASK | SONG_PCM_REC_POL_MASK | SONG_PCM_SLAVE_EN,
                SONG_PCM_TRAN_POL_RISE | SONG_PCM_REC_POL_RISE | SONG_PCM_SLAVE_EN);
        break;
      case AUDIO_HWFMT_CBS_CFS:
        /* but invert the receive polarity to compensate the pad delay for master */
        song_pcm_updatereg(dev, SONG_PCM_MODE,
                SONG_PCM_TRAN_POL_MASK | SONG_PCM_REC_POL_MASK | SONG_PCM_SLAVE_EN,
                SONG_PCM_TRAN_POL_FALL | SONG_PCM_REC_POL_FALL);
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int song_pcm_ioctl(struct i2s_dev_s *dev_, int cmd, unsigned long arg)
{
  struct song_pcm_s *dev = (struct song_pcm_s *) dev_;
  struct audio_caps_s *caps;
  bool playback = arg;
  int ret;

  switch (cmd)
    {
      case AUDIOIOC_START:
      case AUDIOIOC_RESUME:
        ret = clk_enable(dev->sclk);
        if (ret < 0)
          return ret;
        song_pcm_putreg(dev, SONG_PCM_EN, 1);
        if (playback)
          song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_TRAN_EN, SONG_PCM_TRAN_EN);
        else
          song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_REC_EN, SONG_PCM_REC_EN);
        break;
      case AUDIOIOC_STOP:
        if (playback)
          {
            song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_FLUSH_TRAN_BUF, SONG_PCM_FLUSH_TRAN_BUF);
            song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_FLUSH_TRAN_BUF, 0);
          }
        else
          {
            song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_FLUSH_REC_BUF, SONG_PCM_FLUSH_REC_BUF);
            song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_FLUSH_REC_BUF, 0);
          }
      case AUDIOIOC_PAUSE:
        if (playback)
          song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_TRAN_EN, 0);
        else
          song_pcm_updatereg(dev, SONG_PCM_MODE, SONG_PCM_REC_EN, 0);
        clk_disable(dev->sclk);
        break;
      case AUDIOIOC_CONFIGURE:
        caps = (struct audio_caps_s *) arg;
        DEBUGASSERT(caps);
        switch (caps->ac_type)
          {
            case AUDIO_TYPE_EXTENSION:
              switch(caps->ac_format.hw)
                {
                  case AUDIO_EU_HW_FORMAT:
                    return song_pcm_set_fmt(dev, caps->ac_controls.hw[0]);
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

static const struct i2s_ops_s g_pcm_ops =
{
  .i2s_rxchannels    = song_pcm_channels,
  .i2s_rxsamplerate  = song_pcm_samplerate,
  .i2s_rxdatawidth   = song_pcm_datawidth,
  .i2s_txchannels    = song_pcm_channels,
  .i2s_txsamplerate  = song_pcm_samplerate,
  .i2s_txdatawidth   = song_pcm_datawidth,
  .i2s_ioctl         = song_pcm_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2s_dev_s *song_pcm_initialize(uintptr_t base, const char *mclk)
{
  struct song_pcm_s *dev;
  char sclk_name[32];

  if (!mclk)
    return NULL;

  dev = kmm_zalloc(sizeof(struct song_pcm_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_pcm_ops;
  dev->base = base;

  sprintf(sclk_name, "%s.sclk", mclk);
  dev->sclk = clk_register_divider(sclk_name, mclk, CLK_SET_RATE_PARENT,
          dev->base + SONG_PCM_SCLK_CFG, 0, 12,
          (6 << CLK_DIVIDER_MINDIV_OFF) | CLK_DIVIDER_ROUND_CLOSEST |
          CLK_DIVIDER_ONE_BASED);
  if (!dev->sclk)
    {
      kmm_free(dev);
      return NULL;
    }

  song_pcm_putreg(dev, SONG_PCM_DOP_CFG, SONG_PCM_DOP_MARKER);
  return &dev->dev;
}
