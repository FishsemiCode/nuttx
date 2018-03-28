/****************************************************************************
 * drivers/i2s/song_pdm.c
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
#include <nuttx/audio/song_pdm.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_PDM_CTL              0x00
#define SONG_PDM_CFG              0x04
#define SONG_PDM_FIFO_DATA        0x08
#define SONG_PDM_FIFO_STATUS      0x0c
#define SONG_PDM_LP_EN            0x10
#define SONG_PDM_CIC_CFG          0x14

#define SONG_PDM_RESET            0x00000002
#define SONG_PDM_ENABLE           0x00000001

#define SONG_PDM_CLK_SYNC         0x10000000

#define SONG_PDM_REC_SEL_OFF      26
#define SONG_PDM_REC_SEL_MASK     0x0c000000

#define SONG_PDM_CLK_SEL_OFF      24
#define SONG_PDM_CLK_SEL_MASK     0x03000000

#define SONG_PDM_PACK_24          0x00800000
#define SONG_PDM_DATA_OUT         0x00400000

#define SONG_PDM_WIDTH_16         0x00000000
#define SONG_PDM_WIDTH_24         0x00100000
#define SONG_PDM_WIDTH_32         0x00200000
#define SONG_PDM_WIDTH_MASK       0x00300000

#define SONG_PDM_CLK_INVERT       0x00080000
#define SONG_PDM_ENDIAN_BE        0x00040000
#define SONG_PDM_CLK_SLAVE        0x00010000

#define SONG_PDM_LOW_OFF          8
#define SONG_PDM_LOW_MASK         0x0000ff00

#define SONG_PDM_HIGH_OFF         0
#define SONG_PDM_HIGH_MASK        0x000000ff

#define SONG_PDM_TX_POP_ERR       0x00000200
#define SONG_PDM_TX_PUSH_ERR      0x00000100
#define SONG_PDM_RX_POP_ERR       0x00000080
#define SONG_PDM_RX_PUSH_ERR      0x00000040
#define SONG_PDM_ERR              0x00000020
#define SONG_PDM_FULL             0x00000010
#define SONG_PDM_HALF_FULL        0x00000004
#define SONG_PDM_EMPTY_N          0x00000001

#define SONG_PDM_CIC_N_OFF        18
#define SONG_PDM_CIC_N_MASK       0x007c0000

#define SONG_PDM_CIC_R_OFF        8
#define SONG_PDM_CIC_R_MASK       0x0003ff00

#define SONG_PDM_CIC_M_OFF        7
#define SONG_PDM_CIC_M_MASK       0x00000080

#define SONG_PDM_CIC_SHIFT_OFF    1
#define SONG_PDM_CIC_SHIFT_MASK   0x0000007e

#define SONG_PDM_CIC_ENABLE       0x00000001

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_pdm_s
{
  struct i2s_dev_s dev;
  struct clk *sclk;
  uint32_t base;
  uint8_t data_width;
  uint8_t oversampling;
};

struct song_pdm_cic
{
  unsigned int n;
  unsigned int r;
  unsigned int m;
  unsigned int s;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      song_pdm_channels(struct i2s_dev_s *dev_, uint8_t channels);
static uint32_t song_pdm_samplerate(struct i2s_dev_s *dev_, uint32_t rate);
static uint32_t song_pdm_datawidth(struct i2s_dev_s *dev_, int bits);
static int      song_pdm_set_fmt(struct song_pdm_s *dev, uint16_t fmt);
static int      song_pdm_ioctl(struct i2s_dev_s *dev_, int cmd, unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void song_pdm_putreg(struct song_pdm_s *dev,
                                   uint32_t offset, uint32_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = regval;
}

static inline uint32_t song_pdm_getreg(struct song_pdm_s *dev,
                                       uint32_t offset)
{
  return *(volatile uint32_t *)(dev->base + B2C(offset));
}

static inline void song_pdm_updatereg(struct song_pdm_s *dev,
                                      uint32_t offset, uint32_t mask,
                                      uint32_t regval)
{
  song_pdm_putreg(dev, offset, (regval & mask) |
                  (song_pdm_getreg(dev, offset) & ~mask));
}

static int song_pdm_channels(struct i2s_dev_s *dev_, uint8_t channels)
{
  if (channels != 2)
    return -EINVAL;
  return OK;
}

static uint32_t song_pdm_samplerate(struct i2s_dev_s *dev_, uint32_t rate)
{
  struct song_pdm_s *dev = (struct song_pdm_s *)dev_;
  unsigned int bclk_rate, low;

  if (dev->oversampling)
    bclk_rate = rate * dev->oversampling;
  else
    bclk_rate = rate * dev->data_width;

  clk_set_rate(dev->sclk, 2 * bclk_rate);

  low = song_pdm_getreg(dev, SONG_PDM_CFG);
  low  &= SONG_PDM_HIGH_MASK;
  low >>= SONG_PDM_HIGH_OFF;
  low <<= SONG_PDM_LOW_OFF;

  song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_LOW_MASK, low);

  return rate;
}

static uint32_t song_pdm_datawidth(struct i2s_dev_s *dev_, int bits)
{
  struct song_pdm_s *dev = (struct song_pdm_s *)dev_;
  uint32_t fsync_bit;

  switch (bits)
    {
      case 16:
        fsync_bit = SONG_PDM_WIDTH_16;
        break;
      case 32:
        fsync_bit = SONG_PDM_WIDTH_32;
        break;
      default:
        return -EINVAL;
    }
  dev->data_width = bits;

  song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_WIDTH_MASK, fsync_bit);

  return bits;
}

static int song_pdm_set_fmt(struct song_pdm_s *dev, uint16_t fmt)
{
  switch (fmt & AUDIO_HWFMT_FORMAT_MASK)
    {
      case AUDIO_HWFMT_PDM:
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_INV_MASK)
    {
      case AUDIO_HWFMT_NB_NF:
      case AUDIO_HWFMT_NB_IF:
        song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_CLK_INVERT, 0);
        break;
      case AUDIO_HWFMT_IB_NF:
      case AUDIO_HWFMT_IB_IF:
        song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_CLK_INVERT,
                           SONG_PDM_CLK_INVERT);
        break;
      default:
        return -EINVAL;
     }
  switch (fmt & AUDIO_HWFMT_MASTER_MASK)
    {
      case AUDIO_HWFMT_CBM_CFM:
      case AUDIO_HWFMT_CBM_CFS:
        song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_CLK_SLAVE,
                           SONG_PDM_CLK_SLAVE);
        break;
      case AUDIO_HWFMT_CBS_CFM:
      case AUDIO_HWFMT_CBS_CFS:
        song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_CLK_SLAVE, 0);
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int song_pdm_ioctl(struct i2s_dev_s *dev_, int cmd, unsigned long arg)
{
  struct song_pdm_s *dev = (struct song_pdm_s *) dev_;
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
        if (playback)
          song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_DATA_OUT,
                             SONG_PDM_DATA_OUT);
        else
          song_pdm_updatereg(dev, SONG_PDM_CFG, SONG_PDM_DATA_OUT, 0);
        song_pdm_updatereg(dev, SONG_PDM_CTL, SONG_PDM_ENABLE,
                           SONG_PDM_ENABLE);
        break;
      case AUDIOIOC_STOP:
        song_pdm_updatereg(dev, SONG_PDM_CTL, SONG_PDM_RESET,
                           SONG_PDM_RESET);
        song_pdm_updatereg(dev, SONG_PDM_CTL, SONG_PDM_RESET, 0);
          /* fall through */
      case AUDIOIOC_PAUSE:
        song_pdm_updatereg(dev, SONG_PDM_CTL, SONG_PDM_ENABLE, 0);
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
                    return song_pdm_set_fmt(dev, caps->ac_controls.hw[0]);
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

static const struct i2s_ops_s g_song_pdm_ops =
{
  .i2s_rxchannels    = song_pdm_channels,
  .i2s_rxsamplerate  = song_pdm_samplerate,
  .i2s_rxdatawidth   = song_pdm_datawidth,
  .i2s_txchannels    = song_pdm_channels,
  .i2s_txsamplerate  = song_pdm_samplerate,
  .i2s_txdatawidth   = song_pdm_datawidth,
  .i2s_ioctl         = song_pdm_ioctl,
};

static const struct song_pdm_cic g_song_pdm_cic[] =
{
  {.n = 31, .r =  64, .m = 0, .s = 15},
  {.n = 31, .r =  96, .m = 0, .s = 18},
  {.n = 31, .r = 192, .m = 0, .s = 23},
  {.n =  0, .r =   0, .m = 0, .s =  0},
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2s_dev_s *song_pdm_initialize(uintptr_t base, const char *mclk, uint8_t oversampling)
{
  struct song_pdm_s *dev;
  char sclk_name[32];
  int i;

  if (!mclk)
    return NULL;

  dev = kmm_zalloc(sizeof(struct song_pdm_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_song_pdm_ops;
  dev->base = base;
  dev->oversampling = oversampling;

  sprintf(sclk_name, "%s.sclk", mclk);
  dev->sclk = clk_register_divider(sclk_name, mclk, CLK_SET_RATE_PARENT,
                                   dev->base + SONG_PDM_CFG, 0, 8,
                                   (1 << CLK_DIVIDER_MINDIV_OFF) | CLK_DIVIDER_ROUND_CLOSEST |
                                   CLK_DIVIDER_ONE_BASED);
  if (!dev->sclk)
    {
      kmm_free(dev);
      return NULL;
    }

  if (oversampling)
    {
      for (i = 0; ; i++) {
        if (g_song_pdm_cic[i].r == 0)
          {
            kmm_free(dev);
            return NULL;
          }
        if (dev->oversampling != g_song_pdm_cic[i].r)
          continue;
        song_pdm_putreg(dev, SONG_PDM_CIC_CFG,
                        g_song_pdm_cic[i].n << SONG_PDM_CIC_N_OFF |
                        g_song_pdm_cic[i].r << SONG_PDM_CIC_R_OFF |
                        g_song_pdm_cic[i].m << SONG_PDM_CIC_M_OFF |
                        g_song_pdm_cic[i].s << SONG_PDM_CIC_SHIFT_OFF |
                        SONG_PDM_CIC_ENABLE);
        break;
      }
    }

  song_pdm_putreg(dev, SONG_PDM_LP_EN, 0x7f);
  return &dev->dev;
}
