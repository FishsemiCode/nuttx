/****************************************************************************
 * drivers/audio/dp_adc.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/audio/dp_adc.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DP_ADC_SR            0x00
#define DP_ADC_CR_VIC        0x01
#define DP_ADC_AICR_ADC      0x02
#define DP_ADC_AICR_SB_ADC   0x03
#define DP_ADC_AICR_ADC2     0x04
#define DP_ADC_AICR_SB_ADC2  0x05
#define DP_ADC_CR_LTC12      0x06
#define DP_ADC_CR_LTC3       0x07
#define DP_ADC_FCR_ADC       0x08
#define DP_ADC_FCR_ADC2      0x09
#define DP_ADC_CR_DMIC12     0x0a
#define DP_ADC_CR_DMIC3      0x0b
#define DP_ADC_CR_MIC1       0x0c
#define DP_ADC_CR_MIC2       0x0d
#define DP_ADC_CR_MIC3       0x0e
#define DP_ADC_GCR_MIC12     0x0f
#define DP_ADC_GCR_MIC3      0x10
#define DP_ADC_CR_ADC12      0x11
#define DP_ADC_CR_ADC3       0x12
#define DP_ADC_GCR_ADC1      0x13
#define DP_ADC_GCR_ADC2      0x14
#define DP_ADC_GCR_ADC3      0x15
#define DP_ADC_TR_DIG0       0x16
#define DP_ADC_ANA0          0x17
#define DP_ADC_ANA1          0x18
#define DP_ADC_ANA2          0x19
#define DP_ADC_SR_TR_ANA0    0x1a

#define PON_ACK              0x80
#define ADC3_SMUTE_IN        0x02
#define ADC12_SMUTE_IN       0x01

#define ANALOG_SLEEP         0x04
#define ANALOG_SB            0x02
#define DIG_SB               0x01

#define ADC_ADWL_MASK        0xc0
#define ADC_ADWL_16          0x00
#define ADC_ADWL_18          0x40
#define ADC_ADWL_20          0x80
#define ADC_ADWL_24          0xc0
#define ADC_SLAVE            0x20
#define ADC_I2S_RIGHT        0x00
#define ADC_I2S_LEFT         0x04
#define ADC_AUDIOIF_MASK     0x03
#define ADC_AUDIOIF_PARA     0x00
#define ADC_AUDIOIF_DSP      0x02
#define ADC_AUDIOIF_I2S      0x03

#define SB_AICR_ADC          0x01

#define ADC_LTC_CTRL_MASK    0x1f

#define ADC_HPF_EN           0x10
#define ADC_FREQ_MASK        0x0f
#define ADC_FREQ_16K         0x03
#define ADC_FREQ_48K         0x08
#define ADC_FREQ_96K         0x0a
#define ADC_FREQ_192K        0x0c
#define ADC_FREQ_384K        0x0d
#define ADC_FREQ_768K        0x0e

#define MICBIAS_V            0x80
#define MICDIFF              0x40
#define SB_MICBIAS           0x20
#define CAP_CP               0x01

#define GIM2_MASK            0xf0
#define GIM2_SHIFT           4
#define GIM1_MASK            0x0f

#define GIM3_MASK            0x0f

#define ADC12_SOFT_MUTE      0x80
#define SB_ADC2              0x20
#define SB_ADC1              0x10
#define ADC12_LP_MODE        0x08

#define ADC3_SOFT_MUTE       0x80
#define SB_ADC3              0x10
#define ADC3_LP_MODE         0x08

#define GID_MASK             0x7f

#define FAST_ON              0x01

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dp_adc_s
{
  struct audio_lowerhalf_s dev;
  struct clk *mclk;
  uint32_t base;
  uint32_t anc_rate;
  uint8_t idx;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_adc_configure(struct audio_lowerhalf_s *dev_,
                            void *session,
                            const struct audio_caps_s *caps);
static int dp_adc_start(struct audio_lowerhalf_s *dev_,
                        void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int dp_adc_stop(struct audio_lowerhalf_s *dev_,
                       void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int dp_adc_pause(struct audio_lowerhalf_s *dev_,
                        void *session);
static int dp_adc_resume(struct audio_lowerhalf_s *dev_,
                         void *session);
#endif
#else
static int dp_adc_configure(struct audio_lowerhalf_s *dev_,
                            const struct audio_caps_s *caps);
static int dp_adc_start(struct audio_lowerhalf_s *dev_);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int dp_adc_stop(struct audio_lowerhalf_s *dev_);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int dp_adc_pause(struct audio_lowerhalf_s *dev_);
static int dp_adc_resume(struct audio_lowerhalf_s *dev_);
#endif
#endif
static int dp_adc_channels(struct dp_adc_s *dev_,
                           uint8_t channels);
static uint32_t dp_adc_samplerate(struct dp_adc_s *dev_,
                                  uint32_t rate);
static uint32_t dp_adc_datawidth(struct dp_adc_s *dev_,
                                 int bits);
static int dp_adc_set_fmt(struct dp_adc_s *dev_,
                          uint16_t fmt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_dp_adc_ops =
{
  .configure = dp_adc_configure,
  .start = dp_adc_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop = dp_adc_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause = dp_adc_pause,
  .resume = dp_adc_resume,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void dp_adc_putreg(struct dp_adc_s *dev,
                                 uint8_t offset, uint8_t regval)
{
  uint32_t regaddr = dev->base + B2C(offset);
  *(volatile uint8_t *)(regaddr) = regval;
}

static inline uint8_t dp_adc_getreg(struct dp_adc_s *dev,
                                     uint8_t offset)
{
  return *(volatile uint8_t *)(dev->base + B2C(offset));
}

static inline void dp_adc_updatereg(struct dp_adc_s *dev,
                                    uint8_t offset, uint8_t mask,
                                    uint8_t regval)
{
  dp_adc_putreg(dev, offset, (regval & mask) |
                ((dp_adc_getreg(dev, offset) & ~mask)));
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_adc_configure(struct audio_lowerhalf_s *dev_,
                            void *session,
                            const struct audio_caps_s *caps)
#else
static int dp_adc_configure(struct audio_lowerhalf_s *dev_,
                            const struct audio_caps_s *caps)
#endif
{
  struct dp_adc_s *dev = (struct dp_adc_s *)dev_;
  int samprate, nchannels, bpsamp;
  int ret = OK;

  DEBUGASSERT(dp_adc && caps);
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

        ret = dp_adc_channels(dev, nchannels);
        if (ret < 0)
          return -EINVAL;
        ret = dp_adc_datawidth(dev, bpsamp);
        if (ret < 0)
          return ret;
        if (dev->anc_rate)
          return OK;
        return dp_adc_samplerate(dev, samprate);
      case AUDIO_TYPE_EXTENSION:
        switch(caps->ac_format.hw)
          {
            case AUDIO_EU_HW_FORMAT:
              return dp_adc_set_fmt(dev, caps->ac_controls.hw[0]);
            default:
              return -ENOTTY;
           }
      default:
        return -ENOTTY;
    }
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_adc_start(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_adc_start(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_adc_s *dev = (struct dp_adc_s *)dev_;

  dp_adc_putreg(dev, DP_ADC_CR_VIC, ANALOG_SLEEP);
  clk_enable(dev->mclk);
  switch(dev->idx)
    {
      case 0:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC12, SB_ADC1, 0);
        dp_adc_updatereg(dev, DP_ADC_CR_MIC1, SB_MICBIAS, 0);
        break;
      case 1:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC12, SB_ADC2, 0);
        dp_adc_updatereg(dev, DP_ADC_CR_MIC2, SB_MICBIAS, 0);
        break;
      case 2:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC3, SB_ADC3, 0);
        dp_adc_updatereg(dev, DP_ADC_CR_MIC3, SB_MICBIAS, 0);
    }

  dp_adc_putreg(dev, DP_ADC_CR_VIC, 0);

  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_adc_stop(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_adc_stop(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_adc_s *dev = (struct dp_adc_s *)dev_;

  switch(dev->idx)
    {
      case 0:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC12, SB_ADC1, SB_ADC1);
        dp_adc_updatereg(dev, DP_ADC_CR_MIC1, SB_MICBIAS, SB_MICBIAS);
        break;
      case 1:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC12, SB_ADC2, SB_ADC2);
        dp_adc_updatereg(dev, DP_ADC_CR_MIC2, SB_MICBIAS, SB_MICBIAS);
        break;
      case 2:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC3, SB_ADC3, SB_ADC3);
        dp_adc_updatereg(dev, DP_ADC_CR_MIC3, SB_MICBIAS, SB_MICBIAS);
        break;
    }

  clk_disable(dev->mclk);
  if (!clk_is_enabled(dev->mclk))
    dp_adc_putreg(dev, DP_ADC_CR_VIC,
                  ANALOG_SLEEP | ANALOG_SB | DIG_SB);

  return OK;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_adc_pause(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_adc_pause(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_adc_s *dev = (struct dp_adc_s *)dev_;

  switch (dev->idx)
    {
      case 0:
        dp_adc_updatereg(dev, DP_ADC_CR_MIC1, SB_MICBIAS, SB_MICBIAS);
        break;
      case 1:
        dp_adc_updatereg(dev, DP_ADC_CR_MIC2, SB_MICBIAS, SB_MICBIAS);
        break;
      case 2:
        dp_adc_updatereg(dev, DP_ADC_CR_MIC3, SB_MICBIAS, SB_MICBIAS);
        break;
    }

  clk_disable(dev->mclk);
  if (!clk_is_enabled(dev->mclk))
    dp_adc_putreg(dev, DP_ADC_CR_VIC, ANALOG_SLEEP | DIG_SB);

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dp_adc_resume(struct audio_lowerhalf_s *dev_, void *session)
#else
static int dp_adc_resume(struct audio_lowerhalf_s *dev_)
#endif
{
  struct dp_adc_s *dev = (struct dp_adc_s *)dev_;

  clk_enable(dev->mclk);
  switch(dev->idx)
    {
      case 0:
        dp_adc_updatereg(dev, DP_ADC_CR_MIC1, SB_MICBIAS, 0);
        break;
      case 1:
        dp_adc_updatereg(dev, DP_ADC_CR_MIC2, SB_MICBIAS, 0);
        break;
      case 2:
        dp_adc_updatereg(dev, DP_ADC_CR_MIC3, SB_MICBIAS, 0);
        break;
   }

  dp_adc_putreg(dev, DP_ADC_CR_VIC, 0);

  return OK;
}
#endif

static int dp_adc_channels(struct dp_adc_s *dev,
                           uint8_t channels)
{
  if (!channels || channels > 2)
    return -EINVAL;

  return OK;
}

static uint32_t dp_adc_datawidth(struct dp_adc_s *dev, int bits)
{
  switch (bits)
    {
      case 16:
        if (dev->idx < 2)
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC,
                           ADC_ADWL_MASK, ADC_ADWL_16);
        else
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC2,
                           ADC_ADWL_MASK, ADC_ADWL_16);
        break;
      case 18:
        if (dev->idx < 2)
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC,
                           ADC_ADWL_MASK, ADC_ADWL_18);
        else
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC2,
                           ADC_ADWL_MASK, ADC_ADWL_18);
        break;
      case 20:
        if (dev->idx < 2)
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC,
                           ADC_ADWL_MASK, ADC_ADWL_20);
        else
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC2,
                           ADC_ADWL_MASK, ADC_ADWL_20);
        break;
      case 24:
        if (dev->idx < 2)
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC,
                           ADC_ADWL_MASK, ADC_ADWL_24);
        else
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC2,
                           ADC_ADWL_MASK, ADC_ADWL_24);
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static uint32_t dp_adc_samplerate(struct dp_adc_s *dev,
                                  uint32_t rate)
{
  switch (rate)
    {
      case 16000:
        if (dev->idx < 2)
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_16K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC12,
                             ADC12_LP_MODE, ADC12_LP_MODE);
          }
        else
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC2,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_16K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC3,
                             ADC3_LP_MODE, ADC3_LP_MODE);
          }
        break;
      case 48000:
        if (dev->idx < 2)
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_48K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC12,
                             ADC12_LP_MODE, 0);
          }
        else
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC2,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_48K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC3,
                             ADC3_LP_MODE, 0);
          }
        break;
      case 96000:
        if (dev->idx < 2)
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_96K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC12,
                             ADC12_LP_MODE, 0);
          }
        else
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC2,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_96K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC3,
                             ADC3_LP_MODE, 0);
          }
        break;
      case 192000:
       if (dev->idx < 2)
         {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_192K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC12,
                             ADC12_LP_MODE, 0);
         }
       else
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC2,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_192K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC3,
                             ADC3_LP_MODE, 0);
          }
        break;
      case 384000:
        if (dev->idx < 2)
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_384K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC12,
                             ADC12_LP_MODE, 0);
          }
        else
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC2,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_384K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC3,
                             ADC3_LP_MODE, 0);
          }
        break;
      case 768000:
        if (dev->idx < 2)
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_768K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC12,
                             ADC12_LP_MODE, 0);
          }
        else
          {
            dp_adc_updatereg(dev, DP_ADC_FCR_ADC2,
                             ADC_FREQ_MASK | ADC_HPF_EN,
                             ADC_HPF_EN | ADC_FREQ_768K);
            dp_adc_updatereg(dev, DP_ADC_CR_ADC3,
                             ADC3_LP_MODE, 0);
          }
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int dp_adc_set_fmt(struct dp_adc_s *dev, uint16_t fmt)
{
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
        if (dev->idx < 2)
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC,
                           ADC_SLAVE, ADC_SLAVE);
        else
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC2,
                           ADC_SLAVE, ADC_SLAVE);
        break;
      case AUDIO_HWFMT_CBM_CFM:
        if (dev->idx < 2)
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC,
                           ADC_SLAVE, 0);
        else
          dp_adc_updatereg(dev, DP_ADC_AICR_ADC2,
                           ADC_SLAVE, 0);
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *dp_adc_initialize(const char *mclk, uint32_t base,
                                            uint32_t anc_rate, uint8_t idx)
{
  struct dp_adc_s *dev;

  if (idx > 2)
    return NULL;

  dev = kmm_zalloc(sizeof(struct dp_adc_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_dp_adc_ops;
  dev->base = base;
  dev->anc_rate = anc_rate;
  dev->idx = idx;
  dev->mclk = clk_get(mclk);
  if (!dev->mclk)
    {
      kmm_free(dev);
      return NULL;
    }

  while(!(dp_adc_getreg(dev, DP_ADC_SR) & PON_ACK));

  switch (dev->idx)
    {
      case 0:
      case 1:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC12, ADC12_SOFT_MUTE, 0x00);
        break;
      case 2:
        dp_adc_updatereg(dev, DP_ADC_CR_ADC3, ADC3_SOFT_MUTE, 0x00);
        break;
    }

  if (dev->anc_rate && dp_adc_samplerate(dev, dev->anc_rate) < 0)
    {
      kmm_free(dev);
      return NULL;
    }

  return &dev->dev;
}
