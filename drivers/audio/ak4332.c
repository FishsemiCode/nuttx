/****************************************************************************
 * drivers/audio/ak4332.c
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

#include <nuttx/audio/ak4332.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AK4332_PWR1                       0x00
#define AK4332_PWR2                       0x01
#define AK4332_PWR3                       0x02
#define AK4332_PWR4                       0x03
#define AK4332_CLK_MODE                   0x05
#define AK4332_PDM_CTL                    0x08
#define AK4332_DAC_CLK_SRC                0x13
#define AK4332_CODEC_FMT                  0x15
#define AK4332_DAC_ADJ1                   0x26
#define AK4332_DAC_ADJ2                   0x27

#define AK4332_PWR_PMTIM                  0x02

#define AK4332_PWR_PMCP1                  0x01
#define AK4332_PWR_PMCP2                  0x02
#define AK4332_PWR_PMLDO1P                0x10
#define AK4332_PWR_PMLDO1N                0x20

#define AK4332_PWR_PMDA                   0x01

#define AK4332_PWR_PMHP                   0x01
#define AK4332_PWR_LVDTM_MASK             0x70
#define AK4332_PWR_LVDTM_64               0x00
#define AK4332_PWR_LVDTM_128              0x10
#define AK4332_PWR_LVDTM_256              0x20
#define AK4332_PWR_LVDTM_512              0x30
#define AK4332_PWR_LVDTM_1024             0x40
#define AK4332_PWR_LVDTM_2048             0x50
#define AK4332_PWR_LVDTM_4096             0x60
#define AK4332_PWR_LVDTM_8192             0x70

#define AK4332_CLK_FS_48K                 0x0a
#define AK4332_CLK_FS_MASK                0x1f
#define AK4332_CLK_CM_256FS               0x00
#define AK4332_CLK_CM_MASK                0x60

#define AK4332_PDM_CTL_PDM                0x01
#define AK4332_PDM_CTL_FMT_MASK           0x01
#define AK4332_PDM_CTL_PDMMODE1           0x00
#define AK4332_PDM_CTL_PDMMODE_MASK       0x06
#define AK4332_PDM_CTL_PDMCKR             0x40

#define AK4332_DAC_MCLK_SRC_MCLK          0x00
#define AK4332_DAC_MCLK_SRC_MASK          0x01

#define AK4332_CODEC_FMT_MS               0x10
#define AK4332_CODEC_DEVICEID_MASK        0xe0

#define AK4332_I2C_FREQ                   400000
#define AK4332_I2C_ADDR                   0x10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ak4332_s
{
  struct audio_lowerhalf_s dev;
  struct i2c_master_s *i2c;
  int enable_cnt;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ak4332_configure(struct audio_lowerhalf_s *dev_,
                            void *session,
                            const struct audio_caps_s *caps);
static int ak4332_start(struct audio_lowerhalf_s *dev,
                        void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int ak4332_stop(struct audio_lowerhalf_s *dev_,
                       void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int ak4332_pause(struct audio_lowerhalf_s *dev_,
                        void *session);
static int ak4332_resume(struct audio_lowerhalf_s *dev_,
                         void *session);
#endif
#else
static int ak4332_configure(struct audio_lowerhalf_s *dev_,
                            const struct audio_caps_s *caps);
static int ak4332_start(struct audio_lowerhalf_s *dev_);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int ak4332_stop(struct audio_lowerhalf_s *dev_);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int ak4332_pause(struct audio_lowerhalf_s *dev_);
static int ak4332_resume(struct audio_lowerhalf_s *dev_);
#endif
#endif

static uint32_t ak4332_samplerate(struct ak4332_s *dev, uint32_t rate);
static int ak4332_set_fmt(struct ak4332_s *dev, uint16_t fmt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_ak4332_ops =
{
  .configure = ak4332_configure,
  .start = ak4332_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop = ak4332_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause = ak4332_pause,
  .resume = ak4332_resume,
#endif
};

static const struct i2c_config_s g_ak4332_i2c_config =
{
  .frequency = AK4332_I2C_FREQ,
  .address = AK4332_I2C_ADDR,
  .addrlen = 7,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void ak4332_putreg(struct ak4332_s *dev,
                                 uint8_t offset, uint8_t regval)
{
  uint8_t data[] = { offset,regval };
  i2c_write(dev->i2c, &g_ak4332_i2c_config, data, 2);
}

static inline uint8_t ak4332_getreg(struct ak4332_s *dev, uint8_t offset)
{
  uint8_t regval = 0;

  i2c_writeread(dev->i2c, &g_ak4332_i2c_config, &offset, 1, &regval, 1);
  return regval;
}

static inline void ak4332_updatereg(struct ak4332_s *dev,
                                    uint8_t offset, uint8_t mask,
                                    uint8_t regval)
{
  uint8_t origin;

  origin = ak4332_getreg(dev, offset);
  regval &= mask;
  origin &= ~mask;
  regval |= origin;
  ak4332_putreg(dev, offset, regval);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ak4332_configure(struct audio_lowerhalf_s *dev_,
                            void *session,
                            const struct audio_caps_s *caps)
#else
static int ak4332_configure(struct audio_lowerhalf_s *dev_,
                            const struct audio_caps_s *caps)
#endif
{
  struct ak4332_s *dev = (struct ak4332_s *)dev_;
  int samprate, nchannels;

  DEBUGASSERT(ak4332 && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:

        /* Save the current stream configuration */

        samprate  = caps->ac_controls.hw[0] |
                    (caps->ac_controls.b[3] << 16);
        nchannels = caps->ac_channels;

        if (nchannels > 2 || !samprate)
          return -EINVAL;

        return ak4332_samplerate(dev, samprate);
      case AUDIO_TYPE_EXTENSION:
        switch(caps->ac_format.hw)
          {
            case AUDIO_EU_HW_FORMAT:
              return ak4332_set_fmt(dev, caps->ac_controls.hw[0]);
            default:
              return -ENOTTY;
           }
      default:
        return -ENOTTY;
    }
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ak4332_start(struct audio_lowerhalf_s *dev_, void *session)
#else
static int ak4332_start(struct audio_lowerhalf_s *dev_)
#endif
{
  struct ak4332_s *dev = (struct ak4332_s *)dev_;

  if (dev->enable_cnt++)
    return OK;

  ak4332_updatereg(dev, AK4332_PWR1, AK4332_PWR_PMTIM,
                   AK4332_PWR_PMTIM);
  ak4332_updatereg(dev, AK4332_PWR2, AK4332_PWR_PMCP1,
                   AK4332_PWR_PMCP1);
  usleep(6500);
  ak4332_updatereg(dev, AK4332_PWR2,
                   AK4332_PWR_PMLDO1P,
                   AK4332_PWR_PMLDO1P);
  usleep(500);
  ak4332_updatereg(dev, AK4332_PWR2,
                   AK4332_PWR_PMLDO1N,
                   AK4332_PWR_PMLDO1N);
  usleep(500);
  ak4332_updatereg(dev, AK4332_PWR3, AK4332_PWR_PMDA,
                   AK4332_PWR_PMDA);
  ak4332_updatereg(dev, AK4332_PWR2, AK4332_PWR_PMCP2,
                   AK4332_PWR_PMCP2);
  usleep(4500);
  ak4332_updatereg(dev, AK4332_PWR4, AK4332_PWR_PMHP,
                   AK4332_PWR_PMHP);
  return OK;
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ak4332_stop(struct audio_lowerhalf_s *dev_, void *session)
#else
static int ak4332_stop(struct audio_lowerhalf_s *dev_)
#endif
{
  struct ak4332_s *dev = (struct ak4332_s *)dev_;

  if (!dev->enable_cnt || --dev->enable_cnt)
    return OK;

  ak4332_updatereg(dev, AK4332_PWR4, AK4332_PWR_PMHP, 0);
  ak4332_updatereg(dev, AK4332_PWR2, AK4332_PWR_PMCP2, 0);
  ak4332_updatereg(dev, AK4332_PWR3, AK4332_PWR_PMDA, 0);
  ak4332_updatereg(dev, AK4332_PWR2,
                   AK4332_PWR_PMLDO1P | AK4332_PWR_PMLDO1N |
                   AK4332_PWR_PMCP1, 0);
  ak4332_updatereg(dev, AK4332_PWR1, AK4332_PWR_PMTIM, 0);

  return OK;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ak4332_pause(struct audio_lowerhalf_s *dev_, void *session)
#else
static int ak4332_pause(struct audio_lowerhalf_s *dev_)
#endif
{
  struct ak4332_s *dev = (struct ak4332_s *)dev_;

  if (!dev->enable_cnt || --dev->enable_cnt)
    return OK;

  ak4332_updatereg(dev, AK4332_PWR4,
                   AK4332_PWR_PMHP, 0);
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ak4332_resume(struct audio_lowerhalf_s *dev_, void *session)
#else
static int ak4332_resume(struct audio_lowerhalf_s *dev_)
#endif
{
  struct ak4332_s *dev = (struct ak4332_s *)dev_;

  if (dev->enable_cnt++)
    return OK;

  ak4332_updatereg(dev, AK4332_PWR4, AK4332_PWR_PMHP,
                   AK4332_PWR_PMHP);
  return OK;
}
#endif

static uint32_t ak4332_samplerate(struct ak4332_s *dev, uint32_t rate)
{
  ak4332_updatereg(dev, AK4332_CLK_MODE, AK4332_CLK_CM_MASK,
                   AK4332_CLK_CM_256FS);
  ak4332_updatereg(dev, AK4332_CLK_MODE, AK4332_CLK_FS_MASK,
                   AK4332_CLK_FS_48K);
  return rate;
}

static int ak4332_set_fmt(struct ak4332_s *dev, uint16_t fmt)
{
  switch (fmt & AUDIO_HWFMT_FORMAT_MASK)
    {
      case AUDIO_HWFMT_PDM:
        ak4332_updatereg(dev, AK4332_PDM_CTL, AK4332_PDM_CTL_FMT_MASK |
                         AK4332_PDM_CTL_PDMMODE_MASK,
                         AK4332_PDM_CTL_PDM | AK4332_PDM_CTL_PDMMODE1);
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_INV_MASK)
    {
      case AUDIO_HWFMT_NB_NF:
        ak4332_updatereg(dev, AK4332_PDM_CTL, AK4332_PDM_CTL_PDMCKR, 0);
        break;
      case AUDIO_HWFMT_IB_NF:
        ak4332_updatereg(dev, AK4332_PDM_CTL, AK4332_PDM_CTL_PDMCKR,
                         AK4332_PDM_CTL_PDMCKR);
        break;
      default:
        return -EINVAL;
    }
  switch (fmt & AUDIO_HWFMT_MASTER_MASK)
    {
      case AUDIO_HWFMT_CBS_CFS:
        ak4332_updatereg(dev, AK4332_CODEC_FMT, AK4332_CODEC_FMT_MS, 0);
        ak4332_updatereg(dev, AK4332_DAC_CLK_SRC, AK4332_DAC_MCLK_SRC_MASK,
                         AK4332_DAC_MCLK_SRC_MCLK);
        break;
      default:
        return -EINVAL;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *ak4332_initialize(struct i2c_master_s *i2c)
{
  struct ak4332_s *dev;

  dev = kmm_zalloc(sizeof(struct ak4332_s));
  if (!dev)
    return NULL;

  dev->dev.ops = &g_ak4332_ops;
  dev->i2c = i2c;

  if (ak4332_getreg(dev, AK4332_CODEC_FMT) & AK4332_CODEC_DEVICEID_MASK)
    {
       kmm_free(dev);
       return NULL;
    }

  ak4332_putreg(dev, AK4332_DAC_ADJ1, 0x02);
  ak4332_putreg(dev, AK4332_DAC_ADJ2, 0xc0);
  ak4332_set_fmt(dev, AUDIO_HWFMT_CBS_CFS | AUDIO_HWFMT_NB_NF | AUDIO_HWFMT_PDM);
  ak4332_updatereg(dev, AK4332_PWR4, AK4332_PWR_LVDTM_MASK, AK4332_PWR_LVDTM_128);
  return &dev->dev;
}
