/****************************************************************************
 * arch/drivers/analog/song_adc.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: Yuan Zhang<zhangyuan@fishsemi.com>
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

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/song_adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_ADC_PGA_GAIN           0
#define SONG_ADC_ADC_BIAS           2
#define SONG_ADC_PGA_BIAS           3
#define SONG_ADC_MOD_SEL            4
#define SONG_ADC_OFFSET_TEST        5
#define SONG_ADC_PORT_SEL           11
#define SONG_ADC_TEST_SEL           19
#define SONG_ADC_START              20
#define SONG_ADC_DIV                21
#define SONG_ADC_TS_TRIM            23
#define SONG_ADC_TS_PD              28
#define SONG_ADC_ADC_PD             29
#define SONG_ADC_PGA_PD             30

#define SONG_ADC_CLK                5

#define SONG_ADC_EOC_OUT            (1 << 11)
#define SONG_ADC_OUT                (1 << 0)
#define SONG_ADC_OUT_DIGITS         11
#define SONG_ADC_MAX_OUT            (1 << SONG_ADC_OUT_DIGITS)
#define SONG_ADC_OUT_MASK           (SONG_ADC_MAX_OUT - 1)

#define SONG_ADC_PORT_MASK          0x0007f800
#define SONG_ADC_VBAT_PORT(port)    ((1 << port) << SONG_ADC_PORT_SEL)

#define SONG_ADC_PGA_GAIN_MASK      0x00000003
#define SONG_ADC_VBAT_GAIN          (0 << SONG_ADC_PGA_GAIN)

#define SONG_ADC_VBAT_DEFAULT_K     100
#define SONG_ADC_VBAT_DEFAULT_B     0

#define SONG_ADC_VBAT_TIMEOUT       50
#define SONG_ADC_VBAT_PER_INTERVAL  10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_adc_dev_s
{
  FAR const struct song_adc_config_s *cfg;
  FAR struct song_adc_vbat_cal_s vbat_calibrate;
  sem_t sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  song_adc_bind(FAR struct adc_dev_s *dev,
                          FAR const struct adc_callback_s *callback);
static void song_adc_reset(FAR struct adc_dev_s *dev);
static int  song_adc_setup(FAR struct adc_dev_s *dev);
static void song_adc_shutdown(FAR struct adc_dev_s *dev);
static void song_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  song_adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = song_adc_bind,      /* ao_bind */
  .ao_reset    = song_adc_reset,     /* ao_reset */
  .ao_setup    = song_adc_setup,     /* ao_setup */
  .ao_shutdown = song_adc_shutdown,  /* ao_shutdown */
  .ao_rxint    = song_adc_rxint,     /* ao_rxint */
  .ao_ioctl    = song_adc_ioctl      /* ao_read */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void song_adc_write(struct song_adc_dev_s *dev, uint32_t offset,
                           uint32_t regval)
{
  uint32_t regaddr = dev->cfg->base + B2C(offset);
  *(volatile uint32_t *)(regaddr) = (regval);
}

static uint32_t song_adc_read(struct song_adc_dev_s *dev,
                              uint32_t offset)
{
  uint32_t regaddr = dev->cfg->base + B2C(offset);
  return *(volatile uint32_t *)(regaddr);
}

static void song_adc_modify(struct song_adc_dev_s *dev,
                            uint32_t offset, uint32_t clearbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t regval;

  flags   = enter_critical_section();
  regval  = song_adc_read(dev, offset);
  regval &= ~clearbits;
  regval |= setbits;
  song_adc_write(dev, offset, regval);
  leave_critical_section(flags);
}

static void song_adc_update_bits(struct song_adc_dev_s *dev,
                                 uint32_t offset, uint32_t mask, uint32_t regval)
{
  song_adc_write(dev, offset, (regval & mask) |
                  (song_adc_read(dev, offset) & ~mask));
}

static uint32_t song_adc_read_result(struct song_adc_dev_s *dev)
{
  uint32_t adcval;
  uint32_t cnt;

  for (cnt = 0; cnt < SONG_ADC_VBAT_TIMEOUT; cnt += SONG_ADC_VBAT_PER_INTERVAL)
    {
      up_udelay(SONG_ADC_VBAT_PER_INTERVAL);
      adcval = song_adc_read(dev, dev->cfg->sta_off);

      if (adcval & SONG_ADC_EOC_OUT)
        break;
    }

  return adcval & SONG_ADC_OUT_MASK;
}

static int32_t song_adc_vbat_convert(struct song_adc_dev_s *dev, uint32_t val)
{
  int32_t convert_vbat;

  convert_vbat = dev->vbat_calibrate.k * val + dev->vbat_calibrate.b;

  return (convert_vbat > 0) ? (convert_vbat / 100) : 0;
}

static int32_t song_get_vbat_value(struct song_adc_dev_s *dev, uint32_t port, bool convert)
{
  FAR uint32_t data;
  irqstate_t flags;

  flags = enter_critical_section();

  song_adc_modify(dev, dev->cfg->clk_off, 1 << SONG_ADC_CLK, 0);

  song_adc_modify(dev, dev->cfg->ctl_off, \
                  (1 << SONG_ADC_TS_PD) | (1 << SONG_ADC_ADC_PD) | \
                  (1 << SONG_ADC_PGA_PD) | (1 << SONG_ADC_MOD_SEL), 0);

  song_adc_update_bits(dev, dev->cfg->ctl_off, \
                       SONG_ADC_PORT_MASK, SONG_ADC_VBAT_PORT(port));
  song_adc_update_bits(dev, dev->cfg->ctl_off, \
                       SONG_ADC_PGA_GAIN_MASK, SONG_ADC_VBAT_GAIN);
  song_adc_modify(dev, dev->cfg->ctl_off, 0, 1 << SONG_ADC_START);

  data = song_adc_read_result(dev);

  song_adc_modify(dev, dev->cfg->ctl_off, 1 << SONG_ADC_START, 0);

  song_adc_modify(dev, dev->cfg->ctl_off, 0, \
                  (1 << SONG_ADC_TS_PD) | (1 << SONG_ADC_ADC_PD) | \
                  (1 << SONG_ADC_PGA_PD));

  song_adc_modify(dev, dev->cfg->clk_off, 0, 1 << SONG_ADC_CLK);

  leave_critical_section(flags);

  return convert ? song_adc_vbat_convert(dev, data) : data;
}

static int song_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback)
{
  return OK;
}

static void song_adc_reset(FAR struct adc_dev_s *dev)
{
}

static int song_adc_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

static void song_adc_shutdown(FAR struct adc_dev_s *dev)
{
}

static void song_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

static int song_adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct song_adc_dev_s *priv = (FAR struct song_adc_dev_s *)dev->ad_priv;
  int ret = OK;

  if (cmd == ANIOC_SONG_ADC_VBAT)
    {
      while (nxsem_wait(&priv->sem) < 0);

      ret = song_get_vbat_value(priv, (uint32_t)arg, true);

      nxsem_post(&priv->sem);
    }
  else if (cmd == ANIOC_SONG_ADC_VBAT_CALIBRATE)
    {
       while (nxsem_wait(&priv->sem) < 0);

       memcpy(&priv->vbat_calibrate, (void *)arg, sizeof(FAR struct song_adc_vbat_cal_s));

       nxsem_post(&priv->sem);
    }
  else if (cmd == ANIOC_SONG_ADC_VALUE)
    {
       while (nxsem_wait(&priv->sem) < 0);

       ret = song_get_vbat_value(priv, (uint32_t)arg, false);

       nxsem_post(&priv->sem);
    }
  else
    {
      aerr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int song_adc_register(FAR const struct song_adc_config_s *cfg)
{
  FAR struct song_adc_dev_s *adcpriv;
  FAR struct adc_dev_s *adcdev;
  char devname[16];
  int ret;

  adcpriv = (FAR struct song_adc_dev_s *)kmm_malloc(sizeof(struct song_adc_dev_s));
  if (adcpriv == NULL)
    {
      aerr("ERROR: Failed to allocate song adc instance\n");
      return -ENOMEM;
    }

  adcpriv->cfg = cfg;
  adcpriv->vbat_calibrate.k = SONG_ADC_VBAT_DEFAULT_K;
  adcpriv->vbat_calibrate.b = SONG_ADC_VBAT_DEFAULT_B;

  ret = nxsem_init(&adcpriv->sem, 1, 1);
  if (ret < 0)
    {
      kmm_free(adcpriv);
      return ret;
    }

  adcdev = (FAR struct adc_dev_s *)kmm_zalloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      aerr("ERROR: Failed to allocate adc_dev_s instance\n");
      nxsem_destroy(&adcpriv->sem);
      kmm_free(adcpriv);
      return -ENOMEM;
    }

  adcdev->ad_ops = &g_adcops;
  adcdev->ad_priv = adcpriv;

  sprintf(devname, "/dev/adc%d", cfg->minor);
  ret = adc_register(devname, adcdev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register adc driver: %d\n", ret);
      kmm_free(adcdev);
      nxsem_destroy(&adcpriv->sem);
      kmm_free(adcpriv);
    }

  return ret;
}
