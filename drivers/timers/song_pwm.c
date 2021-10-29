/****************************************************************************
 * drivers/timers/song_pwm.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/pwm/song_pwm.h>

#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_PWM_ENABLE                     B2C(0x00)
#define SONG_PWM_UPDATE                     B2C(0x04)
#define SONG_PWM_RESET                      B2C(0x08)
#define SONG_PWM_PERIOD                     B2C(0x0c)
#define SONG_PWM_OCCUPY                     B2C(0x10)
#define SONG_PWM_COUNT                      B2C(0x14)

#define SONG_PWM_BASE(b, c)                 ((b) + (c) * B2C(0x40))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the pwm lower-half driver */

struct song_pwm_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.
   */

  FAR const struct pwm_ops_s *ops;
  FAR struct clk *clk;
  uintptr_t  base;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int song_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int song_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int song_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                          FAR const struct pwm_info_s *info);
static int song_pwm_stop(FAR struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_song_pwm_ops =
{
  .setup    = song_pwm_setup,
  .shutdown = song_pwm_shutdown,
  .start    = song_pwm_start,
  .stop     = song_pwm_stop,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t song_pwm_getreg(uintptr_t base, uint32_t off)
{
  return *(FAR volatile uint32_t *)(base + off);
}

static inline void song_pwm_putreg(uintptr_t base, uint32_t off, uint32_t val)
{
  *(FAR volatile uint32_t *)(base + off) = val;
}

static inline void song_pwm_modifyreg(uintptr_t base, uint32_t off,
                                      uint32_t clearbits, uint32_t setbits)
{
  uint32_t temp;

  temp  = song_pwm_getreg(base, off);
  temp &= ~clearbits;
  temp |= setbits;
  song_pwm_putreg(base, off, temp);
}

static int song_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct song_pwm_lowerhalf_s *pwm = (FAR struct song_pwm_lowerhalf_s *)dev;
  return clk_enable(pwm->clk);
}

static int song_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct song_pwm_lowerhalf_s *pwm = (FAR struct song_pwm_lowerhalf_s *)dev;
  clk_disable(pwm->clk);
  return 0;
}

static int song_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                          FAR const struct pwm_info_s *info)
{
  FAR struct song_pwm_lowerhalf_s *pwm = (FAR struct song_pwm_lowerhalf_s *)dev;
  int ret;

  ret = clk_set_rate(pwm->clk, 200 * info->frequency);
  if (ret < 0)
    {
      return ret;
    }

  song_pwm_modifyreg(pwm->base, SONG_PWM_OCCUPY,
    0xffff, (100 * info->duty + 32768) / 65536);
  song_pwm_putreg(pwm->base, SONG_PWM_UPDATE, 1);
  song_pwm_putreg(pwm->base, SONG_PWM_ENABLE, 1);

  return 0;
}

static int song_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct song_pwm_lowerhalf_s *pwm = (FAR struct song_pwm_lowerhalf_s *)dev;
  song_pwm_putreg(pwm->base, SONG_PWM_ENABLE, 0);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int song_pwm_initialize(int minor, uintptr_t base, int count, FAR const char *mclk)
{
  int i;

  for (i = 0; i < count; i++)
    {
      FAR struct song_pwm_lowerhalf_s *pwm;
      char name[32];

      pwm = kmm_zalloc(sizeof(*pwm));
      if (pwm == NULL)
        {
          return -ENOMEM;
        }

      pwm->ops = &g_song_pwm_ops;
      pwm->base = SONG_PWM_BASE(base, i);

      sprintf(name, "%s.%d", mclk, minor + i);
      pwm->clk = clk_register_divider(name, mclk, CLK_SET_RATE_PARENT,
        pwm->base + SONG_PWM_PERIOD, 0, 8, CLK_DIVIDER_ROUND_CLOSEST);
      if (pwm->clk == NULL)
        {
          kmm_free(pwm);
          return -EINVAL;
        }

      /* Count the duty ratio by percent */
      song_pwm_putreg(pwm->base, SONG_PWM_COUNT, 100);

      sprintf(name, "/dev/pwm%d", minor + i);
      pwm_register(name, (FAR void *)pwm);
    }

   return 0;
}
