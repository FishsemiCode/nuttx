/****************************************************************************
 * drivers/leds/fishled.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@fishsemi.com>
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

#include <nuttx/kmalloc.h>
#include <nuttx/leds/fishled.h>
#include <nuttx/pinctrl/pinctrl.h>
#include <nuttx/ioexpander/ioexpander.h>

#include <errno.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fishled_dev_s
{
  struct ioexpander_dev_s *ioe; /* IOE interface */
  FAR const struct led_config_s *config;
  uint8_t count;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static int fishled_open(FAR struct file *filep);
static int fishled_close(FAR struct file *filep);
static int fishled_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct file_operations g_fishled_ops =
{
  fishled_open,    /* open */
  fishled_close,   /* close */
  NULL,             /* read */
  NULL,             /* write */
  NULL,             /* seek */
  fishled_ioctl,   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL              /* unlink */
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int fishled_open(FAR struct file *filep)
{
  return 0;
}

static int fishled_close(FAR struct file *filep)
{
  return 0;
}

static int fishled_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fishled_dev_s *priv = inode->i_private;
  FAR const struct led_config_s *config = priv->config;
  uint8_t number = priv->count;
  int ret = OK;

  switch (cmd)
    {
      case FISHLED_SET:
        for (uint8_t i = 0; i < number; i++)
          {
            PINCTRL_SELGPIO(g_pinctrl[0], config[i].gpio_port);
            IOEXP_SETDIRECTION(priv->ioe, config[i].gpio_port, IOEXPANDER_DIRECTION_OUT);
            if (arg)
              IOEXP_WRITEPIN(priv->ioe, config[i].gpio_port, config[i].level);
            else
              IOEXP_WRITEPIN(priv->ioe, config[i].gpio_port, !config[i].level);
          }
        break;
      case FISHLED_GET:
        for (uint8_t i = 0; i < number; i++)
          {
            IOEXP_READPIN(priv->ioe, config[i].gpio_port, (bool *)arg);
          }
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int fishled_initialize(FAR struct ioexpander_dev_s *ioe, const struct led_config_s *config, uint8_t count)
{
  struct fishled_dev_s *priv;
  priv = kmm_zalloc(sizeof(struct fishled_dev_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  priv->ioe = ioe;
  priv->config = config;
  priv->count = count;

  return register_driver("/dev/fishled", &g_fishled_ops, 0666, priv);
}
