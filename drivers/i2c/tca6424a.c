/****************************************************************************
 * drivers/i2c/tca6424a.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/tca6424a.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define COMMAND_INPUT_P0              0x00
#define COMMAND_INPUT_P1              0x01
#define COMMAND_INPUT_P2              0x02
#define COMMAND_OUTPUT_P0             0x04
#define COMMAND_OUTPUT_P1             0x05
#define COMMAND_OUTPUT_P2             0x06
#define COMMAND_CONFIG_P0             0x0C
#define COMMAND_CONFIG_P1             0x0D
#define COMMAND_CONFIG_P2             0x0E

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tca6424a_dev_s
{
  FAR struct i2c_master_s *master;
  mutex_t                 mutex;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_config_s config =
{
  .frequency = I2C_SPEED_FAST,
  .address   = 0x22,
  .addrlen   = 7,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     tca6424a_open(FAR struct file *filep);
static int     tca6424a_close(FAR struct file *filep);
static ssize_t tca6424a_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t tca6424a_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int     tca6424a_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

static int tca6424a_readreg(struct tca6424a_dev_s *priv, uint8_t cmd, uint8_t *val);
static int tca6424a_writereg(struct tca6424a_dev_s *priv, uint8_t cmd, uint8_t val);
static int tca6424a_updatereg(struct tca6424a_dev_s *priv, uint8_t cmd, uint8_t mask,
                              uint8_t val);
static int tca6424a_settype(struct tca6424a_dev_s *priv, uint32_t pin,
                            enum tca6424a_direction_e type);
static int tca6424a_gettype(struct tca6424a_dev_s *priv, uint32_t pin,
                            enum tca6424a_direction_e *type);
static int tca6424a_writepin(struct tca6424a_dev_s *priv, uint32_t pin, uint32_t state);
static int tca6424a_readpin(struct tca6424a_dev_s *priv, uint32_t pin, uint32_t *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_tca6424a_drvrops =
{
  tca6424a_open,  /* open */
  tca6424a_close, /* close */
  tca6424a_read,  /* read */
  tca6424a_write, /* write */
  NULL,       /* seek */
  tca6424a_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL      /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tca6424a_open(FAR struct file *filep)
{
  return OK;
}

static int tca6424a_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t tca6424a_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  return 0;
}

static ssize_t tca6424a_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return (ssize_t)buflen;
}

static int tca6424a_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct tca6424a_iotrans_s *trans;
  FAR struct tca6424a_dev_s *dev;
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  trans = (FAR struct tca6424a_iotrans_s *)((uintptr_t)arg);

  switch (cmd)
    {
      case TCA6424A_SETTYPE:
        {
          ret = tca6424a_settype(dev, trans->pin, trans->para.type);
        }
      break;

      case TCA6424A_READTYPE:
        {
          ret = tca6424a_gettype(dev, trans->pin, &trans->para.type);
        }
      break;

      case TCA6424A_WRITEPIN:
        {
          ret = tca6424a_writepin(dev, trans->pin, trans->para.state);
        }
      break;

      case TCA6424A_READPIN:
        {
          ret = tca6424a_readpin(dev, trans->pin, &trans->para.state);
        }
      break;

      default:
        ret = -ENOTTY;
      break;
    }

  return ret;
}

static int tca6424a_readreg(struct tca6424a_dev_s *priv, uint8_t cmd, uint8_t *val)
{
  return i2c_writeread(priv->master, &config, &cmd, 1, val, 1);
}

static int tca6424a_writereg(struct tca6424a_dev_s *priv, uint8_t cmd, uint8_t val)
{
  uint8_t data[] = { cmd, val };
  return i2c_write(priv->master, &config, data, 2);
}

static int tca6424a_updatereg(struct tca6424a_dev_s *priv, uint8_t cmd, uint8_t mask,
                              uint8_t val)
{
  uint8_t origin;
  int ret;

  nxmutex_lock(&priv->mutex);

  ret = tca6424a_readreg(priv, cmd, &origin);
  if (0 != ret)
    {
      goto end;
    }

  origin &= ~mask;
  val &= mask;
  val |= origin;

  ret = tca6424a_writereg(priv, cmd, val);
end:
  nxmutex_unlock(&priv->mutex);
  return ret;
}

static int tca6424a_settype(struct tca6424a_dev_s *priv, uint32_t pin,
                            enum tca6424a_direction_e type)
{
  uint8_t port, bit;

  port = (uint8_t)(pin / 10);
  bit = (uint8_t)((pin % 10) % 8);

  if (0 != port && 1 != port && 2 != port)
    {
      return -EINVAL;
    }

  return tca6424a_updatereg(priv, COMMAND_CONFIG_P0 + port,
           (uint8_t)(1 << bit), (uint8_t)(type << bit));
}

static int tca6424a_gettype(struct tca6424a_dev_s *priv, uint32_t pin,
                            enum tca6424a_direction_e *type)
{
  uint8_t port, bit;
  uint8_t regval;
  int ret;

  port = (uint8_t)(pin / 10);
  bit = (uint8_t)((pin % 10) % 8);

  if (0 != port && 1 != port && 2 != port)
    {
      return -EINVAL;
    }

  ret = tca6424a_readreg(priv, COMMAND_CONFIG_P0 + port, &regval);
  if (0 != ret)
    {
      return ret;
    }

  *type = ((regval & (1 << bit)) != 0);
  return OK;
}

static int tca6424a_writepin(struct tca6424a_dev_s *priv, uint32_t pin, uint32_t state)
{
  uint8_t port, bit;

  port = (uint8_t)(pin / 10);
  bit = (uint8_t)((pin % 10) % 8);

  if (0 != port && 1 != port && 2 != port)
    {
      return -EINVAL;
    }

  return tca6424a_updatereg(priv, COMMAND_OUTPUT_P0 + port,
           (uint8_t)(1 << bit), (uint8_t)((state & 0x1) << bit));
}

static int tca6424a_readpin(struct tca6424a_dev_s *priv, uint32_t pin, uint32_t *state)
{
  uint8_t port, bit;
  uint8_t regval;
  int ret;

  port = (uint8_t)(pin / 10);
  bit = (uint8_t)((pin % 10) % 8);

  if (0 != port && 1 != port && 2 != port)
    {
      return -EINVAL;
    }

  ret = tca6424a_readreg(priv, COMMAND_INPUT_P0 + port, &regval);
  if (0 != ret)
    {
      return ret;
    }

  *state = ((regval & (1 << bit)) != 0);
  return OK;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

int tca6424a_register(FAR struct i2c_master_s *i2c, int minor)
{
  FAR struct tca6424a_dev_s *priv;
  char devname[20];
  int ret;

  snprintf(devname, 20, "/dev/tca6424a%u", (unsigned int)minor);

  priv = (FAR struct tca6424a_dev_s *)kmm_zalloc(sizeof(struct tca6424a_dev_s));
  if (NULL == priv)
    {
      return -ENOMEM;
    }

  priv->master = i2c;
  ret = nxmutex_init(&priv->mutex);
  if (ret < 0)
    {
      goto mutex_fail;
    }

  ret = register_driver(devname, &g_tca6424a_drvrops, 0666, priv);
  if (ret < 0)
    {
      goto reg_fail;
    }

  return OK;

reg_fail:
  nxmutex_destroy(&priv->mutex);
mutex_fail:
  kmm_free(priv);
  return ret;
}
