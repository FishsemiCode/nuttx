/****************************************************************************
 * drivers/pinctrl/pinctrl.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin <zhuyanlin@pinecone.net>
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/pinctrl/pinctrl.h>

#ifdef CONFIG_PINCTRL

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pinctrl_open(FAR struct file *filep);
static int     pinctrl_close(FAR struct file *filep);
static ssize_t pinctrl_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t pinctrl_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     pinctrl_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pinctrl_drvrops =
{
  pinctrl_open,  /* open */
  pinctrl_close, /* close */
  pinctrl_read,  /* read */
  pinctrl_write, /* write */
  NULL,       /* seek */
  pinctrl_ioctl  /* ioctl */
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

/****************************************************************************
 * Name: pinctrl_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int pinctrl_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pinctrl_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int pinctrl_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: pinctrl_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t pinctrl_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: pinctrl_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t pinctrl_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: pinctrl_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int pinctrl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct pinctrl_dev_s *dev;
  FAR struct pinctrl_iotrans_s *trans;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private != NULL);
  dev = inode->i_private;

  switch (cmd)
    {

     /* Command:     PINCTRLC_SETFUNC
      * Description: Set the mux function of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
      */

      case PINCTRLC_SETFUNC:
        {
          trans = (FAR struct pinctrl_iotrans_s *)((uintptr_t)arg);
          ret = PINCTRL_SETFUNC(dev, trans->pin, trans->para.selector);
        }
      break;

     /* Command:     PINCTRLC_SETDS
      * Description: Set the driver strength of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
      */

      case PINCTRLC_SETDS:
        {
          trans = (FAR struct pinctrl_iotrans_s *)((uintptr_t)arg);
          ret = PINCTRL_SETDS(dev, trans->pin, trans->para.level);
        }
      break;

     /* Command:     PINCTRLC_SETDT
      * Description: Set the driver type of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
      */

      case PINCTRLC_SETDT:
        {
          trans = (FAR struct pinctrl_iotrans_s *)((uintptr_t)arg);
          ret = PINCTRL_SETDT(dev, trans->pin, trans->para.type);
        }
      break;

     /* Command:     PINCTRLC_SETSLEW
      * Description: Set slewrate of the pinctrl pin
      * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
      */

      case PINCTRLC_SETSLEW:
        {
          trans = (FAR struct pinctrl_iotrans_s *)((uintptr_t)arg);
          ret = PINCTRL_SETSLEW(dev, trans->pin, trans->para.state);
        }
      break;

     /* Command:     PINCTRLC_SELGPIO
      * Description: Select gpio function of pinctrl pin
      * Argument:    The uint32_t pinctrl number
      */

      case PINCTRLC_SELGPIO:
        {
          ret = PINCTRL_SELGPIO(dev, arg);
        }
      break;

      /* Unrecognized command */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinctrl_register
 *
 * Description:
 *   Register PINCTRL device driver.
 *
 ****************************************************************************/

int pinctrl_register(FAR struct pinctrl_dev_s *dev, int minor)
{
  char devname[16];

  snprintf(devname, 16, "/dev/pinctrl%u", (unsigned int)minor);
  return register_driver(devname, &g_pinctrl_drvrops, 0666, dev);
}

/****************************************************************************
 * Name: pinctrl_unregister
 *
 * Description:
 *   Unregister PINCTRL device driver.
 *
 ****************************************************************************/

void pinctrl_unregister(FAR struct pinctrl_dev_s *dev, int minor)
{
  char devname[16];

  snprintf(devname, 16, "/dev/pinctrl%u", (unsigned int)minor);
  (void)unregister_driver(devname);
}

#endif /* CONFIG_PINCTRL */

