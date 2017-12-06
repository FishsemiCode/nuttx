/****************************************************************************
 * drivers/mbox/mbox.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

#include <nuttx/fs/fs.h>
#include <nuttx/mbox/mbox.h>

#include <errno.h>
#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_MBOX_DEV_NAME      "/dev/mbox"

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static int mbox_ioctl(struct file *filep, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct file_operations g_mbox_drvrops =
{
  NULL,         /* open */
  NULL,         /* close */
  NULL,         /* read */
  NULL,         /* write */
  NULL,         /* seek */
  mbox_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL        /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int mbox_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct mbox_dev_s *dev = inode->i_private;
  int ret = -ENOTTY;

  if (dev->ops && dev->ops->ioctl)
  {
    ret = dev->ops->ioctl(dev, cmd, arg);
  }

  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int mbox_register(struct mbox_dev_s *dev, int minor)
{
  char devname[16];

  snprintf(devname, 16, "%s%d", SONG_MBOX_DEV_NAME, minor);

  return register_driver(devname, &g_mbox_drvrops, 0666, dev);
}

int mbox_unregister(int minor)
{
  char devname[16];

  snprintf(devname, 16, "%s%d", SONG_MBOX_DEV_NAME, minor);

  return unregister_driver(devname);
}

