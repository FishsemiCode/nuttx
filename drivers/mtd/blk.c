/****************************************************************************
 * drivers/mtd/blk.c
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

#include <errno.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     blk_open(FAR struct inode *inode);
static int     blk_close(FAR struct inode *inode);
static ssize_t blk_read(FAR struct inode *inode, unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t blk_write(FAR struct inode *inode, const unsigned char *buffer,
                         size_t start_sector, unsigned int nsectors);
#endif
static int     blk_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     blk_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  blk_open,     /* open     */
  blk_close,    /* close    */
  blk_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  blk_write,    /* write    */
#else
  NULL,         /* write    */
#endif
  blk_geometry, /* geometry */
  blk_ioctl     /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0           /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: blk_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int blk_open(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: blk_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int blk_close(FAR struct inode *inode)
{
  return OK;
}

/****************************************************************************
 * Name: blk_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t blk_read(FAR struct inode *inode, unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  FAR struct mtd_dev_s *dev = inode->i_private;
  return MTD_BREAD(dev, start_sector, nsectors, buffer);
}

/****************************************************************************
 * Name: blk_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t blk_write(FAR struct inode *inode, const unsigned char *buffer,
                         size_t start_sector, unsigned int nsectors)
{
  FAR struct mtd_dev_s *dev = inode->i_private;
  return MTD_BWRITE(dev, start_sector, nsectors, buffer);
}
#endif

/****************************************************************************
 * Name: blk_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int blk_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct mtd_dev_s *dev = inode->i_private;
  struct mtd_geometry_s geo;
  int ret;

  if (geometry == NULL)
    {
      return -EINVAL;
    }

  ret = MTD_IOCTL(dev, MTDIOC_GEOMETRY, (uintptr_t)&geo);
  if (ret < 0)
    {
      return ret;
    }

  geometry->geo_available    = true;
  geometry->geo_mediachanged = false;
#ifdef CONFIG_FS_WRITABLE
  geometry->geo_writeenabled = true;
#else
  geometry->geo_writeenabled = false;
#endif
  geometry->geo_nsectors     = geo.neraseblocks * (geo.erasesize / geo.blocksize);
  geometry->geo_sectorsize   = geo.blocksize;

  return OK;
}

/****************************************************************************
 * Name: blk_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int blk_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct mtd_dev_s *dev = inode->i_private;

  /* Process the ioctl's we care about first, pass any we don't respond
   * to directly to the underlying MTD device.
   */

  switch (cmd)
    {
    case MTDIOC_ERASE:
      {
        FAR const struct mtd_erase_s *erase;

        erase = (FAR const struct mtd_erase_s *)arg;
        return MTD_ERASE(dev, erase->startblock, erase->nblocks);
      }

    case BIOC_XIPBASE:

      /* Just change the BIOC_XIPBASE command to the MTDIOC_XIPBASE command. */

      cmd = MTDIOC_XIPBASE;

      /* Go through */
    default:
      return MTD_IOCTL(dev, cmd, arg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: blk_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *   but without automatically erasing
 *
 * Input Parameters:
 *   minor - The minor device number. The MTD block device will be
 *      registered as /dev/mtdrawN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int blk_initialize(int minor, FAR struct mtd_dev_s *mtd)
{
  char devname[16];

  if (minor < 0 || !mtd)
    {
      return -EINVAL;
    }

  /* Create a MTD block device name */

  snprintf(devname, 16, "/dev/mtdraw%d", minor);

  /* Inode private data is a reference to the MTD device structure */

  return register_blockdriver(devname, &g_bops, 0, mtd);
}
