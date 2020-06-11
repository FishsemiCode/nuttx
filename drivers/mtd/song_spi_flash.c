/****************************************************************************
 * drivers/mtd/song_spi_flash.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: zhuyanlin<zhuyanlin@fishsemi.com>
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

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/song_spi_flash.h>

#include <ctype.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* flash controller Registers */

#define CTL                   0x000
#define MODE_CTL              0x004
#define CODE_DATA             0x008
#define OP_CTL                0x00C
#define CMD_ST_CTL            0x010
#define CTL_STATUS            0x014
#define RD_DATA_CTL           0x018
#define INT_EN                0x01C
#define INT_RAW               0x020
#define INT_STATUS            0x024
#define PG_FIFO               0x028

/* CTL register map */

#define CMD_CODE_LD           20
#define DUMMY_CTL             12
#define CON_LD                11
#define DUMMY_LD              8
#define FOUR_LINE_MODE        7
#define WLINE_NUM_CTL         4
#define AUTO_DS_EN            3
#define LINE_NUM_LD           0

#define CMD_CODE_LD_MSK       (0xff << CMD_CODE_LD)
#define CON_LD_MSK            (1 << CON_LD)
#define DUMMY_LD_MSK          (7 << DUMMY_LD)
#define FOUR_LINE_MODE_MSK    (1 << FOUR_LINE_MODE)
#define WLINE_NUM_CTL_MSK     (7 << WLINE_NUM_CTL)
#define LINE_NUM_LD_MSK       (7 << LINE_NUM_LD)

/* CODE_DATA register map */

#define CMD_CODE_CTL          24
#define WDATA_CTL             0

/* OP_CTL register map */

#define CMD_EN                (1 << 0)
#define WR_EN                 (1 << 1)
#define RD_EN                 (1 << 2)
#define WR1_EN                (1 << 3)
#define WR_NUM_CTL            4
#define RD_NUM_CTL            8
#define WR1_NUM_CTL           16
#define WAIT_STATUS_CTL       28

/* CMD_ST_CTL register map */

#define CMD_START_CTL         (1 << 0)

/* CTL_STATUS register map */

#define CTL_LOAD_READY        (1 << 11)
#define LD_DATA_FIFO_FULL     (1 << 10)
#define LD_DATA_FIFO_EMPTY    (1 << 9)
#define LD_CTL_FIFO_FULL      (1 << 8)
#define LD_CTL_FIFO_EMPTY     (1 << 7)
#define PG_DATA_FIFO_AF       (1 << 6)
#define PG_DATA_FIFO_FULL     (1 << 5)
#define PG_DATA_FIFO_EMPTY    (1 << 4)
#define CMD_STATE             (0)

/* PG FIFO write depth */
#define PG_FIFO_DEPTH         (16)

/* NOR flash command */

#define WREN                  (0x06 << CMD_CODE_CTL)   /* Write enable               */
#define WRDI                  (0x04 << CMD_CODE_CTL)   /* Write Disable              */
#define RDSR                  (0x05 << CMD_CODE_CTL)   /* Read status register       */
#define RDSR1                 (0x35 << CMD_CODE_CTL)   /* Read status register-1     */
#define WRSR                  (0x01 << CMD_CODE_CTL)   /* Write Status Register      */
#define RDDATA                (0x03 << CMD_CODE_CTL)   /* Read data bytes            */
#define FRD                   (0x0b << CMD_CODE_CTL)   /* Higher speed read          */
#define FRDD                  (0x3b << CMD_CODE_CTL)   /* Fast read, dual output     */
#define PP                    (0x02 << CMD_CODE_CTL)   /* Program page               */
#define QPP                   (0x32 << CMD_CODE_CTL)   /* Quad Program page          */
#define SE                    (0x20 << CMD_CODE_CTL)   /* Sector erase (4KB)         */
#define BE                    (0xd8 << CMD_CODE_CTL)   /* Block Erase (64KB)         */
#define CE                    (0xc7 << CMD_CODE_CTL)   /* Chip erase                 */
#define PD                    (0xb9 << CMD_CODE_CTL)   /* Power down                 */
#define PURDID                (0xab << CMD_CODE_CTL)   /* Release PD, Device ID      */
#define RDMFID                (0x90 << CMD_CODE_CTL)   /* Read Manufacturer / Device */
#define RDID                  (0x9f << CMD_CODE_CTL)   /* Read JEDED ID */
#define DISQPI                (0xff << CMD_CODE_CTL)   /* Disable quad mode */

/* Status register bit definitions */

#define SR_WIP                (1 << 0)  /* Bit 0: Write in Progress */
#define SR_WEL                (1 << 1)  /* Bit 1: Write Enable Latch */
#define SR_QE                 (1 << 9)

/* Identifies the flash smallest block write unit */

#define SECTOR_SHIFT           12        /* Sector size 1 << 12 = 4Kb */
#define SECTOR_SIZE            (1 << 12) /* Sector size 1 << 12 = 4Kb */
#define PAGE_SHIFT             8         /* Sector size 1 << 8 = 256b */
#define PAGE_SIZE              (1 << 8)  /* Sector size 1 << 8 = 256b */

/* JEDEC Read ID register values */

#define JEDEC_CAPACITY_2MBIT   0x12  /* 64x4096 = 2Mbit memory capacity */
#define JEDEC_CAPACITY_8MBIT   0x14  /* 256x4096 = 8Mbit memory capacity */
#define JEDEC_CAPACITY_16MBIT  0x15  /* 512x4096  = 16Mbit memory capacity */
#define JEDEC_CAPACITY_32MBIT  0x16  /* 1024x4096 = 32Mbit memory capacity */
#define JEDEC_CAPACITY_64MBIT  0x17  /* 2048x4096 = 64Mbit memory capacity */
#define JEDEC_CAPACITY_128MBIT 0x18  /* 4096x4096 = 128Mbit memory capacity */
#define JEDEC_CAPACITY_256MBIT 0x19  /* 8192x4096 = 256Mbit memory capacity */
#define NSECTORS_2MBIT         64    /* 64  sectors x 4096 bytes/sector = 256Kb */
#define NSECTORS_8MBIT         256   /* 256 sectors x 4096 bytes/sector = 1Mb */
#define NSECTORS_16MBIT        512   /* 512 sectors x 4096 bytes/sector = 2Mb */
#define NSECTORS_32MBIT        1024  /* 1024 sectors x 4096 bytes/sector = 4Mb */
#define NSECTORS_64MBIT        2048  /* 2048 sectors x 4096 bytes/sector = 8Mb */
#define NSECTORS_128MBIT       4096  /* 4096 sectors x 4096 bytes/sector = 16Mb */
#define NSECTORS_256MBIT       8192  /* 8192 sectors x 4096 bytes/sector = 32Mb */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_spi_flash_dev_s
{
  struct mtd_dev_s mtd;
  uint8_t          *write_buf;
  uint16_t         nsectors;
  FAR const struct song_spi_flash_config_s *cfg;
};

struct jedec_capacity_type_s
{
  uint8_t  capacity_code;
  uint16_t capacity_nsectors;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t flash_regread(FAR struct song_spi_flash_dev_s *priv,
                          uint32_t offset);
static inline void flash_regwrite(FAR struct song_spi_flash_dev_s *priv,
                          uint32_t offset, uint32_t val);
static inline void flash_regupdate(FAR struct song_spi_flash_dev_s *priv,
                          uint32_t offset, uint32_t mask, uint32_t val);
static int flash_get_capacity(FAR struct song_spi_flash_dev_s *priv);

/* MTD driver methods */

static int song_spi_flash_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks);
static ssize_t song_spi_flash_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t song_spi_flash_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR const uint8_t *buf);
static ssize_t song_spi_flash_read(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR uint8_t *buffer);
static int song_spi_flash_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct jedec_capacity_type_s g_capacity_table[] =
{
  {JEDEC_CAPACITY_2MBIT,  NSECTORS_2MBIT},
  {JEDEC_CAPACITY_8MBIT,  NSECTORS_8MBIT},
  {JEDEC_CAPACITY_16MBIT, NSECTORS_16MBIT},
  {JEDEC_CAPACITY_32MBIT, NSECTORS_32MBIT},
  {JEDEC_CAPACITY_64MBIT, NSECTORS_64MBIT},
  {JEDEC_CAPACITY_128MBIT,NSECTORS_128MBIT},
  {JEDEC_CAPACITY_256MBIT,NSECTORS_256MBIT},
  {0, 0},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline volatile uint32_t flash_regread(
                          FAR struct song_spi_flash_dev_s *priv, uint32_t offset)
{
  return (*(volatile uint32_t *)(priv->cfg->base + offset));
}

static inline void flash_regwrite(FAR struct song_spi_flash_dev_s *priv,
                          uint32_t offset, uint32_t val)
{
  *(volatile uint32_t *)(priv->cfg->base + offset) = val;
}

static inline void flash_regupdate(FAR struct song_spi_flash_dev_s *priv,
                          uint32_t offset, uint32_t mask, uint32_t val)
{
  flash_regwrite(priv, offset, (val & mask) |
            (flash_regread(priv, offset) & ~mask));
}

static inline irqstate_t flash_enter_critical(
                          FAR struct song_spi_flash_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  flash_regupdate(priv, MODE_CTL, 1, 1);
  return flags;
}

static inline void flash_leave_critical(FAR struct song_spi_flash_dev_s *priv,
                          irqstate_t flags)
{
  flash_regupdate(priv, MODE_CTL, 1, 0);
  leave_critical_section(flags);
}

__ramfunc__ static void flash_wren(FAR struct song_spi_flash_dev_s *priv)
{
  /* send write enable */
  flash_regwrite(priv, CODE_DATA, WREN);
  flash_regwrite(priv, OP_CTL, CMD_EN);

  /* wait cmd send */
  flash_regwrite(priv, CMD_ST_CTL, CMD_START_CTL);
  while((flash_regread(priv, CMD_ST_CTL) & 0x1));
}

__ramfunc__ static void flash_wait_wip_finish(FAR struct song_spi_flash_dev_s *priv)
{
  uint8_t status;

  do
    {
      /* send read status */
      flash_regwrite(priv, CODE_DATA, RDSR);
      flash_regwrite(priv, OP_CTL, CMD_EN | RD_EN | (1 << RD_NUM_CTL));
      /* wait cmd send */
      flash_regwrite(priv, CMD_ST_CTL, CMD_START_CTL);
      while((flash_regread(priv, CMD_ST_CTL) & 0x1));

      status = flash_regread(priv, RD_DATA_CTL);
    }
  while ((status & SR_WIP) != 0);
}

__ramfunc__ static void flash_enable_quad_spi(FAR struct song_spi_flash_dev_s *priv)
{
  irqstate_t flags;

  flags = flash_enter_critical(priv);

  flash_wren(priv);
  /* send write register QE = 1*/
  flash_regwrite(priv, CODE_DATA, WRSR | SR_QE);
  flash_regwrite(priv, OP_CTL, CMD_EN | WR_EN | (2 << WR_NUM_CTL));
  /* wait cmd send */
  flash_regwrite(priv, CMD_ST_CTL, CMD_START_CTL);
  while((flash_regread(priv, CMD_ST_CTL) & 0x1));

  flash_wait_wip_finish(priv);

  flash_leave_critical(priv, flags);
}

__ramfunc__ static int flash_get_capacity(FAR struct song_spi_flash_dev_s *priv)
{
  const struct jedec_capacity_type_s *ptr = g_capacity_table;
  irqstate_t flags;
  uint8_t memory;
  int ret = -ENODEV;

  flags = flash_enter_critical(priv);

  /* send rdid cmd */
  flash_regwrite(priv, CODE_DATA, RDID);

  /* cmd_en = 1, wr_num_ctl = 3 */
  flash_regwrite(priv, OP_CTL, CMD_EN | RD_EN | (3 << RD_NUM_CTL));
  /* wait cmd send */
  flash_regwrite(priv, CMD_ST_CTL, CMD_START_CTL);
  while((flash_regread(priv, CMD_ST_CTL) & 0x1));

  /* bit0 - bit7     | bit8 - bit15 | bit16 - bit23  */
  /* manufacturer-id | memory type  | memory density */
  memory = (flash_regread(priv, RD_DATA_CTL) >> 16) & 0xff;

  flash_leave_critical(priv, flags);

  while (ptr->capacity_code)
    {
      if (ptr->capacity_code == memory)
        {
          priv->nsectors = ptr->capacity_nsectors;
          ret = 0;
          break;
        }
      ptr++;
    }

  return ret;
}

__ramfunc__ static ssize_t song_spi_flash_read(FAR struct mtd_dev_s *dev,
                          off_t offset, size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct song_spi_flash_dev_s *priv = (FAR struct song_spi_flash_dev_s *)dev;

  memcpy(buffer, (void*)(priv->cfg->cpu_base + offset), nbytes);
  return nbytes;
}

__ramfunc__ static ssize_t song_spi_flash_bread(FAR struct mtd_dev_s *dev,
                          off_t startblock, size_t nblocks, FAR uint8_t *buf)
{
  song_spi_flash_read(dev, startblock << PAGE_SHIFT, nblocks << PAGE_SHIFT, buf);
  return nblocks;
}

__ramfunc__ static int song_spi_flash_erase(FAR struct mtd_dev_s *dev,
                          off_t startblock, size_t nblocks)
{
  FAR struct song_spi_flash_dev_s *priv = (FAR struct song_spi_flash_dev_s *)dev;
  irqstate_t flags;
  size_t nb;

  flags = flash_enter_critical(priv);

  for (nb = 0; nb < nblocks; nb++)
    {
      flash_wren(priv);

      /* send sector erase cmd */
      flash_regwrite(priv, CODE_DATA, SE | ((startblock + nb) << SECTOR_SHIFT));
      /* cmd_en = 1, wr_en = 1, wr_num_ctl = 3 */
      flash_regwrite(priv, OP_CTL, CMD_EN | WR_EN | (3 << WR_NUM_CTL));

      flash_regwrite(priv, CMD_ST_CTL, CMD_START_CTL);
      while((flash_regread(priv, CMD_ST_CTL) & 0x1));

      flash_wait_wip_finish(priv);
    }

  flash_leave_critical(priv, flags);

  up_invalidate_icache(startblock << SECTOR_SHIFT,
      (startblock + nblocks) << SECTOR_SHIFT);
  return nblocks;
}

__ramfunc__ static ssize_t song_spi_flash_bwrite(FAR struct mtd_dev_s *dev,
                          off_t startblock, size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct song_spi_flash_dev_s *priv = (FAR struct song_spi_flash_dev_s *)dev;
  const uint8_t *write_buf = buf;
  irqstate_t flags;
  size_t nb, i;
  extern uintptr_t _sflash;
  extern uintptr_t _eflash;

  /* address copy situation: in flash section or not 32bit align */

  if ((((uintptr_t)buf >= (uintptr_t)&_sflash) &&
        ((uintptr_t)buf <= (uintptr_t)&_eflash)) ||
        (uintptr_t)buf & (sizeof(uint32_t) - 1))
    {
      write_buf = priv->write_buf;
    }

  for (nb = 0; nb < nblocks; nb++)
    {
      if (write_buf == priv->write_buf)
        {
          memcpy(priv->write_buf, &buf[nb * PAGE_SIZE], PAGE_SIZE);
        }
      else
        {
          write_buf = &buf[nb * PAGE_SIZE];
        }

      flags = flash_enter_critical(priv);
      flash_wren(priv);

      /* send page program cmd */
      flash_regwrite(priv, CODE_DATA, PP | ((startblock + nb) << PAGE_SHIFT));
      /* cmd_en = 1,wr_en = 1, wr1_en = 1, wr_num_ctl = 3, wr_num_ctl1 = 256 */
      flash_regwrite(priv, OP_CTL, CMD_EN | WR_EN | WR1_EN        \
                    | (3 << WR_NUM_CTL) | (PAGE_SIZE << WR1_NUM_CTL));

      /* send spi data and wait transfer finish */
      for (i = 0; i < PAGE_SIZE; )
        {
          if (!(flash_regread(priv, CTL_STATUS) & PG_DATA_FIFO_FULL))
            {
              flash_regwrite(priv, PG_FIFO, *((uint32_t *)&write_buf[i]));
              i += 4;
            }
          if (i == PG_FIFO_DEPTH * 4)
            {
              flash_regwrite(priv, CMD_ST_CTL, CMD_START_CTL);
            }
        }

      while((flash_regread(priv, CMD_ST_CTL) & 0x1));
      flash_wait_wip_finish(priv);

      flash_leave_critical(priv,flags);
    }

  up_invalidate_icache(startblock << PAGE_SHIFT,
      (startblock + nblocks) << PAGE_SHIFT);
  return nblocks;
}

__ramfunc__ static int song_spi_flash_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct song_spi_flash_dev_s *priv = (FAR struct song_spi_flash_dev_s *)dev;
  int ret = -EINVAL;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              geo->blocksize    = PAGE_SIZE;
              geo->erasesize    = SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          song_spi_flash_erase(dev, 0, priv->nsectors);
          ret  = OK;
        }
        break;

      case MTDIOC_XIPBASE:
        {
          FAR void **ptr_base = (FAR void**)((uintptr_t)arg);
          if (ptr_base)
            {
              *ptr_base = (FAR void *)priv->cfg->cpu_base;
              ret       = OK;
            }
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: song_spi_flash_initialize
 *
 * Description:
 *  Create and initialize a song spi flash controller MTD device instance
 *  that can be used to access the spi program memory.
 *
 ****************************************************************************/

__ramfunc__ FAR struct mtd_dev_s *song_spi_flash_initialize(
                          FAR const struct song_spi_flash_config_s *cfg)
{
  struct song_spi_flash_dev_s *priv;

  if (cfg->mclk)
    {
      struct clk *mclk;
      int ret;

      mclk = clk_get(cfg->mclk);
      if (!mclk)
        return NULL;

      ret = clk_enable(mclk);
      if (ret < 0)
        return NULL;

      if (cfg->rate)
        {
          ret = clk_set_rate(mclk, cfg->rate);
          if (ret < 0)
            return NULL;
        }
    }

  priv = kmm_zalloc(sizeof(struct song_spi_flash_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->write_buf = kmm_zalloc(PAGE_SIZE);
  if (priv->write_buf == NULL)
    {
      kmm_free(priv);
      return NULL;
    }

  priv->mtd.erase    = song_spi_flash_erase;
  priv->mtd.bread    = song_spi_flash_bread;
  priv->mtd.bwrite   = song_spi_flash_bwrite;
  priv->mtd.read     = song_spi_flash_read;
  priv->mtd.ioctl    = song_spi_flash_ioctl;
  priv->mtd.name     = "song_spi_flash";
  priv->cfg          = cfg;

  if (flash_get_capacity(priv))
    {
      ferr("ERROR: flash get capacity fail\n");
      kmm_free(priv->write_buf);
      kmm_free(priv);
      return NULL;
    }

  flash_enable_quad_spi(priv);

  return (FAR struct mtd_dev_s *)priv;
}
