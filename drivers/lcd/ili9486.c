/**************************************************************************************
 * drivers/lcd/ili9486.c
 *
 *   Copyright (C) 2019 Fishsemi Inc. All rights reserved.
 *   Author: clyde <liuyan@fishsemi.com>
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
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <errno.h>
#include <assert.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/lcd/ili9486_lcd.h>
#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PIXEL_SIZE 2
#define LCD_WIDTH 480
#define LCD_HEIGHT 320
#define SPI_SEND_PIXEL_BLOCK 16

#if defined(CONFIG_LCD_ILI9486)
#define LCD_ILI9486_DEVNAME "/dev/ili9486"
#else
#error No ILI9486 device configured
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct ili9486_dev_s
{
  FAR struct spi_dev_s *spi;                        /* Cached SPI device reference */
  FAR const struct lcd_ili9486_config_s *config;    /* Cached ilitek 9486 instance */
  struct ioexpander_dev_s *ioe;                     /* Cached IO expander device reference */

  FAR struct ili9486_lcd_matrix_s matrix;           /* Matrix to draw */

  sem_t exclsem;                                    /* Mutual exclusion semaphore */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ili9486_spi_select(FAR struct ili9486_dev_s *priv);
static void ili9486_spi_deselect(FAR struct ili9486_dev_s *priv);

static void ili9486_ioe_direction(FAR struct ili9486_dev_s *priv);
static void ili9486_poweron(FAR struct ili9486_dev_s *priv, bool on);
static void ili9486_reset(FAR struct ili9486_dev_s *priv);
static void ili9486_init(FAR struct ili9486_dev_s *priv);

static void draw_matrix(FAR struct ili9486_dev_s *priv,
                      uint16_t* pixel_buffer, int len);

/* Character driver methods */

static int ili9486_open(FAR struct file *filep);
static int ili9486_close(FAR struct file *filep);
static ssize_t ili9486_read(FAR struct file *filep, FAR char *buffer,
                          size_t len);
static ssize_t ili9486_write(FAR struct file *filep, FAR const char *buffer,
                           size_t len);
static int ili9486_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* init command sequence */

static int ili9486_init_code[] = {
  // Interface Mode Control
  -0x01, 0xb0, 0x00,
  // Sleep OUT
  -0x01, 0x11,
  // Interface Pixel Format, 16 bits / pixel
  -0x01, 0x3A, 0x55,
  // Memory Access Control
  -0x01, 0x36, 0x28,
  -0x01, 0x36, 0x00,
  // Power Control 3 (For Normal Mode)
  -0x01, 0xC2, 0x44,
  // VCOM Control
  -0x01, 0xC5, 0x00, 0x00, 0x00, 0x00,
  // PGAMCTRL(Positive Gamma Control)
  -0x01, 0xE0, 0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A,
        0x13, 0x04, 0x11, 0x0D, 0x00,
  // NGAMCTRL (Negative Gamma Correction)
  -0x01, 0xE1, 0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06,
        0x10, 0x03, 0x24, 0x20, 0x00,
  // Digital Gamma Control 1
  -0x01, 0xE2, 0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06,
        0x10, 0x03, 0x24, 0x20, 0x00,
  // Memory Access Control, BGR
  -0x01, 0x36, 0x28,
  // # Sleep OUT
  -0x01, 0x11,
  // Display ON
  -0x01, 0x29,
  -0x02, 0x28,
  -0x03
};

/* Driver operations */

static const struct file_operations g_ili9486_fops =
{
  ili9486_open,   /* open */
  ili9486_close,  /* close */
  ili9486_read,   /* read */
  ili9486_write,  /* write */
  NULL,           /* seek */
  ili9486_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL          /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/****************************************************************************
 * Name: ili9486_ioe_direction
 *
 * Description:
 *   Set ili9486 ioexpander.
 *
 ****************************************************************************/

static void ili9486_ioe_direction(FAR struct ili9486_dev_s *priv)
{
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->power_gpio, IOEXPANDER_DIRECTION_OUT);
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->rst_gpio, IOEXPANDER_DIRECTION_OUT);
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->spi_rs_gpio, IOEXPANDER_DIRECTION_OUT);
}

/****************************************************************************
 * Name: ili9486_poweron
 *
 * Description:
 *   Power supply.
 *
 ****************************************************************************/

static void ili9486_poweron(FAR struct ili9486_dev_s *priv, bool on)
{
  if (on)
    {
      (void)IOEXP_WRITEPIN(priv->ioe, priv->config->power_gpio, true);
    }
  else
    {
      (void)IOEXP_WRITEPIN(priv->ioe, priv->config->power_gpio, false);
    }
}

/****************************************************************************
 * Name: ili9486_reset
 *
 * Description:
 *   Reset ili9486 lcd panel.
 *
 ****************************************************************************/

static void ili9486_reset(FAR struct ili9486_dev_s *priv)
{
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->rst_gpio, true);
  up_mdelay(1);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->rst_gpio, false);
  up_mdelay(1);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->rst_gpio, true);
}

/****************************************************************************
 * Name: ili9486_spi_select
 *
 * Description:
 *   Select the ili9486 lcd device.
 *
 ****************************************************************************/

static void ili9486_spi_select(FAR struct ili9486_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  lcdinfo("Mode: %d Frequency: %d\n", SPIDEV_MODE0, priv->config->spi_freq);

  /* Lock the SPI bus */

  (void)SPI_LOCK(priv->spi, true);

  /* Configure SPI for the ILI9486 */

  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, priv->config->spi_nbits);
  (void)SPI_SETFREQUENCY(priv->spi, priv->config->spi_freq);

  /* Select SPI device */

  SPI_SELECT(priv->spi, priv->config->spi_cs_num, true);
}

/****************************************************************************
 * Name: ili9486_spi_deselect
 *
 * Description:
 *   des select the Ili9486 lcd device.
 *
 ****************************************************************************/

static void ili9486_spi_deselect(FAR struct ili9486_dev_s *priv)
{
  SPI_SELECT(priv->spi, priv->config->spi_cs_num, false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: ili9486_init
 *
 * Description:
 *   Send init command to initialize the Ili9486.
 *
 ****************************************************************************/

static void ili9486_init(FAR struct ili9486_dev_s *priv)
{
  bool done = false;
  bool is_cmd = false;
  int const *cmd = ili9486_init_code;

  while (!done)
    {
      switch (*cmd)
        {
        case -1:
          is_cmd = true;
          break;
        case -2:
          cmd++;
          up_mdelay(*cmd);
          break;
        case -3:
          done = true;
          break;
        default:
          ili9486_spi_select(priv);
          if (is_cmd)
            {
              (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, false);
              is_cmd = false;
            }
          else
            {
              (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, true);
            }
          (void)SPI_SEND(priv->spi, (uint16_t)*cmd);
          ili9486_spi_deselect(priv);
          break;
        }
      cmd++;
  }
}

/****************************************************************************
 * Name: draw_matrix
 *
 * Description:
 *   Draw a matrix area.
 *
 ****************************************************************************/

static void draw_matrix(FAR struct ili9486_dev_s *priv,
                      uint16_t* pixel_buffer, int len)
{
#ifdef CONFIG_SPI_EXCHANGE

  uint16_t buffer[4];
  int count = len;

  /* Set X coord */

  ili9486_spi_select(priv);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, false);
  (void)SPI_SEND(priv->spi, 0x2A);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, true);

  buffer[0] = priv->matrix.x >> 8;
  buffer[1] = priv->matrix.x;
  buffer[2] = (priv->matrix.x + priv->matrix.w) >> 8;
  buffer[3] = priv->matrix.x + priv->matrix.w - 1;

  SPI_SNDBLOCK(priv->spi, buffer, 4);

  /* Set Y coord */

  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, false);
  (void)SPI_SEND(priv->spi, 0x2B);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, true);

  buffer[0] = priv->matrix.y >> 8;
  buffer[1] = priv->matrix.y;
  buffer[2] = (priv->matrix.y + priv->matrix.h) >> 8;
  buffer[3] = priv->matrix.y + priv->matrix.h - 1;

  SPI_SNDBLOCK(priv->spi, buffer, 4);

  /* Write data */

  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, false);
  (void)SPI_SEND(priv->spi, 0x2C);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->spi_rs_gpio, true);

  while(count)
    {
      if (count > SPI_SEND_PIXEL_BLOCK)
        {
          SPI_SNDBLOCK(priv->spi, pixel_buffer, SPI_SEND_PIXEL_BLOCK);
          pixel_buffer += SPI_SEND_PIXEL_BLOCK;
          count -= SPI_SEND_PIXEL_BLOCK;
        }
      else
        {
          SPI_SNDBLOCK(priv->spi, pixel_buffer, count);
          count = 0;
        }
    }

  ili9486_spi_deselect(priv);
#endif
}

/****************************************************************************
 * Name: ili9486_open
 *
 * Description:
 *   This function is called whenever the ILI9486 device is opened.
 *
 ****************************************************************************/

static int ili9486_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ili9486_close
 *
 * Description:
 *   This function is called whenever the ILI9486 device is closed.
 *
 ****************************************************************************/

static int ili9486_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ili9486_read
 *
 ****************************************************************************/

static ssize_t ili9486_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  return OK;
}

/****************************************************************************
 * Name: ili9486_write
 *
 * Description:
 *   Draw a buffer to ili9486 device.
 *
 ****************************************************************************/

static ssize_t ili9486_write(FAR struct file *filep, FAR const char *buffer,
                           size_t len)
{
  FAR struct inode *inode;
  FAR struct ili9486_dev_s *priv;
  int ret;

  lcdinfo("buffer: %p len %lu\n", buffer, (unsigned long)len);
  DEBUGASSERT(buffer != NULL && len > 0);

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = inode->i_private;

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  draw_matrix(priv, (uint16_t *)buffer, len);

  nxsem_post(&priv->exclsem);
  return len;
}

/****************************************************************************
 * Name: ili9486_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ili9486_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct ili9486_dev_s *priv;
  int ret = OK;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  lcdinfo("cmd: %d arg: %lu\n", cmd, arg);

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case ILI9486_LCDIOC_GETATTRIBUTES:
        {
          FAR struct ili9486_lcd_attributes_s *attr =
                  (FAR struct ili9486_lcd_attributes_s *)((uintptr_t)arg);
          if (!attr)
            {
              ret = -EINVAL;
              goto out;
            }

          attr->lcd_width = LCD_WIDTH;
          attr->lcd_height = LCD_HEIGHT;
        }
        break;
      case ILI9486_LCDIOC_CLEAR:
        {
          ili9486_reset(priv);
          ili9486_init(priv);
        }
        break;
      case ILI9486_LCDIOC_SET_MATRIX:
        {
          FAR struct ili9486_lcd_matrix_s *matrix =
                  (FAR struct ili9486_lcd_matrix_s *)((uintptr_t)arg);
          if (!matrix)
            {
              ret = -EINVAL;
              goto out;
            }

          priv->matrix.x = matrix->x;
          priv->matrix.y = matrix->y;
          priv->matrix.w = matrix->w;
          priv->matrix.h = matrix->h;
        }
        break;

      default:
        lcderr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

out:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcd_ili9486_register
 *
 * Description:
 *   Register a ili9486 lcd component.
 *
 ****************************************************************************/

int lcd_ili9486_register(FAR const struct lcd_ili9486_config_s *config,
                         FAR struct spi_dev_s *spi,
                         FAR struct ioexpander_dev_s *ioe)
{
  FAR struct ili9486_dev_s *priv;
  int ret;

  DEBUGASSERT(config != NULL && spi != NULL && ioe != NULL);

  priv = (FAR struct ili9486_dev_s *)kmm_zalloc(sizeof(struct ili9486_dev_s));
  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate ili9486 device structure\n");
      return -ENOMEM;
    }

  priv->spi = spi;
  priv->config = config;
  priv->ioe = ioe;
  sem_init(&priv->exclsem, 0, 1);

  ili9486_ioe_direction(priv);
  ili9486_poweron(priv, true);
  ili9486_reset(priv);
  ili9486_init(priv);

  ret = register_driver(LCD_ILI9486_DEVNAME, &g_ili9486_fops, 0666, priv);
  if (ret < 0)
    {
      nxsem_destroy(&priv->exclsem);
      kmm_free(priv);
      return ret;
    }

  return OK;
}
