/**************************************************************************************
 * drivers/lcd/vk1056b.c
 *
 *   Copyright (C) 2020 Fishsemi Inc. All rights reserved.
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
#include <time.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/lcd/vk1056b_lcd.h>
#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* base function command */

#define OSC_OFF 0x00           /* Close oscillator */
#define OSC_ON 0x01            /* Open oscillator */
#define DISP_OFF 0x02          /* Close LCD Bias */
#define DISP_ON 0x03           /* Open LCD Bias */
#define COM_1_3__4 0x29        /* 1/3bias 4com */
#define COM_1_3__3 0x25        /* 1/3bias 3com */
#define COM_1_3__2 0x21        /* 1/3bias 2com */
#define COM_1_2__4 0x28        /* 1/2bias 4com */
#define COM_1_2__3 0x24        /* 1/2bias 3com */
#define COM_1_2__2 0x20        /* 1/2bias 2com */

/* Extended function command shutdown to reduce power consumption */

#define TIMER_DIS 0x04
#define WDT_DIS 0x05
#define BUZZ_OFF 0x08
#define RC32K 0X18
#define IRQ_DIS 0X80

#if defined(CONFIG_LCD_VK1056B)
#define LCD_VK1056B_DEVNAME "/dev/vk1056b"
#else
#error No VK1056B device configured
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct vk1056b_dev_s
{
  FAR const struct lcd_vk1056b_config_s *config;    /* Cached vk1056b instance */
  struct ioexpander_dev_s *ioe;                     /* Cached IO expander device reference */

  sem_t exclsem;                                    /* Mutual exclusion semaphore */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void vk1056b_write_clock(FAR struct vk1056b_dev_s *priv);
static void vk1056b_write_command(FAR struct vk1056b_dev_s *priv, uint8_t functioncode);
static void vk1056b_write_data(FAR struct vk1056b_dev_s *priv);
static void draw_all(bool state);
static void draw_body_temperature(uint32_t temperature);
static void draw_real_time(struct tm *time);
static void draw_battery(bool state);
static void vk1056b_ioe_direction(FAR struct vk1056b_dev_s *priv);
static void vk1056b_poweron(FAR struct vk1056b_dev_s *priv, bool on);
static void vk1056b_init(FAR struct vk1056b_dev_s *priv);

/* Character driver methods */

static int vk1056b_open(FAR struct file *filep);
static int vk1056b_close(FAR struct file *filep);
static ssize_t vk1056b_read(FAR struct file *filep, FAR char *buffer,
                          size_t len);
static ssize_t vk1056b_write(FAR struct file *filep, FAR const char *buffer,
                           size_t len);
static int vk1056b_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define ADDR_N_BIT 6
#define BIGEN_ADDR 9

#define COM_NUM 4
#define SEG_NUM 10

static uint8_t ddr_map[SEG_NUM] = {0};

static uint8_t digit[30] = {
  0x06, 0x0a, 0x06, // 0
  0x06, 0x00, 0x00, // 1
  0x02, 0x0e, 0x04, // 2
  0x06, 0x0e, 0x00, // 3
  0x06, 0x04, 0x02, // 4
  0x04, 0x0e, 0x02, // 5
  0x04, 0x0e, 0x06, // 6
  0x06, 0x02, 0x00, // 7
  0x06, 0x0e, 0x06, // 8
  0x06, 0x0e, 0x02, // 9
};

/* Driver operations */

static const struct file_operations g_vk1056b_fops =
{
  vk1056b_open,   /* open */
  vk1056b_close,  /* close */
  vk1056b_read,   /* read */
  vk1056b_write,  /* write */
  NULL,           /* seek */
  vk1056b_ioctl   /* ioctl */
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

static inline void __set_bit(uint8_t *val, int nr)
{
  uint8_t mask = 1 << (nr);
  *val |= mask;
}

static inline void __clear_bit(uint8_t *val, int nr)
{
  uint8_t mask = 1 << (nr);
  *val &= ~mask;
}

static void set_earmark(uint8_t *seg, int nr)
{
  __set_bit(seg, nr);
}

static void clear_earmark(uint8_t *seg, int nr)
{
  __clear_bit(seg, nr);
}

/****************************************************************************
 * Name: vk1056b_write_clock
 *
 * Description:
 *  set Vk1056B clk timing sequence
 *
 ****************************************************************************/

static void vk1056b_write_clock(FAR struct vk1056b_dev_s *priv)
{
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->wr_gpio, false);
  up_udelay(5);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->wr_gpio, true);
  up_udelay(5);
}

/****************************************************************************
 * Name: vk1056b_write_command
 *
 * Description:
 *  write function command
 ****************************************************************************/

static void vk1056b_write_command(FAR struct vk1056b_dev_s *priv, uint8_t functioncode)
{
  uint8_t i;
  uint8_t shift = 0x80;

  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->cs_gpio, false);
  up_udelay(3);

  /* the cmd types header 100 bit */
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
  vk1056b_write_clock(priv);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
  vk1056b_write_clock(priv);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
  vk1056b_write_clock(priv);

  for (i = 0; i < 8; i++)
    {
      if (shift & functioncode)
        {
          (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
        }
      else
        {
          (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
        }

      vk1056b_write_clock(priv);
      shift = shift >> 1;
    }

  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
  vk1056b_write_clock(priv);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->cs_gpio, true);
  up_udelay(3);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
}

/****************************************************************************
 * Name: vk1056b_write_data
 *
 * Description:
 *  write data to vk1056b
 ****************************************************************************/

static void vk1056b_write_data(FAR struct vk1056b_dev_s *priv)
{
  uint8_t i, j;
  uint8_t shift;

  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->cs_gpio, false);
  up_udelay(3);

  /* the cmd types header 101 bit */
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
  vk1056b_write_clock(priv);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
  vk1056b_write_clock(priv);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
  vk1056b_write_clock(priv);

  shift = 0x20;
  for (i = 0; i < ADDR_N_BIT; i++)
    {
      if (BIGEN_ADDR & shift)
        {
          (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
        }
      else
        {
          (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
        }

      vk1056b_write_clock(priv);
      shift = shift >> 1;
    }

  for (i = 0; i < SEG_NUM; i++)
    {
      shift = 0x08;
      for (j = 0; j < COM_NUM; j++)
        {
          if (ddr_map[i] & shift)
            {
              (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
            }
          else
            {
              (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, false);
            }

          vk1056b_write_clock(priv);
          shift = shift >> 1;
        }
    }

  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->cs_gpio, true);
  up_udelay(3);
  (void)IOEXP_WRITEPIN(priv->ioe, priv->config->data_gpio, true);
}

/****************************************************************************
 * Name: draw_all
 *
 * Description:
 *  draw all or not
 ****************************************************************************/

static void draw_all(bool state)
{
  syslog(LOG_INFO, "set display all\n");
  if (state)
    {
      memset(ddr_map, 0xff, sizeof(ddr_map));
    }
  else
    {
      memset(ddr_map, 0x00, sizeof(ddr_map));
    }
}

/****************************************************************************
 * Name: draw_body_temperature
 *
 * Description:
 *  draw a body temperature
 ****************************************************************************/

static void draw_body_temperature(uint32_t temperature)
{
  uint8_t temp_digit;

  temp_digit = temperature % 10;
  ddr_map[1] = digit[3 * temp_digit];
  ddr_map[2] = digit[3 * temp_digit + 1];
  ddr_map[3] = digit[3 * temp_digit + 2];

  temp_digit = (temperature / 10) % 10;
  ddr_map[4] = digit[3 * temp_digit];
  ddr_map[5] = digit[3 * temp_digit + 1];
  ddr_map[6] = digit[3 * temp_digit + 2];

  temp_digit = (temperature / 100) % 10;
  ddr_map[7] = digit[3 * temp_digit];
  ddr_map[8] = digit[3 * temp_digit + 1];
  ddr_map[9] = digit[3 * temp_digit + 2];

  //todo, split as a new ioc if display delay more.
  /* 4A */
  set_earmark(&ddr_map[0], 1);
  /* 4B */
  set_earmark(&ddr_map[0], 2);
  /* 4C */
  set_earmark(&ddr_map[0], 3);
  /* I3 */
  set_earmark(&ddr_map[3], 3);
  /* H2 */
  set_earmark(&ddr_map[4], 3);
}

/****************************************************************************
 * Name: draw_real_time
 *
 * Description:
 *  draw a real time
 ****************************************************************************/

static void draw_real_time(struct tm *time)
{
  uint8_t tm_digit;

  tm_digit = time->tm_min % 10;
  ddr_map[1] = digit[3 * tm_digit];
  ddr_map[2] = digit[3 * tm_digit + 1];
  ddr_map[3] = digit[3 * tm_digit + 2];

  tm_digit = (time->tm_min / 10) % 10;
  ddr_map[4] = digit[3 * tm_digit];
  ddr_map[5] = digit[3 * tm_digit + 1];
  ddr_map[6] = digit[3 * tm_digit + 2];

  if (time->tm_hour > 12)
      time->tm_hour -= 12;

  tm_digit = time->tm_hour % 10;
  ddr_map[7] = digit[3 * tm_digit];
  ddr_map[8] = digit[3 * tm_digit + 1];
  ddr_map[9] = digit[3 * tm_digit + 2];

  /* H1 */
  if (time->tm_hour > 9)
    {
      set_earmark(&ddr_map[9], 3);
    }
  else
    {
      clear_earmark(&ddr_map[9], 3);
    }

  /* COL */
  set_earmark(&ddr_map[7], 3);
}

/****************************************************************************
 * Name: draw_battery
 *
 * Description:
 *  draw a battery
 ****************************************************************************/

static void draw_battery(bool state)
{
  /* H3 */
  if (state)
    {
      set_earmark(&ddr_map[1], 3);
    }
  else
    {
      clear_earmark(&ddr_map[1], 3);
    }
}

#ifdef VK1056B_STANDBY_MODE

/****************************************************************************
 * Name: vk1056b_enter_standby
 *
 * Description:
 *  enter lower power state when using on-chip clk source
 ****************************************************************************/

void vk1056b_enter_standby(FAR struct vk1056b_dev_s *priv)
{
  vk1056b_write_command(priv, OSC_OFF);
  vk1056b_write_command(priv, DISP_OFF);
}

/****************************************************************************
 * Name: vk1056b_exit_standby
 *
 * Description:
 *  exit lower power state when using on-chip clk source
 ****************************************************************************/

void vk1056B_exit_standby(FAR struct vk1056b_dev_s *priv)
{
  vk1056b_write_command(priv, OSC_ON);
  vk1056b_write_command(priv, DISP_ON);
  vk1056b_write_command(priv, COM_1_3__3);
}
#endif

/****************************************************************************
 * Name: vk1056b_ioe_direction
 *
 * Description:
 *   Set vk1056b ioexpander.
 *
 ****************************************************************************/

static void vk1056b_ioe_direction(FAR struct vk1056b_dev_s *priv)
{
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->power_gpio, IOEXPANDER_DIRECTION_OUT);
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->cs_gpio, IOEXPANDER_DIRECTION_OUT);
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->wr_gpio, IOEXPANDER_DIRECTION_OUT);
  (void)IOEXP_SETDIRECTION(priv->ioe,
                         priv->config->data_gpio, IOEXPANDER_DIRECTION_OUT);
}

/****************************************************************************
 * Name: vk1056b_poweron
 *
 * Description:
 *  vk1056b power on
 ****************************************************************************/

void vk1056b_poweron(FAR struct vk1056b_dev_s *priv, bool on)
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
 * Name: vk1056b_init
 *
 * Description: 
 *  vk1056b initialization
 ****************************************************************************/

void vk1056b_init(FAR struct vk1056b_dev_s *priv)
{
  vk1056b_write_command(priv, OSC_ON);
  vk1056b_write_command(priv, DISP_ON);
  vk1056b_write_command(priv, COM_1_3__3);

  /* disable following function to reduce power consumption */

  vk1056b_write_command(priv, TIMER_DIS);
  vk1056b_write_command(priv, WDT_DIS);
  vk1056b_write_command(priv, BUZZ_OFF);
  vk1056b_write_command(priv, IRQ_DIS);
}

/****************************************************************************
 * Name: vk1056b_open
 *
 * Description:
 *   This function is called whenever the vk1056b device is opened.
 *
 ****************************************************************************/

static int vk1056b_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vk1056b_close
 *
 * Description:
 *   This function is called whenever the vk1056b device is closed.
 *
 ****************************************************************************/

static int vk1056b_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vk1056b_read
 *
 ****************************************************************************/

static ssize_t vk1056b_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  return OK;
}

/****************************************************************************
 * Name: vk1056b_write
 *
 ****************************************************************************/

static ssize_t vk1056b_write(FAR struct file *filep, FAR const char *buffer,
                           size_t len)
{
  return OK;
}

/****************************************************************************
 * Name: vk1056b_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int vk1056b_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct vk1056b_dev_s *priv;
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
      case VK1056B_LCDIOC_DISP_ALL:
        {
          bool state = (bool)arg;
          draw_all(state);
        }
        break;

      case VK1056B_LCDIOC_BODY_TEMP:
          {
            uint32_t temp = (uint32_t)(*((float *)((uintptr_t)arg)) * 10);
            draw_body_temperature(temp);
          }
          break;

      case VK1056B_LCDIOC_RT_TIME:
          {
            FAR struct tm *time = (struct tm *)((uintptr_t)arg);
            draw_real_time(time);
          }
          break;

      case VK1056B_LCDIOC_BATTERY:
          {
            bool state = (bool)arg;
            draw_battery(state);
          }
	  break;

      default:
          lcderr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
          ret = -ENOTTY;
          break;
    }

  vk1056b_write_data(priv);

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcd_vk1056b_register
 *
 * Description:
 *   Register a vk1056b lcd component.
 *
 ****************************************************************************/

int lcd_vk1056b_register(FAR const struct lcd_vk1056b_config_s *config,
                         FAR struct ioexpander_dev_s *ioe)
{
  FAR struct vk1056b_dev_s *priv;
  int ret;

  DEBUGASSERT(config != NULL && ioe != NULL);

  priv = (FAR struct vk1056b_dev_s *)kmm_zalloc(sizeof(struct vk1056b_dev_s));
  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate vk1056b device structure\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->ioe = ioe;
  sem_init(&priv->exclsem, 0, 1);

  vk1056b_ioe_direction(priv);
  vk1056b_poweron(priv, true);
  vk1056b_init(priv);

  ret = register_driver(LCD_VK1056B_DEVNAME, &g_vk1056b_fops, 0666, priv);
  if (ret < 0)
    {
      nxsem_destroy(&priv->exclsem);
      kmm_free(priv);
      return ret;
    }

  return OK;
}
