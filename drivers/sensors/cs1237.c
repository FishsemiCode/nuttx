/****************************************************************************
 * drivers/sensors/cs1237.c
 * Character driver for the ChipSea Temperature Sensor
 *
 *   Copyright (C) 2020 Fishsemi. All rights reserved.
 *   Author: Dongjiuzhu <dongjiuzhu@fishsemi.com>
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
#include <debug.h>
#include <time.h>
#include <math.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/cs1237.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CS1237_ADC_RESOLUTION (3.0 / 16777216)
#define CS1237_LEN(array) (sizeof(array) / sizeof(array[0]))

/****************************************************************************
 * Private
 ****************************************************************************/

struct cs1237_dev_s
{
  FAR struct pinctrl_dev_s *pctl;
  FAR struct ioexpander_dev_s *ioe;
  FAR struct cs1237_config_s *cfg;

  struct work_s work;
  sem_t datasem;

  float temperature;
};

struct cs1237_t_v_table {
  float temperature;
  double voltage;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    cs1237_worker(FAR void *arg);
static int     cs1237_open(FAR struct file *filep);
static int     cs1237_close(FAR struct file *filep);
static ssize_t cs1237_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t cs1237_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     cs1237_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cs1237fops =
{
  cs1237_open,
  cs1237_close,
  cs1237_read,
  cs1237_write,
  NULL,
  cs1237_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
};

static const struct cs1237_t_v_table g_cs1237_table[] = {
  { 32.0, 0.810330859 },
  { 32.1, 0.807842808 },
  { 32.2, 0.805365151 },
  { 32.3, 0.802889932 },
  { 32.4, 0.800417188 },
  { 32.5, 0.797955034 },
  { 32.6, 0.795495462 },
  { 32.7, 0.793046626 },
  { 32.8, 0.790600478 },
  { 32.9, 0.788165209 },
  { 33.0, 0.785732738 },
  { 33.1, 0.7833031   },
  { 33.2, 0.780884539 },
  { 33.3, 0.778477146 },
  { 33.4, 0.776064524 },
  { 33.5, 0.773663178 },
  { 33.6, 0.771273197 },
  { 33.7, 0.768886377 },
  { 33.8, 0.766502753 },
  { 33.9, 0.764122362 },
  { 34.0, 0.761753591 },
  { 34.1, 0.759396531 },
  { 34.2, 0.757034501 },
  { 34.3, 0.754678408 },
  { 34.4, 0.833497025 },
  { 34.5, 0.831327914 },
  { 34.6, 0.747671656 },
  { 34.7, 0.745344136 },
  { 34.8, 0.74302029  },
  { 34.9, 0.740708664 },
  { 35.0, 0.738400823 },
  { 35.1, 0.736096805 },
  { 35.2, 0.733805205 },
  { 35.3, 0.731517539 },
  { 35.4, 0.729233843 },
  { 35.5, 0.726954156 },
  { 35.6, 0.724687144 },
  { 35.7, 0.722424252 },
  { 35.8, 0.719645633 },
  { 35.9, 0.717910976 },
  { 36.0, 0.715660669 },
  { 36.1, 0.713423348 },
  { 36.2, 0.711190372 },
  { 36.3, 0.708961778 },
  { 36.4, 0.706737605 },
  { 36.5, 0.704526673 },
  { 36.6, 0.702311475 },
  { 36.7, 0.700109628 },
  { 36.8, 0.69791239  },
  { 36.9, 0.6957198   },
  { 37.0, 0.693531895 },
  { 37.1, 0.691357597 },
  { 37.2, 0.689179196 },
  { 37.3, 0.687014514 },
  { 37.4, 0.684854705 },
  { 37.5, 0.682699809 },
  { 37.6, 0.680549864 },
  { 37.7, 0.678404909 },
  { 37.8, 0.676273983 },
  { 37.9, 0.674139141 },
  { 38.0, 0.672009405 },
  { 38.1, 0.669893864 },
  { 38.2, 0.667783539 },
  { 38.3, 0.665669389 },
  { 38.4, 0.663569598 },
  { 38.5, 0.661475141 },
  { 38.6, 0.659386057 },
  { 38.7, 0.657302384 },
  { 38.8, 0.655224161 },
  { 38.9, 0.653151426 },
  { 39.0, 0.65108422  },
  { 39.1, 0.649031793 },
  { 39.2, 0.646975776 },
  { 39.3, 0.644925403 },
  { 39.4, 0.642880714 },
  { 39.5, 0.640841748 },
  { 39.6, 0.638817835 },
  { 39.7, 0.636790446 },
  { 39.8, 0.634768897 },
  { 39.9, 0.632762566 },
  { 40.0, 0.630752828 },
  { 40.1, 0.628749047 },
  { 40.2, 0.62675126  },
  { 40.3, 0.624759507 },
  { 40.4, 0.622783245 },
  { 40.5, 0.620803693 },
  { 40.6, 0.61883029  },
  { 40.7, 0.616863077 },
  { 40.8, 0.614902092 },
  { 40.9, 0.612947373 },
  { 41.0, 0.610998961 },
  { 41.1, 0.609047365 },
  { 41.2, 0.607111664 },
  { 41.3, 0.605182385 },
  { 41.4, 0.603249993 },
  { 41.5, 0.601333658 },
  { 41.6, 0.599414257 },
  { 41.7, 0.597511021 },
  { 41.8, 0.595604766 },
  { 41.9, 0.593705133 },
  { 42.0, 0.591812161 },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs1237_output_clk
 ****************************************************************************/

static void cs1237_output_clk(FAR struct cs1237_dev_s *priv, int num)
{
  DEBUGASSERT(priv != NULL);

  while (num--)
    {
      IOEXP_WRITEPIN(priv->ioe, priv->cfg->clk_io, TRUE);
      up_udelay_light(10);
      IOEXP_WRITEPIN(priv->ioe, priv->cfg->clk_io, FALSE);
      up_udelay_light(10);
    }
}

/****************************************************************************
 * Name: cs1237_convert_temp
 ****************************************************************************/

static float cs1237_convert_temp(uint32_t temp)
{
  int i, len = CS1237_LEN(g_cs1237_table), k = 0;
  double t = temp * CS1237_ADC_RESOLUTION;
  double min = fabs(t - g_cs1237_table[0].voltage);

  if (t > g_cs1237_table[0].voltage)
    {
      return g_cs1237_table[0].temperature;
    }

  if (t < g_cs1237_table[len-1].voltage)
    {
      return g_cs1237_table[len-1].temperature;
    }

  for (i = 1; i < len; i++)
    {
      if (min > fabs(t - g_cs1237_table[i].voltage))
        {
          min = fabs(t - g_cs1237_table[i].voltage);
          k = i;
        }
    }

  return g_cs1237_table[k].temperature;
}

/****************************************************************************
 * Name: cs1237_read_temp
 ****************************************************************************/

static int32_t cs1237_read_temp(FAR struct cs1237_dev_s *priv)
{
  int i, ret, times = 5;
  bool level;
  uint32_t temp = 0;
  irqstate_t flags;

  ret = nxsem_wait(&priv->datasem);
  if (ret < 0)
    {
      snerr("ERROR: Could not aquire priv->datasem: %d\n", ret);
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  IOEXP_SETDIRECTION(priv->ioe, priv->cfg->data_io, IOEXPANDER_DIRECTION_IN);

  /* wait data pin to output low that adc data has been converted */

  do
    {
      if (times == 0)
        {
          nxsem_post(&priv->datasem);
          return -ETIMEDOUT;
        }
      IOEXP_READPIN(priv->ioe, priv->cfg->data_io, &level);
      up_udelay_light(5);
      times--;
    }
  while(level);

  /* send 24 clk, and every raising edge to receive a bit of adc data, MSB first */

  flags = enter_critical_section();

  for (i = 0; i < 24; i++)
    {
      temp <<= 1;
      cs1237_output_clk(priv, 1);
      IOEXP_READPIN(priv->ioe, priv->cfg->data_io, &level);
      if (level)
        {
          temp++;
        }
    }

  leave_critical_section(flags);

  priv->temperature = cs1237_convert_temp(temp);

  nxsem_post(&priv->datasem);

  flags = enter_critical_section();

  cs1237_output_clk(priv, 3);

  /* clock 27, the data pin is forced high */

  IOEXP_SETDIRECTION(priv->ioe, priv->cfg->data_io, IOEXPANDER_DIRECTION_OUT);
  IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, TRUE);

  leave_critical_section(flags);

  return 0;
}

/****************************************************************************
 * Name: cs1237_read_reg
 ****************************************************************************/

static unsigned char cs1237_read_reg(FAR struct cs1237_dev_s *priv)
{
  unsigned char cmd = (0x56 << 1), regval = 0;
  bool level;
  int i;
  irqstate_t flags = enter_critical_section();

  /* clock 1 ~ clock 27*/

  cs1237_output_clk(priv, 27);

  /* clock 28 ~ clock 29, set data pin as output*/

  IOEXP_SETDIRECTION(priv->ioe, priv->cfg->data_io, IOEXPANDER_DIRECTION_OUT);
  cs1237_output_clk(priv, 2);

  /* clock 30 ~ clock 36, send read cmd data */

  for (i = 0; i < 7; i++)
    {
      if (cmd & 0x80)
        {
          IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, TRUE);
        }
      else
        {
          IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, FALSE);
        }
      cmd <<= 1;
      cs1237_output_clk(priv, 1);
    }

  /* clock 37, set data pin as input*/

  IOEXP_SETDIRECTION(priv->ioe, priv->cfg->data_io, IOEXPANDER_DIRECTION_IN);
  cs1237_output_clk(priv, 1);

  /* clock 38 ~ clock 45, read register data */

  for (i = 0; i < 8; i++)
    {
      regval <<= 1;
      cs1237_output_clk(priv, 1);
      IOEXP_READPIN(priv->ioe, priv->cfg->data_io, &level);
      if (level)
        {
          regval++;
        }
    }
  cs1237_output_clk(priv, 1);

  leave_critical_section(flags);

  return regval;
}

/****************************************************************************
 * Name: cs1237_write_reg
 ****************************************************************************/

static void cs1237_write_reg(FAR struct cs1237_dev_s *priv, unsigned char regval)
{
  unsigned char cmd = (0x65 << 1);
  int i;
  irqstate_t flags = enter_critical_section();

  /* clock 1 ~ clock 27*/

  cs1237_output_clk(priv, 27);

  /* clock 28 ~ clock 29, set data pin as output */

  IOEXP_SETDIRECTION(priv->ioe, priv->cfg->data_io, IOEXPANDER_DIRECTION_OUT);
  cs1237_output_clk(priv, 2);

  /* clock 30 ~ clock 36, send write cmd data */

  for (i = 0; i < 7; i++)
    {
      if (cmd & 0x80)
        {
          IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, TRUE);
        }
      else
        {
          IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, FALSE);
        }
      cmd <<= 1;
      cs1237_output_clk(priv, 1);
    }

  /* clock 37 */

  cs1237_output_clk(priv, 1);

  /* clock 38 ~ clock 45, write register data */

  for (i = 0; i < 8; i++)
    {
      if (regval & 0x80)
        {
          IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, TRUE);
        }
      else
        {
          IOEXP_WRITEPIN(priv->ioe, priv->cfg->data_io, FALSE);
        }
      regval <<= 1;
      cs1237_output_clk(priv, 1);
    }

  /* clock 46, set data pin as input */

  IOEXP_SETDIRECTION(priv->ioe, priv->cfg->data_io, IOEXPANDER_DIRECTION_IN);
  cs1237_output_clk(priv, 1);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cs1237_config
 ****************************************************************************/

static void cs1237_config(FAR struct cs1237_dev_s *priv)
{
  uint8_t regval = 0;

  regval |= priv->cfg->refo;
  regval |= priv->cfg->pga;
  regval |= priv->cfg->freq;
  regval |= priv->cfg->ch;
  regval &= 0xEF;

  cs1237_write_reg(priv, regval);
}

/****************************************************************************
 * Name: cs1237_convert_odr
 ****************************************************************************/

static int cs1237_convert_odr(CS1237_FREQ freq)
{
  switch (freq)
    {
    case CS1237_FREQ_10HZ:
      return 10;
    case CS1237_FREQ_40HZ:
      return 40;
    case CS1237_FREQ_640HZ:
      return 640;
    case CS1237_FREQ_1280HZ:
      return 1280;
    default:
      return 10;
    }
}

/****************************************************************************
 * Name: cs1237_convert_freq
 ****************************************************************************/

static CS1237_FREQ cs1237_convert_freq(int odr)
{
  if (odr < 40 && odr > 0)
    {
      return CS1237_FREQ_10HZ;
    }
  else if (odr >= 40 && odr < 640)
    {
      return CS1237_FREQ_40HZ;
    }
  else if (odr >= 640 && odr < 1280)
    {
      return CS1237_FREQ_640HZ;
    }
  else if (odr >= 1280)
    {
      return CS1237_FREQ_1280HZ;
    }
  else
    {
      return CS1237_FREQ_10HZ;
    }
}

/****************************************************************************
 * Name: cs1237_open
 ****************************************************************************/

static int cs1237_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cs1237_dev_s *priv = inode->i_private;

  /* cs1237 enter normal state */

  IOEXP_WRITEPIN(priv->ioe, priv->cfg->clk_io, FALSE);

  /* start read adc data */

  return work_queue(LPWORK, &priv->work, cs1237_worker, priv, 0);
}

/****************************************************************************
 * Name: cs1237_close
 ****************************************************************************/

static int cs1237_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cs1237_dev_s *priv = inode->i_private;

  /* cancel work queue to stop read adc data */

  work_cancel(HPWORK, &priv->work);

  /* cs1237 enter suspend state, save power */

  IOEXP_WRITEPIN(priv->ioe, priv->cfg->clk_io, TRUE);

  return OK;
}

/****************************************************************************
 * Name: cs1237_read
 ****************************************************************************/

static ssize_t cs1237_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cs1237_dev_s *priv = inode->i_private;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(float))
    {
      snerr("ERROR: Not enough memory for reading out a sensor data sample\n");
      return -EINVAL;
    }

  /* Aquire the semaphore before the data is copied */

  ret = nxsem_wait(&priv->datasem);
  if (ret < 0)
    {
      snerr("ERROR: Could not aquire priv->datasem: %d\n", ret);
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      return ret;
    }

  memset(buffer, 0, buflen);

  *((float *)buffer) = priv->temperature;

  /* Give back the semaphore */

  nxsem_post(&priv->datasem);

  return sizeof(float);
}

/****************************************************************************
 * Name: cs1237_write
 ****************************************************************************/

static ssize_t cs1237_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: cs1237_ioctl
 ****************************************************************************/

static int cs1237_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cs1237_dev_s *priv = inode->i_private;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
    case CMD_CFG_REFO:
      if (arg == CS1237_REFO_ON || arg == CS1237_REFO_OFF)
        {
          priv->cfg->refo = arg;
          ret = OK;
        }
      break;
    case CMD_ODR_GET:
      arg = cs1237_convert_odr(priv->cfg->freq);
      ret = OK;
      return ret;
    case CMD_CFG_ODR:

      /* arg is odr(int) not CS1237_FREQ */

      priv->cfg->freq = cs1237_convert_freq(arg);
      ret = OK;
      break;
    case CMD_CFG_PGA:
      if (arg == CS1237_PGA_1 || arg == CS1237_PGA_2 ||
          arg == CS1237_PGA_64 || arg == CS1237_PGA_128)
        {
          priv->cfg->pga = arg;
          ret = OK;
        }
      break;
    case CMD_CFG_CH:
      if (arg == CS1237_CH_A || arg == CS1237_CH_RESERVE ||
          arg == CS1237_CH_TEMP || arg == CS1237_CH_TEST)
        {
          priv->cfg->ch = arg;
          ret = OK;
        }
      break;
    default:
      snerr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  if (ret == OK)
    {
      cs1237_config(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: cs1237_worker
 ****************************************************************************/

static void cs1237_worker(FAR void *arg)
{
  FAR struct cs1237_dev_s *priv = arg;
  int odr_tick = MSEC2TICK(MSEC_PER_SEC / cs1237_convert_odr(priv->cfg->freq) + 5);

  DEBUGASSERT(priv != NULL);

  cs1237_read_temp(priv);

  work_queue(LPWORK, &priv->work, cs1237_worker, priv, odr_tick);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs1237_register
 *
 * Description:
 *   Register the CS1237 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/cs12370"
 *   config - configuration for the cs1237 driver. For details see description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cs1237_register(FAR const char *devpath, FAR struct pinctrl_dev_s *pctl,
                        FAR struct ioexpander_dev_s *ioe, FAR struct cs1237_config_s *config)
{
  FAR struct cs1237_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(pctl != NULL);
  DEBUGASSERT(ioe != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the cs1237 device structure */

  priv = (FAR struct cs1237_dev_s *)kmm_malloc(sizeof(struct cs1237_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(struct cs1237_dev_s));

  priv->pctl = pctl;
  priv->ioe = ioe;
  priv->cfg = config;
  priv->temperature = 0.0f;
  priv->work.worker = NULL;

  nxsem_init(&priv->datasem, 0, 1);

  /* select gpio funtion for clock and data gpio pin */

  PINCTRL_SELGPIO(pctl, config->clk_io);
  PINCTRL_SELGPIO(pctl, config->data_io);
  PINCTRL_SETDT(pctl, config->clk_io, BIAS_DISABLE);
  PINCTRL_SETDT(pctl, config->data_io, BIAS_PULLUP);
  IOEXP_SETDIRECTION(ioe, config->data_io, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETDIRECTION(ioe, config->clk_io, IOEXPANDER_DIRECTION_OUT);

  /* default config cs1237 */

  cs1237_config(priv);

  /* cs1237 enter powerdata, save power */

  IOEXP_WRITEPIN(priv->ioe, priv->cfg->clk_io, TRUE);

  ret = register_driver(devpath, &g_cs1237fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      nxsem_destroy(&priv->datasem);
    }

  return ret;
}
