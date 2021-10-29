/****************************************************************************
 *  arch/risc-v/src/song/song_sdio.c
 *
 *   Copyright (C)
 *   Author:
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
#include <nuttx/cache.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sdio.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>

#include "song_sdio.h"

#define SDIOD_FN0_CCCR                   (0x0000)
#define SDIOD_FN0_BLK_SIZE               (0x0004)
#define SDIOD_FN0_CIS_PTR                (0x0008)
#define SDIOD_FN0_MANF                   (0x000C)
#define SDIOD_FN1_FBR                    (0x0010)
#define SDIOD_FN1_MANF                   (0x0014)
#define SDIOD_FN1_BLK_SIZE               (0x0018)
#define SDIOD_FN1_IF_CODE                (0x001C)
#define SDIOD_FN1_CIS_PTR                (0x0020)
#define SDIOD_FN1_DATP                   (0x0024)
#define SDIOD_FN1_TPL_MISC               (0x0028)
#define SDIOD_FN1_TPL_PSN                (0x002C)
#define SDIOD_FN1_TPL_CSA_SIZE           (0x0030)
#define SDIOD_FN1_TPL_MAX_BLK_SIZE       (0x0034)
#define SDIOD_FN1_TPL_OP_PWR             (0x0038)
#define SDIOD_FN1_TPL_SB_PWR             (0x003C)
#define SDIOD_FN1_TPL_DAT_BW             (0x0040)
#define SDIOD_FN1_TPL_RDY_TO             (0x0044)
#define SDIOD_FN1_TPL_HP_PWR             (0x0048)
#define SDIOD_FN1_TPL_LP_PWR             (0x004C)
#define SDIOD_FN1_INTR_EN                (0x0050)
#define SDIOD_FN1_INTR_RAW               (0x0054)
#define SDIOD_FN1_INTR_STATUS            (0x0058)
#define SDIOD_HST_INFO0                  (0x005C)
#define SDIOD_HST_INFO1                  (0x0060)
#define SDIOD_DEV_INFO                   (0x0064)
#define SDIOD_IO_EXT_CMD_ARG             (0x0068)
#define SDIOD_RX_BLK_CNT                 (0x006C)
#define SDIOD_TX_BLK_CNT                 (0x0070)
#define SDIOD_CMD_INTR_EN                (0x0074)
#define SDIOD_CMD_INTR_RAW               (0x0078)
#define SDIOD_CMD_INTR_STATUS            (0x007C)
#define SDIOD_OP_COND                    (0x0080)
#define SDIOD_OP_MODE                    (0x0084)
#define SDIOD_BUS_LPWR                   (0x0088)
#define SDIOD_CARD_PWR                   (0x008C)
#define SDIOD_CARD_RDY                   (0x0090)
#define SDIOD_DBG_CFG                    (0x0094)
#define SDIOD_CLK_CTRL                   (0x0098)
#define SDIOD_RST_CTRL                   (0x009C)
#define SDIOD_BUF_STATUS                 (0x00A0)
#define SDIOD_MON_STATUS                 (0x00A4)
#define SDIOD_DMA_BUS_MODE               (0x00A8)
#define SDIOD_DMA_TX_POLL                (0x00AC)
#define SDIOD_DMA_RX_POLL                (0x00B0)
#define SDIOD_DMA_RDSC_ADR               (0x00B4)
#define SDIOD_DMA_TDSC_ADR               (0x00B8)
#define SDIOD_DMA_STATUS                 (0x00BC)
#define SDIOD_DMA_OP_MODE                (0x00C0)
#define SDIOD_DMA_INTR_ENA               (0x00C4)
#define SDIOD_DMA_MISS_BLK_BUF_OVR_CNT   (0x00C8)
#define SDIOD_DMA_RX_STAT_WD_TIMER       (0x00CC)
#define SDIOD_DMA_CURR_TDSC              (0x00D0)
#define SDIOD_DMA_CURR_RDSC              (0x00D4)
#define SDIOD_DMA_CURR_TBUF              (0x00D8)
#define SDIOD_DMA_CURR_RBUF              (0x00DC)
#define SDIOD_DMA_CH_PRIOR               (0x00E0)
#define SDIOD_DMA_CH_STATUS              (0x00E4)
#define SDIOD_DMA_CH_SOFT_CLOSE          (0x00E8)
#define SDIOD_DMA_CH_OUTSTD_CFG          (0x00EC)
#define SDIOD_DMA_CH_OUTSTD_CLR          (0x00F0)
#define SDIOD_DMA_ATC_TCH_MONITOR        (0x00F4)
#define SDIOD_DMA_ATC_RCH_MONITOR        (0x00F8)
#define SDIOD_DMA_ARC_TCH_MONITOR        (0x00FC)
#define SDIOD_DMA_ARC_RCH_MONITOR        (0x0100)
#define SDIOD_DMA_ATC_TCH_INTR_EN        (0x0104)
#define SDIOD_DMA_ATC_TCH_INTR_RAW       (0x0108)
#define SDIOD_DMA_ATC_TCH_INTR_STATUS    (0x010C)
#define SDIOD_DMA_ATC_RCH_INTR_EN        (0x0110)
#define SDIOD_DMA_ATC_RCH_INTR_RAW       (0x0114)
#define SDIOD_DMA_ATC_RCH_INTR_STATUS    (0x0118)
#define SDIOD_DMA_ARC_TCH_INTR_EN        (0x011C)
#define SDIOD_DMA_ARC_TCH_INTR_RAW       (0x0120)
#define SDIOD_DMA_ARC_TCH_INTR_STATUS    (0x0124)
#define SDIOD_DMA_ARC_RCH_INTR_EN        (0x0128)
#define SDIOD_DMA_ARC_RCH_INTR_RAW       (0x012C)
#define SDIOD_DMA_ARC_RCH_INTR_STATUS    (0x0130)
#define SDIOD_DMA_CH_INTR_EN             (0x0134)
#define SDIOD_DMA_CH_INTR_RAW            (0x0138)
#define SDIOD_DMA_CH_INTR_STATUS         (0x013C)
#define SDIOD_DMA_TCH_DEBUG              (0x0140)
#define SDIOD_DMA_ATC_TCH_RD_DEBUG0      (0x0144)
#define SDIOD_DMA_ATC_TCH_RD_DEBUG1      (0x0148)
#define SDIOD_DMA_ATC_TCH_WR_DEBUG0      (0x014C)
#define SDIOD_DMA_ATC_TCH_WR_DEBUG1      (0x0150)
#define SDIOD_DMA_ATC_TCH_DEBUG          (0x0154)
#define SDIOD_DMA_ARC_TCH_RD_DEBUG0      (0x0158)
#define SDIOD_DMA_ARC_TCH_RD_DEBUG1      (0x015C)
#define SDIOD_DMA_ARC_TCH_WR_DEBUG0      (0x0160)
#define SDIOD_DMA_ARC_TCH_WR_DEBUG1      (0x0164)

#define U_ram1_BASEADDR    0x70010000
#define D_DATA_TXBUF_ADDR   ((volatile unsigned *)(U_ram1_BASEADDR + 0x0))
#define D_DATA_RXBUF_ADDR   ((volatile unsigned *)(U_ram1_BASEADDR + 0x6000))
#define D_DESC_TXBUF_ADDR   ((volatile unsigned *)(U_ram1_BASEADDR + 0xc000))
#define D_DESC_RXBUF_ADDR   ((volatile unsigned *)(U_ram1_BASEADDR + 0xe000))

static int     sdiodrvr_open(FAR struct file *filep);
static int     sdiodrvr_close(FAR struct file *filep);
static ssize_t sdiodrvr_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t sdiodrvr_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     sdiodrvr_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     sdiodrvr_unlink(FAR struct inode *inode);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sdio_putreg32(v, a)  (*(volatile uint32_t *)(a) = (v))
#define sdio_getreg32(a)     (*(volatile uint32_t *)(a))

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct dw_sdio_dev_s
{
  struct sdio_dev_s *dev;
  struct clk *mclk;
  struct clk *pclk;
  uint32_t base;
  uint32_t crefs;
  sem_t sem;
};

static const struct file_operations sdiodrvr_fops =
{
  sdiodrvr_open,    /* open */
  sdiodrvr_close,   /* close */
  sdiodrvr_read,    /* read */
  sdiodrvr_write,   /* write */
  NULL,             /* seek */
  sdiodrvr_ioctl,   /* ioctl */
  NULL,             /* poll */
  sdiodrvr_unlink,  /* unlink */
};

static int sdiodrvr_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct dw_sdio_dev_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct dw_sdio_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  ret = nxsem_wait(&priv->sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxsem_post(&priv->sem);
  return 0;
}

static int sdiodrvr_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct dw_sdio_dev_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct i2c_driver_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the I2C driver state structure */

  ret = nxsem_wait(&priv->sem);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then commit Hara-Kiri now.
   */

  if (priv->crefs < 0)
    {
      nxsem_destroy(&priv->sem);
      kmm_free(priv);
      return OK;
    }

  nxsem_post(&priv->sem);
  return 0;
}

static ssize_t sdiodrvr_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  return 0;
}

static ssize_t sdiodrvr_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return 0;
}

static int sdiodrvr_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  return 0;
}

static int sdiodrvr_unlink(FAR struct inode *inode)
{
 return 0;
}


static int song_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t nbytes)
{
  struct dw_sdio_dev_s *priv = (struct dw_sdio_dev_s *)dev;

  DEBUGASSERT(priv->mclk != NULL && priv->pclk != NULL && priv->dev != NULL);
}

static int song_recvsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t nbytes)
{
  struct dw_sdio_dev_s *priv = (struct dw_sdio_dev_s *)dev;

  DEBUGASSERT(priv->mclk != NULL && priv->pclk != NULL && priv->dev != NULL);

  /* ARC recv prior is seven */

  sdio_putreg32(0x3f, (priv->base + SDIOD_DMA_CH_PRIOR));

  /* Interrupt enable and rx interrupt enable */

  sdio_putreg32(0x10040, (priv->base + SDIOD_DMA_INTR_ENA));

  /* SDIO OP MODE start dma recv ,don't clean up recv frame */

  sdio_putreg32(0x21, (priv->base + SDIOD_DMA_OP_MODE));

  /* Set DMA startup address */

  sdio_putreg32(0, (priv->base + SDIOD_DMA_RDSC_ADR));
}

struct sdio_dev_s g_sdiodev =
{
  .sendsetup = song_sendsetup,
  .recvsetup = song_recvsetup,
};

static void sdiod_des_config(uint32_t sdio_base, uint8_t direction, int blk_size, int blk_num, int data_addr, int desc_start_addr)
{
  uint32_t *desc_addr;
  uint32_t desc0;
  uint32_t desc1;
  uint32_t desc2;
  uint32_t desc3;

  if (direction == 1)
    {
      sdio_putreg32(desc_start_addr, (sdio_base + SDIOD_DMA_RDSC_ADR));

      desc_addr = (uint32_t *)(desc_start_addr);
      for (int i = 0; i < blk_num; i++)
      {
        /* DMA OWNER */

        desc0 = 0x80000000;
        desc1 = blk_size | 0x2000;

        /* RX_BUFFER_ADDR */

        desc2 = data_addr + blk_size * i;
        desc3 = desc_start_addr + 16 + i * 16;
        *desc_addr = desc0;
        desc_addr++;
        *desc_addr = desc1;
        desc_addr++;
        *desc_addr = desc2;
        desc_addr++;
        *desc_addr = desc3;
        desc_addr++;
      }
    }

  /* Tx descriptor init */

  else
    {
      sdio_putreg32(desc_start_addr, (sdio_base + SDIOD_DMA_TDSC_ADR));
      desc_addr = (uint32_t *)(desc_start_addr);

      for (int i = 0; i < blk_num; i++)
        {
          /* DMA OWNER */

          desc0 = 0xf0100000;
          desc1 = blk_size;

          /* RX_BUFFER_ADDR */

          desc2 = data_addr + blk_size * i;
          desc3 = desc_start_addr + 16 + i * 16;
          *desc_addr = desc0;
          desc_addr++;
          *desc_addr = desc1;
          desc_addr++;
          *desc_addr = desc2;
          desc_addr++;
          *desc_addr = desc3;
          desc_addr++;
        }
    }
}

static int song_sdio_initialize(uint32_t base)
{
  //uint32_t data[4] = {0x12345678, 0x23456789, 0x3456789a, 0x456789ab};
  uint32_t data[4] = {0x11223344, 0x55667788, 0xaabbccdd, 0x116699aa};

  /* SDCAD Power up */

  sdio_putreg32(0x01, (SDIOD_CARD_PWR + base));

  /* Rx descriptor init */

  sdiod_des_config(base, 1, 0x10, 1, D_DATA_RXBUF_ADDR, D_DESC_RXBUF_ADDR);

  /* Tx descriptor init */

  sdiod_des_config(base, 0, 0x10, 1, D_DATA_TXBUF_ADDR, D_DESC_TXBUF_ADDR);

  for(int i =0;i < 4;i++)
    sdio_putreg32(data[i], 0x70010000 + i * 4);

  /* ARC_RCH priori: 7, ARC_TCH priori:3 */

  sdio_putreg32(0x3f | sdio_getreg32(base + SDIOD_DMA_CH_PRIOR), (base + SDIOD_DMA_CH_PRIOR));

  /* Interrupt Enable, Rx interrupt enable, Sum interrupt enable and tx interrupt */

  sdio_putreg32(0x10041 | sdio_getreg32(base + SDIOD_DMA_INTR_ENA), (base + SDIOD_DMA_INTR_ENA));

  /* OP_Mode, Rx flush disable, start rx and tx */

  sdio_putreg32(0x29 | sdio_getreg32(base + SDIOD_DMA_OP_MODE), (base + SDIOD_DMA_OP_MODE));

  /* Set DMA Rx and Tx poll */

  sdio_putreg32(0x5678, (base + SDIOD_DMA_RX_POLL));
  sdio_putreg32(0x5678, (base + SDIOD_DMA_TX_POLL));

  return 0;
}

static int song_sdio_isr(int irq, FAR void *context, FAR void *arg)
{
  int rdata;
  struct dw_sdio_dev_s *sdio = (struct dw_sdio_dev_s *)arg;

  rdata = sdio_getreg32(sdio->base + SDIOD_DMA_STATUS);

  /* Clear interrupt flag */

  sdio_putreg32(0x01 << 0x06, (sdio->base + SDIOD_DMA_STATUS));
  sdio_putreg32(0x01 << 0x00, (sdio->base + SDIOD_DMA_STATUS));

  /* 4 * sizeof(int32) */

//  sdiod_des_config(1, 0x10, 1, (D_DATA_RXBUF_ADDR + 0x04), (D_DESC_RXBUF_ADDR + 0x4));

  /* Reset Rx description */

  sdiod_des_config(sdio->base, 1, 0x10, 1, (D_DATA_RXBUF_ADDR), (D_DESC_RXBUF_ADDR));

  /* Reset Tx description */

  sdiod_des_config(sdio->base, 0, 0x10, 1, (D_DATA_TXBUF_ADDR), (D_DESC_TXBUF_ADDR));

  sdio_putreg32(0x5678, (sdio->base + SDIOD_DMA_RX_POLL));
  sdio_putreg32(0x5678, (sdio->base + SDIOD_DMA_TX_POLL));
}

void dw_sdio_allinitialize(FAR const struct dw_sdio_config_s *config, int config_num, FAR struct sdio_dev_s *sdio_dev)
{
  struct dw_sdio_dev_s *sdio;
  struct clk *clk;
  int ret;

  DEBUGASSERT(config && config->base && config->irq && config->mclk && config->pclk);

  sdio = kmm_zalloc(sizeof(struct dw_sdio_dev_s));
  sdio->base = config->base;

  clk = clk_get(config->clk_en);
  if (clk != NULL)
    clk_enable(clk);
  else
    syslog(0,"sdio_clk is not find\n");

  /* Low level call back */

  sdio->dev = &g_sdiodev;

  ret = nxsem_init(&sdio->sem, 0, 0);
  if (ret < 0)
    goto sem_err;

  ret = nxsem_setprotocol(&sdio->sem, SEM_PRIO_NONE);
  if (ret < 0)
    goto sem_prio_err;

  if (config->mclk != NULL)
  {
    sdio->mclk = clk_get(config->mclk);
    if (!sdio->mclk)
      {
        syslog(0, "sdio:%p mclk invalid\n", config->base);
      }
  }

  if (config->pclk != NULL)
  {
    sdio->pclk = clk_get(config->pclk);
    if (!sdio->pclk)
      {
        syslog(0, "sdio:%p pclk invalid\n", config->base);
      }
  }

  if (config->rate != 0)
  {
    ret = clk_set_rate(sdio->mclk, config->rate);
    if (ret)
      {
        syslog(0, "sdio:%p mclk set rate failed\n", config->base);
        goto sem_prio_err;
      }
  }

  if (sdio->pclk != NULL)
    ret = clk_enable(sdio->pclk);
  if (sdio->mclk != NULL)
    ret = clk_enable(sdio->mclk);

  ret = song_sdio_initialize(sdio->base);

  if(config->irq >= 0)
    {
      ret = irq_attach(config->irq, song_sdio_isr, sdio);
      if (ret < 0)
        {
          syslog(0, "irq attach failed\n");
          goto sem_prio_err;
        }
        up_enable_irq(config->irq);
    }

  ret = register_driver("/dev/sdio", &sdiodrvr_fops, 0666, sdio);
  if (ret < 0)
    {
      kmm_free(sdio);
      return ret;
    }

  return 0;

sem_prio_err:
  nxsem_destroy(&sdio->sem);
sem_err:
  free(sdio);
  return NULL;
}
