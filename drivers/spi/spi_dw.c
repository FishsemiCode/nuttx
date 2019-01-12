/****************************************************************************
 * drivers/spi/spi_dw.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Qian Wenfa <qianwenfa@pinecone.net>
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
 * POSPIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdio.h>
#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/dma.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/spi/spi_transfer.h>

#define SPI_CTRL0_DFS_MASK_V4   (0x1f << 16)
#define SPI_CTRL0_DFS_SHIFT_V4  (16)
#define SPI_CTRL0_TMOD_MASK     (3 << 8)
#define SPI_CTRL0_TMOD_RXTX     (0 << 8)
#define SPI_CTRL0_TMOD_TX       (1 << 8)
#define SPI_CTRL0_TMOD_RX       (2 << 8)
#define SPI_CTRL0_OLPH_MASK     (3 << 6)
#define SPI_CTRL0_OLPH_SHIFT    (6)
#define SPI_CTRL0_FRF_MASK      (3 << 4)
#define SPI_CTRL0_FRF_MOTO      (0 << 4)
#define SPI_CTRL0_FRF_TI        (1 << 4)
#define SPI_CTRL0_DFS_MASK_V3   (0xf << 0)
#define SPI_CTRL0_DFS_SHIFT_V3  (0)

#define SPI_EN_ENABLE           (1 << 0)

#define SPI_BAUD_MASK           (0xfffe)

#define SPI_STS_BUSY            (1 << 0)

#define SPI_INT_MASK            (0x1f << 0)
#define SPI_INT_RXFI            (1 << 4)
#define SPI_INT_TXEI            (1 << 0)

#define SPI_DMAC_RDMAE          (1 << 0)
#define SPI_DMAC_TDMAE          (1 << 1)

#ifndef MIN
#  define MIN(a,b)              ((a) < (b) ? (a) : (b))
#endif

enum dw_spi_version
{
  DW_SPI_V3 = 0,
  DW_SPI_V4,
};

struct dw_spi_hw_s
{
  volatile uint32_t CTRL0;
  volatile uint32_t CTRL1;
  volatile uint32_t EN;
  volatile uint32_t RESERVE0;
  volatile uint32_t SE;
  volatile uint32_t BAUD;
  volatile uint32_t TXFTL;
  volatile uint32_t RXFTL;
  volatile uint32_t TXFL;
  volatile uint32_t RXFL;
  volatile uint32_t STS;
  volatile uint32_t IE;
  volatile uint32_t IS;
  volatile uint32_t RIS;
  volatile uint32_t TXOIC;
  volatile uint32_t RXOIC;
  volatile uint32_t RXUIC;
  volatile uint32_t RESERVE1;
  volatile uint32_t IC;
  volatile uint32_t DMAC;
  volatile uint32_t DMATDL;
  volatile uint32_t DMARDL;
  volatile uint32_t IDR;
  volatile uint32_t VERSION;
  volatile uint32_t DATA;
};

struct dw_spi_s
{
  struct spi_dev_s spi_dev;
  struct clk *clk;
  const struct dw_spi_config_s *config;
  struct ioexpander_dev_s *ioe;
  struct dma_chan_s *tx_chan;
  struct dma_chan_s *rx_chan;
  enum dw_spi_version ver;
  mutex_t mutex;
  sem_t sem;
  uint32_t fifo_len;
  uint32_t n_bytes;
  const void *tx;
  const void *tx_end;
  void *rx;
  void *rx_end;
  size_t len;
};

struct dw_spi_dfs_set
{
  uint32_t mask;
  uint32_t shift;
};

static const struct dw_spi_dfs_set g_dw_spi_dfs_set[2] =
{
  { SPI_CTRL0_DFS_MASK_V3, SPI_CTRL0_DFS_SHIFT_V3 },
  { SPI_CTRL0_DFS_MASK_V4, SPI_CTRL0_DFS_SHIFT_V4 },
};

static void dw_spi_update_reg(volatile uint32_t *reg, uint32_t bits, uint32_t value)
{
  uint32_t tmp = *reg;

  tmp &= ~bits;
  tmp |= value & bits;
  *reg = tmp;
}

static uint32_t dw_spi_tx_max(struct dw_spi_s *spi)
{
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;
  uint32_t tx_left, tx_room, rxtx_gap;

  tx_left = (spi->tx_end - spi->tx) / spi->n_bytes;
  tx_room = spi->fifo_len - hw->TXFL;

  rxtx_gap = ((spi->rx_end - spi->rx) / spi->n_bytes) - tx_left;

  return MIN(MIN(tx_left, tx_room), (spi->fifo_len - rxtx_gap));
}

static uint32_t dw_spi_write_fifo(struct dw_spi_s *spi)
{
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;
  uint32_t max = dw_spi_tx_max(spi);
  uint32_t written = max;
  uint16_t txw = 0;

  while (max--)
    {
      if (spi->tx_end - spi->len)
        {
          if (spi->n_bytes == 1)
            txw = *(uint8_t *)(spi->tx);
          else
            txw = *(uint16_t *)(spi->tx);
        }
      hw->DATA = txw;
      spi->tx += spi->n_bytes;
    }

  return written;
}

static uint32_t dw_spi_rx_max(struct dw_spi_s *spi)
{
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;
  uint32_t rx_left = (spi->rx_end - spi->rx) / spi->n_bytes;

  return MIN(rx_left, hw->RXFL);
}

static uint32_t dw_spi_read_fifo(struct dw_spi_s *spi)
{
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;
  uint32_t max = dw_spi_rx_max(spi);
  uint32_t read = max;
  uint16_t rxw;

  while (max--)
    {
      rxw = hw->DATA;
      if (spi->rx_end - spi->len)
        {
          if (spi->n_bytes == 1)
            *(uint8_t *)(spi->rx) = rxw;
          else
            *(uint16_t *)(spi->rx) = rxw;
        }
      spi->rx += spi->n_bytes;
    }

  return read;
}

static inline void dw_spi_enable(struct dw_spi_hw_s *hw, bool enable)
{
  hw->EN = enable ? SPI_EN_ENABLE : 0;
}

static void dw_spi_set_duplex(struct dw_spi_hw_s *hw, uint32_t duplex, uint32_t nwords)
{
  if (duplex == SPI_CTRL0_TMOD_RX)
    {
      hw->CTRL1 = nwords - 1;
    }

  dw_spi_update_reg(&hw->CTRL0, SPI_CTRL0_TMOD_MASK, duplex);
}

static inline void dw_spi_set_tfifo_thresh(struct dw_spi_hw_s *hw, uint8_t thresh)
{
  hw->TXFTL = thresh;
}

static inline void dw_spi_set_rfifo_thresh(struct dw_spi_hw_s *hw, uint8_t thresh)
{
  hw->RXFTL = thresh;
}

static inline void dw_spi_mask_intr(struct dw_spi_hw_s *hw, uint32_t mask)
{
  dw_spi_update_reg(&hw->IE, mask, 0);
}

static inline void dw_spi_unmask_intr(struct dw_spi_hw_s *hw, uint32_t mask)
{
  dw_spi_update_reg(&hw->IE, mask, mask);
}

/****************************************************************************
 * Name: SPI_LOCK
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int dw_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  struct dw_spi_s *dw_spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  int ret;

  if (lock)
    ret = nxmutex_lock(&dw_spi->mutex);
  else
    ret = nxmutex_unlock(&dw_spi->mutex);

  return ret;
}

/****************************************************************************
 * Name: SPI_SELECT
 *
 * Description:
 *   Enable/disable the SPI chip select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *   Required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dw_spi_select(FAR struct spi_dev_s *dev,
                              uint32_t devid, bool selected)
{
  struct dw_spi_s *spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  const struct dw_spi_config_s *config = spi->config;
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)config->base;
  uint32_t cs = SPIDEVID_INDEX(devid);
  bool cs_values[config->cs_num];
  int i;

  if (cs >= config->cs_num)
    {
      spierr("devid %d out of device range %d\n", devid, config->cs_num);
      return;
    }

  if (spi->ioe)
    {
      for (i = 0; i < config->cs_num; i++)
        cs_values[i] = true;

      cs_values[cs] = !selected;
      IOEXP_MULTIWRITEPIN(spi->ioe, (uint8_t *)config->cs_gpio,
          cs_values, config->cs_num);

      /* in IOE mode, we can expand the cs_num to more than the available SE bits.
       * thus, to avoid set an invalid SE bit, we simply set bit 0 in IOE mode */
      hw->SE = selected;
    }
  else
    {
      if (selected)
        hw->SE = 1 << cs;
      else
        hw->SE = 0;
    }
}

/****************************************************************************
 * Name: SPI_SETFREQUENCY
 *
 * Description:
 *   Set the SPI frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t dw_spi_setfrequency(FAR struct spi_dev_s *dev,
                                            uint32_t frequency)
{
  struct dw_spi_s *spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  int ret;

  ret = clk_set_rate(spi->clk, frequency);
  if (ret < 0)
    spierr("DW_SPI-%p set freq to %d failed:%d\n", spi->config->base, frequency, ret);

  return clk_get_rate(spi->clk);
}
/****************************************************************************
 * Name: SPI_SETMODE
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void dw_spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct dw_spi_s *spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  const struct dw_spi_config_s *config = spi->config;
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)config->base;

  dw_spi_update_reg(&hw->CTRL0, SPI_CTRL0_OLPH_MASK, mode << SPI_CTRL0_OLPH_SHIFT);
}

/****************************************************************************
 * Name: SPI_SETBITS
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits in an SPI word.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void dw_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  struct dw_spi_s *spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  const struct dw_spi_config_s *config = spi->config;
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)config->base;

  DEBUGASSERT(nbits >= 4 || nbits <= 16);

  if (nbits <= 8)
    spi->n_bytes = 1;
  else
    spi->n_bytes = 2;

  dw_spi_update_reg(&hw->CTRL0, g_dw_spi_dfs_set[spi->ver].mask,
                (nbits - 1) << g_dw_spi_dfs_set[spi->ver].shift);
}

/****************************************************************************
 * Name: SPI_SEND
 *
 * Description:
 *   Exchange one word on SPI. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

static uint16_t dw_spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  struct dw_spi_s *spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;

  if (clk_enable(spi->clk) < 0)
    {
      spierr("DW_SPI-%p enable clock failed:%d\n", spi->config->base, ret);
      return 0;
    }

  dw_spi_set_duplex(hw, SPI_CTRL0_TMOD_RXTX, 1);
  dw_spi_enable(hw, true);
  hw->DATA = wd;
  while (!hw->RXFL || (hw->STS & SPI_STS_BUSY));
  wd = hw->DATA;
  dw_spi_enable(hw, false);
  clk_disable(spi->clk);

  return wd;
}

#ifdef CONFIG_SPI_EXCHANGE
static void dw_spi_dma_cb(FAR struct dma_chan_s *chan, FAR void *arg, ssize_t len)
{
  struct dw_spi_s *spi = arg;

  nxsem_post(&spi->sem);
}

static void dw_spi_dma_transfer(FAR struct dw_spi_s *spi,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords)
{
  const struct dw_spi_config_s *config = spi->config;
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)config->base;
  struct dma_config_s cfg;
  uint32_t duplex;
  int ret;

  if (rxbuffer && txbuffer)
    duplex = SPI_CTRL0_TMOD_RXTX;
  else if (txbuffer)
    duplex = SPI_CTRL0_TMOD_TX;
  else
    duplex = SPI_CTRL0_TMOD_RX;

  dw_spi_set_duplex(hw, duplex, nwords);
  dw_spi_enable(hw, true);

  if (rxbuffer)
    {
      hw->DMAC |= SPI_DMAC_RDMAE;
      /* XXX: what the threshold exactly */
      hw->DMARDL = 4;

      memset(&cfg, 0, sizeof(cfg));
      cfg.direction = DMA_DEV_TO_MEM;
      cfg.src_width = spi->n_bytes;
      DMA_CONFIG(spi->rx_chan, &cfg);
      DMA_START(spi->rx_chan, dw_spi_dma_cb,
                spi, up_addrenv_va_to_pa(rxbuffer),
                up_addrenv_va_to_pa(&hw->DATA), nwords);
    }

  if (txbuffer)
    {
      hw->DMAC |= SPI_DMAC_TDMAE;
      /* XXX: what the threshold exactly */
      hw->DMATDL = 4;

      memset(&cfg, 0, sizeof(cfg));
      cfg.direction = DMA_MEM_TO_DEV;
      cfg.dst_width = spi->n_bytes;
      DMA_CONFIG(spi->tx_chan, &cfg);

      up_clean_dcache((uintptr_t)txbuffer, (uintptr_t)txbuffer + nwords * spi->n_bytes);
      DMA_START(spi->tx_chan, rxbuffer ? NULL : dw_spi_dma_cb,
                spi, up_addrenv_va_to_pa(&hw->DATA),
                up_addrenv_va_to_pa((void *)txbuffer), nwords);
    }
  else /* RECEIVE-only mode, we need to trigger the transfer */
    {
      hw->DATA = 0;
    }

  do
    {
      ret = nxsem_wait(&spi->sem);
    }
  while (ret == -EINTR);

  if (ret)
    spierr("DW_SPI-%p transfer ret=%d\n", config->base, ret);

  if (rxbuffer)
    up_invalidate_dcache((uintptr_t)rxbuffer, (uintptr_t)rxbuffer + nwords * spi->n_bytes);
  else
    /* we still have some data in the fifo for TX-only mode,
     * let's poll for the transfer finish */
     while (hw->STS & SPI_STS_BUSY);

  hw->DMAC = 0;
  dw_spi_enable(hw, false);
}

static void dw_spi_cpu_transfer(FAR struct dw_spi_s *spi,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords)
{
  const struct dw_spi_config_s *config = spi->config;
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)config->base;
  int ret;

  spi->len = nwords * spi->n_bytes;
  spi->tx = txbuffer;
  spi->tx_end = txbuffer + spi->len;
  spi->rx = rxbuffer;
  spi->rx_end = rxbuffer + spi->len;

  /* set to tx/rx mode */
  dw_spi_set_duplex(hw, SPI_CTRL0_TMOD_RXTX, nwords);
  dw_spi_set_tfifo_thresh(hw, MIN(spi->fifo_len / 2, nwords));

  /* enable the interrupt to tigger the transfer procedure */
  dw_spi_unmask_intr(hw, SPI_INT_TXEI);
  dw_spi_enable(hw, true);

  do
    {
      ret = nxsem_wait(&spi->sem);
    }
  while (ret == -EINTR);

  if (ret)
    spierr("DW_SPI-%p transfer ret=%d\n", config->base, ret);

  dw_spi_enable(hw, false);
  dw_spi_mask_intr(hw, 0xff);
}

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dw_spi_exchange(FAR struct spi_dev_s *dev,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords)
{
  struct dw_spi_s *spi = container_of(dev,
                                struct dw_spi_s, spi_dev);
  int ret;
  size_t nsize;
  bool using_dma;

  if (!nwords || (!txbuffer && !rxbuffer))
    {
      return;
    }

  ret = clk_enable(spi->clk);
  if (ret < 0)
    {
      spierr("DW_SPI-%p enable clock failed:%d\n", spi->config->base, ret);
      return;
    }

  /* using dma only when: the data is longer than the fifo length and
   * the dma channel is valid. additionally, the rx buffer should be 4-aligned */
  using_dma = (nwords > spi->fifo_len) &&
      (txbuffer ? !!spi->tx_chan : true) &&
      (rxbuffer ? (spi->rx_chan && !((uintptr_t)rxbuffer & 0x03)) : true);

  if (using_dma)
    {
      while (nwords > 0)
        {
          if (!txbuffer && nwords > UINT16_MAX)
            nsize = UINT16_MAX;
          else
            nsize = nwords;

          dw_spi_dma_transfer(spi, txbuffer, rxbuffer, nsize);

          if (txbuffer)
            txbuffer += nsize * spi->n_bytes;
          if (rxbuffer)
            rxbuffer += nsize * spi->n_bytes;

          nwords -= nsize;
        }
    }
  else
    dw_spi_cpu_transfer(spi, txbuffer, rxbuffer, nwords);

  clk_disable(spi->clk);
}
#endif

static const struct spi_ops_s dw_spi_ops =
{
  .lock = dw_spi_lock,
  .select = dw_spi_select,
  .setfrequency = dw_spi_setfrequency,
  .setmode = dw_spi_setmode,
  .setbits = dw_spi_setbits,
  .send = dw_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange = dw_spi_exchange,
#endif
};

static int dw_spi_isr(int irq, void *context, FAR void *arg)
{
  struct dw_spi_s *spi = arg;
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;
  uint16_t irq_status = hw->IS & SPI_INT_MASK;
  uint16_t rxlevel;

  if (!irq_status)
    goto none_irq;

  dw_spi_read_fifo(spi);
  if (spi->rx_end == spi->rx)
    {
      dw_spi_mask_intr(hw, SPI_INT_RXFI);
      goto transfer_done;
    }

  rxlevel = MIN(spi->fifo_len, (spi->rx_end - spi->rx) / spi->n_bytes - 1);
  if ((spi->tx_end == spi->tx))
    {
        /* dynamically update RX threshold if TX data has been totally got into fifo */
        dw_spi_set_rfifo_thresh(hw, rxlevel);
    }
  else
    {
      dw_spi_write_fifo(spi);
      if ((spi->tx_end == spi->tx))
        {
          /* now that all the TX data has been got into the fifo, we disable
           * the TXEI interrupt and enable the RXFI interrupt to reduce the
           * interrupt times */
          dw_spi_mask_intr(hw, SPI_INT_TXEI);
          dw_spi_set_rfifo_thresh(hw, rxlevel);
          dw_spi_unmask_intr(hw, SPI_INT_RXFI);
        }
    }

  return 0;

transfer_done:
  nxsem_post(&spi->sem);
none_irq:
  return 0;
}

static int dw_spi_cs_init(FAR struct dw_spi_s *spi)
{
  const struct dw_spi_config_s *config = spi->config;
  int i, ret;

  if (spi->ioe)
    {
      for (i = 0; i < config->cs_num; i++)
        {
          ret = IOEXP_SETDIRECTION(spi->ioe, config->cs_gpio[i], IOEXPANDER_DIRECTION_OUT);
          if (ret)
            {
              spierr("set direction for cs gpio %d failed\n", config->cs_gpio[i]);
              return -EIO;
            }
        }
    }

  dw_spi_select(&spi->spi_dev, 0, false);

  return OK;
}

static void dw_spi_hw_init(struct dw_spi_s *spi)
{
  struct dw_spi_hw_s *hw = (struct dw_spi_hw_s *)spi->config->base;
  uint32_t mode;
  uint32_t fifo;
  char ver;

  /* initially disable the controller */
  dw_spi_enable(hw, false);

  /* initially disable all the interrupt */
  dw_spi_mask_intr(hw, 0xff);

  /* set to MOTO spi mode */
  dw_spi_update_reg(&hw->CTRL0, SPI_CTRL0_FRF_MASK, SPI_CTRL0_FRF_MOTO);
  if (spi->config->mode_ctrl)
    {
      mode = 1 << spi->config->mode_sel_bit;
      dw_spi_update_reg((volatile uint32_t *)spi->config->mode_ctrl, mode, mode);
    }

  /* detect the fifo depth */
  for (fifo = 2; fifo <= 257; fifo++)
    {
      hw->TXFTL = fifo;
      if (fifo != hw->TXFTL)
        break;
    }
  spi->fifo_len = (fifo == 257) ? 0 : fifo;
  hw->TXFTL = 0;

  /* get the IP version */
  ver = hw->VERSION >> 24;
  if (ver == '3')
    spi->ver = DW_SPI_V3;
  else if (ver == '4')
    spi->ver = DW_SPI_V4;

  dw_spi_cs_init(spi);
}

FAR struct spi_dev_s *dw_spi_initialize(FAR const struct dw_spi_config_s *config,
                                        FAR struct ioexpander_dev_s *ioe,
                                        FAR struct dma_dev_s *dma)
{
  struct dw_spi_s *spi;
  char name[32];
  int ret;

  DEBUGASSERT(config && config->cs_num &&
              config->cs_num <= CONFIG_SPI_DW_MAX_CS &&
              config->base && config->irq);

  spi = kmm_zalloc(sizeof(*spi));
  if (!spi)
    {
      spierr("no memory for spi %p\n", config->base);
      return NULL;
    }

  sprintf(name, "spi%d_clk", config->bus);
  spi->clk = clk_register_divider(name, config->mclk, CLK_SET_RATE_PARENT,
          config->base + offsetof(struct dw_spi_hw_s, BAUD),
          0, 16, CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_DIV_NEED_EVEN);
  if (!spi->clk)
    {
      spierr("DW_SPI-%p create clock failed\n", config->base);
      kmm_free(spi);
      return NULL;
    }

  spi->spi_dev.ops = &dw_spi_ops;
  spi->config = config;
  spi->ioe = ioe;
  if (dma)
    {
      spi->tx_chan = DMA_GET_CHAN(dma, config->tx_dma);
      spi->rx_chan = DMA_GET_CHAN(dma, config->rx_dma);
    }

  ret = nxmutex_init(&spi->mutex);
  if (ret)
    goto mutex_err;

  ret = nxsem_init(&spi->sem, 0, 0);
  if (ret)
    goto sem_err;

  ret = nxsem_setprotocol(&spi->sem, SEM_PRIO_NONE);
  if (ret)
    goto sem_prio_err;

  dw_spi_hw_init(spi);

  ret = irq_attach(config->irq, dw_spi_isr, spi);
  if (ret)
    goto irq_err;

  up_enable_irq(config->irq);

#ifdef CONFIG_SPI_DRIVER
  if (config->bus >= 0)
    spi_register(&spi->spi_dev, config->bus);
#endif

  return &spi->spi_dev;

irq_err:
sem_prio_err:
  nxsem_destroy(&spi->sem);
sem_err:
  nxmutex_destroy(&spi->mutex);
mutex_err:
  kmm_free(spi);
  return NULL;
}

void dw_spi_allinitialize(FAR const struct dw_spi_config_s *config, int config_num,
                          FAR struct ioexpander_dev_s *ioe,
                          FAR struct dma_dev_s *dma,
                          FAR struct spi_dev_s **spi)
{
  struct spi_dev_s *spi_dev;
  int i;

  for (i = 0; i < config_num; i++)
    {
      spi_dev = dw_spi_initialize(&config[i], ioe, dma);
      if (spi_dev && config[i].bus >= 0)
        {
          spi[config[i].bus] = spi_dev;
        }
    }
}
