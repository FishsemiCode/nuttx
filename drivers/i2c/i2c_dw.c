/****************************************************************************
 * drivers/i2c/i2c_dw.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Dong Jiuzhu<dongjiuzhu@pinecone.net>
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

#include <errno.h>
#include <nuttx/clk/clk.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Controller config bits */

#define DW_I2C_CON_MASTER                   0x01
#define DW_I2C_CON_SPEED_STD                0x02
#define DW_I2C_CON_SPEED_FAST               0x04
#define DW_I2C_CON_SPEED_HIGH               0x06
#define DW_I2C_CON_SPEED_MASK               0x06
#define DW_I2C_CON_10BITADDR_MASTER         0x10
#define DW_I2C_CON_RESTART_EN               0x20
#define DW_I2C_CON_SLAVE_DISABLE            0x40

#define DW_I2C_CONTROLLER_CONFIG  (DW_I2C_CON_MASTER | \
                DW_I2C_CON_SLAVE_DISABLE | \
                DW_I2C_CON_RESTART_EN) \

/* I2c TAR bits */

#define DW_I2C_TAR_10BITADDR_MASTER         0x1000

/* I2c Data Cmd bits */

#define DW_I2C_DATA_CMD_DAT                 0x0ff
#define DW_I2C_DATA_CMD_READ                0x100
#define DW_I2C_DATA_CMD_STOP                0x200
#define DW_I2C_DATA_CMD_RESTART             0x400

/* I2c intr mask */

#define DW_I2C_INTR_RX_UNDER                0x001
#define DW_I2C_INTR_RX_OVER                 0x002
#define DW_I2C_INTR_RX_FULL                 0x004
#define DW_I2C_INTR_TX_OVER                 0x008
#define DW_I2C_INTR_TX_EMPTY                0x010
#define DW_I2C_INTR_RD_REQ                  0x020
#define DW_I2C_INTR_TX_ABRT                 0x040
#define DW_I2C_INTR_RX_DONE                 0x080
#define DW_I2C_INTR_ACTIVITY                0x100
#define DW_I2C_INTR_STOP_DET                0x200
#define DW_I2C_INTR_START_DET               0x400
#define DW_I2C_INTR_GEN_CALL                0x800

#define DW_I2C_INTR_DEFAULT_MASK  (DW_I2C_INTR_RX_FULL | \
                DW_I2C_INTR_TX_EMPTY | \
                DW_I2C_INTR_TX_ABRT | \
                DW_I2C_INTR_STOP_DET)

#define DW_I2C_STATUS_ACTIVITY              0x01

/* status codes */

#define DW_I2C_STATUS_IDLE                  0x00
#define DW_I2C_STATUS_WRITE_IN_PROGRESS     0x01
#define DW_I2C_STATUS_READ_IN_PROGRESS      0x02

#define DW_I2C_WAIT_BUS_FREE_TIMEOUT        MSEC2TICK(20)  /* 20ms */

#define DW_I2C_FORCE_UINT8(x)               ((x) & DW_I2C_DATA_CMD_DAT)

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)                     (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dw_i2c_hw_s
{
  volatile uint32_t CON;                /* 0x00 */
  volatile uint32_t TAR;                /* 0x04 */
  volatile uint32_t SAR;                /* 0x08 */
  volatile uint32_t HS_MADDR;           /* 0x0c */
  volatile uint32_t DATA_CMD;           /* 0x10 */
  volatile uint32_t SS_SCL_HCNT;        /* 0x14 */
  volatile uint32_t SS_SCL_LCNT;        /* 0x18 */
  volatile uint32_t FS_SCL_HCNT;        /* 0x1c */
  volatile uint32_t FS_SCL_LCNT;        /* 0x20 */
  volatile uint32_t HS_SCL_HCNT;        /* 0x24 */
  volatile uint32_t HS_SCL_LCNT;        /* 0x28 */
  volatile uint32_t INTR_STAT;          /* 0x2c */
  volatile uint32_t INTR_MASK;          /* 0x30 */
  volatile uint32_t RAW_INTR_STAT;      /* 0x34 */
  volatile uint32_t RX_TL;              /* 0x38 */
  volatile uint32_t TX_TL;              /* 0x3c */
  volatile uint32_t CLR_INTR;           /* 0x40 */
  volatile uint32_t CLR_RX_UNDER;       /* 0x44 */
  volatile uint32_t CLR_RX_OVER;        /* 0x48 */
  volatile uint32_t CLR_TX_OVER;        /* 0x4c */
  volatile uint32_t CLR_RD_REQ;         /* 0x50 */
  volatile uint32_t CLR_TX_ABRT;        /* 0x54 */
  volatile uint32_t CLR_RX_DONE;        /* 0x58 */
  volatile uint32_t CLR_ACTIVITY;       /* 0x5c */
  volatile uint32_t CLR_STOP_DET;       /* 0x60 */
  volatile uint32_t CLR_START_DET;      /* 0x64 */
  volatile uint32_t CLR_GEN_CALL;       /* 0x68 */
  volatile uint32_t ENABLE;             /* 0x6c */
  volatile uint32_t STATUS;             /* 0x70 */
  volatile uint32_t TXFLR;              /* 0x74 */
  volatile uint32_t RXFLR;              /* 0x78 */
  volatile uint32_t SDA_HOLD;           /* 0x7c */
  volatile uint32_t TX_ABRT_SOURCE;     /* 0x80 */
  volatile uint32_t SLV_DATA_NAK_ONLY;  /* 0x84 */
  volatile uint32_t DMA_CR;             /* 0x88 */
  volatile uint32_t DMA_TDLR;           /* 0x8c */
  volatile uint32_t DMA_RDLR;           /* 0x90 */
  volatile uint32_t SDA_SETUP;          /* 0x94 */
  volatile uint32_t ACK_GENERAL_CALL;   /* 0x98 */
  volatile uint32_t ENABLE_STATUS;      /* 0x9c */
  volatile uint32_t FS_SPKLEN;          /* 0xa0 */
  volatile uint32_t HS_SPKLEN;          /* 0xa4 */
  volatile uint32_t RESERVED4[19];
  volatile uint32_t COMP_PARAM_1;       /* 0xf4 */
  volatile uint32_t COMP_VERSION;       /* 0xf8 */
  volatile uint32_t COMP_TYPE;          /* 0xfc */
};

struct dw_i2c_timing_s
{
  uint32_t rate;
  uint32_t sda_hold;
  uint32_t fs_spklen;
  uint32_t hs_spklen;
  uint32_t ss_hcnt;
  uint32_t ss_lcnt;
  uint32_t fs_hcnt;
  uint32_t fs_lcnt;
  uint32_t hs_hcnt;
  uint32_t hs_lcnt;
};

struct dw_i2c_dev_s
{
  struct i2c_master_s dev;              /* I2c private data */
  struct clk *mclk;                     /* I2c master clk */
  FAR struct dw_i2c_hw_s *hw;           /* Hardware i2c controller info */

  uint32_t msgs_num;                    /* number of mesgs */
  struct i2c_msg_s *msgs;               /* msgs for master transfer */
  uint32_t internal_status;             /* Status of driver internal operation */

  uint32_t msg_write_idx;               /* current idx for master wirte msgs operation */
  uint32_t msg_read_idx;                /* current idx for master read msgs operation */
  uint32_t rx_outstanding;              /* cuttent bytes for rx fifo */

  /* master transfer using variate */
  uint32_t tx_buf_len;
  uint32_t rx_buf_len;
  uint8_t  *tx_buf;
  uint8_t  *rx_buf;
  uint32_t tx_fifo_depth;
  uint32_t rx_fifo_depth;

  sem_t sem;                            /* use to sync i2c transfer */
  mutex_t mutex;                        /* use to protect i2c transfer in multi process */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dw_i2c_timing_s g_i2c_timings[] =
{
  {16000000, 7, 1, 1, 62 , 92 , 14, 17, 6, 8},
  {25600000, 7, 1, 1, 120, 121, 26, 27, 6, 8},
  {26000000, 8, 1, 1, 100, 124, 16, 35, 4, 8},
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dw_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s dw_i2c_ops =
{
  .transfer = dw_i2c_transfer,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dw_i2c_get_timing
 *
 * Description:
 *   get timing config from static g_i2c_timings array
 *
 * Input Parameters:
 *   rate    - I2c contolller mclk
 *
 * Returned Value:
 *  NULL(don't find timing config according to rate)
 *  timing config pointer
 ****************************************************************************/

const struct dw_i2c_timing_s *dw_i2c_get_timing(uint32_t rate)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(g_i2c_timings); i++)
    {
      if (g_i2c_timings[i].rate == rate)
        return &g_i2c_timings[i];
    }

  return NULL;
}

/****************************************************************************
 * Name: dw_i2c_enable_disable
 *
 * Description:
 *   enable or disable i2c controller
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *   enable - true to enable or false to disable
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_enable_disable(FAR struct dw_i2c_dev_s *i2c, bool enable)
{
  int times = 10;

  do
    {
      i2c->hw->ENABLE = enable;
      if ((i2c->hw->ENABLE_STATUS & 1) == enable)
          return;

      /*
       * Wait 10 times the signaling period of the highest I2C
       * transfer supported by the driver (for 400KHz this is
       * 25us) as described in the DesignWare I2C databook.
       */
      up_udelay(25);
    }
  while (times--);
}

/****************************************************************************
 * Name: dw_i2c_disable
 *
 * Description:
 *   disable i2c controller and disable all interrupts
 *   a wrapper function
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_disable(FAR struct dw_i2c_dev_s *i2c)
{
  /* Disable controller */
  dw_i2c_enable_disable(i2c, false);

  /* Disable all interrupts */
  i2c->hw->INTR_MASK = 0;
  i2c->hw->CLR_INTR;
}

/****************************************************************************
 * Name: dw_i2c_enable
 *
 * Description:
 *   enable i2c controller
 *   a wrapper function
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_enable(FAR struct dw_i2c_dev_s *i2c)
{
  /* enforce disabled interrupts (due to HW issues) */
  i2c->hw->INTR_MASK = 0;

  /* Enable controller */
  dw_i2c_enable_disable(i2c, true);

  /* Clear and enable interrupts */
  i2c->hw->CLR_INTR;
  i2c->hw->INTR_MASK = DW_I2C_INTR_DEFAULT_MASK;
}

/****************************************************************************
 * Name: dw_i2c_wait_bus_nobusy
 *
 * Description:
 *   wait i2c bus no busy until timeout, the time is 20ms
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dw_i2c_wait_bus_nobusy(FAR struct dw_i2c_dev_s *i2c)
{
  clock_t tick;

  tick = clock_systimer();
  do
    {
      if (!(i2c->hw->STATUS & DW_I2C_STATUS_ACTIVITY))
        return 0;
    }
  while ((clock_systimer() - tick) < DW_I2C_WAIT_BUS_FREE_TIMEOUT);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: dw_i2c_xfer_init
 *
 * Description:
 *   do some init and ready state for starting i2c transfer
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_xfer_init(FAR struct dw_i2c_dev_s *i2c)
{
  struct i2c_msg_s *msgs = i2c->msgs;
  uint32_t con = DW_I2C_CONTROLLER_CONFIG;
  uint32_t tar = msgs[0].addr;

  /* if the slave address is ten bit address, enable 10BITADDR */
  if (msgs[i2c->msg_write_idx].flags & I2C_M_TEN)
    {
      con |= DW_I2C_CON_10BITADDR_MASTER;
      tar |= DW_I2C_TAR_10BITADDR_MASTER;
    }

  /* set the i2c frequency */
  if (msgs[0].frequency <= I2C_SPEED_STANDARD)
    {
      /* standard mode */
      con |= DW_I2C_CON_SPEED_STD;
    }
  else if (msgs[0].frequency <= I2C_SPEED_FAST)
    {
      /* Fast mode */
      con |= DW_I2C_CON_SPEED_FAST;
    }
  else
    {
      /* High mode */
      con |= DW_I2C_CON_SPEED_HIGH;
    }

  dw_i2c_disable(i2c);

  i2c->hw->CON = con;
  i2c->hw->TAR = tar;

  /* Enable the adapter */
  dw_i2c_enable(i2c);
}

/****************************************************************************
 * Name: dw_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers, each transfer is started with a
 *   START and the final transfer is completed with a STOP. Each sequence
 *   will be an 'atomic'  operation in the sense that any other I2C actions
 *   will be serialized and pend until this sequence of transfers completes.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   msgs  - A pointer to a set of message descriptors
 *   count - The number of transfers to perform
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dw_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs, int count)
{
  struct dw_i2c_dev_s *i2c = container_of(dev, struct dw_i2c_dev_s, dev);
  struct timespec abstime;
  uint16_t addr;
  int ret, i;

  if (!count)
    return -EINVAL;

  addr = msgs[0].addr;
  for (i = 0; i < count; i++)
    {
      if (addr != msgs[i].addr || msgs[i].frequency > I2C_SPEED_HIGH
           || !msgs[i].buffer || !msgs[i].length)
        {
          return -EINVAL;
        }
    }

  nxmutex_lock(&i2c->mutex);

  i2c->msgs             = msgs;
  i2c->msgs_num         = count;
  i2c->msg_write_idx    = 0;
  i2c->msg_read_idx     = 0;
  i2c->internal_status  = 0;
  i2c->rx_outstanding   = 0;

  ret = clk_enable(i2c->mclk);
  if (ret < 0)
    {
      nxmutex_unlock(&i2c->mutex);
      return ret;
    }

  /* don't enable irq. otherwise when master transfer first
   * bytes(address+write/read) to slave, slave may NACK
   */
  dw_i2c_enable_disable(i2c, true);

  ret = dw_i2c_wait_bus_nobusy(i2c);
  if (ret < 0)
    {
      goto done;
    }

  /* wait for transfer complete with in a predefined timeout */
  clock_gettime(CLOCK_REALTIME, &abstime);
  abstime.tv_sec++;

  /* start the transfers */
  dw_i2c_xfer_init(i2c);

  /* wait i2c transfer complete with in 1 second */
  do
    {
      ret = nxsem_timedwait(&i2c->sem, &abstime);
    }
  while(ret == -EINTR);

  if (ret < 0)
    {
      i2cerr("i2c transfer failed, ret:%d\n", ret);
    }
  else if (i2c->internal_status != DW_I2C_STATUS_IDLE)
    {
      ret = -EIO;
    }

done:
  dw_i2c_disable(i2c);
  clk_disable(i2c->mclk);
  nxmutex_unlock(&i2c->mutex);
  return ret;
}

/****************************************************************************
 * Name: dw_i2c_xfer_msg
 *
 * Description:
 *   When i2c controller happen DW_I2C_INTR_TX_EMPTY interrupt, this function
 *   is called to generate start and stop single, and send bytes to txfifo
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_xfer_msg(FAR struct dw_i2c_dev_s *i2c)
{
  struct i2c_msg_s *msgs = i2c->msgs;
  uint32_t intr_mask;
  uint16_t tx_limit, rx_limit;
  uint16_t buf_len = i2c->tx_buf_len;
  uint8_t *buf = i2c->tx_buf;
  bool need_restart = false;

  intr_mask = DW_I2C_INTR_DEFAULT_MASK;

  for (; i2c->msg_write_idx < i2c->msgs_num; i2c->msg_write_idx++)
    {
      if (!(i2c->internal_status & DW_I2C_STATUS_WRITE_IN_PROGRESS))
        {
          /* new i2c_msg */
          buf = msgs[i2c->msg_write_idx].buffer;
          buf_len = msgs[i2c->msg_write_idx].length;

          if (i2c->msg_write_idx > 0 && !(msgs[i2c->msg_write_idx].flags & I2C_M_NOSTART))
            need_restart = true;
        }

      tx_limit = i2c->tx_fifo_depth - i2c->hw->TXFLR;
      rx_limit = i2c->rx_fifo_depth - i2c->hw->RXFLR;

      while (buf_len > 0 && tx_limit > 0 && rx_limit > 0)
        {
          uint32_t cmd = 0;

          /*
           * If IC_EMPTYFIFO_HOLD_MASTER_EN is set we must
           * manually set the stop bit. However, it cannot be
           * detected from the registers so we set it always
           * when writing/reading the last byte.
           */
          if ( i2c->msg_write_idx == i2c->msgs_num - 1 && buf_len == 1 )
            cmd |= DW_I2C_DATA_CMD_STOP;

          if (need_restart == true)
            {
              cmd |= DW_I2C_DATA_CMD_RESTART;
              need_restart = false;
            }

          if (msgs[i2c->msg_write_idx].flags & I2C_M_READ)
            {
              /* avoid rx buffer overrun */
              if (i2c->rx_outstanding >= i2c->rx_fifo_depth)
                break;

              i2c->hw->DATA_CMD = cmd | DW_I2C_DATA_CMD_READ;
              rx_limit--;
              i2c->rx_outstanding++;
            }
          else
            {
              i2c->hw->DATA_CMD = cmd | DW_I2C_FORCE_UINT8(*buf++);
            }
          tx_limit--;
          buf_len--;
        }

      if (buf_len > 0)
        {
          /* more bytes to be written */
          i2c->tx_buf = buf;
          i2c->tx_buf_len = buf_len;
          i2c->internal_status |= DW_I2C_STATUS_WRITE_IN_PROGRESS;
          break;
        }
      else
        {
          i2c->internal_status &= ~DW_I2C_STATUS_WRITE_IN_PROGRESS;
        }
    }

  /*
   * If i2c_msg index search is completed, we don't need TX_EMPTY
   * interrupt any more.
   */
  if (i2c->msg_write_idx == i2c->msgs_num)
    intr_mask &= ~DW_I2C_INTR_TX_EMPTY;
   i2c->hw->INTR_MASK = intr_mask;
}

/****************************************************************************
 * Name: dw_i2c_read_data
 *
 * Description:
 *   When i2c controller happen DW_I2C_INTR_RX_FULL interrupt, this function
 *   is called to receive bytes to rxfifo
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_read_data(FAR struct dw_i2c_dev_s *i2c)
{
  struct i2c_msg_s *msgs = i2c->msgs;
  int32_t rx_valid;

  for (; i2c->msg_read_idx < i2c->msgs_num; i2c->msg_read_idx++)
    {
      uint32_t len;
      uint8_t *buf;

      if (!(msgs[i2c->msg_read_idx].flags & I2C_M_READ))
        continue;

      if (!(i2c->internal_status & DW_I2C_STATUS_READ_IN_PROGRESS))
        {
          len = msgs[i2c->msg_read_idx].length;
          buf = msgs[i2c->msg_read_idx].buffer;
        }
      else
        {
          len = i2c->rx_buf_len;
          buf = i2c->rx_buf;
        }

      rx_valid = i2c->hw->RXFLR;

      for (; len > 0 && rx_valid > 0; len--, rx_valid--)
        {
          *buf++ = DW_I2C_FORCE_UINT8(i2c->hw->DATA_CMD);
          i2c->rx_outstanding--;
        }

      if (len > 0)
        {
          i2c->internal_status |= DW_I2C_STATUS_READ_IN_PROGRESS;
          i2c->rx_buf_len = len;
          i2c->rx_buf = buf;
          break;
        }
      else
        {
          i2c->internal_status &= ~DW_I2C_STATUS_READ_IN_PROGRESS;
        }
    }
}

/****************************************************************************
 * Name: dw_i2c_read_clear_intr
 *
 * Description:
 *   clear interrupt bits
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *
 * Returned Value:
 *   Reg INTR_STAT content
 *
 ****************************************************************************/

static uint32_t dw_i2c_read_clear_intr(FAR struct dw_i2c_dev_s *i2c)
{
  uint32_t stat;

  /*
   * The IC_INTR_STAT register just indicates "enabled" interrupts.
   * Ths unmasked raw version of interrupt status bits are available
   * in the IC_RAW_INTR_STAT register.
   *
   * That is,
   *  stat = IC_RAW_INTR_STAT;
   * equals to,
   *   stat = IC_RAW_INTR_STAT & IC_INTR_MASK;
   *
   * The raw version might be useful for debugging purposes.
   */

  stat = i2c->hw->INTR_STAT;

  /*
   * Do not use the IC_CLR_INTR register to clear interrupts, or
   * you'll miss some interrupts, triggered during the period from
   * IC_INTR_STAT to IC_CLR_INTR.
   *
   * Instead, use the separately-prepared IC_CLR_* registers.
   */
  if (stat & DW_I2C_INTR_RX_UNDER)
    i2c->hw->CLR_RX_UNDER;
  if (stat & DW_I2C_INTR_RX_OVER)
    i2c->hw->CLR_RX_OVER;
  if (stat & DW_I2C_INTR_TX_OVER)
    i2c->hw->CLR_TX_OVER;
  if (stat & DW_I2C_INTR_RD_REQ)
    i2c->hw->CLR_RD_REQ;
  if (stat & DW_I2C_INTR_TX_ABRT)
    {
      i2cerr("i2c transfer happened abort, IC_TX_ABRT_SOURCE:0x%x\n", i2c->hw->TX_ABRT_SOURCE);
      i2c->hw->CLR_TX_ABRT;
    }
  if (stat & DW_I2C_INTR_RX_DONE)
    i2c->hw->CLR_RX_DONE;
  if (stat & DW_I2C_INTR_ACTIVITY)
    i2c->hw->CLR_ACTIVITY;
  if (stat & DW_I2C_INTR_START_DET)
    i2c->hw->CLR_START_DET;
  if (stat & DW_I2C_INTR_GEN_CALL)
    i2c->hw->CLR_GEN_CALL;

  return stat;
}

/****************************************************************************
 * Name: dw_i2c_isr
 *
 * Description:
 *   interrupt service routine, is called when dw i2c controller happens interrupt
 *
 * Input Parameters:
 *   irq     - Dw i2c controller irq number
 *   context - NULL
 *   arg     - Dw i2c controller run-time data
 *
 * Returned Value:
 *   Zero
 *
 ****************************************************************************/

static int dw_i2c_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct dw_i2c_dev_s *i2c = arg;
  uint32_t stat, enabled;

  enabled = i2c->hw->ENABLE;
  stat = i2c->hw->RAW_INTR_STAT;

  if (!enabled || !(stat & ~DW_I2C_INTR_ACTIVITY))
    goto transfer_done;

  stat = dw_i2c_read_clear_intr(i2c);

  if (stat & DW_I2C_INTR_TX_ABRT)
    {
      /*
       * Anytime TX_ABRT is set, the contents of the tx/rx
       * buffers are flushed.  Make sure to skip them.
       */
      i2c->hw->INTR_MASK = 0;
      goto tx_aborted;
    }

  if (stat & DW_I2C_INTR_RX_FULL)
    dw_i2c_read_data(i2c);

  if (stat & DW_I2C_INTR_TX_EMPTY)
    dw_i2c_xfer_msg(i2c);

  /*
   * No need to modify or disable the interrupt mask here.
   * i2c_dw_xfer_msg() will take care of it according to
   * the current transmit status.
   */
  if (stat & DW_I2C_INTR_STOP_DET)
    {
tx_aborted:
      /* stop intr bit is cleared at the end of interrupt service routine */
      i2c->hw->CLR_STOP_DET;
      goto transfer_done;
    }

  return 0;

transfer_done:
  nxsem_post(&i2c->sem);
  return 0;
}

/****************************************************************************
 * Name: dw_i2c_hw_init
 *
 * Description:
 *   init DesignWare i2c controller as i2c master
 *   this function need to init controller mode, sda hold time, fs_spklen and irq
 *
 * Input Parameters:
 *   i2c    - Dw i2c controller run-time data
 *   config - DW i2c controller hardware resource
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void dw_i2c_hw_init(FAR struct dw_i2c_dev_s *i2c, FAR const struct dw_i2c_config_s *config)
{
  const struct dw_i2c_timing_s *timing = dw_i2c_get_timing(config->rate);
  uint32_t param;

  dw_i2c_disable(i2c);

  param = i2c->hw->COMP_PARAM_1;
  i2c->tx_fifo_depth = ((param >> 16) & 0xff) + 1;
  i2c->rx_fifo_depth = ((param >> 8)  & 0xff) + 1;

  /* Configure Tx/Rx FIFO threshold levels */

  i2c->hw->TX_TL = i2c->tx_fifo_depth / 2;
  i2c->hw->RX_TL = 0;

  /* set i2c scl high/low count reg */

  if (config->ss_hcnt)
    i2c->hw->SS_SCL_HCNT = config->ss_hcnt;
  else if (timing)
    i2c->hw->SS_SCL_HCNT = timing->ss_hcnt;

  if (config->ss_lcnt)
    i2c->hw->SS_SCL_LCNT = config->ss_lcnt;
  else if (timing)
    i2c->hw->SS_SCL_LCNT = timing->ss_lcnt;

  if (config->fs_hcnt)
    i2c->hw->FS_SCL_HCNT = config->fs_hcnt;
  else if (timing)
    i2c->hw->FS_SCL_HCNT = timing->fs_hcnt;

  if (config->fs_lcnt)
    i2c->hw->FS_SCL_LCNT = config->fs_lcnt;
  else if (timing)
    i2c->hw->FS_SCL_LCNT = timing->fs_lcnt;

  if (config->hs_hcnt)
    i2c->hw->HS_SCL_HCNT = config->hs_hcnt;
  else if (timing)
    i2c->hw->HS_SCL_HCNT = timing->hs_hcnt;

  if (config->hs_lcnt)
    i2c->hw->HS_SCL_LCNT = config->hs_lcnt;
  else if (timing)
    i2c->hw->HS_SCL_LCNT = timing->hs_lcnt;

  /* set i2c sda hold time */

  if (config->sda_hold)
    i2c->hw->SDA_HOLD = config->sda_hold;
  else if (timing)
    i2c->hw->SDA_HOLD = timing->sda_hold;

  /* set fs/hs spklen */

  if (config->fs_spklen)
    i2c->hw->FS_SPKLEN = config->fs_spklen;
  else if (timing)
    i2c->hw->FS_SPKLEN = timing->fs_spklen;

  if (config->hs_spklen)
    i2c->hw->HS_SPKLEN = config->hs_spklen;
  else if (timing)
    i2c->hw->HS_SPKLEN = timing->hs_spklen;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

FAR struct i2c_master_s *dw_i2c_initialize(FAR const struct dw_i2c_config_s *config)
{
  struct dw_i2c_dev_s *i2c;
  int ret;

  DEBUGASSERT(config && config->base && config->irq);

  i2c = kmm_zalloc(sizeof(struct dw_i2c_dev_s));
  if (NULL == i2c)
    {
      i2cerr("i2c:%p no memory apply\n", config->base);
      return NULL;
    }

  i2c->hw= (struct dw_i2c_hw_s *)config->base;
  i2c->dev.ops = &dw_i2c_ops;

  ret = nxmutex_init(&i2c->mutex);
  if (ret < 0)
    goto mutex_err;

  ret = nxsem_init(&i2c->sem, 0, 0);
  if (ret < 0)
    goto sem_err;

  ret = nxsem_setprotocol(&i2c->sem, SEM_PRIO_NONE);
  if (ret < 0)
    goto sem_prio_err;

  i2c->mclk = clk_get(config->mclk);
  if (!i2c->mclk)
    {
      i2cerr("i2c:%p mclk invalid\n", config->base);
      goto sem_prio_err;
    }

  if (config->rate)
    {
      ret = clk_set_rate(i2c->mclk, config->rate);
      if (ret)
        {
          i2cerr("i2c:%p mclk set rate failed\n", config->base);
          goto sem_prio_err;
        }
    }

  ret = clk_enable(i2c->mclk);
  if (ret < 0)
    {
      i2cerr("i2c:%p open mclk failed\n", config->base);
      goto sem_prio_err;
    }

  dw_i2c_hw_init(i2c, config);

  clk_disable(i2c->mclk);

  ret = irq_attach(config->irq, dw_i2c_isr, i2c);
  if (ret < 0)
    goto irq_err;

  up_enable_irq(config->irq);

#ifdef CONFIG_I2C_DRIVER
  if (config->bus >= 0)
    i2c_register(&i2c->dev, config->bus);
#endif

  return &i2c->dev;

irq_err:
sem_prio_err:
  nxsem_destroy(&i2c->sem);
sem_err:
  nxmutex_destroy(&i2c->mutex);
mutex_err:
  kmm_free(i2c);
  i2cerr("i2c%p: initialize failed %d\n", config->base, ret);
  return NULL;
}

void dw_i2c_allinitialize(FAR const struct dw_i2c_config_s *config, int config_num,
                          FAR struct i2c_master_s **i2c)
{
  struct i2c_master_s *i2c_dev;
  int i;

  for (i = 0; i < config_num; i++)
    {
      i2c_dev = dw_i2c_initialize(&config[i]);
      if (i2c_dev && config[i].bus >= 0)
        i2c[config[i].bus] = i2c_dev;
    }
}
