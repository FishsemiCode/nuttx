/****************************************************************************
 * drivers/serial/uart_cmsdk.c
 * Serial driver for cmsdk UART
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Zhuang Liu <liuzhuang@pinecone.net>
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
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/uart_cmsdk.h>

#include <arch/board/board.h>

#ifdef CONFIG_CMSDK_UART

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_cmsdk_s
{
  uint32_t         uartbase;  /* Base address of UART registers */
  uint32_t         bauddiv;
  uint8_t          tx_irq;
  uint8_t          rx_irq;
  uint8_t          ov_irq;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_cmsdk_setup(FAR struct uart_dev_s *dev);
static void uart_cmsdk_shutdown(FAR struct uart_dev_s *dev);
static int  uart_cmsdk_attach(FAR struct uart_dev_s *dev);
static void uart_cmsdk_detach(FAR struct uart_dev_s *dev);
static int  uart_cmsdk_rx_interrupt(int irq, FAR void *context, FAR void *arg);
static int  uart_cmsdk_ov_interrupt(int irq, FAR void *context, FAR void *arg);
static int  uart_cmsdk_tx_interrupt(int irq, FAR void *context, FAR void *arg);
static int  uart_cmsdk_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  uart_cmsdk_receive(FAR struct uart_dev_s *dev, uint32_t *status);
static void uart_cmsdk_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_cmsdk_rxavailable(FAR struct uart_dev_s *dev);
static void uart_cmsdk_send(FAR struct uart_dev_s *dev, int ch);
static void uart_cmsdk_txint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_cmsdk_txready(FAR struct uart_dev_s *dev);
static bool uart_cmsdk_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct uart_ops_s g_uart_ops =
{
  .setup          = uart_cmsdk_setup,
  .shutdown       = uart_cmsdk_shutdown,
  .attach         = uart_cmsdk_attach,
  .detach         = uart_cmsdk_detach,
  .ioctl          = uart_cmsdk_ioctl,
  .receive        = uart_cmsdk_receive,
  .rxint          = uart_cmsdk_rxint,
  .rxavailable    = uart_cmsdk_rxavailable,
  .send           = uart_cmsdk_send,
  .txint          = uart_cmsdk_txint,
  .txready        = uart_cmsdk_txready,
  .txempty        = uart_cmsdk_txempty,
};

/* I/O buffers */

#ifdef CONFIG_CMSDK_UART0
static char g_uart0rxbuffer[CONFIG_CMSDK_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_CMSDK_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_CMSDK_UART1
static char g_uart1rxbuffer[CONFIG_CMSDK_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_CMSDK_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_CMSDK_UART2
static char g_uart2rxbuffer[CONFIG_CMSDK_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_CMSDK_UART2_TXBUFSIZE];
#endif


/* This describes the state of the CMSDK uart0 port. */

#ifdef CONFIG_CMSDK_UART0
static struct uart_cmsdk_s g_uart0priv =
{
  .uartbase       = CONFIG_CMSDK_UART0_BASE,
  .bauddiv        = CONFIG_CMSDK_UART0_BAUDDIV,
  .tx_irq         = CONFIG_CMSDK_UART0_TX_IRQ,
  .rx_irq         = CONFIG_CMSDK_UART0_RX_IRQ,
  .ov_irq         = CONFIG_CMSDK_UART0_OV_IRQ,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_CMSDK_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_CMSDK_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the CMSDK uart1 port. */

#ifdef CONFIG_CMSDK_UART1
static struct uart_cmsdk_s g_uart1priv =
{
  .uartbase       = CONFIG_CMSDK_UART1_BASE,
  .bauddiv        = CONFIG_CMSDK_UART1_BAUDDIV,
  .tx_irq         = CONFIG_CMSDK_UART1_TX_IRQ,
  .rx_irq         = CONFIG_CMSDK_UART1_RX_IRQ,
  .ov_irq         = CONFIG_CMSDK_UART1_OV_IRQ,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_CMSDK_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_CMSDK_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

#ifdef CONFIG_CMSDK_UART2
static struct uart_cmsdk_s g_uart2priv =
{
  .uartbase       = CONFIG_CMSDK_UART2_BASE,
  .bauddiv        = CONFIG_CMSDK_UART2_BAUDDIV,
  .tx_irq         = CONFIG_CMSDK_UART2_TX_IRQ,
  .rx_irq         = CONFIG_CMSDK_UART2_RX_IRQ,
  .ov_irq         = CONFIG_CMSDK_UART2_OV_IRQ,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_CMSDK_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_CMSDK_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/* Which UART with be console */

#if defined(CONFIG_CMSDK_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port    /* UART0=console */
#elif defined(CONFIG_CMSDK_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port    /* UART1=console */
#elif defined(CONFIG_CMSDK_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port    /* UART2=console */
#endif

#ifdef CONFIG_CMSDK_UART0
#  define TTYS0_DEV       g_uart0port
#endif

#ifdef CONFIG_CMSDK_UART1
#  define TTYS1_DEV       g_uart1port
#endif

#ifdef CONFIG_CMSDK_UART2
#  define TTYS2_DEV       g_uart2port
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_cmsdk_serialin
 ****************************************************************************/

static inline uint32_t uart_cmsdk_serialin(FAR struct uart_cmsdk_s *priv, uint32_t offset)
{
  return *((FAR volatile uint32_t *)priv->uartbase + offset);
}

/****************************************************************************
 * Name: uart_cmsdk_serialout
 ****************************************************************************/

static inline void uart_cmsdk_serialout(FAR struct uart_cmsdk_s *priv, uint32_t offset,
                                    uint32_t value)
{
  *((FAR volatile uint32_t *)priv->uartbase + offset) = value;
}

/****************************************************************************
 * Name: uart_cmsdk_serialmodify
 ****************************************************************************/

void uart_cmsdk_serialmodify(FAR struct uart_cmsdk_s *priv, uint32_t offset,
                              uint32_t clearbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t   regval;

  flags   = enter_critical_section();
  regval  = uart_cmsdk_serialin(priv, offset);
  regval &= ~clearbits;
  regval |= setbits;
  uart_cmsdk_serialout(priv, offset, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: uart_cmsdk_disableuartint
 ****************************************************************************/

static inline void uart_cmsdk_disableuartint(FAR struct uart_cmsdk_s *priv)
{
  uart_cmsdk_serialmodify(priv, UART_CTRL_OFFSET, UART_CTRL_ALLIE, 0);
}

/****************************************************************************
 * Name: uart_cmsdk_restoreuartint
 ****************************************************************************/

static inline void uart_cmsdk_restoreuartint(FAR struct uart_cmsdk_s *priv)
{
  uart_cmsdk_serialmodify(priv, UART_CTRL_OFFSET, 0, UART_CTRL_ALLIE);
}

/****************************************************************************
 * Name: uart_cmsdk_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int uart_cmsdk_setup(FAR struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;

  uart_cmsdk_serialout(priv, UART_BAUDDIV_OFFSET, priv->bauddiv);

  uart_cmsdk_serialout(priv, UART_CTRL_OFFSET,
                 (UART_CTRL_TX_ENABLE | UART_CTRL_RX_ENABLE |
                 UART_CTRL_RX_INT_ENABLE | UART_CTRL_TX_INT_ENABLE |
                 UART_CTRL_RX_OVERRUN_INT_ENABLE | UART_CTRL_TX_OVERRUN_INT_ENABLE));

  return OK;
}

/****************************************************************************
 * Name: uart_cmsdk_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void uart_cmsdk_shutdown(struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  uart_cmsdk_disableuartint(priv);
}

/****************************************************************************
 * Name: uart_cmsdk_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int uart_cmsdk_attach(struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */
  ret = irq_attach(priv->tx_irq, uart_cmsdk_tx_interrupt, dev);
  ret |= irq_attach(priv->rx_irq, uart_cmsdk_rx_interrupt, dev);
  ret |= irq_attach(priv->ov_irq, uart_cmsdk_ov_interrupt, dev);
#ifndef CONFIG_ARCH_NOINTC
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */
      up_enable_irq(priv->tx_irq);
      up_enable_irq(priv->rx_irq);
      up_enable_irq(priv->ov_irq);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: uart_cmsdk_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void uart_cmsdk_detach(FAR struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;

  up_disable_irq(priv->tx_irq);
  irq_detach(priv->tx_irq);
  up_disable_irq(priv->rx_irq);
  irq_detach(priv->rx_irq);
  up_disable_irq(priv->ov_irq);
  irq_detach(priv->ov_irq);
}

/****************************************************************************
 * Name: uart_cmsdk_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_cmsdk_s structure in order to call these functions.
 *
 ****************************************************************************/

static int uart_cmsdk_rx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  FAR struct uart_cmsdk_s *priv;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (FAR struct uart_cmsdk_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */
  uart_cmsdk_serialout(priv, UART_INTSTS_OFFSET, UART_INTSTATUS_RX);
  uart_recvchars(dev);

  return OK;
}

static int uart_cmsdk_ov_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  FAR struct uart_cmsdk_s *priv;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (FAR struct uart_cmsdk_s *)dev->priv;

  if(uart_cmsdk_serialin(priv, UART_INTSTS_OFFSET) && UART_INTSTATUS_RX_OVERRUN)
    {
      uart_cmsdk_serialout(priv, UART_INTSTS_OFFSET, UART_INTSTATUS_RX_OVERRUN);
      uart_cmsdk_serialout(priv, UART_STATE_OFFSET, UART_STATE_RX_BUF_OVERRUN);
    }
  if(uart_cmsdk_serialin(priv, UART_INTSTS_OFFSET) && UART_INTSTATUS_TX_OVERRUN)
    {
      uart_cmsdk_serialout(priv, UART_INTSTS_OFFSET, UART_INTSTATUS_TX_OVERRUN);
      uart_cmsdk_serialout(priv, UART_STATE_OFFSET, UART_STATE_TX_BUF_OVERRUN);
    }

  return OK;
}

static int uart_cmsdk_tx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  FAR struct uart_cmsdk_s *priv;
  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (FAR struct uart_cmsdk_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  uart_cmsdk_serialout(priv, UART_INTSTS_OFFSET, UART_INTSTATUS_TX);
  uart_xmitchars(dev);

  return OK;
}

/****************************************************************************
 * Name: uart_cmsdk_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int uart_cmsdk_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Name: uart_cmsdk_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int uart_cmsdk_receive(struct uart_dev_s *dev, uint32_t *status)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  uint32_t rbr;
  rbr = uart_cmsdk_serialin(priv, UART_RBR_OFFSET);
  *status = uart_cmsdk_serialin(priv, UART_STATE_OFFSET);
  return rbr;
}

/****************************************************************************
 * Name: uart_cmsdk_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void uart_cmsdk_rxint(struct uart_dev_s *dev, bool enable)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;

  if (enable)
    {
      uart_cmsdk_serialmodify(priv, UART_CTRL_OFFSET, 0, UART_CTRL_RX_INT_ENABLE);
    }
  else
    {
      uart_cmsdk_serialmodify(priv, UART_CTRL_OFFSET, UART_CTRL_RX_INT_ENABLE, 0);
    }
}

/****************************************************************************
 * Name: uart_cmsdk_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool uart_cmsdk_rxavailable(struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  return ((uart_cmsdk_serialin(priv, UART_STATE_OFFSET) & UART_STATE_RX_BUF_FULL) != 0);
}

/****************************************************************************
 * Name: uart_cmsdk_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void uart_cmsdk_send(struct uart_dev_s *dev, int ch)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  uart_cmsdk_serialout(priv, UART_THR_OFFSET, ch);
}

/****************************************************************************
 * Name: uart_cmsdk_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void uart_cmsdk_txint(struct uart_dev_s *dev, bool enable)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
      uart_cmsdk_serialmodify(priv, UART_CTRL_OFFSET, 0, UART_CTRL_TX_INT_ENABLE);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
    }
  else
    {
      uart_cmsdk_serialmodify(priv, UART_CTRL_OFFSET ,UART_CTRL_TX_INT_ENABLE, 0);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: uart_cmsdk_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool uart_cmsdk_txready(struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  return ((uart_cmsdk_serialin(priv, UART_STATE_OFFSET) & UART_STATE_TX_BUF_FULL) == 0);
}

/****************************************************************************
 * Name: uart_cmsdk_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool uart_cmsdk_txempty(struct uart_dev_s *dev)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)dev->priv;
  return ((uart_cmsdk_serialin(priv, UART_STATE_OFFSET) & UART_STATE_TX_BUF_FULL) == 0);
}

#ifdef HAVE_CMSDK_CONSOLE
/****************************************************************************
 * Name: uart_cmsdk_putc
 *
 * Description:
 *   Write one character to the UART (polled)
 *
 ****************************************************************************/

static void uart_cmsdk_putc(FAR struct uart_cmsdk_s *priv, int ch)
{
  if(!(uart_cmsdk_serialin(priv, UART_CTRL_OFFSET) & UART_CTRL_TX_ENABLE))
    return;

  while ((uart_cmsdk_serialin(priv, UART_STATE_OFFSET) & UART_STATE_TX_BUF_FULL) != 0)
    ;
  uart_cmsdk_serialout(priv, UART_THR_OFFSET, ch);
}
#endif

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before uart_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by uart_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */
#ifdef CONFIG_CMSDK_UART0
  uart_cmsdk_disableuartint(&g_uart0priv);
#endif
#ifdef CONFIG_CMSDK_UART1
  uart_cmsdk_disableuartint(&g_uart1priv);
#endif
#ifdef CONFIG_CMSDK_UART2
  uart_cmsdk_disableuartint(&g_uart2priv);
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  uart_cmsdk_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef CONSOLE_DEV
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_CMSDK_CONSOLE
int up_putc(int ch)
{
  FAR struct uart_cmsdk_s *priv = (FAR struct uart_cmsdk_s *)CONSOLE_DEV.priv;

  uart_cmsdk_disableuartint(priv);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      uart_cmsdk_putc(priv, '\r');
    }

  uart_cmsdk_putc(priv, ch);
  uart_cmsdk_restoreuartint(priv);
  return ch;
}
#endif

#endif /* CONFIG_CMSDK_UART */
