/****************************************************************************
 * arch/csky/src/ch2201/ch2201_serial.c
 *
 *   Copyright (C) 2007-2009, 2012-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include "up_arch.h"
#include "up_internal.h"
#include <drv_usart.h>
#include "chip/dw_usart.h"

extern usart_handle_t console_handle;
extern void USART0_IRQHandler(int irq, uint32_t *regs, void *arg);

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint8_t  ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#if 0
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the LPC214X uart0 port. */

static struct up_dev_s g_uart0priv =
{
  .uartbase       = CSKY_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = UART0_IRQn,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};

#if 0
/* This describes the state of the LPC214X uart1 port. */

static struct up_dev_s g_uart1priv =
{
  .uartbase       = CSKY_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = UART1_IRQn,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif
/* Now, which one with be tty0/console and which tty1? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port
#  define TTYS0_DEV       g_uart0port
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint8_t parity;
  uint8_t bits;
  uint8_t stopbits2;

  if (priv->parity == 0) {
      parity = USART_PARITY_NONE;
  } else if (priv->parity == 1) {
      parity = USART_PARITY_ODD;
  } else if(priv->parity == 2) {
      parity = USART_PARITY_EVEN;
  } else {
      parity = USART_PARITY_NONE;
  }

  if (priv->bits == 7) {
      bits = USART_DATA_BITS_7;
  } else {
      bits = USART_DATA_BITS_8;
  }

  if (priv->stopbits2 == true) {
      stopbits2 = USART_STOP_BITS_2;
  } else {
      stopbits2 = USART_STOP_BITS_1;
  }

  csi_usart_config(console_handle, priv->baud, USART_MODE_ASYNCHRONOUS, parity, stopbits2, bits);
  return OK;
}


/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
 csi_usart_abort_send(console_handle);
 csi_usart_abort_receive(console_handle);
}

/****************************************************************************
 * Name: up_attach
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

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}


/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint8_t            status;
  int                passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  dw_usart_reg_t *addr = (dw_usart_reg_t *)(priv->uartbase);

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = addr->IIR & 0xf;

      /* The NO INTERRUPT should be zero if there are pending
       * interrupts
       */

      if (status == 0 || status == 1 || status == 7)
        {
          /* Break out of the loop when there is no longer a
           * pending interrupt
           */
          break;
        }

        switch (status) {
            case DW_IIR_THR_EMPTY:       /* interrupt source:transmitter holding register empty */
                uart_xmitchars(dev);
                break;
    
            case DW_IIR_RECV_DATA:       /* interrupt source:receiver data available or receiver fifo trigger level reached */
                uart_recvchars(dev);
                break;
    
            case DW_IIR_RECV_LINE:
                break;

            case DW_IIR_CHAR_TIMEOUT:
                {
                uint32_t timecount = 0;
                while (addr->LSR & 0x1) {
                    addr->RBR;
                    timecount++;
                    if (timecount >= UART_BUSY_TIMEOUT) {
                        break;
                    }
                }
                }

            default:
                break;
        }

      if (status == 0 || status == 1 || status == 7)
        {
          /* Break out of the loop when there is no longer a
           * pending interrupt
           */

          break;
        }

    }

    return OK;
}


/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  uint8_t ch;

  csi_usart_getchar(console_handle, &ch);
  return ch;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  flags = irqsave();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      csi_usart_set_interrupt(console_handle, USART_INTR_READ, 1);
#endif
    }
  else
    {
      csi_usart_set_interrupt(console_handle, USART_INTR_READ, 0);
    }

    irqrestore(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  dw_usart_reg_t *addr = (dw_usart_reg_t *)(priv->uartbase);
  uint8_t status = addr->IIR & 0xf;

  if (status == DW_IIR_RECV_DATA) {
      return true;
  } else {
      return false;
  }
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  /* FIXME: \r */
  if (ch == '\n')
    {
      /* Add CR */

      csi_usart_putchar(console_handle, '\r');
    }

  csi_usart_putchar(console_handle, ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  flags = irqsave();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      csi_usart_set_interrupt(console_handle, USART_INTR_WRITE, 1);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      csi_usart_set_interrupt(console_handle, USART_INTR_WRITE, 0);
    }

    irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  dw_usart_reg_t *addr = (dw_usart_reg_t *)(priv->uartbase);
  uint8_t status = addr->IIR & 0xf;

  if (status == DW_IIR_THR_EMPTY) {
      return true;
  } else {
      return false;
  }
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Enable UART0 and 1 */


  /* Disable both UARTS */
  csi_usart_set_interrupt(console_handle, USART_INTR_READ, 0);
  csi_usart_set_interrupt(console_handle, USART_INTR_WRITE, 0);

  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  (void)uart_register("/dev/console", &CONSOLE_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
    if (console_handle == NULL) {
        return -1;
    }

    if (ch == '\n') {
        csi_usart_putchar(console_handle, '\r');
    }

    csi_usart_putchar(console_handle, ch);

    return 0;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      csi_usart_putchar(console_handle, '\r');
    }

  csi_usart_putchar(console_handle, ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
