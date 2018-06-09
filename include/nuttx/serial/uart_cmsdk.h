/************************************************************************************
 * include/nuttx/serial/uart_cmsdk.h
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_SERIAL_UART_CMSDK_H
#define __INCLUDE_NUTTX_SERIAL_UART_CMSDK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_CMSDK_UART

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CONFIGURATION ********************************************************************/

/* Register offsets *****************************************************************/

#define UART_RBR_OFFSET          0 /* Receiver Buffer Register */
#define UART_THR_OFFSET          0 /* Transmit Holding Register */
#define UART_STATE_OFFSET        1 /* Interrupt State Register */
#define UART_CTRL_OFFSET         2 /* Interrupt Control Register */
#define UART_INTSTS_OFFSET       3 /* Interrupt Status Clear Register */
#define UART_BAUDDIV_OFFSET      4 /* Baud rate divider Register */

/* Register bit definitions *********************************************************/

#define UART_STATE_TX_BUF_FULL            (1 << 0)
#define UART_STATE_RX_BUF_FULL            (1 << 1)
#define UART_STATE_TX_BUF_OVERRUN         (1 << 2)
#define UART_STATE_RX_BUF_OVERRUN         (1 << 3)

#define UART_INTSTATUS_TX                 (1 << 0)
#define UART_INTSTATUS_RX                 (1 << 1)
#define UART_INTSTATUS_TX_OVERRUN         (1 << 2)
#define UART_INTSTATUS_RX_OVERRUN         (1 << 3)

#define UART_CTRL_TX_ENABLE               (1 << 0)
#define UART_CTRL_RX_ENABLE               (1 << 1)
#define UART_CTRL_TX_INT_ENABLE           (1 << 2)
#define UART_CTRL_RX_INT_ENABLE           (1 << 3)
#define UART_CTRL_TX_OVERRUN_INT_ENABLE   (1 << 4)
#define UART_CTRL_RX_OVERRUN_INT_ENABLE   (1 << 5)
#define UART_CTRL_TSTMODE_ENABLE          (1 << 6)
#define UART_CTRL_ALLIE                   (0x3C)

#define UART_BAUDDIV_MIN                  (16)
#define UART_BAUDDIV_MAX                  (0xfffff)

/************************************************************************************
 * Public Types
 ************************************************************************************/

#if defined(CONFIG_CMSDK_UART0_SERIAL_CONSOLE) && defined(CONFIG_CMSDK_UART0)
#  undef CONFIG_CMSDK_UART1_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART2_SERIAL_CONSOLE
#  define HAVE_CMSDK_CONSOLE 1
#elif defined(CONFIG_CMSDK_UART1_SERIAL_CONSOLE) && defined(CONFIG_CMSDK_UART1)
#  undef CONFIG_CMSDK_UART0_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART2_SERIAL_CONSOLE
#  define HAVE_CMSDK_CONSOLE 1
#elif defined(CONFIG_CMSDK_UART2_SERIAL_CONSOLE) && defined(CONFIG_CMSDK_UART2)
#  undef CONFIG_CMSDK_UART0_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART1_SERIAL_CONSOLE
#  define HAVE_CMSDK_CONSOLE 1
#else
#  undef CONFIG_CMSDK_UART0_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART1_SERIAL_CONSOLE
#  undef CONFIG_CMSDK_UART2_SERIAL_CONSOLE
#  undef HAVE_CMSDK_CONSOLE
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
#endif /* CONFIG_CMSDK_UART */
#endif /* __INCLUDE_NUTTX_SERIAL_UART_CMSDK_H */
