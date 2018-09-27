/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     isr.c
 * @brief    source file for the interrupt server route
 * @version  V1.0
 * @date     25. August 2017
 ******************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "up_internal.h"
#include "up_arch.h"

#include <drv_common.h>
#include <nuttx/irq.h>
extern void dw_usart_irqhandler(int32_t idx);
#if 0
extern void dw_gpio_irqhandler(int32_t idx);
extern void dw_timer_irqhandler(int32_t idx);
extern void dw_iic_irqhandler(int32_t idx);
extern void ck_rtc_irqhandler(int32_t idx);
extern void dw_spi_irqhandler(int32_t idx);
extern void dw_wdt_irqhandler(int32_t idx);
extern void ck_dma_irqhandler(int32_t idx);
extern void ck_aes_irqhandler(int32_t idx);
extern void ck_sha_irqhandler(int32_t idx);
extern void dw_dmac_irqhandler(int32_t idx);
extern void ck_adc_irqhandler(int32_t idx);
extern void ck_i2s_irqhandler(int32_t idx);
#endif

#define readl(addr) \
    ({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#ifndef CONFIG_KERNEL_NONE
#define CSI_INTRPT_ENTER() csi_kernel_intrpt_enter()
#define CSI_INTRPT_EXIT()  csi_kernel_intrpt_exit()
#else
#define CSI_INTRPT_ENTER()
#define CSI_INTRPT_EXIT()
#endif

#ifdef CONFIG_HAVE_VIC
#define ATTRIBUTE_ISR __attribute__((isr))
#else
#define ATTRIBUTE_ISR
#endif

void CORET_IRQHandler(int irq, uint32_t *regs, void *arg)
{
    readl(0xE000E010);
    sched_process_timer();
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
    if (clock_timer != NULL)
#endif
    {
        clock_timer();
    }
}

void USART0_IRQHandler(int irq, uint32_t *regs, void *arg)
{
    dw_usart_irqhandler(0);
}

void USART1_IRQHandler(int irq, uint32_t *regs, void *arg)
{
    dw_usart_irqhandler(1);
}

void USART2_IRQHandler(int irq, uint32_t *regs, void *arg)
{
    dw_usart_irqhandler(2);
}

void USART3_IRQHandler(int irq, uint32_t *regs, void *arg)
{
    dw_usart_irqhandler(3);
}

#if 0
ATTRIBUTE_ISR void TIMA0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_timer_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void TIMA1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_timer_irqhandler(1);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void TIMB0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_timer_irqhandler(2);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void TIMB1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_timer_irqhandler(3);
    CSI_INTRPT_EXIT();
}
#endif

#if 0
ATTRIBUTE_ISR void GPIO0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_gpio_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void GPIO1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_gpio_irqhandler(1);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void I2C0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_iic_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void I2C1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_iic_irqhandler(1);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void RTC_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    ck_rtc_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void RTC1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    ck_rtc_irqhandler(1);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void AES_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    ck_aes_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void TRNG_IRQHandler(void)
{
    CSI_INTRPT_ENTER();

    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void RSA_IRQHandler(void)
{
    CSI_INTRPT_ENTER();

    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void SPI0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_spi_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void SPI1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_spi_irqhandler(1);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void WDT_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_wdt_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void DMAC0_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_dmac_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void DMAC1_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    dw_dmac_irqhandler(1);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void ADC_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    ck_adc_irqhandler(0);
    CSI_INTRPT_EXIT();
}

ATTRIBUTE_ISR void I2S_IRQHandler(void)
{
    CSI_INTRPT_ENTER();
    ck_i2s_irqhandler(0);
    CSI_INTRPT_EXIT();
}
#endif
