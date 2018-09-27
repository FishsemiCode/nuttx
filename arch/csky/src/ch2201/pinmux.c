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
 * @file     pinmux.c
 * @brief    source file for the pinmux
 * @version  V1.0
 * @date     02. June 2017
 ******************************************************************************/

#include <stdint.h>
#include <nuttx/irq.h>

#define readl(addr) \
    ({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#define writel(b,addr) (void)((*(volatile unsigned int *) (addr)) = (b))
#if 0
/* saving the pulldown register status because the register write only */
static uint32_t pulldown_flag0;     /* saving the PortA and PortB pulldown register status */
static uint8_t pulldown_flag1;      /* saving the PortC pulldown register status */
#endif
/**
  \brief       initial hobbit pinmux.
  \return      none
*/
void ioreuse_initial(void)
{
    unsigned int value;

    value = readl(CH2201_GIPO0_PORTCTL_REG);
    value &= ~(GPIO0_REUSE_DIS);
    writel(value, CH2201_GIPO0_PORTCTL_REG);

}

/**
  \brief       config the pin function.
  \param[in]   pin       refs to pin_name_e.
  \param[in]   pin_func  refs to pin_func_e.
  \return      0-success or -1-failure
*/
int32_t drv_pinmux_config(pin_name_e pin, pin_func_e pin_func)
{
    unsigned int val = 0;
    unsigned int reg_val = 0;

    uint8_t offset;

    if (pin_func > 3) {
        if (pin <= PB3) {
            if (pin <= PA5) {
                offset = pin;
                /* gpio data source select */
                val = readl(CH2201_GIPO0_PORTCTL_REG);
                val &= ~(1 << offset);
                writel(val, CH2201_GIPO0_PORTCTL_REG);
                return 0;
            } else if (pin >= PB0) {
                offset = pin - 6;
                /* gpio data source select */
                val = readl(CH2201_GIPO1_PORTCTL_REG);
                val &= ~(1 << offset);
                writel(val, CH2201_GIPO1_PORTCTL_REG);
                return 0;
            }
        }

        if ((pin >= PA6) && (pin <= PA27)) {
            offset = pin - 4;
            /* gpio data source select */
            val = readl(CH2201_GIPO0_PORTCTL_REG);
            val &= ~(1 << offset);
            writel(val, CH2201_GIPO0_PORTCTL_REG);
            return 0;
        }

        return -1;
    }

    if ((pin >= PA6) && (pin <= PA27)) {
        offset = pin - 4;

        /* gpio data source select */
        val = readl(CH2201_GIPO0_PORTCTL_REG);
        val |= (1 << offset);
        writel(val, CH2201_GIPO0_PORTCTL_REG);

        if (pin <= PA11) {
            offset = pin;
            reg_val = (0x3 << (offset * 2));
            /* reuse function select */
            val = readl(CH2201_IOMUX0L_REG);
            val &= ~(reg_val);
            val |= (pin_func << (2 * offset));
            writel(val, CH2201_IOMUX0L_REG);
            return 0;
        } else {
            offset = pin - 16;
            reg_val = (0x3 << (offset * 2));
            /* reuse function select */
            val = readl(CH2201_IOMUX0H_REG);
            val &= ~(reg_val);
            val |= (pin_func << (2 * offset));
            writel(val, CH2201_IOMUX0H_REG);
            return 0;
        }
    }

    if ((pin >= PA0) && (pin <= PB3)) {
        if (pin >= PB0) {
            offset = pin - 6;
            val = readl(CH2201_GIPO1_PORTCTL_REG);
            val |= (1 << offset);
            writel(val, CH2201_GIPO1_PORTCTL_REG);

            offset = pin;
            reg_val = (0x3 << (offset * 2));
            /* reuse function select */
            val = readl(CH2201_IOMUX0L_REG);
            val &= ~(reg_val);
            val |= (pin_func << (2 * offset));
            writel(val, CH2201_IOMUX0L_REG);
            return 0;
        }

        if (pin <= PA5) {
            offset = pin;
            /* gpio data source select */
            val = readl(CH2201_GIPO0_PORTCTL_REG);
            val |= (1 << offset);
            writel(val, CH2201_GIPO0_PORTCTL_REG);

            reg_val = (0x3 << (offset * 2));
            /* reuse function select */
            val = readl(CH2201_IOMUX0L_REG);
            val &= ~(reg_val);
            val |= (pin_func << (2 * offset));
            writel(val, CH2201_IOMUX0L_REG);
            return 0;
        }
    }

    if (pin > PA27) {
        offset = pin - PC0;
        reg_val = (0x3 << (offset * 2));
        val = readl(CH2201_IOMUX1L_REG);
        val &= ~(reg_val);
        val |= (pin_func << (2 * offset));
        writel(val, CH2201_IOMUX1L_REG);
        return 0;
    }

    return -1;
}
#if 0
/**
  \brief       config the pin mode.
  \param[in]   port      refs to port_name_e.
  \param[in]   offset    the offset of the pin in corresponding port.
  \param[in]   pin_mode  refs to gpio_mode_e.
  \return      0-success or -1-failure
*/
int32_t drv_pin_config_mode(port_name_e port, uint8_t offset, gpio_mode_e pin_mode)
{
    unsigned int val = 0;
    uint32_t reg = 0;

    if (port == PORTA) {
        if (offset > PA5) {
            offset += 4;
        }
    } else if (port == PORTB) {
        offset += 6;
    }

    switch (pin_mode) {
        case GPIO_MODE_PULLUP:
            if (port == PORTC) {
                reg = CH2201_IOPU1_REG;

                /* disable Portc corresponding pin pulldown status */
                if (pulldown_flag1 & (1 << offset)) {
                    pulldown_flag1 &= ~(1 << offset);
                    writel(pulldown_flag1, CH2201_IOPD1_REG);
                }
            } else {
                /* disable PortA&PortB corresponding pin pulldown status */
                reg = CH2201_IOPU0_REG;

                if (pulldown_flag0 & (1 << offset)) {
                    pulldown_flag0 &= ~(1 << offset);
                    writel(pulldown_flag0, CH2201_IOPD0_REG);
                }
            }

            break;

        case GPIO_MODE_PULLDOWN:
            if (port == PORTC) {
                reg = CH2201_IOPD1_REG;
                val = readl(CH2201_IOPU1_REG);
                pulldown_flag1 |= (1 << offset);

                /* disable Portc corresponding pin pullup status */
                if (val & (1 << offset)) {
                    val &= ~(1 << offset);
                    writel(val, CH2201_IOPU1_REG);
                }
            } else {
                reg = CH2201_IOPD0_REG;
                val = readl(CH2201_IOPU0_REG);
                pulldown_flag0 |= (1 << offset);

                /* disable PortA&PortB corresponding pin pullup status */
                if (val & (1 << offset)) {
                    val &= ~(1 << offset);
                    writel(val, CH2201_IOPU0_REG);
                }
            }

            break;

        case GPIO_MODE_OPEN_DRAIN:
            if (port == PORTC) {
                reg = CH2201_IOOD1_REG;
            } else {
                reg = CH2201_IOOD0_REG;
            }

            break;

        case GPIO_MODE_PUSH_PULL:
            return (CSI_DRV_ERRNO_GPIO_BASE | DRV_ERROR_UNSUPPORTED);

        case GPIO_MODE_PULLNONE:
            if (port == PORTC) {
                /* disable Portc corresponding pin pulldown status */
                if (pulldown_flag1 & (1 << offset)) {
                    pulldown_flag1 &= ~(1 << offset);
                    writel(pulldown_flag1, CH2201_IOPD1_REG);
                }

                val = readl(CH2201_IOPU1_REG);

                /* disable Portc corresponding pin pullup status */
                if (val & (1 << offset)) {
                    val &= ~(1 << offset);
                    writel(val, CH2201_IOPU1_REG);
                }
            } else {
                /* disable PortA&PortB corresponding pin pulldown status */
                if (pulldown_flag0 & (1 << offset)) {
                    pulldown_flag0 &= ~(1 << offset);
                    writel(pulldown_flag0, CH2201_IOPD0_REG);
                }

                val = readl(CH2201_IOPU0_REG);

                /* disable PortA&PortB corresponding pin pullup status */
                if (val & (1 << offset)) {
                    val &= ~(1 << offset);
                    writel(val, CH2201_IOPU0_REG);
                }
            }

            return 0;

        default:
            return (CSI_DRV_ERRNO_GPIO_BASE | DRV_ERROR_PARAMETER);
    }

    val = readl(reg);
    val |= (1 << offset);
    writel(val, reg);
    return 0;
}
#endif
