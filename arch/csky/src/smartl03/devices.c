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
 * @file     devices.c
 * @brief    source file for the devices
 * @version  V1.0
 * @date     24. August 2017
 ******************************************************************************/
#include <drv_usart.h>
#include <nuttx/irq.h>
#define readl(addr) \
    ({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#define writel(b,addr) (void)((*(volatile unsigned int *) (addr)) = (b))

struct {
    uint32_t base;
    uint32_t irq;
}
const sg_usart_config[CONFIG_USART_NUM] = {
    {CSKY_UART0_BASE, UART0_IRQn},
};
typedef struct {
    int32_t    tx;
    int32_t    rx;
    uint16_t cfg_idx;    //idx of sg_usart_config[]
} usart_pin_map_t;
const static usart_pin_map_t s_usart_pin_map[] = {
    {
        PAD_UART0_SIN,
        PAD_UART0_SOUT,
        0
    }
};

/**
  \param[in]   instance idx, must not exceed return value of target_get_usart_count()
  \brief       get usart instance.
  \return      pointer to usart instance
*/
int32_t target_usart_init(int32_t idx, uint32_t *base, uint32_t *irq)
{
    if (idx >= sizeof(s_usart_pin_map) / sizeof(usart_pin_map_t)) {
        return -1;
    }

    *base = sg_usart_config[s_usart_pin_map[idx].cfg_idx].base;
    *irq = sg_usart_config[s_usart_pin_map[idx].cfg_idx].irq;
    return s_usart_pin_map[idx].cfg_idx;
}

struct {
    uint32_t base;
    uint32_t irq;
}
const sg_timer_config[CONFIG_TIMER_NUM] = {
    {CSKY_TIMER0_BASE, TIM0_IRQn},
    {CSKY_TIMER1_BASE, TIM1_IRQn},
    {CSKY_TIMER2_BASE, TIM2_IRQn},
    {CSKY_TIMER3_BASE, TIM3_IRQn},
};

int32_t target_get_timer_count(void)
{
    return CONFIG_TIMER_NUM;
}

int32_t target_get_timer(int32_t idx, uint32_t *base, uint32_t *irq)
{
    if (idx >= target_get_timer_count()) {
        return 0;
    }

    *base = sg_timer_config[idx].base;
    *irq = sg_timer_config[idx].irq;
    return idx;
}

struct {
    uint32_t base;
    uint32_t irq;
    uint32_t pin_num;
    port_name_e port;
}
const sg_gpio_config[CONFIG_GPIO_NUM] = {
    {CSKY_GPIOA_BASE, GPIO0_IRQn, 0, PORTA},
    {CSKY_GPIOA_BASE, GPIO1_IRQn, 0, PORTB},
    {CSKY_GPIOA_BASE, GPIO2_IRQn, 0, PORTC},
    {CSKY_GPIOA_BASE, GPIO3_IRQn, 0, PORTD},
    {CSKY_GPIOA_BASE, GPIO4_IRQn, 0, PORTE},
    {CSKY_GPIOA_BASE, GPIO5_IRQn, 0, PORTF},
    {CSKY_GPIOA_BASE, GPIO6_IRQn, 0, PORTG},
    {CSKY_GPIOA_BASE, GPIO7_IRQn, 0, PORTH},
};

typedef struct {
    int32_t    gpio_pin;
    uint32_t cfg_idx;    //idx of sg_gpio_config[]
} gpio_pin_map_t;
const static gpio_pin_map_t s_gpio_pin_map[] = {
    {PA0, 0},
    {PA1, 1},
    {PA2, 2},
    {PA3, 3},
    {PA4, 4},
    {PA5, 5},
    {PA6, 6},
    {PA7, 7},
};

int32_t target_gpio_port_init(port_name_e port, uint32_t *base, uint32_t *irq, uint32_t *pin_num)
{
    int i;

    for (i = 0; i < CONFIG_GPIO_NUM; i++) {
        if (sg_gpio_config[i].port == port) {
            *base = sg_gpio_config[i].base;
            *irq = sg_gpio_config[i].irq;
            *pin_num = sg_gpio_config[i].pin_num;
            return i;
        }
    }

    return -1;
}
/**
  \param[in]   instance idx, must not exceed return value of target_get_gpio_count()
  \brief       get gpio instance.
  \return      pointer to gpio instance
*/
int32_t target_gpio_pin_init(int32_t gpio_pin, uint32_t *port_idx)
{
    uint32_t idx;

    for (idx = 0; idx < sizeof(s_gpio_pin_map) / sizeof(gpio_pin_map_t); idx++) {
        if (s_gpio_pin_map[idx].gpio_pin == gpio_pin) {
            *port_idx = s_gpio_pin_map[idx].cfg_idx;
            return idx;
        }
    }

    return -1;
}
