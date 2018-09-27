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
 * @file     sys_freq.c
 * @brief    source file for setting system frequency.
 * @version  V1.0
 * @date     18. July 2018
 ******************************************************************************/
#include <stdint.h>
#include <nuttx/irq.h>

#define readl(addr) \
    ({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#define writel(b,addr) (void)((*(volatile unsigned int *) (addr)) = (b))

/* FIXME */
#define CK_EFLASH_TRC     0x4003f020
#define CK_EFLASH_TNVS    0x4003f024
#define CK_EFLASH_TPGS    0x4003f028
#define CK_EFLASH_TPROG   0x4003f02c
#define CK_EFLASH_TRCV    0x4003f030
#define CK_EFLASH_TERASE  0x4003f034

typedef struct {
    uint8_t trc;
    uint16_t tnvs;
    uint8_t tpgs;
    uint16_t tprog;
    uint16_t trcv_erase;
} eflash_opt_time_t;

extern int g_system_clock;

static const eflash_opt_time_t eflash_opt_time[] = {
    {0x0, 0x35, 0x16, 0x35, 0x1b9},
    {0x0, 0x6a, 0x2d, 0x6b, 0x371},
    {0x0, 0x9f, 0x44, 0xa1, 0x528},
    {0x1, 0xd4, 0x5a, 0xd7, 0x6e1},
    {0x1, 0x109, 0x71, 0x10c, 0x89b},
    {0x1, 0x13e, 0x88, 0x141, 0xa56},
};

void drv_set_sys_freq(clk_src_e source, clk_val_e val)
{
    /* calculate the pllout frequence */
    uint8_t osr_freq = val >> 16;
    uint8_t pllm = val & 0x3f;
    uint8_t plln = (val >> 8) & 0x3f;
    uint8_t pllout = osr_freq * (pllm / plln);

    if (osr_freq != (EHS_VALUE / 1000000) || pllout < 16 || pllout >= 60 || source == IHS_CLK) {
        return;
    }

    g_system_clock = pllout * 1000000;

    /* set eflash control operation time */
    uint8_t index = ((pllout + 7) / 8) - 1;

    if (index >= sizeof(eflash_opt_time) / sizeof(eflash_opt_time_t)) {
        return;
    }

    uint32_t flag = csi_irq_save();

    eflash_opt_time_t *opt_time = (eflash_opt_time_t *)&eflash_opt_time[index];

    writel(opt_time->trc, (uint32_t *)CK_EFLASH_TRC);
    writel(opt_time->tnvs, (uint32_t *)CK_EFLASH_TNVS);
    writel(opt_time->tpgs, (uint32_t *)CK_EFLASH_TPGS);
    writel(opt_time->tprog, (uint32_t *)CK_EFLASH_TPROG);
    writel(opt_time->trcv_erase, (uint32_t *)CK_EFLASH_TRCV);

    val = val & 0xfff;
    int timeout = 10000;

    /* config pll and wait until stable */
    if (source == EHS_CLK) {
        val |= (3 << 18);
    }

    writel(val, (uint32_t *)PMU_PLL_CTRL);

    while (timeout--) {
        if (readl((uint32_t *)PMU_CLK_STABLE) & 0x00000010) {
            break;
        }
    }

    writel(MCLK_REG_VAL, (uint32_t *)PMU_MCLK_SEL);
    csi_irq_restore(flag);

}

int32_t drv_get_sys_freq(void)
{
    return g_system_clock;
}
