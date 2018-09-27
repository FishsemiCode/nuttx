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
 * @file     sys_freq.h
 * @brief    header file for setting system frequency.
 * @version  V1.0
 * @date     18. July 2018
 ******************************************************************************/
#ifndef _SYS_FREQ_H_
#define _SYS_FREQ_H_
#ifndef __ASSEMBLY__
#include <stdint.h>
#include "soc.h"

#define PMU_MCLK_SEL  (CSKY_CLKGEN_BASE + 0x4)
#define MCLK_REG_VAL  0x8UL

#define PMU_CLK_STABLE  (CSKY_CLKGEN_BASE + 0x18)
#define PMU_PLL_CTRL  (CSKY_CLKGEN_BASE + 0x2c)

typedef enum {
    IHS_CLK       = 0,         /* internal high speed clock */
    EHS_CLK       = 1          /* external high speed clock */
} clk_src_e;

typedef enum {
    OSR_8M_CLK_16M      = 0x80204,
    OSR_8M_CLK_24M      = 0x80206,
    OSR_8M_CLK_32M      = 0x80208,
    OSR_8M_CLK_40M      = 0x8020a,
    OSR_8M_CLK_48M      = 0x8020c
} clk_val_e;

void drv_set_sys_freq(clk_src_e source, clk_val_e val);
int32_t drv_get_sys_freq(void);

#endif /* __ASSEMBLY__ */
#endif /* _SYS_FREQ_H_ */

