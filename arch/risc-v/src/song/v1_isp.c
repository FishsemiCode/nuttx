/****************************************************************************
 * arch/risc-v/src/song/v1_isp.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Doujianglong <doujianglong@pinecone.net>
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

#ifdef CONFIG_ARCH_CHIP_V1_ISP

#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/fs/partition.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/power/regulator.h>
#include <nuttx/pwm/song_pwm.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>

#include <stdio.h>

#include "chip.h"
#include "up_arch.h"

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOP_PWR_BASE                (0xf9210000)

/****************************************************************************
 * Private Data
 ****************************************************************************/
#ifdef CONFIG_I2C_DW
FAR struct i2c_master_s *g_i2c[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/
#ifdef CONFIG_I2C_DW
static void up_i2c_init(void)
{
    static const struct dw_i2c_config_s config[] =
    {
        {
            .bus = 0,
            .base = 0xfa3e2000,
            .rate = 26000000,
            .mclk = "isp_i2c0_mclk",
            .irq = 17,
        },
        {
            .bus = 1,
            .base = 0xfa3e3000,
            .rate = 26000000,
            .mclk = "isp_i2c1_mclk",
            .irq = 18,
        }
    };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_i2c_allinitialize(config, config_num, g_i2c);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void riscv_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
  {
    .minor      = -1,
    .base       = TOP_PWR_BASE,
    .irq        = 21,
    .c1_max     = 2600,
    .c1_freq    = 26000000,
    .ctl_off    = 0x160,
    .calib_off  = 0x174,
    .c1_off     = 0x164,
    .c2_off     = 0x168,
    .spec_off   = 0x660,
    .intren_off = 0x3fc,
    .intrst_off = 0x3f0,
    .intr_bit   = 27,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif

}

void up_lateinitialize(void)
{
#ifdef CONFIG_I2C_DW
    up_i2c_init();
#endif
}

#endif /* CONFIG_ARCH_CHIP_V1_ISP */
