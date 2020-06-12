/****************************************************************************
 * arch/arm/src/song/u1_common.h
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

#ifndef __ARCH_ARM_SRC_SONG_U1_COMMON_H
#define __ARCH_ARM_SRC_SONG_U1_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOP_PMICFSM_BASE                (0xb2010000)
#define TOP_PMICFSM_INT_STATUS          (TOP_PMICFSM_BASE + 0x04)
#define TOP_PMICFSM_INT_MASK            (TOP_PMICFSM_BASE + 0x08)
#define TOP_PMICFSM_CONFIG1             (TOP_PMICFSM_BASE + 0x0c)
#define TOP_PMICFSM_CONFIG2             (TOP_PMICFSM_BASE + 0x10)
#define TOP_PMICFSM_WAKEUP_ENABLE       (TOP_PMICFSM_BASE + 0x14)
#define TOP_PMICFSM_WAKEUP_REASON       (TOP_PMICFSM_BASE + 0x18)
#define TOP_PMICFSM_BUCK1               (TOP_PMICFSM_BASE + 0x24)
#define TOP_PMICFSM_LDO0                (TOP_PMICFSM_BASE + 0x28)
#define TOP_PMICFSM_LDO1                (TOP_PMICFSM_BASE + 0x2c)
#define TOP_PMICFSM_IOMOD_CTL           (TOP_PMICFSM_BASE + 0xbc)
#define TOP_PMICFSM_PLLTIME             (TOP_PMICFSM_BASE + 0xe8)
#define TOP_PMICFSM_TRIM0               (TOP_PMICFSM_BASE + 0xf0)
#define TOP_PMICFSM_AP_M4_INT2SLP_MK0   (TOP_PMICFSM_BASE + 0xf4)
#define TOP_PMICFSM_CP_M4_INT2SLP_MK0   (TOP_PMICFSM_BASE + 0xf8)

#define TOP_PMICFSM_SLP_U0RXD_ACT       (1 << 10)

#define TOP_PMICFSM_DS_SLP_VALID        (1 << 0)

#define TOP_PMICFSM_PON_ENABLE          (5 << 4)
#define TOP_PMICFSM_UART_ENABLE         (1 << 8)
#define TOP_PMICFSM_RTC_ENABLE          (1 << 12)
#define TOP_PMICFSM_GPIO0_ENABLE        (1 << 16)
#define TOP_PMICFSM_GPIO1_ENABLE        (1 << 20)
#define TOP_PMICFSM_GPIO2_ENABLE        (1 << 24)
#define TOP_PMICFSM_GPIO3_ENABLE        (1 << 28)

#define TOP_PMICFSM_FIRST_PON           (1 << 0)
#define TOP_PMICFSM_PON                 (1 << 1)
#define TOP_PMICFSM_UART                (1 << 2)
#define TOP_PMICFSM_RTC                 (1 << 3)
#define TOP_PMICFSM_GPIO0               (1 << 4)
#define TOP_PMICFSM_GPIO1               (1 << 5)
#define TOP_PMICFSM_GPIO2               (1 << 6)
#define TOP_PMICFSM_GPIO3               (1 << 7)
#define TOP_PMICFSM_WDT_RSTN            (1 << 8)
#define TOP_PMICFSM_SOFT_RSTN           (1 << 9)
#define TOP_PMICFSM_BUTTON_RSTN         (1 << 10)

#define TOP_PMICFSM_DS_RF_RST_MK        (1 << 6)

#define TOP_PMICFSM_LDO0_RF_ICTRL_MK    (3 << 14)
#define TOP_PMICFSM_LDO0_RF_ICTRL_1     (1 << 14)

#define TOP_PMICFSM_LDO0_ACTIVE_ON      (0x1 << 0)
#define TOP_PMICFSM_LDO0_SLEEP_ON       (0x1 << 1)
#define TOP_PMICFSM_LDO0_VOLT_900mV     (0x0 << 8)
#define TOP_PMICFSM_LDO0_DEFAULT        (TOP_PMICFSM_LDO0_ACTIVE_ON | \
                                         TOP_PMICFSM_LDO0_SLEEP_ON  | \
                                         TOP_PMICFSM_LDO0_VOLT_900mV)

#define TOP_PMICFSM_LDO1_VOLT_MK        (0x3f << 8)
#define TOP_PMICFSM_LDO1_VOLT_0x19      (0x19 << 8)

#define TOP_PMICFSM_IOMOD_PUD_CTL_MK    (0x3 << 2)
#define TOP_PMICFSM_IOMOD_PUD_CTL_NO    (0x0 << 2)

#define TOP_PMICFSM_PLL_STABLE_TIME     (0xf << 8)
#define TOP_PMICFSM_OSC_STABLE_TIME     (0x52 << 0)

/* RAMDISK */

#define U1_SP_RAMRETENT_BASE    (0xb1000000)
#define U1_RAMDISK_SECTOR_SZ    (128)
#define U1_SP_RAMDISK_OFFSET    (0)
#define U1_SP_RAMDISK_SECTOR    (16 * 8)
#define U1_SP_RAMDISK_SIZE      (U1_SP_RAMDISK_SECTOR * U1_RAMDISK_SECTOR_SZ)
#define U1_SP_RAMDISK_BASE      (U1_SP_RAMRETENT_BASE + U1_SP_RAMDISK_OFFSET)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum wakeup_reason_e
{
  WAKEUP_REASON_FIRST_PON,
  WAKEUP_REASON_PON_RSTN,
  WAKEUP_REASON_UART_RSTN,
  WAKEUP_REASON_GPIO_RSTN,
  WAKEUP_REASON_RTC_RSTN,
  WAKEUP_REASON_WDT_RSTN,
  WAKEUP_REASON_SOFT_RSTN,
  WAKEUP_REASON_BUTTON_RSTN,
  WAKEUP_REASON_MAX
};

/****************************************************************************
 * Static Inline Functions
 ****************************************************************************/

static inline enum wakeup_reason_e up_get_wkreason(void)
{
  uint32_t val = getreg32(TOP_PMICFSM_WAKEUP_REASON);

  if (val & TOP_PMICFSM_FIRST_PON)
    {
      return WAKEUP_REASON_FIRST_PON;
    }
  else if (val & TOP_PMICFSM_PON)
    {
      return WAKEUP_REASON_PON_RSTN;
    }
  else if (val & TOP_PMICFSM_UART)
    {
      return WAKEUP_REASON_UART_RSTN;
    }
  else if (val & (TOP_PMICFSM_GPIO0 |
                  TOP_PMICFSM_GPIO1 |
                  TOP_PMICFSM_GPIO2 |
                  TOP_PMICFSM_GPIO3))
    {
      return WAKEUP_REASON_GPIO_RSTN;
    }
  else if (val & TOP_PMICFSM_RTC)
    {
      return WAKEUP_REASON_RTC_RSTN;
    }
  else if (val & TOP_PMICFSM_WDT_RSTN)
    {
      return WAKEUP_REASON_WDT_RSTN;
    }
  else if (val & TOP_PMICFSM_SOFT_RSTN)
    {
      return WAKEUP_REASON_SOFT_RSTN;
    }
  else if (val & TOP_PMICFSM_BUTTON_RSTN)
    {
      return WAKEUP_REASON_BUTTON_RSTN;
    }
  else
    {
      return WAKEUP_REASON_WDT_RSTN;
    }
}

static inline char *up_get_wkreason_env(void)
{
  enum wakeup_reason_e reason = up_get_wkreason();
  char *env;

  switch (reason)
    {
      case WAKEUP_REASON_FIRST_PON:
        env = "first_pon";
        break;
      case WAKEUP_REASON_UART_RSTN:
        env = "uart_rstn";
        break;
      case WAKEUP_REASON_GPIO_RSTN:
        env = "gpio_rstn";
        break;
      case WAKEUP_REASON_RTC_RSTN:
        env = "rtc_rstn";
        break;
      case WAKEUP_REASON_WDT_RSTN:
        env = "wdt_rstn";
        break;
      case WAKEUP_REASON_SOFT_RSTN:
        env = "soft_rstn";
        break;
      case WAKEUP_REASON_BUTTON_RSTN:
        env = "button_rstn";
        break;
      default:
        env = "pon_rstn";
        break;
    }

  return env;
}

static inline bool up_is_warm_rstn(void)
{
  enum wakeup_reason_e wakeup_reason = up_get_wkreason();

  if (wakeup_reason == WAKEUP_REASON_GPIO_RSTN ||
      wakeup_reason == WAKEUP_REASON_UART_RSTN ||
      wakeup_reason == WAKEUP_REASON_PON_RSTN ||
      wakeup_reason == WAKEUP_REASON_RTC_RSTN)
    {
      return true;
    }

  return false;
}

static inline bool up_is_u1v1(void)
{
  return getreg32(TOP_PMICFSM_PLLTIME) == 0xdeadbeaf;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_file_copy(char *dstfile, char *srcfile);
int up_folder_copy(char *dstdir, char *srcdir);
int up_folder_copy_append(char *dstdir, char *srcdir);

#endif //__ARCH_ARM_SRC_SONG_U1_COMMON_H
