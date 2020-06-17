/****************************************************************************
 * include/nuttx/lcd/vk1056b_lcd.h
 *
 *   Copyright (C) 2020 Fishsemi Inc. All rights reserved.
 *   Author: clyde <liuyan@fishsemi.com>
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

#ifndef __INCLUDE_VK1056B_H
#define __INCLUDE_VK1056B_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
*****************************************************************************/

/* VK1056B IOCTL commands *****************************************************
 *
 * VK1056B_LCDIOC_DISP_ALL:
 *   Description:   Display all or not.
 *   Argument:		A boolean value.
 *   Returns:		None
 *
 * VK1056B_LCDIOC_BODY_TEMP:
 *   Description:	Dsiplay a body temperature.
 *   Argument:		A temperature.
 *   Returns:		None
 *
 * VK1056B_LCDIOC_RT_TIME:
 *   Description:	Display a real time.
 *   Argument:		A time.
 *   Returns:       None.
 *
 * VK1056B_LCDIOC_BATTERY:
 *   Description:	Display a battery.
 *   Argument:		None.
 *   Returns:       None.
 */
#define VK1056B_LCDIOC_DISP_ALL (0x01)
#define VK1056B_LCDIOC_BODY_TEMP (0x02)
#define VK1056B_LCDIOC_RT_TIME (0x03)
#define VK1056B_LCDIOC_BATTERY (0x04)

/****************************************************************************
 * Public Types
 ****************************************************************************/
struct lcd_vk1056b_config_s
{
  uint8_t power_gpio;
  uint8_t cs_gpio;
  uint8_t wr_gpio;
  uint8_t data_gpio;
};

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int lcd_vk1056b_register(FAR const struct lcd_vk1056b_config_s *config,
                         FAR struct ioexpander_dev_s *ioe);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_VK1056B_H  */
