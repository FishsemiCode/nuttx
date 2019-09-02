/****************************************************************************
 * include/nuttx/lcd/ili9486_lcd_ioctl.h
 *
 *   Copyright (C) 2019 Fishsemi Inc. All rights reserved.
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

#ifndef __INCLUDE_ILI9486_IOCTL_H
#define __INCLUDE_ILI9486_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* IOCTL commands that may be supported by ILI9486 LCD drivers */

/* ILI9486_LCDIOC_GETATTRIBUTES:  Get the attributes of the IL9486 LCD
 *
 * argument:  Pointer to struct ili9486_lcd_attributes_s in which values will be
 *            returned
 */

#define ILI9486_LCDIOC_GETATTRIBUTES  _LCDIOC(0x0001)

/* ILI9486_LCDIOC_CLEAR: clear ILI9486 LCD screen
 *
 * argument:  NULL
 */

#define ILI9486_LCDIOC_CLEAR  _LCDIOC(0x0002)

/* ILI9486_LCDIOC_SET_CURSOR: Set position of cursor.
 *
 * argument: Pointer to ili9486_lcd_curpos_s which row and column be set
 */

#define ILI9486_LCDIOC_SET_MATRIX  _LCDIOC(0x0003)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Used with the ILI9486_LCDIOC_GETATTRIBUTES ioctl call */

struct ili9486_lcd_attributes_s
{
  uint16_t lcd_width;       /* Width of the LCD */
  uint16_t lcd_height;      /* Height of the LCD */
};

/* Used with the ILI9486_LCDIOC_CURPOS ioctl call */

struct ili9486_lcd_matrix_s
{
  uint16_t x;             /* X pointer in the horizontal coordinate (zero-based) */
  uint16_t y;             /* Y pointer in the vertical coordinate (zero-based) */
  uint16_t w;             /* Width of the matrix */
  uint16_t h;             /* Height of the matrix */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif
