/****************************************************************************
 * include/nuttx/sensors/cs1237.h
 *
 *   Copyright (C) 2020 Fishsemi. All rights reserved.
 *   Author: dongjiuzhu<dongjiuzhu@fishsemi.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_CS1237_H
#define __INCLUDE_NUTTX_SENSORS_CS1237_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/pinctrl/pinctrl.h>

#if defined(CONFIG_SENSORS_CS1237)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CMD_CFG_REFO 1
#define CMD_CFG_ODR  2
#define CMD_ODR_GET  3
#define CMD_CFG_PGA  4
#define CMD_CFG_CH   5

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum {
  CS1237_FREQ_10HZ   = (0 << 4),
  CS1237_FREQ_40HZ   = (1 << 4),
  CS1237_FREQ_640HZ  = (2 << 4),
  CS1237_FREQ_1280HZ = (3 << 4),
}CS1237_FREQ;

typedef enum {
  CS1237_PGA_1   = (0 << 2),
  CS1237_PGA_2   = (1 << 2),
  CS1237_PGA_64  = (2 << 2),
  CS1237_PGA_128 = (3 << 2),
}CS1237_PGA;

typedef enum {
  CS1237_CH_A       = (0 << 0),
  CS1237_CH_RESERVE = (1 << 0),
  CS1237_CH_TEMP    = (2 << 0),
  CS1237_CH_TEST    = (3 << 0),
}CS1237_CH;

typedef enum {
  CS1237_REFO_ON  = (0 << 6),
  CS1237_REFO_OFF = (1 << 6),
}CS1237_REFO;

/* A reference to a structure of this type must be passed to the cs1237
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct cs1237_config_s
{
  CS1237_REFO refo;
  CS1237_FREQ freq;
  CS1237_PGA pga;
  CS1237_CH ch;

  /* The gpio number of clock must be provided for each CS1237 device */

  int clk_io;

  /* The gpio number of data must be provided for each CS1237 device */

  int data_io;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: cs1237_register
 *
 * Description:
 *   Register the cs1237 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/cs12370"
 *   pctl - An instance of the Pinctrl interface to use to communicate with cs1237
 *   ioe - An instance of the IOEXPANDER interface to use to communicate with cs1237
 *   config - configuration for the cs1237 driver. For details see description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cs1237_register(FAR const char *devpath, FAR struct pinctrl_dev_s *pctl,
                        FAR struct ioexpander_dev_s *ioe, FAR struct cs1237_config_s *config);
#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_CS1237 */
#endif /* __INCLUDE_NUTTX_SENSORS_CS1237_H */
