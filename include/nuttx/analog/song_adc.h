/****************************************************************************
 * include/nuttx/analog/song_adc.h
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: Yuan Zhang<zhangyuan@fishsemi.com>
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

#ifndef __INCLUDE_NUTTX_ANALOG_SONG_ADC_H
#define __INCLUDE_NUTTX_ANALOG_SONG_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>
#include <stdint.h>

#if defined(CONFIG_ADC_SONG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get PGADC vbat value
 * IN: adc port
 * OUT: vbat value */

#define ANIOC_SONG_ADC_VBAT             _ANIOC(AN_SONG_ADC_FIRST + 0)

/* Calibrate PGADC
 * IN: calibration parameter
 * OUT: calibration result */

#define ANIOC_SONG_ADC_VBAT_CALIBRATE   _ANIOC(AN_SONG_ADC_FIRST + 1)

/* Get PGADC origin value
 * IN: adc port
 * OUT: origin value */

#define ANIOC_SONG_ADC_VALUE            _ANIOC(AN_SONG_ADC_FIRST + 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* vbat value = (k * origin value + b) / 100 */

struct song_adc_vbat_cal_s
{
  int32_t k;
  int32_t b;
};

struct song_adc_config_s
{
  uint32_t  minor;
  uint32_t  base;
  uint32_t  ctl_off;
  uint32_t  sta_off;
  uint32_t  clk_off;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: song_adc_register
 *
 * Description:
 *   Register the song adc device as 'devpath'
 *
 * Input Parameters:
 *   song_adc_config_s - adc config
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int song_adc_register(FAR const struct song_adc_config_s *cfg);

#endif /* CONFIG_ADC_SONG */
#endif /* __INCLUDE_NUTTX_ANALOG_SONG_ADC_H */
