/****************************************************************************
 * include/nuttx/i2c/i2c_dw.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Jiuzhu Dong<dongjiuzhu@pinecone.net>
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

#ifndef __INCLUDE_I2C_DW_H
#define __INCLUDE_I2C_DW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct dw_i2c_config_s
{
  int bus;
  uintptr_t base;
  FAR const char *mclk;
  FAR const char *pclk;
  uint32_t rate;
  uint32_t irq;
  uint32_t sda_hold;
  uint32_t fs_spklen;
  uint32_t hs_spklen;
  uint32_t ss_hcnt;
  uint32_t ss_lcnt;
  uint32_t fs_hcnt;
  uint32_t fs_lcnt;
  uint32_t hs_hcnt;
  uint32_t hs_lcnt;
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

FAR struct i2c_master_s *dw_i2c_initialize(FAR const struct dw_i2c_config_s *config);

void dw_i2c_allinitialize(FAR const struct dw_i2c_config_s *config, int config_num,
                          FAR struct i2c_master_s **i2c);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_I2C_I2C_DW_H */
