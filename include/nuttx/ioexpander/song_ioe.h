/****************************************************************************
 * include/nuttx/ioexpander/song_ioe.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin<zhuyanlin@pinecone.net>
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_SONG_IOE_H
#define __INCLUDE_NUTTX_IOEXPANDER_SONG_IOE_H

/****************************************************************************
 * Included Files
****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct song_ioe_config_s
{
  uint32_t        cpu;

  /* mask: extra MASK pins for other cpus,
   * (en | cpu0) | (en | cpu1) << 8 | (en | cpu2) << 16 | (en | cpu3) << 24
   * en = 0x80, means active.
   * en = 0x00, means inactive.
   */

  uint32_t        mask;
  uint32_t        base;
  uint32_t        irq;
  FAR const char *mclk;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:song_ioe_initialize
 *
 * Description:
 *   Create ioe device driver instances for song platform.
 *
 * Input Parameters:
 *   cfg - Pointer to struct song_ioe_config_s
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *song_ioe_initialize(FAR const struct song_ioe_config_s *cfg);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_IOE_H */
