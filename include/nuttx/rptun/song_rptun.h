/********************************************************************************************
 * include/nuttx/rptun/song_rptun.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_RPTUN_SONG_RPTUN_H
#define __INCLUDE_NUTTX_RPTUN_SONG_RPTUN_H

#ifdef CONFIG_SONG_RPTUN

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/rptun/rptun.h>
#include <nuttx/mbox/mbox.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct song_rptun_config_s
{
  const char            *cpu_name;
  uint32_t              role;
  int32_t               ch_start_rx;
  int32_t               ch_vring_rx;
  int32_t               ch_start_tx;
  int32_t               ch_vring_tx;

  /* for talk between peer processor, like tl420 and cp */
  struct proc_shm       shm;

  /* the resource table info */
  struct rsc_table_info rsc;

  /* the resource table flash address */
  uintptr_t             rsc_flash;
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

struct rptun_dev_s *song_rptun_initialize(
                const struct song_rptun_config_s *config,
                struct mbox_dev_s *mbox_rx,
                struct mbox_dev_s *mbox_tx,
                int minor);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SONG_RPTUN */
#endif /* __INCLUDE_NUTTX_RPTUN_SONG_RPTUN_H */
