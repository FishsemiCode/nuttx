/****************************************************************************
 * arch/ceva/src/xm6/up_hardfault.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

#include <assert.h>
#include <debug.h>

#include "cpm.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_P_ECADD                             0x051c
#define REG_P_MAPAR                             0x0520
#define REG_P_MAPSR                             0x0524
#define REG_P_MECCCOR                           0x0530
#define REG_D_ECCCOR                            0x0784
#define REG_D_ECCERR                            0x0788
#define REG_D_MECCCOR                           0x078c
#define REG_D_MECCERR                           0x0790
#define REG_D_SECCCOR                           0x0794
#define REG_D_SECCERR                           0x0798
#define REG_UOP_STS                             0x0c58
#define REG_UOP_PAR                             0x0c5c
#define REG_MAPAR                               0x0c80
#define REG_MAPSR                               0x0c84
#define REG_DBG_GEN                             0x0d14
#define REG_DBG_GEN_2                           0x0d24
#define REG_DBG_DUNMPD                          0x0d30
#define REG_HIST_OVERFLOW                       0x0d4c
#define REG_DBG_DESC_ID                         0x0d84
#define REG_DBG_QMAN_ID                         0x0d88

#ifdef CONFIG_DEBUG_HARDFAULT
# define hfalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define hfalert(x...)
#endif

#define hfdumpreg1(reg)                           \
  hfalert("%s: %08x\n",                           \
          #reg, getcpm(REG_##reg))

#define hfdumpreg2(reg1, reg2)                    \
  hfalert("%s: %08x %s: %08x\n",                  \
          #reg1, getcpm(REG_##reg1),              \
          #reg2, getcpm(REG_##reg2))

#define hfdumpreg3(reg1, reg2, reg3)              \
  hfalert("%s: %08x %s: %08x %s: %08x\n",         \
          #reg1, getcpm(REG_##reg1),              \
          #reg2, getcpm(REG_##reg2),              \
          #reg3, getcpm(REG_##reg3))

#define hfdumpreg4(reg1, reg2, reg3, reg4)        \
  hfalert("%s: %08x %s: %08x %s: %08x %s: %08x\n",\
          #reg1, getcpm(REG_##reg1),              \
          #reg2, getcpm(REG_##reg2),              \
          #reg3, getcpm(REG_##reg3),              \
          #reg4, getcpm(REG_##reg4))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_hardfault
 *
 * Description:
 *   This is Hard Fault exception handler.
 *
 ****************************************************************************/

int up_hardfault(int irq, FAR void *context, FAR void *arg)
{
  /* Dump some hard fault info */

  hfalert("Hard Fault:\n");
  hfdumpreg4(P_ECADD   , P_MAPAR      , P_MAPSR    , P_MECCCOR  );
  hfdumpreg4(D_ECCCOR  , D_ECCERR     , D_MECCCOR  , D_MECCERR  );
  hfdumpreg4(D_SECCCOR , D_SECCERR    , UOP_STS    , UOP_PAR    );
  hfdumpreg4(MAPAR     , MAPSR        , DBG_GEN    , DBG_GEN_2  );
  hfdumpreg4(DBG_DUNMPD, HIST_OVERFLOW, DBG_DESC_ID, DBG_QMAN_ID);

  PANIC();
  return OK;
}
