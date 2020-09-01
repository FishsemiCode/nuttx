/********************************************************************************************
 * include/nuttx/eeprom/song_efuse.h
 *
 *   Copyright (C) 2020 Fishsemi Inc. All rights reserved.
 *   Author:Fishsemi
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

#ifndef __INCLUDE_NUTTX_EEPROM_SONG_EFUSE_H
#define __INCLUDE_NUTTX_EEPROM_SONG_EFUSE_H

#ifdef CONFIG_SONG_EFUSE

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* EFUSE ADDRESS AND SIZE */

#define EFUSE_PUBKEY_HASH_ADDR   (0x20)
#define EFUSE_PUBKEY_HASH_SIZE   (32)
#define EFUSE_FUNC_INFO_ADDR     (0x40)
#define EFUSE_FUNC_INFO_SIZE     (2)
#define EFUSE_FUNC_INFO_SEC_BOOT (0x10)
#define EFUSE_FUNC_INFO_USER     (0x46)
#define EFUSE_FUNC_READ          (1<<0)
#define EFUSE_FUNC_WRITE         (1<<1)

union efuse_ioctl_parameter
{
  struct
  {
    uint16_t data:16;
    uint8_t  length:8;
    uint8_t  address:8;
  };
  uint32_t all;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int song_efuse_initialize(int minor, uint32_t base, uint8_t irq);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SONG_EFUSE */
#endif /* __INCLUDE_NUTTX_EEPROM_SONG_EFUSE_H */
