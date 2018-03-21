/****************************************************************************
 * drivers/ioexpander/song_ioe.h
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

#ifndef __DRIVER_IOEXPANDER_SONG_IOE_H
#define __DRIVER_IOEXPANDER_SONG_IOE_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define SONG_IOE_INT_DISABLE              0x00
#define SONG_IOE_INT_HIGHLEVEL            0x01
#define SONG_IOE_INT_LOWLEVEL             0x03
#define SONG_IOE_INT_GPIOCLK_RISING       0x05
#define SONG_IOE_INT_GPIOCLK_FALLING      0x07
#define SONG_IOE_INT_GPIOCLK_BOTHEDGES    0x09

#define SONG_IOE_REG(base, ioe, num)      ((base) + ((ioe) / (num)) * 4)
#define SONG_IOE_PORT_DR(ioe)             SONG_IOE_REG(0x000, ioe, 16)
#define SONG_IOE_PORT_DDR(ioe)            SONG_IOE_REG(0x040, ioe, 16)
#define SONG_IOE_EXT_PORT(ioe)            SONG_IOE_REG(0x080, ioe, 32)
#define SONG_IOE_INTR_CTRL(ioe)           SONG_IOE_REG(0x0A0, ioe, 4)
#define SONG_IOE_DEBOUNCE(ioe)            SONG_IOE_REG(0x1A0, ioe, 16)
#define SONG_IOE_INTR_RAW(ioe)            SONG_IOE_REG(0x1E0, ioe, 32)
#define SONG_IOE_INTR_CLR(ioe)            SONG_IOE_REG(0x200, ioe, 32)
#define SONG_IOE_INTR_MASK(cpu, ioe)      SONG_IOE_REG((0x220 + cpu * 0x60), ioe, 16)
#define SONG_IOE_INTR_STATUS(cpu, ioe)    SONG_IOE_REG((0x260 + cpu * 0x60), ioe, 32)


#endif /* DRIVER_IOEXPANDER_SONG_IOE_H */
