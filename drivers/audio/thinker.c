/****************************************************************************
 * drivers/audio/thinker.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Pinecone <pinecone@pinecone.net>
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/thinker.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S/PCM MCTRL            ( 0x4000 ~  0x400b) */
/* IRQ CTRL                 ( 0x400c ~  0x402b) */
/* EXTRA REG                ( 0x4404 ~  0x4417) */
/* EXTRA COEF               ( 0x4500 ~  0x4503) SRAM */
/* FFT REG                  ( 0x4800 ~  0x480F) */
/* FFT COEF                 ( 0x4900 ~  0x4903) SRAM */
/* MELFILTER REG            ( 0x4C00 ~  0x4C13) */
/* MELFILTER COEF           ( 0x4D00 ~  0x4D03) SRAM */
/* CNN LAYER/WEIGHT         ( 0x5424 ~  0x542B) SRAM */
/* DNN REG                  ( 0x5800 ~  0x5807) */
/* SMART REG                ( 0x6000 ~  0x603F) */
/* SMART COEF               ( 0x6200 ~  0x6203) SRAM */
/* SMOOTH REG               ( 0x6800 ~  0x68BB) */
/* CNN WAKEUP WEIGHT        ( 0x8000 ~  0x8003) SRAM */
/* CNN WAKEUP COEF          ( 0x8004 ~  0x8007) SRAM */
/* CNN/DNN WAKEUP PARAM     (0x14400 ~ 0x15527) */
/* CNN CMD LAYER/WEIGHT     (0x25424 ~ 0x2542B) SRAM */
/* CNN CMD WEIGHT           (0x28000 ~ 0x28003) SRAM */
/* CNN CMD COEF             (0x28004 ~ 0x28007) SRAM */
/* DNN COEF/WEIGHT          (0x2A000 ~ 0x2A003) SRAM */
/* CMD NET STRUCT           (0x34400 ~ 0x35527) */

#define THINKER_CFG_IR      (0x4014)
#define THINKER_CTRL_REG    (0x401c)
#define THINKER_IR_REG      (0x4020)
#define THINKER_CMD_INDEX   (0x4028)

#define THINKER_IR_FBANK    (0)
#define THINKER_IR_SMOOTH1  (1)
#define THINKER_IR_SMOOTH2  (2)
#define THINKER_IR_SMART    (3)
#define THINKER_IR_COUNT    (4)

#define THINKER_CTRL_ENABLE (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct thinker_dev_s
{
  struct audio_lowerhalf_s  dev;
  uintptr_t                 base;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int thinker_getcaps(struct audio_lowerhalf_s *dev, int type, struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_reserve(struct audio_lowerhalf_s *dev, void **session);
static int thinker_release(struct audio_lowerhalf_s *dev, void *session);
static int thinker_configure(struct audio_lowerhalf_s *dev, void *session, const struct audio_caps_s *caps);
static int thinker_start(struct audio_lowerhalf_s *dev, void *session);
static int thinker_stop(struct audio_lowerhalf_s *dev, void *session);
#else
static int thinker_reserve(struct audio_lowerhalf_s *dev);
static int thinker_release(struct audio_lowerhalf_s *dev);
static int thinker_configure(struct audio_lowerhalf_s *dev, const struct audio_caps_s *caps);
static int thinker_start(struct audio_lowerhalf_s *dev);
static int thinker_stop(struct audio_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_thinker_audioops =
{
  .getcaps        = thinker_getcaps,
  .configure      = thinker_configure,
  .shutdown       = thinker_stop,
  .start          = thinker_start,
  .stop           = thinker_stop,
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .resume         = thinker_start,
  .pause          = thinker_stop,
#endif
  .reserve        = thinker_reserve,
  .release        = thinker_release,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void thinker_write(struct thinker_dev_s *thinker,
        uint32_t offset, uint32_t regval)
{
  uint32_t regaddr = thinker->base + offset;
  *(volatile uint32_t *)(regaddr) = (regval);
}

static inline uint32_t thinker_read(struct thinker_dev_s *thinker,
        uint32_t offset)
{
  uint32_t regaddr = thinker->base + offset;
  return *(volatile uint32_t *)(regaddr);
}

static inline void thinker_update_bits(struct thinker_dev_s *thinker,
                                  uint32_t offset, uint32_t mask,
                                  uint32_t regval)
{
  thinker_write(thinker, offset, (regval & mask) |
                 (thinker_read(thinker, offset) & ~mask));
}

static int song_thinker_irq_handler(int irq, void *context, void *args)
{
  struct thinker_dev_s *thinker = args;
  uint32_t status, id, index;
  struct audio_msg_s msg;
  int i;

  status = thinker_read(thinker, THINKER_IR_REG);

  thinker_write(thinker, THINKER_CFG_IR, status);

  for (i = 0; i < THINKER_IR_COUNT; i++)
    {
      if (!(status & (1 << i)))
        continue;

      if (i == THINKER_IR_FBANK)
        {
          id = AUDIO_MSG_SLIENCE;
          index = 0;
        }
      else if (i == THINKER_IR_SMOOTH1)
        {
          id = AUDIO_MSG_WAKEUP;
          index = 0;
        }
      else if (i == THINKER_IR_SMOOTH2)
        {
          id = AUDIO_MSG_WAKEUP;
          index = 1;
        }
      else if (i == THINKER_IR_SMART)
        {
          id = AUDIO_MSG_COMMAND;
          index = thinker_read(thinker, THINKER_CMD_INDEX);
        }

      msg.msgId = id;
      msg.u.data = index;

      thinker->dev.upper(thinker->dev.priv, AUDIO_CALLBACK_MESSAGE, (struct ap_buffer_s *)&msg, OK);
    }

  return OK;
}

static int thinker_getcaps(struct audio_lowerhalf_s *dev, int type,
        struct audio_caps_s *caps)
{
  return sizeof(struct audio_caps_s);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_configure(struct audio_lowerhalf_s *dev,
        void *session,
        const struct audio_caps_s *caps)
#else
static int thinker_configure(struct audio_lowerhalf_s *dev,
        const struct audio_caps_s *caps)
#endif
{
  struct thinker_dev_s *thinker = (struct thinker_dev_s *)dev;
  uint32_t *vector = (uint32_t *)&caps->ac_controls.qw;
  uint32_t size, *start, *end;

  if (caps->ac_type != AUDIO_TYPE_EXTENSION
        || caps->ac_format.hw != AUDIO_EU_LOAD_MODULE)
    return -EINVAL;

  size = vector[0];
  start = (uint32_t *)vector[1];

  for (end = start + size; start < end; start += 2)
    thinker_write(thinker, (*start & 0xFFFFF), *(start + 1));

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_start(struct audio_lowerhalf_s *dev, void *session)
#else
static int thinker_start(struct audio_lowerhalf_s *dev)
#endif
{
  struct thinker_dev_s *thinker = (struct thinker_dev_s *)dev;

  thinker_update_bits(thinker, THINKER_CTRL_REG, THINKER_CTRL_ENABLE, 1);

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_stop(struct audio_lowerhalf_s *dev, void *session)
#else
static int thinker_stop(struct audio_lowerhalf_s *dev)
#endif
{
  struct thinker_dev_s *thinker = (struct thinker_dev_s *)dev;

  thinker_update_bits(thinker, THINKER_CTRL_REG, THINKER_CTRL_ENABLE, 0);

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_reserve(struct audio_lowerhalf_s *dev,
        void **session)
#else
static int thinker_reserve(struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_release(struct audio_lowerhalf_s *dev,
        void *session)
#else
static int thinker_release(struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

struct audio_lowerhalf_s *thinker_initialize(uintptr_t base, int irq)
{
  struct thinker_dev_s *thinker;

  thinker = kmm_zalloc(sizeof(struct thinker_dev_s));
  if (!thinker)
    return NULL;

  thinker->dev.ops = &g_thinker_audioops;
  thinker->base = base;

  irq_attach(irq, song_thinker_irq_handler, thinker);
  up_enable_irq(irq);

  return &thinker->dev;
}
