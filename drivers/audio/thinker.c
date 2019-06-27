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

#define THINKER_IR              (0x002c)
#define THINKER_IMR             (0x0030)
#define THINKER_SMOOTH_CONFIG2  (0x0094)
#define THINKER_MCTRL_CR        (0x01fc)

#define THINKER_VAD_T_INTR      (0)
#define THINKER_VAD_F_INTR      (1)
#define THINKER_VAD_P_INTR      (2)
#define THINKER_FBANK_INTR      (3)
#define THINKER_DNN_INTR        (4)
#define THINKER_SMOOTH_INTR     (5)
#define THINKER_FIFO_ERROR_INTR (6)
#define THINKER_IR_COUNT        (7)

#define THINKER_MCTRL_WAKEUP    (0)
#define THINKER_MCTRL_COMMAND   (1)
#define THINKER_MCTRL_PRE       (2)

#define THINKER_MCTRL_STRAT     (1 << 0)

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
  uint32_t status, id, index, cmode, imr;
  struct audio_msg_s msg;
  int i;

  status = thinker_read(thinker, THINKER_IR);
  imr = thinker_read(thinker, THINKER_IMR);
  cmode = (thinker_read(thinker, THINKER_MCTRL_CR) >> 1) & 0x3;
  index = thinker_read(thinker, THINKER_SMOOTH_CONFIG2) & 0xf;
  thinker_write(thinker, THINKER_IR, status);

  switch (cmode)
    {
      case THINKER_MCTRL_WAKEUP:
          id = AUDIO_MSG_WAKEUP;
          break;
      case THINKER_MCTRL_COMMAND:
          id = AUDIO_MSG_COMMAND;
          break;
      default :
          syslog(LOG_ERR, "Invalid mode ctrl: %x\n", cmode);
          return OK;
    }

  for (i = 0; i < THINKER_IR_COUNT; i++)
    {
      if (!(imr & (status & (1 << i))))
        continue;

      if (i != THINKER_FBANK_INTR
              && i != THINKER_SMOOTH_INTR)
        {
          syslog(LOG_ERR, "Uncaught INTR: %d\n", i);
          continue;
        }

      msg.msgId = (i == THINKER_FBANK_INTR) ? AUDIO_MSG_SLIENCE : id;
      msg.u.data = index;

      if (thinker->dev.upper && thinker->dev.priv)
        {
          thinker->dev.upper(thinker->dev.priv, AUDIO_CALLBACK_MESSAGE, (struct ap_buffer_s *)&msg, OK);
        }
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
    return 0;

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

  thinker_update_bits(thinker, THINKER_MCTRL_CR, THINKER_MCTRL_STRAT, 1);

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int thinker_stop(struct audio_lowerhalf_s *dev, void *session)
#else
static int thinker_stop(struct audio_lowerhalf_s *dev)
#endif
{
  struct thinker_dev_s *thinker = (struct thinker_dev_s *)dev;

  thinker_update_bits(thinker, THINKER_MCTRL_CR, THINKER_MCTRL_STRAT, 0);

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

  thinker_write(thinker, THINKER_IMR, 1 << THINKER_SMOOTH_INTR);

  return &thinker->dev;
}
