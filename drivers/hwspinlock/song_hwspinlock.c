/****************************************************************************
 * drivers/hwspinlock/song_hwspinlock.c
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/hwspinlock/hwspinlock.h>
#include <nuttx/hwspinlock/song_hwspinlock.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_hwspinlock_dev_s
{
  struct hwspinlock_dev_s                hwspinlock;
  const struct song_hwspinlock_config_s *config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool song_hwspinlock_trylock(struct hwspinlock_dev_s *dev,
                                    int id, int priority);
static void song_hwspinlock_unlock(struct hwspinlock_dev_s *dev, int id);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct hwspinlock_ops_s g_song_hwspinlock_ops =
{
  .trylock = song_hwspinlock_trylock,
  .unlock  = song_hwspinlock_unlock,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t song_hwspinlock_read(
                                const struct song_hwspinlock_config_s *config,
                                uint32_t offset)
{
  return (*(volatile uint32_t *)(config->base + B2C(offset)));
}

static inline void song_hwspinlock_write(
                                const struct song_hwspinlock_config_s *config,
                                uint32_t offset, uint32_t val)
{
  *(volatile uint32_t *)(config->base + B2C(offset)) = val;
}

static void song_hwspinlock_request(
                                const struct song_hwspinlock_config_s *config,
                                int id, int priority, bool req)
{
  uint32_t offset;
  uint32_t value;
  uint32_t bit;
  uint32_t en;

  offset = config->arbiter_req + (id / 4) * 4;
  bit    = (id % 4) * 8;
  en     = (1 << 7) | (1 << 1);
  value  = ((priority << 4) | req | en) << bit;

  song_hwspinlock_write(config, offset, value);
}

static bool song_hwspinlock_trylock(struct hwspinlock_dev_s *dev,
                                    int id, int priority)
{
  struct song_hwspinlock_dev_s *priv = (struct song_hwspinlock_dev_s *)dev;
  const struct song_hwspinlock_config_s *config = priv->config;
  uint32_t ack;

  song_hwspinlock_request(config, id, priority, true);

  ack = song_hwspinlock_read(config, config->arbiter_ack);
  if ((ack & (1 << id)) == 0)
    {
      song_hwspinlock_request(config, id, priority, false);
      return false;
    }

  return true;
}

static void song_hwspinlock_unlock(struct hwspinlock_dev_s *dev, int id)
{
  struct song_hwspinlock_dev_s *priv = (struct song_hwspinlock_dev_s *)dev;

  song_hwspinlock_request(priv->config, id, 0, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct hwspinlock_dev_s *song_hwspinlock_initialize(
                            const struct song_hwspinlock_config_s *config)
{
  struct song_hwspinlock_dev_s *priv;

  priv = kmm_zalloc(sizeof(struct song_hwspinlock_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->hwspinlock.ops = &g_song_hwspinlock_ops;
  priv->config = config;

  song_hwspinlock_write(config, config->arbiter_en, 0xff);

  return (struct hwspinlock_dev_s *)priv;
}
