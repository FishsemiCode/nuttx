/********************************************************************************************
 * include/nuttx/hwspinlock/hwspinlock.h
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H
#define __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H

#ifdef CONFIG_HWSPINLOCK

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hwspinlock_dev_s;

struct hwspinlock_ops_s
{
  bool (*trylock)(struct hwspinlock_dev_s *dev, int id, int priority);
  void (*unlock)(struct hwspinlock_dev_s *dev, int id);
};

struct hwspinlock_dev_s
{
  const struct hwspinlock_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

static inline bool hwspinlock_trylock(struct hwspinlock_dev_s *dev,
                                      int id, int priority)
{
  return dev->ops->trylock(dev, id, priority);
}

static inline void hwspinlock_lock(struct hwspinlock_dev_s *dev,
                                   int id, int priority)
{
  while (!dev->ops->trylock(dev, id, priority));
}

static inline void hwspinlock_unlock(struct hwspinlock_dev_s *dev,
                                     int id, int priority)
{
  dev->ops->unlock(dev, id);
}

#endif /* CONFIG_HWSPINLOCK */
#endif /* __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H */
