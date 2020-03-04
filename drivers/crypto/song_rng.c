/****************************************************************************
 * drivers/crypto/song_rng.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
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

#include <poll.h>
#include <string.h>
#include <sys/random.h>

#include <nuttx/arch.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/clk/clk.h>

#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_RNG_STOP             (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_rng_s
{
  volatile uint32_t STA;
  volatile uint32_t CTRL;
  volatile uint32_t SKIP[34];
  volatile uint32_t DATA[8];
};

struct song_rng_dev_s
{
    uint8_t open_count;    /* Number of times the device has been opened */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int song_rng_open(FAR struct file *filep);
static int song_rng_close(FAR struct file *filep);
static ssize_t song_rng_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int song_rng_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct song_rng_s * const g_song_rng
  = (FAR struct song_rng_s *)CONFIG_SONG_RNG_BASE;

static sem_t g_song_rng_sem = SEM_INITIALIZER(1);

static const struct file_operations g_song_rng_fops =
{
  song_rng_open,                /* open */
  song_rng_close,               /* close */
  song_rng_read,                /* read */
  NULL,                         /* write */
  NULL,                         /* seek */
  NULL                          /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , song_rng_poll               /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

static struct song_rng_dev_s g_song_rng_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: song_rng_open
 ****************************************************************************/

static int song_rng_open(FAR struct file *filep)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct song_rng_dev_s *dev   = inode->i_private;
  uint8_t                   use_count;

  /* Increment the count of references to the device. */

  use_count = dev->open_count + 1;
  if (use_count == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      return -EMFILE;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (use_count == 1)
    {
      clk_enable(clk_get("security_clk32k"));
    }

  dev->open_count = use_count;

  return 0;
}

/****************************************************************************
 * Name: song_rng_close
 ****************************************************************************/

static int song_rng_close(FAR struct file *filep)
{
  FAR struct inode          *inode = filep->f_inode;
  FAR struct song_rng_dev_s *dev   = inode->i_private;
  uint8_t                   use_count;

  use_count = dev->open_count - 1;

  if (use_count > 1)
    {
      dev->open_count = use_count;
      return OK;
    }

  /* Check if this is the last time that the driver has been closed. */

  if (use_count == 0)
    {
      clk_disable(clk_get("security_clk32k"));
    }

  if (use_count < 0)
    {
      /* More close than open */

      return -EMFILE;
    }

  dev->open_count = use_count;

  return 0;
}

/****************************************************************************
 * Name: song_rng_read
 ****************************************************************************/

static ssize_t song_rng_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  size_t nbytes = len;

  while (nxsem_wait(&g_song_rng_sem) != 0)
    ; /* Ignore -EINTR error */

  g_song_rng->CTRL &= ~SONG_RNG_STOP;
  while (nbytes > 0)
    {
      size_t copied = sizeof(g_song_rng->DATA);

      if (copied > nbytes)
        {
          copied = nbytes;
        }

      up_udelay(31); /* Wait one 32KHz before copy */
      memcpy(buffer, (FAR void *)g_song_rng->DATA, copied);

      buffer += copied;
      nbytes -= copied;
    }
  g_song_rng->CTRL |= SONG_RNG_STOP;

  nxsem_post(&g_song_rng_sem);
  return len;
}

/****************************************************************************
 * Name: song_rng_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int song_rng_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  if (setup)
    {
      fds->revents = fds->events & POLLIN;
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Register /dev/random
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  g_song_rng->CTRL |= SONG_RNG_STOP; /* Ensure the hardware stop */
  register_driver("/dev/random", &g_song_rng_fops, 0444, &g_song_rng_dev);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
#ifndef CONFIG_DEV_RANDOM
  g_song_rng->CTRL |= SONG_RNG_STOP; /* Ensure the hardware stop */
#endif
  register_driver("/dev/urandom", &g_song_rng_fops, 0444, &g_song_rng_dev);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
