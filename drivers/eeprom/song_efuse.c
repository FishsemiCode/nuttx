/****************************************************************************
 * drivers/eeprom/song_efuse.c
 * Character driver for the fishsemi efuse driver
 *
 *   Copyright (C) . All rights reserved.
 *   Author: fishsemi
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

#include <stdlib.h>
#include <debug.h>
#include <stdio.h>
#include <nuttx/arch.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/eeprom/song_efuse.h>

#if defined(CONFIG_SONG_EFUSE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EFUSE_CTRL_MASK          (1)
#define EFUSE_CTRL_ENABLE        (1)
#define EFUSE_CTRL_DISABLE       (0)
#define EFUSE_CTRL_IDLE          (0)
#define EFUSE_CTRL_BUSY          (1)

#define EFUSE_MODE_MASK          (1)
#define EFUSE_MODE_READ          (0)
#define EFUSE_MODE_PROG          (1)

#define EFUSE_INTR_EN_MASK       (0x03)
#define EFUSE_INTR_EFUSE_EN      (1 << 1)
#define EFUSE_INTR_EFUSE_ST      (1 << 1)
#define EFUSE_INTR_PGM_EN        (1 << 2)

#define EFUSE_INTR_STS_MASK      (0x03)
#define EFUSE_INTR_EFUSE_STS     (1 << 1)
#define EFUSE_INTR_PGM_STS       (1 << 2)
#define EFUSE_INTR_CLEAR         (1 << 1)

/* EFUSE REGISTER */

#define EFUSE_EN_OFFSET          (0x80)
#define EFUSE_STS_OFFSET         (0x84)
#define EFUSE_RAW_OFFSET         (0x88)
#define EFUSE_CTRL_OFFSET        (0x200)
#define EFUSE_MODE_OFFSET        (0x204)
#define EFUSE_ADDR_OFFSET        (0x208)
#define EFUSE_LENGTH_OFFSET      (0x20C)
#define EFUSE_CNT_OFFSET         (0x210)
#define EFUSE_DATA_OFFSET        (0x220)

#define EFUSE_SEM_WAIT_10MS      (10)
#define EFUSE_READ_MAX_NUM       (128)

/****************************************************************************
 * Private
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

# define getreg32(a)          (*(volatile uint32_t *)(a))
# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))
# define modreg32(v,m,a)      (putreg32((getreg32(a) & ~(m)) | ((v) & (m)), a))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct efuse_dev_s
{
  uint32_t base;
  mutex_t  mutex;
  sem_t    sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int inline efuse_lock(FAR struct efuse_dev_s *efuse);
static int inline efuse_unlock(FAR struct efuse_dev_s *efuse);
static int efuse_read_register(FAR struct efuse_dev_s *priv, uint32_t addr,
                               uint8_t *data, uint32_t cnt);
static int32_t efuse_programbit(FAR struct efuse_dev_s *priv, uint32_t bit);
static int32_t efuse_programdata(FAR struct efuse_dev_s *priv, uint32_t addr,
                                 const uint8_t *data, uint32_t cnt);
/* Character driver methods */

static int     efuse_open(FAR struct file *filep);
static int     efuse_close(FAR struct file *filep);
static ssize_t efuse_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t efuse_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     efuse_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static off_t   efuse_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_efusefops =
{
  efuse_open,
  efuse_close,
  efuse_read,
  efuse_write,
  efuse_seek,
  efuse_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_efuse_isr
 *
 * Description:
 *   operate efuse interrupt.
 *
 ****************************************************************************/

static int up_efuse_isr(int irq, FAR void* context, FAR void *arg)
{
  struct efuse_dev_s *priv = (struct efuse_dev_s *)(arg);
  int ret = OK;

  /* Clear interrupt bit */

  modreg32(EFUSE_INTR_CLEAR, EFUSE_INTR_EFUSE_STS, priv->base + EFUSE_STS_OFFSET);

  ret = nxsem_post(&priv->sem);
  if (ret < 0)
    {
      ferr("Error: nx_sem post failed :%d", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: efuse_sem_wait
 *
 * Description:
 *   sem wait timed.
 *
 ****************************************************************************/

static int efuse_sem_wait(sem_t *sem, unsigned int timeout_ms)
{
  struct timespec abstime;
  unsigned int timeout_sec;

  /* Get the current time */

  clock_gettime(CLOCK_REALTIME, &abstime);

  timeout_sec = timeout_ms / 1000;
  abstime.tv_sec += timeout_sec;
  abstime.tv_nsec += (1000 * 1000) * (timeout_ms % 1000);

  if (abstime.tv_nsec >= 1000 * 1000 * 1000)
    {
      abstime.tv_sec++;
      abstime.tv_nsec -= 1000 * 1000 * 1000;
    }

  return nxsem_timedwait(sem, &abstime);
}

/****************************************************************************
 * Name: efuse_lock
 *
 * Description:
 *   Lock and configure the efuse interface.
 *
 ****************************************************************************/

static inline int efuse_lock(FAR struct efuse_dev_s *efuse)
{
  return nxmutex_lock(&efuse->mutex);
}

/****************************************************************************
 * Name: efuse_unlock
 *
 * Description:
 *   Unlock the efuse interface.
 *
 ****************************************************************************/

static inline int efuse_unlock(FAR struct efuse_dev_s *efuse)
{
  return nxmutex_unlock(&efuse->mutex);
}

/****************************************************************************
 * Name: efuse_read_register
 *
 * Description:
 *   Read register data specified by addr and cnt
 *
 ****************************************************************************/

static int efuse_read_register(FAR struct efuse_dev_s *priv, uint32_t addr,
                               uint8_t *data, uint32_t cnt)
{
  int ret;
  uint8_t i;

  ret = efuse_lock(priv);
  if (ret < 0)
    {
      ferr("Error: Failed to lock\n");
      return ret;
    }

  /* Clear efuse interrupt register */

  putreg32(EFUSE_INTR_EFUSE_STS, priv->base + EFUSE_STS_OFFSET);

  /* Chip is busy or idle? */

  if (getreg32(priv->base + EFUSE_CTRL_OFFSET) & EFUSE_CTRL_BUSY)
    {
      finfo("Efuse device is busy\n");
      return -EBUSY;
    }

  /* Set read mode */

  modreg32(EFUSE_MODE_READ, EFUSE_MODE_MASK, priv->base + EFUSE_MODE_OFFSET);
  putreg32(addr * 8, priv->base + EFUSE_ADDR_OFFSET);
  putreg32(cnt * 8, priv->base + EFUSE_LENGTH_OFFSET);

  /* Enable efuse interrupt register */

  modreg32(EFUSE_INTR_EFUSE_EN, EFUSE_MODE_MASK << 1, priv->base + EFUSE_EN_OFFSET);

  /* Enable efuse ctrl register */

  modreg32(EFUSE_CTRL_ENABLE, EFUSE_MODE_MASK, priv->base + EFUSE_CTRL_OFFSET);
  efuse_sem_wait(&priv->sem, EFUSE_SEM_WAIT_10MS);

  /* Disable efuse ctrl register */

  modreg32(EFUSE_CTRL_DISABLE, EFUSE_MODE_MASK << 1, priv->base + EFUSE_EN_OFFSET);

  for (i = 0; i < cnt; i++)
    data[i] = getreg32(priv->base + EFUSE_DATA_OFFSET + addr + i);

  ret = efuse_unlock(priv);
  if (ret < 0)
    {
      ferr("Error: Failed to unlock\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: efuse_programbit
 *
 * Description:
 *   Write register data one bit
 *
 ****************************************************************************/

static int32_t efuse_programbit(FAR struct efuse_dev_s *priv, uint32_t bit)
{

  /* Clear efuse interrupt register */

  putreg32(EFUSE_INTR_EFUSE_STS, priv->base + EFUSE_STS_OFFSET);

  if (getreg32(priv->base + EFUSE_CTRL_OFFSET) & EFUSE_CTRL_BUSY)
    {
      finfo("Efuse device is busy\n");
      return -EBUSY;
    }

  /* Set efuse ctrl disable */

  modreg32(EFUSE_CTRL_DISABLE, EFUSE_MODE_MASK, priv->base + EFUSE_MODE_OFFSET);

  /* Set efuse grogram function */

  modreg32(EFUSE_MODE_PROG, EFUSE_MODE_MASK, priv->base + EFUSE_MODE_OFFSET);
  putreg32(bit, priv->base + EFUSE_ADDR_OFFSET);
  putreg32(1, priv->base + EFUSE_LENGTH_OFFSET);

  /* Enable efuse interrupt register */

  modreg32(EFUSE_INTR_EFUSE_EN, EFUSE_MODE_MASK << 1, priv->base + EFUSE_EN_OFFSET);

  /* Enable efuse ctrl register */

  modreg32(EFUSE_CTRL_ENABLE, EFUSE_MODE_MASK, priv->base + EFUSE_CTRL_OFFSET);

  efuse_sem_wait(&priv->sem, EFUSE_SEM_WAIT_10MS);

  /* Disable efuse ctrl register */

  modreg32(EFUSE_CTRL_DISABLE, EFUSE_MODE_MASK << 1, priv->base + EFUSE_EN_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: efuse_programdata
 *
 * Description:
 *   This function is called whenever the data is writen by one bit.
 *
 ****************************************************************************/

static int32_t efuse_programdata(FAR struct efuse_dev_s *priv, uint32_t addr,
                                 const uint8_t *data, uint32_t cnt)
{
  uint8_t i;
  uint8_t j;
  uint8_t ret;

  ret = efuse_lock(priv);
  if (ret < 0)
    {
      ferr("Error: Failed to lock\n");
      return ret;
    }

  for (i = 0; i < cnt; i++)
    {
      for (j = 0; j < 8; j++)
        {
          if (data[i] & (1 << j))
            efuse_programbit(priv, addr * 8 + i * 8 + j);
        }
    }

  ret = efuse_unlock(priv);
  if (ret < 0)
    {
      ferr("Error: Failed to unlock\n");
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: efuse_open
 *
 * Description:
 *   This function is called whenever the efuse device is opened.
 *
 ****************************************************************************/

static int efuse_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: efuse_close
 *
 * Description:
 *   This routine is called when the efuse device is closed.
 *
 ****************************************************************************/

static int efuse_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: efuse_seek_
 *
 * Description:
 *   This routine is called when the efuse device set read offset.
 *
 ****************************************************************************/

static off_t efuse_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  off_t newpos;
  off_t ret = 0;

  DEBUGASSERT(inode && inode->i_private);

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = EFUSE_READ_MAX_NUM;
      break;

    default:

      /* Return EINVAL if the whence argument is invalid */

      return -EINVAL;
    }

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
      finfo("SEEK new pos :%d\n", newpos);
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: efuse_ioctl
 *
 * Description:
 *   This routine is called when the efuse device choose read efuse register.
 *
 * Input Parameters:
 *    cmd -  2 means write
 *    arg -  32-24bits mean address, 24-16bits mean length, 16-8bits mean data1, 8-0bits mean data0
 *
 * Returned Value:
 *   Zero () on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int efuse_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct efuse_dev_s *priv = inode->i_private;
  FAR union efuse_ioctl_parameter parameter;
  uint8_t i;
  uint8_t ret = 0;
  uint8_t data;

  DEBUGASSERT(priv != NULL);

  parameter.all = arg;

  if (cmd == EFUSE_FUNC_WRITE)
    {
      if (parameter.length > 2)
        {
          finfo("Failed to write, length is more than 2 bytes\n");
          return -EINVAL;
        }

      for (i = 0; i < parameter.length; i++)
        {
          data = (parameter.data >> (i * 8)) & 0xff;
          ret = efuse_programdata(priv, parameter.address + i, &data, 1);
        }
    }
  else
    {
      finfo("Failed invalid parameter\n");
      return -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: efuse_read
 ****************************************************************************/

static ssize_t efuse_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct efuse_dev_s *priv = inode->i_private;
  int ret;

  /* Check for issues */

  DEBUGASSERT(priv != NULL && buffer != NULL);

  if (filep->f_pos + buflen > EFUSE_READ_MAX_NUM)
    {
      buflen = EFUSE_READ_MAX_NUM - filep->f_pos;
    }

  if (buflen == 0)
    {
      goto done;
    }

  ret = efuse_read_register(priv, filep->f_pos, buffer, buflen);
  if (ret == -EBUSY)
    {
      return -EAGAIN;
    }

  /* update the file position */

  filep->f_pos += buflen;

done:
  return buflen;
}

/****************************************************************************
 * Name: efuse_write
 ****************************************************************************/

static ssize_t efuse_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: song_efuse_initialize
 *
 * Description:
 *   This function will register the efuse driver as /dev/efuse
 *   where N is the minor device number
 *
 * Input Parameters:
 *   minor- number of the /dev/efuse
 *   base - the register base
 *   irq  - the number of irq
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int song_efuse_initialize(int minor, uint32_t base, uint8_t irq)
{
  FAR struct efuse_dev_s *priv;
  int ret;
  char devname[32];

  /* Sanity check */

  DEBUGASSERT(base != NULL);

  sprintf(devname, "/dev/efuse%d", minor);

  /* Initialize the efuse device structure */

  priv = (FAR struct efuse_dev_s *)kmm_zalloc(sizeof(struct efuse_dev_s));
  if (priv == NULL)
    {
      ferr("Error: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->base = base;

  ret = nxmutex_init(&priv->mutex);
  if (ret)
    {
      kmm_free(priv);
      ferr("Error: Failed to init mutex: %d\n", ret);
      return ret;
    }

  ret = nxsem_init(&priv->sem, 0, 0);
  if (ret)
    {
      kmm_free(priv);
      ferr("Error: Failed to init sem: %d\n", ret);
      return ret;
    }

  /* Attach efuse interrupt */

  irq_attach(irq, up_efuse_isr, priv);
  up_enable_irq(irq);

  /* Register the character driver */

  ret = register_driver(devname, &g_efusefops, 0666, priv);
  if (ret < 0)
    {
      ferr("Error: Failed to register efuse driver: %d\n", ret);
      up_disable_irq(irq);
      irq_detach(irq);
      nxsem_destroy(&priv->sem);
      nxmutex_destroy(&priv->mutex);
      kmm_free(priv);
    }

  return ret;
}

int song_efuse_read_info(int base, uint32_t addr, uint8_t *buffer, uint32_t cnt)
{
  uint8_t i;

  while (getreg32(base + EFUSE_CTRL_OFFSET) & EFUSE_CTRL_BUSY);

  /* Set read mode */

  modreg32(EFUSE_MODE_READ, EFUSE_MODE_MASK, base + EFUSE_MODE_OFFSET);
  putreg32(addr * 8, base + EFUSE_ADDR_OFFSET);
  putreg32(cnt * 8, base + EFUSE_LENGTH_OFFSET);

  /* Enable efuse ctrl register */

  modreg32(EFUSE_CTRL_ENABLE, EFUSE_MODE_MASK, base + EFUSE_CTRL_OFFSET);
  while ((getreg32(base + EFUSE_RAW_OFFSET) & EFUSE_INTR_EFUSE_ST) == 0);
  putreg32(EFUSE_INTR_EFUSE_ST, base + EFUSE_RAW_OFFSET);

  for (i = 0; i < cnt; i++)
    buffer[i] = getreg32(base + EFUSE_DATA_OFFSET + addr + i);

  return 0;
}
#endif /* CONFIG_EFUSE */
