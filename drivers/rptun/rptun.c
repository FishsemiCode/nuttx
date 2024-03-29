/****************************************************************************
 * drivers/rptun/rptun.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdio.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/signal.h>
#include <metal/utilities.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b)              ((a) > (b) ? (a) : (b))
#endif

#ifndef ALIGN_UP
#  define ALIGN_UP(s, a)        (((s) + (a) - 1) & ~((a) - 1))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_priv_s
{
  FAR struct rptun_dev_s       *dev;
  struct remoteproc            rproc;
  int                          vdev_num;
  struct rpmsg_virtio_device   *vdev[CONFIG_RPTUN_VDEV_NUM];
  struct rpmsg_virtio_shm_pool *shm_pool[CONFIG_RPTUN_VDEV_NUM];
  struct metal_list            bind;
  struct metal_list            node;
#ifdef CONFIG_RPTUN_USE_HPWORK
  struct work_s                work;
#else
  int                          pid;
#endif
};

struct rptun_bind_s
{
  char                      name[RPMSG_NAME_SIZE];
  uint32_t                  dest;
  struct metal_list         node;
  FAR struct rpmsg_device   *rdev;
};

struct rptun_cb_s
{
  FAR void          *priv;
  uint32_t          buf_size;
  rpmsg_dev_cb_t    device_created;
  rpmsg_dev_cb_t    device_destroy;
  rpmsg_bind_cb_t   ns_bind;
  struct metal_list node;
};

struct rptun_store_s
{
  struct file file;
  FAR char   *buf;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct remoteproc *rptun_init(FAR struct remoteproc *rproc,
                                         FAR struct remoteproc_ops *ops,
                                         FAR void *arg);
static void rptun_remove(FAR struct remoteproc *rproc);
static int rptun_mmap(FAR struct remoteproc *rproc,
                      FAR metal_phys_addr_t *pa, FAR metal_phys_addr_t *da,
                      FAR void **va, size_t size, unsigned int attribute,
                      FAR struct metal_io_region **io_);
static int rptun_config(FAR struct remoteproc *rproc, FAR void *data);
static int rptun_start(FAR struct remoteproc *rproc);
static int rptun_stop(FAR struct remoteproc *rproc);
static int rptun_notify(FAR struct remoteproc *rproc, uint32_t id);

static void rptun_ns_bind(FAR struct rpmsg_device *rdev,
                          FAR const char *name, uint32_t dest);

static int rptun_dev_start(FAR struct remoteproc *rproc);
static int rptun_dev_stop(FAR struct remoteproc *rproc);
static int rptun_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

static int rptun_store_open(FAR void *store_, FAR const char *path,
                            FAR const void **img_data);
static void rptun_store_close(FAR void *store_);
static int rptun_store_load(FAR void *store_, size_t offset,
                            size_t size, FAR const void **data,
                            metal_phys_addr_t pa,
                            FAR struct metal_io_region *io,
                            char is_blocking);

static int rptun_get_vdev_num(FAR struct rptun_rsc_s *rsc);
static int rptun_get_vdev_index(FAR void *priv_, uint32_t buf_size);
static metal_phys_addr_t rptun_pa_to_da(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t pa);
static metal_phys_addr_t rptun_da_to_pa(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t da);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct remoteproc_ops g_rptun_ops =
{
  .init   = rptun_init,
  .remove = rptun_remove,
  .mmap   = rptun_mmap,
  .config = rptun_config,
  .start  = rptun_start,
  .stop   = rptun_stop,
  .notify = rptun_notify,
};

static const struct file_operations g_rptun_devops =
{
  .ioctl = rptun_dev_ioctl,
};

static struct image_store_ops g_rptun_storeops =
{
  .open     = rptun_store_open,
  .close    = rptun_store_close,
  .load     = rptun_store_load,
  .features = SUPPORT_SEEK,
};

static sem_t g_rptun_sem = SEM_INITIALIZER(1);

static METAL_DECLARE_LIST(g_rptun_cb);
static METAL_DECLARE_LIST(g_rptun_priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RPTUN_USE_HPWORK
static void rptun_worker(FAR void *arg)
{
  FAR struct rptun_priv_s *priv = arg;

  remoteproc_get_notification(&priv->rproc, RPTUN_NOTIFY_ALL);
}

static int rptun_callback(FAR void *arg, uint32_t vqid)
{
  FAR struct rptun_priv_s *priv = arg;

  return work_queue(HPWORK, &priv->work, rptun_worker, priv, 0);
}
#else
static int rptun_thread(int argc, FAR char *argv[])
{
  FAR struct rptun_priv_s *priv;
  sigset_t set;
  int ret;

  priv = (FAR struct rptun_priv_s *)((uintptr_t)strtoul(argv[1], NULL, 0));

  sigemptyset(&set);
  sigaddset(&set, SIGUSR1);
  nxsig_procmask(SIG_BLOCK, &set, NULL);

  while (1)
    {
      ret = nxsig_timedwait(&set, NULL, NULL);
      if (ret == SIGUSR1)
        {
          remoteproc_get_notification(&priv->rproc, RPTUN_NOTIFY_ALL);
        }
    }

  return 0;
}

static int rptun_callback(FAR void *arg, uint32_t vqid)
{
  FAR struct rptun_priv_s *priv = arg;

  return nxsig_kill(priv->pid, SIGUSR1);
}
#endif

static FAR struct remoteproc *rptun_init(FAR struct remoteproc *rproc,
                                         FAR struct remoteproc_ops *ops,
                                         FAR void *arg)
{
  rproc->ops = ops;
  rproc->priv = arg;

  return rproc;
}

static void rptun_remove(FAR struct remoteproc *rproc)
{
  rproc->priv = NULL;
}

static int rptun_mmap(FAR struct remoteproc *rproc,
                      FAR metal_phys_addr_t *pa, FAR metal_phys_addr_t *da,
                      FAR void **va, size_t size, unsigned int attribute,
                      FAR struct metal_io_region **io_)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct metal_io_region *io = metal_io_get_region();

  if (*pa != METAL_BAD_PHYS)
    {
      *da = rptun_pa_to_da(priv->dev, *pa);
      *va = metal_io_phys_to_virt(io, *pa);
      if (!*va)
        {
          return -RPROC_EINVAL;
        }
    }
  else if (*da != METAL_BAD_PHYS)
    {
      *pa = rptun_da_to_pa(priv->dev, *da);
      *va = metal_io_phys_to_virt(io, *pa);
      if (!*va)
        {
          return -RPROC_EINVAL;
        }
    }
  else if (*va)
    {
      *pa = metal_io_virt_to_phys(io, *va);
      if (*pa == METAL_BAD_PHYS)
        {
          return -RPROC_EINVAL;
        }

      *da = rptun_pa_to_da(priv->dev, *pa);
    }
  else
    {
      return -RPROC_EINVAL;
    }

  if (io_)
    {
      *io_ = io;
    }

  return 0;
}

static int rptun_config(FAR struct remoteproc *rproc, FAR void *data)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      return RPTUN_CONFIG(priv->dev, data);
    }

  return 0;
}

static int rptun_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      return RPTUN_START(priv->dev);
    }

  return 0;
}

static int rptun_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  if (RPTUN_IS_MASTER(priv->dev))
    {
      return RPTUN_STOP(priv->dev);
    }

  return 0;
}

static int rptun_notify(FAR struct remoteproc *rproc, uint32_t id)
{
  FAR struct rptun_priv_s *priv = rproc->priv;

  RPTUN_NOTIFY(priv->dev, RPTUN_NOTIFY_ALL);

  return 0;
}

static void *rptun_get_priv_by_rdev(FAR struct rpmsg_device *rdev)
{
  struct rpmsg_virtio_device *rvdev;
  struct virtio_device *vdev;
  struct remoteproc_virtio *rpvdev;
  struct remoteproc *rproc;

  rvdev = metal_container_of(rdev, struct rpmsg_virtio_device, rdev);
  vdev  = rvdev->vdev;
  if (!vdev)
    {
      return NULL;
    }

  rpvdev = metal_container_of(vdev, struct remoteproc_virtio, vdev);
  rproc  = rpvdev->priv;
  if (!rproc)
    {
      return NULL;
    }

  return rproc->priv;
}

static void rptun_ns_bind(FAR struct rpmsg_device *rdev,
                          FAR const char *name, uint32_t dest)
{
  FAR struct rptun_priv_s *priv = rptun_get_priv_by_rdev(rdev);
  FAR struct rptun_bind_s *bind;

  bind = kmm_malloc(sizeof(struct rptun_bind_s));
  if (bind)
    {
      FAR struct metal_list *node;
      FAR struct rptun_cb_s *cb;

      bind->dest = dest;
      strncpy(bind->name, name, RPMSG_NAME_SIZE);
      bind->rdev = rdev;

      nxsem_wait(&g_rptun_sem);

      metal_list_add_tail(&priv->bind, &bind->node);

      metal_list_for_each(&g_rptun_cb, node)
        {
          cb = metal_container_of(node, struct rptun_cb_s, node);
          if (cb->ns_bind)
            {
              cb->ns_bind(rdev, cb->priv, name, dest);
            }
        }

      nxsem_post(&g_rptun_sem);
    }
}

static int rptun_dev_start(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct virtio_device *vdev;
  FAR struct rptun_rsc_s *rsc;
  FAR struct metal_list *node;
  FAR struct rptun_cb_s *cb;
  unsigned int role = RPMSG_REMOTE;
  int ret;
  int i;

  ret = remoteproc_config(&priv->rproc, NULL);
  if (ret)
    {
      return ret;
    }

  if (RPTUN_GET_FIRMWARE(priv->dev))
    {
      struct rptun_store_s store =
      {
        0
      };

      ret = remoteproc_load(rproc, RPTUN_GET_FIRMWARE(priv->dev),
                            &store, &g_rptun_storeops, NULL);
      if (ret)
        {
          return ret;
        }

      rsc = rproc->rsc_table;

      if (rsc)
        priv->vdev_num = 1;
      else
        priv->vdev_num = 0;
    }
  else
    {
      rsc = RPTUN_GET_RESOURCE(priv->dev);
      if (!rsc)
        {
          return -EINVAL;
        }

      priv->vdev_num = rptun_get_vdev_num(rsc);

      if (!priv->vdev_num || priv->vdev_num > CONFIG_RPTUN_VDEV_NUM)
        {
          return -EINVAL;
        }

      ret = remoteproc_set_rsc_table(rproc, (struct resource_table *)rsc,
                                     (priv->vdev_num - 1) * sizeof(struct rptun_rsc_vdev_s)
                                     + sizeof(struct rptun_rsc_s));
      if (ret)
        {
          return ret;
        }
    }

  if (priv->vdev_num)
    {
      priv->vdev[0] = kmm_zalloc(priv->vdev_num * (sizeof(struct rpmsg_virtio_device)
                                 + sizeof(struct rpmsg_virtio_shm_pool)));
      if (priv->vdev[0] == NULL)
        {
          return -ENOMEM;
        }

      for (i = 0; i < priv->vdev_num - 1; i++)
        {
          priv->shm_pool[i] = (struct rpmsg_virtio_shm_pool *)(priv->vdev[i] + 1);
          priv->vdev[i + 1] = (struct rpmsg_virtio_device *)(priv->shm_pool[i] + 1);
        }

      priv->shm_pool[i] = (struct rpmsg_virtio_shm_pool *)(priv->vdev[i] + 1);
    }

  /* Update resource table on MASTER side */

  if (RPTUN_IS_MASTER(priv->dev) && rsc)
    {
      int j;
      uint32_t tbsz;
      uint32_t v0sz;
      uint32_t v1sz;
      uint32_t shbufsz;
      metal_phys_addr_t da0;
      metal_phys_addr_t da1;
      uint32_t align0;
      uint32_t align1;
      FAR void *va0;
      FAR void *va1;
      FAR void *shbuf;
      FAR char *start;
      FAR struct rptun_rsc_vdev_s *rsc_vdev;

      role = RPMSG_MASTER;

      tbsz = (priv->vdev_num - 1) * sizeof(struct rptun_rsc_vdev_s)
             + sizeof(struct rptun_rsc_s);

      for (i = 0, j = 0; i < rsc->rsc_tbl_hdr.num; i++)
        {
          start = (char *)rsc;
          start += B2C(rsc->offset[i]);

          if (*((uint32_t *)start) != RSC_VDEV)
            {
              continue;
            }

          rsc_vdev = (struct rptun_rsc_vdev_s *)start;

          align0 = B2C(rsc_vdev->rpmsg_vring0.align);
          align1 = B2C(rsc_vdev->rpmsg_vring1.align);

          tbsz = ALIGN_UP(tbsz, MAX(align0, align1));
          v0sz = ALIGN_UP(vring_size(rsc_vdev->rpmsg_vring0.num, align0), align0);
          v1sz = ALIGN_UP(vring_size(rsc_vdev->rpmsg_vring1.num, align1), align1);

          va0 = (char *)rsc + tbsz;
          va1 = (char *)rsc + tbsz + v0sz;

          da0 = da1 = METAL_BAD_PHYS;

          remoteproc_mmap(rproc, NULL, &da0, &va0, v0sz, 0, NULL);
          remoteproc_mmap(rproc, NULL, &da1, &va1, v1sz, 0, NULL);

          rsc_vdev->rpmsg_vring0.da = da0;
          rsc_vdev->rpmsg_vring1.da = da1;

          shbuf   = (FAR char *)rsc + tbsz + v0sz + v1sz;
          shbufsz = rsc_vdev->buf_size *
                    (rsc_vdev->rpmsg_vring0.num + rsc_vdev->rpmsg_vring1.num);

          rpmsg_virtio_init_shm_pool(priv->shm_pool[j++], shbuf, shbufsz);

          tbsz += (v0sz + v1sz + shbufsz);
        }
    }

  /* Remote proc create */

  for (i = 0; i < priv->vdev_num; i++)
    {
      vdev = remoteproc_create_virtio(rproc, i, role, NULL);
      if (!vdev)
        {
          ret = -ENOMEM;
          goto clean;
        }

      ret = rpmsg_init_vdev(priv->vdev[i], vdev, rptun_ns_bind,
                            metal_io_get_region(), priv->shm_pool[i]);
      if (ret)
        {
          goto clean;
        }
    }

  /* Remote proc start */

  ret = remoteproc_start(rproc);
  if (ret)
    {
      goto clean;
    }

  nxsem_wait(&g_rptun_sem);

  /* Add priv to list */

  metal_list_add_tail(&g_rptun_priv, &priv->node);

  /* Broadcast device_created to all registers */

  if (priv->vdev_num > 0)
    {
      metal_list_for_each(&g_rptun_cb, node)
        {
          cb = metal_container_of(node, struct rptun_cb_s, node);
          if (cb->device_created)
            {
              i = rptun_get_vdev_index(priv, cb->buf_size);
              cb->device_created(&priv->vdev[i]->rdev, cb->priv);
            }
        }
    }

  nxsem_post(&g_rptun_sem);

  /* Register callback to mbox for receiving remote message */

  RPTUN_REGISTER_CALLBACK(priv->dev, rptun_callback, priv);

  return 0;

clean:
  for (i = 0; i < priv->vdev_num; i++)
    {
      if (priv->vdev[i]->vdev)
        {
          remoteproc_remove_virtio(rproc, priv->vdev[i]->vdev);
        }
    }

    kmm_free(priv->vdev[0]);

    return ret;
}

static int rptun_dev_stop(FAR struct remoteproc *rproc)
{
  FAR struct rptun_priv_s *priv = rproc->priv;
  FAR struct metal_list *node;
  FAR struct rptun_cb_s *cb;
  int i;

  /* Unregister callback from mbox */

  RPTUN_UNREGISTER_CALLBACK(priv->dev);

  nxsem_wait(&g_rptun_sem);

  /* Remove priv from list */

  metal_list_del(&priv->node);

  /* Broadcast device_destroy to all registers */

  metal_list_for_each(&g_rptun_cb, node)
    {
      cb = metal_container_of(node, struct rptun_cb_s, node);
      if (cb->device_destroy)
        {
          i = rptun_get_vdev_index(priv, cb->buf_size);
          cb->device_destroy(&priv->vdev[i]->rdev, cb->priv);
        }
    }

  nxsem_post(&g_rptun_sem);

  /* Remote proc stop and shutdown */

  remoteproc_shutdown(rproc);

  /* Remote proc remove */

  for (i = 0; i < priv->vdev_num; i++)
    {
      remoteproc_remove_virtio(rproc, priv->vdev[i]->vdev);
      rpmsg_deinit_vdev(priv->vdev[i]);
    }

  /* Free bind list */

  metal_list_for_each(&priv->bind, node)
    {
      struct rptun_bind_s *bind;

      bind = metal_container_of(node, struct rptun_bind_s, node);
      kmm_free(bind);
    }

  kmm_free(priv->vdev[0]);

  return 0;
}

static int rptun_dev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rptun_priv_s *priv = inode->i_private;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case RPTUNIOC_START:
        if (priv->rproc.state == RPROC_OFFLINE)
          {
            ret = rptun_dev_start(&priv->rproc);
          }
        break;

      case RPTUNIOC_STOP:
        if (priv->rproc.state != RPROC_OFFLINE)
          {
            ret = rptun_dev_stop(&priv->rproc);
          }
        break;
    }

  return ret;
}

static int rptun_store_open(FAR void *store_, FAR const char *path,
                            FAR const void **img_data)
{
  FAR struct rptun_store_s *store = store_;
  int len = 0x100;
  int ret;

  ret = file_open(&store->file, path, O_RDONLY);
  if (ret < 0)
    {
      return ret;
    }

  store->buf = kmm_malloc(len);
  if (!store->buf)
    {
      file_close(&store->file);
      return -ENOMEM;
    }

  *img_data = store->buf;

  return file_read(&store->file, store->buf, len);
}

static void rptun_store_close(FAR void *store_)
{
  FAR struct rptun_store_s *store = store_;

  kmm_free(store->buf);
  file_close(&store->file);
}

static int rptun_store_load(FAR void *store_, size_t offset,
                            size_t size, FAR const void **data,
                            metal_phys_addr_t pa,
                            FAR struct metal_io_region *io,
                            char is_blocking)
{
  FAR struct rptun_store_s *store = store_;
  FAR char *tmp;
  FAR char *bounce = NULL;
  ssize_t ret;

  if (pa == METAL_BAD_PHYS)
    {
      tmp = kmm_realloc(store->buf, size);
      if (!tmp)
        {
          return -ENOMEM;
        }

      store->buf = tmp;
      *data = tmp;
    }
  else
    {
      if ((size % sizeof(unsigned int)) == 0)
        {
          tmp = kmm_zalloc(size);
          if (!tmp)
            {
              return -ENOMEM;
            }

          bounce = metal_io_phys_to_virt(io, pa);
          if (!bounce)
            {
              kmm_free(tmp);
              return -EINVAL;
            }
        }
      else
        {
          tmp = metal_io_phys_to_virt(io, pa);
          if (!tmp)
            {
              return -EINVAL;
            }
        }
    }

  file_seek(&store->file, offset, SEEK_SET);
  ret = file_read(&store->file, tmp, size);
  if (bounce != NULL)
    {
      int i = 0;
      unsigned int *src = (unsigned int *)tmp;
      unsigned int *dst = (unsigned int *)bounce;

      for (i = 0; i < (size / sizeof(unsigned int)); i++)
        {
          *dst++ = *src++;
        }
      kmm_free(tmp);
    }

  return ret;
}

static int rptun_get_vdev_num(FAR struct rptun_rsc_s *rsc)
{
  uint32_t type;
  uint32_t i;
  char *start;
  int num = 0;

  for (i = 0; i < rsc->rsc_tbl_hdr.num; i++)
    {
      start = (char *)rsc;
      start += B2C(rsc->offset[i]);
      type = *((uint32_t *)start);

      if (type == RSC_VDEV)
        {
          num++;
        }
    }

  return num;
}

static int rptun_get_vdev_index(FAR void *priv_, uint32_t buf_size)
{
  FAR struct rptun_priv_s *priv = priv_;
  int i;

  for (i = 0; i < priv->vdev_num; i++)
    {
      if (priv->vdev[i]->shbuf_size >= buf_size)
        {
          return i;
        }
    }

  return priv->vdev_num - 1;
}

static metal_phys_addr_t rptun_pa_to_da(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t pa)
{
  FAR const struct rptun_addrenv_s *addrenv;
  uint32_t i;

  addrenv = RPTUN_GET_ADDRENV(dev);
  if (!addrenv)
    {
      return pa;
    }

  for (i = 0; addrenv[i].size; i++)
    {
      if (pa - addrenv[i].pa < addrenv[i].size)
        {
          return addrenv[i].da + (pa - addrenv[i].pa);
        }
    }

  return pa;
}

static metal_phys_addr_t rptun_da_to_pa(FAR struct rptun_dev_s *dev,
                                        metal_phys_addr_t da)
{
  FAR const struct rptun_addrenv_s *addrenv;
  uint32_t i;

  addrenv = RPTUN_GET_ADDRENV(dev);
  if (!addrenv)
    {
      return da;
    }

  for (i = 0; addrenv[i].size; i++)
    {
      if (da - addrenv[i].da < addrenv[i].size)
        {
          return addrenv[i].pa + (da - addrenv[i].da);
        }
    }

  return da;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR const char *rpmsg_get_cpuname(FAR struct rpmsg_device *rdev)
{
  FAR struct rptun_priv_s *priv = rptun_get_priv_by_rdev(rdev);

  return RPTUN_GET_CPUNAME(priv->dev);
}

int rpmsg_register_callback(FAR void *priv_,
                            rpmsg_dev_cb_t device_created,
                            rpmsg_dev_cb_t device_destroy,
                            rpmsg_bind_cb_t ns_bind)
{
  return rpmsg_register_channel_callback(priv_, 0,
                            device_created, device_destroy, ns_bind);
}

int rpmsg_register_channel_callback(FAR void *priv_,
                                    uint32_t chan_size,
                                    rpmsg_dev_cb_t device_created,
                                    rpmsg_dev_cb_t device_destroy,
                                    rpmsg_bind_cb_t ns_bind)
{
  FAR struct metal_list *node;
  FAR struct metal_list *bnode;
  FAR struct rptun_cb_s *cb;

  cb = kmm_zalloc(sizeof(struct rptun_cb_s));
  if (!cb)
    {
      return -ENOMEM;
    }

  cb->priv           = priv_;
  cb->buf_size       = chan_size;
  cb->device_created = device_created;
  cb->device_destroy = device_destroy;
  cb->ns_bind        = ns_bind;

  nxsem_wait(&g_rptun_sem);

  metal_list_add_tail(&g_rptun_cb, &cb->node);

  metal_list_for_each(&g_rptun_priv, node)
    {
      struct rptun_priv_s *priv;
      int i;

      priv = metal_container_of(node, struct rptun_priv_s, node);
      if (device_created)
        {
          i = rptun_get_vdev_index(priv, chan_size);
          device_created(&priv->vdev[i]->rdev, priv_);
        }

      if (ns_bind)
        {
          metal_list_for_each(&priv->bind, bnode)
            {
              struct rptun_bind_s *bind;

              bind = metal_container_of(bnode, struct rptun_bind_s, node);
              ns_bind(bind->rdev, priv_, bind->name, bind->dest);
            }
        }
    }

  nxsem_post(&g_rptun_sem);

  return 0;
}

void rpmsg_unregister_callback(FAR void *priv_,
                               rpmsg_dev_cb_t device_created,
                               rpmsg_dev_cb_t device_destroy,
                               rpmsg_bind_cb_t ns_bind)
{
  FAR struct metal_list *node;
  FAR struct metal_list *pnode;

  nxsem_wait(&g_rptun_sem);

  metal_list_for_each(&g_rptun_cb, node)
    {
      struct rptun_cb_s *cb = NULL;

      cb = metal_container_of(node, struct rptun_cb_s, node);
      if (cb->priv == priv_ &&
          cb->device_created == device_created &&
          cb->device_destroy == device_destroy &&
          cb->ns_bind == ns_bind)
        {
          if (device_destroy)
            {
              metal_list_for_each(&g_rptun_priv, pnode)
                {
                  struct rptun_priv_s *priv;
                  int i;

                  priv = metal_container_of(pnode, struct rptun_priv_s, node);
                  i = rptun_get_vdev_index(priv, cb->buf_size);
                  device_destroy(&priv->vdev[i]->rdev, priv_);
                }
            }

          metal_list_del(&cb->node);
          kmm_free(cb);
        }
    }

  nxsem_post(&g_rptun_sem);
}

int rptun_initialize(FAR struct rptun_dev_s *dev)
{
  struct metal_init_params params = METAL_INIT_DEFAULTS;
  FAR struct rptun_priv_s *priv;
#ifndef CONFIG_RPTUN_USE_HPWORK
  FAR char *argv[2];
  char arg0[16];
#endif
  char name[32];
  int ret;

  ret = metal_init(&params);
  if (ret)
    {
      return ret;
    }

  priv = kmm_zalloc(sizeof(struct rptun_priv_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }


#ifndef CONFIG_RPTUN_USE_HPWORK
  snprintf(name, 32, "rptun%s", RPTUN_GET_CPUNAME(dev));
  snprintf(arg0, 16, "0x%" PRIxPTR, (uintptr_t)priv);

  argv[0] = arg0;
  argv[1] = NULL;

  ret = kthread_create(name,
                       CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE,
                       rptun_thread,
                       argv);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  priv->pid = ret;
#endif

  priv->dev = dev;

  metal_list_init(&priv->bind);
  remoteproc_init(&priv->rproc, &g_rptun_ops, priv);

  if (RPTUN_IS_AUTOSTART(dev))
    {
      rptun_dev_start(&priv->rproc);
    }

  snprintf(name, 32, "/dev/rptun/%s", RPTUN_GET_CPUNAME(dev));
  return register_driver(name, &g_rptun_devops, 0666, priv);
}

int rptun_boot(FAR const char *cpuname)
{
  struct file file;
  char name[32];
  int ret;

  if (!cpuname)
    {
      return -EINVAL;
    }

  snprintf(name, 32, "/dev/rptun/%s", cpuname);
  ret = file_open(&file, name, 0, 0);
  if (ret)
    {
      return ret;
    }

  ret = file_ioctl(&file, RPTUNIOC_START, 0);
  file_close(&file);

  return ret;
}
