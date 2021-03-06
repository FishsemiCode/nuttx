/****************************************************************************
 * fs/hostfs/hostfs_rpmsg_server.c
 * Hostfs rpmsg driver on server
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

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/rptun/openamp.h>

#include <sys/mount.h>

#include "hostfs_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hostfs_rpmsg_server_s
{
  struct rpmsg_endpoint                ept;
  struct file                          files[CONFIG_NFILE_DESCRIPTORS];
  void                                 *dirs[CONFIG_NFILE_DESCRIPTORS];
  sem_t                                sem;
#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
  const struct hostfs_server_config_s *cfg;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hostfs_rpmsg_open_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_close_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_write_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_lseek_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_sync_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_dup_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_fstat_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_ftruncate_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_opendir_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_rewinddir_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_closedir_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv_);
static int hostfs_rpmsg_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int hostfs_rpmsg_unlink_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int hostfs_rpmsg_mkdir_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int hostfs_rpmsg_rmdir_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int hostfs_rpmsg_rename_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int hostfs_rpmsg_stat_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_);

static void hostfs_rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv_, FAR const char *name,
                                 uint32_t dest);
static void hostfs_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept);
static int  hostfs_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len, uint32_t src,
                                FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_hostfs_rpmsg_handler[] =
{
  [HOSTFS_RPMSG_OPEN]      = hostfs_rpmsg_open_handler,
  [HOSTFS_RPMSG_CLOSE]     = hostfs_rpmsg_close_handler,
  [HOSTFS_RPMSG_READ]      = hostfs_rpmsg_read_handler,
  [HOSTFS_RPMSG_WRITE]     = hostfs_rpmsg_write_handler,
  [HOSTFS_RPMSG_LSEEK]     = hostfs_rpmsg_lseek_handler,
  [HOSTFS_RPMSG_IOCTL]     = hostfs_rpmsg_ioctl_handler,
  [HOSTFS_RPMSG_SYNC]      = hostfs_rpmsg_sync_handler,
  [HOSTFS_RPMSG_DUP]       = hostfs_rpmsg_dup_handler,
  [HOSTFS_RPMSG_FSTAT]     = hostfs_rpmsg_fstat_handler,
  [HOSTFS_RPMSG_FTRUNCATE] = hostfs_rpmsg_ftruncate_handler,
  [HOSTFS_RPMSG_OPENDIR]   = hostfs_rpmsg_opendir_handler,
  [HOSTFS_RPMSG_READDIR]   = hostfs_rpmsg_readdir_handler,
  [HOSTFS_RPMSG_REWINDDIR] = hostfs_rpmsg_rewinddir_handler,
  [HOSTFS_RPMSG_CLOSEDIR]  = hostfs_rpmsg_closedir_handler,
  [HOSTFS_RPMSG_STATFS]    = hostfs_rpmsg_statfs_handler,
  [HOSTFS_RPMSG_UNLINK]    = hostfs_rpmsg_unlink_handler,
  [HOSTFS_RPMSG_MKDIR]     = hostfs_rpmsg_mkdir_handler,
  [HOSTFS_RPMSG_RMDIR]     = hostfs_rpmsg_rmdir_handler,
  [HOSTFS_RPMSG_RENAME]    = hostfs_rpmsg_rename_handler,
  [HOSTFS_RPMSG_STAT]      = hostfs_rpmsg_stat_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
static int hostfs_rpmsg_try_mount(const struct hostfs_server_config_s *cfg,
                                     const char *pathname)
{
  const struct hostfs_server_mount_s *mntp = cfg->mnt;
  const struct hostfs_mkdir_s *dirp = cfg->dir;
  int ret = ERROR;
  int i;

  for (i = 0; i < cfg->mntcnt; i++)
    {
      if (!strncmp(pathname, mntp[i].target,
            strchr(pathname + 1, '/') - pathname))
        {
          ret = mount(mntp[i].source, mntp[i].target, mntp[i].fstype,
              0, mntp[i].options);
        }
    }

  if (ret == OK)
    {
      for (i = 0; i < cfg->dircnt; i++)
        {
          if (!strncmp(pathname, dirp[i].parent_dir,
                strchr(pathname + 1, '/') - pathname))
            {
              mkdir(dirp[i].full_dir, 0777);
            }
        }
    }

  return ret;
}
#endif

static int hostfs_rpmsg_open_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_open_s *msg = data;
  int i;
#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
  int ret = 0;
#else
  int ret = -ENOENT;
#endif

  nxsem_wait(&priv->sem);
  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      if (!priv->files[i].f_inode)
        {
          ret = file_open(&priv->files[i], msg->pathname, msg->flags,
                          msg->mode);

#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
          if (ret == -ENOENT)
            {
              /* No direntory, Try mount the direntory on server Once */
              if (hostfs_rpmsg_try_mount(priv->cfg, msg->pathname) == OK)
                {
                  ret = file_open(&priv->files[i], msg->pathname, msg->flags,
                                msg->mode);
                }
            }
#endif
          if (ret >= 0)
            {
              ret = i;
            }

          break;
        }
    }

  nxsem_post(&priv->sem);

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_close_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_close_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      nxsem_wait(&priv->sem);
      ret = file_close(&priv->files[msg->fd]);
      nxsem_post(&priv->sem);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_read_s *msg = data;
  FAR struct hostfs_rpmsg_read_s *rsp;
  int ret = -ENOENT;
  uint32_t space;

  rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (!rsp)
    {
      return -ENOMEM;
    }

  *rsp = *msg;

  space -= sizeof(*msg);
  if (space > msg->count)
    {
      space = msg->count;
    }

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_read(&priv->files[msg->fd], rsp->buf, space);
    }

  rsp->header.result = ret;
  return rpmsg_send_nocopy(ept, rsp, (ret < 0 ? 0 : ret) + sizeof(*rsp));
}

static int hostfs_rpmsg_write_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_write_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_write(&priv->files[msg->fd], msg->buf, msg->count);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_lseek_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_lseek_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_seek(&priv->files[msg->fd], msg->offset, msg->whence);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_ioctl_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_ioctl(&priv->files[msg->fd], msg->request, msg->arg);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_sync_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_sync_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_fsync(&priv->files[msg->fd]);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_dup_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_dup_s *msg = data;
  int i;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      nxsem_wait(&priv->sem);
      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (!priv->files[i].f_inode)
            {
              ret = file_dup2(&priv->files[msg->fd], &priv->files[i]);
              if (ret >= 0)
                {
                  ret = i;
                }

              break;
            }
        }

      nxsem_post(&priv->sem);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_fstat_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_fstat_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_fstat(&priv->files[msg->fd], &msg->buf);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_ftruncate_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_ftruncate_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_truncate(&priv->files[msg->fd], msg->length);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_opendir_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_opendir_s *msg = data;
  FAR void *dir;
  int i;
  int ret = -ENOTDIR;

  dir = opendir(msg->pathname);

#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
  if (!dir && get_errno() == ENOTDIR)
    {
      if (hostfs_rpmsg_try_mount(priv->cfg, msg->pathname) == OK)
        {
          dir = opendir(msg->pathname);
        }
    }
#endif

  if (dir)
    {
      nxsem_wait(&priv->sem);
      for (i = 1; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (!priv->dirs[i])
            {
              priv->dirs[i] = dir;
              ret = i;
              break;
            }
        }

      nxsem_post(&priv->sem);

      if (ret < 0)
        {
          closedir(dir);
        }
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_readdir_s *msg = data;
  FAR struct dirent *entry;
  int ret = -ENOENT;

  if (msg->fd >= 1 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      entry = readdir(priv->dirs[msg->fd]);
      if (entry)
        {
          msg->type = entry->d_type;
          strcpy(msg->name, entry->d_name);
          len += strlen(entry->d_name) + 1;
          ret = 0;
        }
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, len);
}

static int hostfs_rpmsg_rewinddir_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_rewinddir_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 1 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      rewinddir(priv->dirs[msg->fd]);
      ret = 0;
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_closedir_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_closedir_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 1 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = closedir(priv->dirs[msg->fd]);
      nxsem_wait(&priv->sem);
      priv->dirs[msg->fd] = NULL;
      nxsem_post(&priv->sem);
      ret = ret ? get_errno(ret) : 0;
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_statfs_s *msg = data;
  int ret;

  ret = statfs(msg->pathname, &msg->buf);
  msg->header.result = ret ? get_errno(ret) : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_unlink_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_unlink_s *msg = data;
  int ret;

  ret = unlink(msg->pathname);
  msg->header.result = ret ? get_errno(ret) : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_mkdir_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_mkdir_s *msg = data;
  int ret;

  ret = mkdir(msg->pathname, msg->mode);
  msg->header.result = ret ? get_errno(ret) : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_rmdir_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_rmdir_s *msg = data;
  int ret;

  ret = rmdir(msg->pathname);
  msg->header.result = ret ? get_errno(ret) : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_rename_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_rename_s *msg = data;
  FAR char *newpath;
  size_t oldlen;
  int ret;

  oldlen = (strlen(msg->pathname) + 1 + 0x7) & ~0x7;
  newpath = msg->pathname + oldlen;

  ret = rename(msg->pathname, newpath);
  msg->header.result = ret ? get_errno(ret) : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int hostfs_rpmsg_stat_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_)
{
  FAR struct hostfs_rpmsg_server_s *priv = priv_;
  FAR struct hostfs_rpmsg_stat_s *msg = data;
  int ret;

  ret = stat(msg->pathname, &msg->buf);
#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
  if (ret && get_errno() == ENOENT)
    {
      if (hostfs_rpmsg_try_mount(priv->cfg, msg->pathname) == OK)
        {
          ret = stat(msg->pathname, &msg->buf);
        }
    }
#endif
  msg->header.result = ret ? get_errno(ret) : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static void hostfs_rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv_, FAR const char *name,
                                 uint32_t dest)
{
  FAR struct hostfs_rpmsg_server_s *priv;
  int ret;

  if (strcmp(name, HOSTFS_RPMSG_EPT_NAME))
    {
      return;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      return;
    }

#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
  priv->cfg = priv_;
#endif
  priv->ept.priv = priv;
  nxsem_init(&priv->sem, 0, 1);

  ret = rpmsg_create_ept(&priv->ept, rdev, HOSTFS_RPMSG_EPT_NAME,
                         RPMSG_ADDR_ANY, dest,
                         hostfs_rpmsg_ept_cb, hostfs_rpmsg_ns_unbind);
  if (ret)
    {
      nxsem_destroy(&priv->sem);
      kmm_free(priv);
    }
}

static void hostfs_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct hostfs_rpmsg_server_s *priv = ept->priv;
  int i;

  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      if (priv->files[i].f_inode)
        {
          file_close(&priv->files[i]);
        }
    }

  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      if (priv->dirs[i])
        {
          closedir(priv->dirs[i]);
        }
    }

  rpmsg_destroy_ept(&priv->ept);
  nxsem_destroy(&priv->sem);

  kmm_free(priv);
}

static int hostfs_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                               size_t len, uint32_t src, FAR void *priv)
{
  struct hostfs_rpmsg_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_hostfs_rpmsg_handler))
    {
      return g_hostfs_rpmsg_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
int hostfs_rpmsg_server_init(FAR const struct hostfs_server_config_s* cfg)
#else
int hostfs_rpmsg_server_init(void)
#endif
{
#ifdef CONFIG_FS_HOSTFS_SERVER_MOUNT
  return rpmsg_register_callback((void *)cfg,
#else
  return rpmsg_register_callback(NULL,
#endif
                                 NULL,
                                 NULL,
                                 hostfs_rpmsg_ns_bind);
}
