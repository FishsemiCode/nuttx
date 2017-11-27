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

#include <openamp/open_amp.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>

#include "hostfs_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hostfs_rpmsg_server_s
{
  struct file files[CONFIG_NFILE_DESCRIPTORS];
  void        *dirps[CONFIG_NFILE_DESCRIPTORS];
  sem_t       sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void hostfs_rpmsg_open_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_close_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_read_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_write_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_lseek_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_ioctl_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_sync_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_dup_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_fstat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_opendir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_readdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_rewinddir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_closedir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_statfs_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_unlink_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_mkdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_rmdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_rename_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_stat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_channel_created(struct rpmsg_channel *channel);
static void hostfs_rpmsg_channel_destroyed(struct rpmsg_channel *channel);
static void hostfs_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_rx_cb_t g_hostfs_rpmsg_handler[] =
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

static void hostfs_rpmsg_open_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_open_s *msg = data;
  int i, fd, ret = -ENOENT;

  fd = open(msg->pathname, msg->flags, msg->mode);
  if (fd >= 0)
    {
      nxsem_wait(&priv->sem);

      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (!priv->files[i].f_inode)
            {
              break;
            }
        }

      if (i != CONFIG_NFILE_DESCRIPTORS)
        {
          ret = file_detach(fd, &priv->files[i]);
          if (ret == 0)
            {
              ret = i;
            }
        }

      nxsem_post(&priv->sem);

      if (ret < 0)
        {
          close(fd);
        }
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_close_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_close_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      nxsem_wait(&priv->sem);

      ret = file_close_detached(&priv->files[msg->fd]);

      nxsem_post(&priv->sem);
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_read_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_read_s *msg = data;
  struct hostfs_rpmsg_read_s *rsp;
  uint32_t space;
  int ret = -ENOENT;

  rsp = rpmsg_get_tx_payload_buffer(channel, &space, true);
  if (!rsp)
    {
      return;
    }

  space -= sizeof(*msg);
  if (space > msg->count)
    {
      space = msg->count;
    }

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_read(&priv->files[msg->fd], rsp->buf, space);
    }

  *rsp = *msg;
  rsp->header.result = ret;

  rpmsg_send_nocopy(channel, rsp, (ret < 0 ? 0 : ret) + sizeof(*rsp));
}

static void hostfs_rpmsg_write_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_write_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_write(&priv->files[msg->fd], msg->buf, msg->count);
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_lseek_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_lseek_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      msg->offset = file_seek(&priv->files[msg->fd], msg->offset, msg->whence);
      ret = msg->offset;
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_ioctl_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_ioctl_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_ioctl(&priv->files[msg->fd], msg->request, msg->arg);
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_sync_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_sync_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_fsync(&priv->files[msg->fd]);
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_dup_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_dup_s *msg = data;
  int i, newfd, ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      newfd = file_dup(&priv->files[msg->fd], 0);
      if (newfd >= 0)
        {
          nxsem_wait(&priv->sem);

          for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
            {
              if (!priv->files[i].f_inode)
                {
                  break;
                }
            }

          if (i != CONFIG_NFILE_DESCRIPTORS)
            {
              ret = file_detach(newfd, &priv->files[i]);
              if (ret == 0)
                {
                  ret = i;
                }
            }

          nxsem_post(&priv->sem);

          if (ret < 0)
            {
              close(newfd);
            }
        }
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_fstat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_fstat_s *msg = data;
  int ret = -ENOENT;

  if (msg->fd >= 0 && msg->fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = file_fstat(&priv->files[msg->fd], &msg->buf);
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_opendir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_opendir_s *msg = data;
  int i, ret = -ENOENT;
  void *dirp;

  dirp = opendir(msg->pathname);
  if (dirp)
    {
      nxsem_wait(&priv->sem);

      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (!priv->dirps[i])
            {
              break;
            }
        }

      if (i != CONFIG_NFILE_DESCRIPTORS)
        {
          priv->dirps[i] = dirp;
          msg->dirp = (uintptr_t)dirp;
          ret = 0;
        }

      nxsem_post(&priv->sem);

      if (ret < 0)
        {
          closedir(dirp);
        }
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_readdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_readdir_s *msg = data;
  struct dirent *entry;
  int ret = -ENOENT;

  entry = readdir((void *)(uintptr_t)msg->dirp);
  if (entry)
    {
      memcpy(&msg->entry, entry, sizeof(*entry));
      ret = 0;
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_rewinddir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_rewinddir_s *msg = data;

  rewinddir((void *)(uintptr_t)msg->dirp);

  msg->header.result = 0;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_closedir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  struct hostfs_rpmsg_closedir_s *msg = data;
  int i, ret;

  ret = closedir((void *)(uintptr_t)msg->dirp);
  if (ret == 0)
    {
      nxsem_wait(&priv->sem);

      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          if (priv->dirps[i] == (void *)(uintptr_t)msg->dirp)
            {
              priv->dirps[i] = NULL;
            }
        }

      nxsem_post(&priv->sem);
    }

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_statfs_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_statfs_s *msg = data;
  int ret;

  ret = statfs(msg->pathname, &msg->buf);

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_unlink_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_unlink_s *msg = data;
  int ret;

  ret = unlink(msg->pathname);

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_mkdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_mkdir_s *msg = data;
  int ret;

  ret = mkdir(msg->pathname, msg->mode);

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_rmdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_rmdir_s *msg = data;
  int ret;

  ret = rmdir(msg->pathname);

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_rename_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_rename_s *msg = data;
  char *newpath;
  int ret;

  newpath = msg->pathname + strlen(msg->pathname) + 1;
  if (newpath[0] == 0)
    {
      newpath += 1;
    }

  ret = rename(msg->pathname, newpath);

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_stat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_stat_s *msg = data;
  int ret;

  ret = stat(msg->pathname, &msg->buf);

  msg->header.result = ret;

  rpmsg_send(channel, msg, sizeof(*msg));
}

static void hostfs_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct hostfs_rpmsg_server_s *priv;

  priv = kmm_malloc(sizeof(*priv));
  if (!priv)
    {
      return;
    }

  nxsem_init(&priv->sem, 0, 1);

  rpmsg_set_privdata(channel, priv);
}

static void hostfs_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
  struct hostfs_rpmsg_server_s *priv = rpmsg_get_privdata(channel);
  int i;

  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      if (priv->files[i].f_inode)
        {
          file_close_detached(&priv->files[i]);
        }
    }

  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      if (priv->dirps[i])
        {
          closedir(priv->dirps[i]);
        }
    }

  nxsem_destroy(&priv->sem);
  kmm_free(priv);
}

static void hostfs_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_hostfs_rpmsg_handler))
    {
      g_hostfs_rpmsg_handler[command](channel, data, len, priv_, src);
    }
}

int hostfs_rpmsg_server_init(void)
{
  return rpmsg_register_callback(
                HOSTFS_RPMSG_CHANNEL_NAME,
                NULL,
                NULL,
                NULL,
                hostfs_rpmsg_channel_created,
                hostfs_rpmsg_channel_destroyed,
                hostfs_rpmsg_channel_received);
}
