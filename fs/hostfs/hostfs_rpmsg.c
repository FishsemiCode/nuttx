/****************************************************************************
 * fs/hostfs/hostfs_rpmsg.c
 * Hostfs rpmsg driver
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

#include <errno.h>
#include <string.h>

#include <openamp/open_amp.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/hostfs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/semaphore.h>

#include "hostfs_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hostfs_rpmsg_s
{
  struct rpmsg_channel *channel;
  const char           *cpu_name;
};

struct hostfs_rpmsg_cookie_s
{
  sem_t                 sem;
  int                   result;
  void                  *data;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void hostfs_rpmsg_default_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv, unsigned long src);
static void hostfs_rpmsg_read_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv, unsigned long src);
static void hostfs_rpmsg_readdir_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv, unsigned long src);
static void hostfs_rpmsg_statfs_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv, unsigned long src);
static void hostfs_rpmsg_stat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src);
static void hostfs_rpmsg_device_created(struct remote_device *rdev, void *priv_);
static void hostfs_rpmsg_channel_created(struct rpmsg_channel *channel);
static void hostfs_rpmsg_channel_destroyed(struct rpmsg_channel *channel);
static void hostfs_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src);
static int  hostfs_rpmsg_send_recv(uint32_t command, bool copy,
                    struct hostfs_rpmsg_header_s *msg, int len, void *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct hostfs_rpmsg_s g_hostfs_rpmsg;

static const rpmsg_rx_cb_t g_hostfs_rpmsg_handler[] =
{
  [HOSTFS_RPMSG_OPEN]      = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_CLOSE]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_READ]      = hostfs_rpmsg_read_handler,
  [HOSTFS_RPMSG_WRITE]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_LSEEK]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_IOCTL]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_SYNC]      = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_DUP]       = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_FSTAT]     = hostfs_rpmsg_stat_handler,
  [HOSTFS_RPMSG_FTRUNCATE] = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_OPENDIR]   = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_READDIR]   = hostfs_rpmsg_readdir_handler,
  [HOSTFS_RPMSG_REWINDDIR] = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_CLOSEDIR]  = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_STATFS]    = hostfs_rpmsg_statfs_handler,
  [HOSTFS_RPMSG_UNLINK]    = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_MKDIR]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_RMDIR]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_RENAME]    = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_STAT]      = hostfs_rpmsg_stat_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void hostfs_rpmsg_default_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;

  cookie->result = header->result;
  if (cookie->result >= 0 && cookie->data)
    {
      memcpy(cookie->data, data, len);
    }
  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_read_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  struct hostfs_rpmsg_read_s *rsp = data;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      memcpy(cookie->data, rsp->buf, B2C(cookie->result));
    }
  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_readdir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  struct hostfs_rpmsg_readdir_s *rsp = data;
  struct dirent *entry = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      nbstr2cstr(entry->d_name, rsp->name, NAME_MAX);
      entry->d_name[NAME_MAX] = '\0';
      entry->d_type = rsp->type;
    }
  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_statfs_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  struct hostfs_rpmsg_statfs_s *rsp = data;
  struct statfs *buf = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      buf->f_type    = rsp->buf.f_type;
      buf->f_namelen = rsp->buf.f_namelen;
      buf->f_bsize   = B2C(rsp->buf.f_bsize);
      buf->f_blocks  = rsp->buf.f_blocks;
      buf->f_bfree   = rsp->buf.f_bfree;
      buf->f_bavail  = rsp->buf.f_bavail;
      buf->f_files   = rsp->buf.f_files;
      buf->f_ffree   = rsp->buf.f_ffree;
    }
  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_stat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  struct hostfs_rpmsg_stat_s *rsp = data;
  struct stat *buf = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      buf->st_mode    = rsp->buf.st_mode;
      buf->st_size    = B2C(rsp->buf.st_size);
      buf->st_blksize = B2C(rsp->buf.st_blksize);
      buf->st_blocks  = rsp->buf.st_blocks;
      buf->st_atime   = rsp->buf.st_atime;
      buf->st_mtime   = rsp->buf.st_mtime;
      buf->st_ctime   = rsp->buf.st_ctime;
    }
  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_device_created(struct remote_device *rdev, void *priv_)
{
  struct hostfs_rpmsg_s *priv = priv_;
  struct rpmsg_channel *channel;

  if (strcmp(priv->cpu_name, rdev->proc->cpu_name) == 0)
    {
      channel = rpmsg_create_channel(rdev, HOSTFS_RPMSG_CHANNEL_NAME);
      if (channel != NULL)
        {
          rpmsg_set_privdata(channel, priv);
        }
    }
}

static void hostfs_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct hostfs_rpmsg_s *priv = rpmsg_get_privdata(channel);

  if (priv != NULL)
    {
      priv->channel = channel;
    }
}

static void hostfs_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
  struct hostfs_rpmsg_s *priv = rpmsg_get_privdata(channel);

  if (priv != NULL)
    {
      priv->channel = NULL;
    }
}

static void hostfs_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_hostfs_rpmsg_handler))
    {
      g_hostfs_rpmsg_handler[command](channel, data, len, priv, src);
    }
}

static int hostfs_rpmsg_send_recv(uint32_t command, bool copy,
                struct hostfs_rpmsg_header_s *msg, int len, void *data)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_cookie_s cookie;
  int ret;

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);
  nxsem_setprotocol(&cookie.sem, SEM_PRIO_NONE);

  if (data)
    {
      cookie.data = data;
    }
  else if (copy)
    {
      cookie.data = msg;
    }

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)&cookie;

  if (copy)
    {
      ret = rpmsg_send(priv->channel, msg, len);
    }
  else
    {
      ret = rpmsg_send_nocopy(priv->channel, msg, len);
    }
  if (ret < 0)
    {
      goto fail;
    }

  while (1)
    {
      ret = nxsem_wait(&cookie.sem);
      if (ret != -EINTR)
        {
          if (ret == 0)
            {
              ret = cookie.result;
            }
          break;
        }
    }

fail:
  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_open(const char *pathname, int flags, int mode)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_open_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  msg->flags = flags;
  msg->mode  = mode;
  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_OPEN, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_close(int fd)
{
  struct hostfs_rpmsg_close_s msg =
  {
    .fd = fd,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_CLOSE, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

ssize_t host_read(int fd, void *buf, size_t count)
{
  size_t read = 0;
  int ret = 0;

  while (read < count)
    {
      struct hostfs_rpmsg_read_s msg =
      {
        .fd    = fd,
        .count = C2B(count - read),
      };

      ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_READ, true,
              (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), buf);
      if (ret <= 0)
        {
          break;
        }

      read += B2C(ret);
      buf  += B2C(ret);
    }

  return read ? read : ret;
}

ssize_t host_write(int fd, const void *buf, size_t count)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  size_t written = 0;
  int ret = 0;

  while (written < count)
    {
      struct hostfs_rpmsg_write_s *msg;
      uint32_t space;

      msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
      if (!msg)
        {
          ret = -ENOMEM;
          break;
        }

      space -= sizeof(*msg);
      if (space > count - written)
        {
          space = count - written;
        }

      msg->fd    = fd;
      msg->count = C2B(space);
      memcpy(msg->buf, buf + written, space);

      ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_WRITE, false,
                (struct hostfs_rpmsg_header_s *)msg, sizeof(*msg) + space, NULL);
      if (ret <= 0)
        {
          break;
        }

      written += B2C(ret);
    }

  return written ? written : ret;
}

off_t host_lseek(int fd, off_t offset, int whence)
{
  struct hostfs_rpmsg_lseek_s msg =
  {
    .fd     = fd,
    .offset = C2B(offset),
    .whence = whence,
  };
  int ret;

  ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_LSEEK, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);

  return ret < 0 ? ret : B2C(ret);
}

int host_ioctl(int fd, int request, unsigned long arg)
{
  struct hostfs_rpmsg_ioctl_s msg =
  {
    .fd      = fd,
    .request = request,
    .arg     = arg,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_IOCTL, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

void host_sync(int fd)
{
  struct hostfs_rpmsg_sync_s msg =
  {
    .fd = fd,
  };

  hostfs_rpmsg_send_recv(HOSTFS_RPMSG_SYNC, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_dup(int fd)
{
  struct hostfs_rpmsg_dup_s msg =
  {
    .fd = fd,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_DUP, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_fstat(int fd, struct stat *buf)
{
  struct hostfs_rpmsg_fstat_s msg =
  {
    .fd = fd,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_FSTAT, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), buf);
}

int host_ftruncate(int fd, off_t length)
{
  struct hostfs_rpmsg_ftruncate_s msg =
  {
    .fd     = fd,
    .length = length,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_FTRUNCATE, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

void *host_opendir(const char *name)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_opendir_s *msg;
  uint32_t space;
  size_t len;
  int ret;

  len  = sizeof(*msg);
  len += B2C(strlen(name) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return NULL;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return NULL;
    }

  cstr2bstr(msg->pathname, name);

  ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_OPENDIR, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);

  return ret < 0 ? NULL : (void *)ret;
}

int host_readdir(void *dirp, struct dirent* entry)
{
  struct hostfs_rpmsg_readdir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_READDIR, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), entry);
}

void host_rewinddir(void *dirp)
{
  struct hostfs_rpmsg_rewinddir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  hostfs_rpmsg_send_recv(HOSTFS_RPMSG_REWINDDIR, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_closedir(void *dirp)
{
  struct hostfs_rpmsg_closedir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_CLOSEDIR, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_statfs(const char *path, struct statfs *buf)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_statfs_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(path) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  cstr2bstr(msg->pathname, path);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_STATFS, false,
          (struct hostfs_rpmsg_header_s *)msg, len, buf);
}

int host_unlink(const char *pathname)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_unlink_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_UNLINK, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_mkdir(const char *pathname, mode_t mode)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_mkdir_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  msg->mode = mode;
  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_MKDIR, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_rmdir(const char *pathname)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_rmdir_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_RMDIR, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_rename(const char *oldpath, const char *newpath)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_rename_s *msg;
  size_t len, oldlen;
  uint32_t space;

  len     = sizeof(*msg);
  oldlen  = B2C((strlen(oldpath) + 1 + 0x7) & ~0x7);
  len    += oldlen + B2C(strlen(newpath) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  cstr2bstr(msg->pathname, oldpath);
  cstr2bstr(msg->pathname + oldlen, newpath);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_RENAME, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_stat(const char *path, struct stat *buf)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_stat_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(path) + 1);

  space = rpmsg_get_buffer_size(priv->channel);
  if (len > space)
    {
      return -ENOMEM;
    }

  msg = rpmsg_get_tx_payload_buffer(priv->channel, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  cstr2bstr(msg->pathname, path);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_STAT, false,
          (struct hostfs_rpmsg_header_s *)msg, len, buf);
}

int hostfs_rpmsg_init(const char *cpu_name)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;

  priv->cpu_name = cpu_name;

  return rpmsg_register_callback(
                HOSTFS_RPMSG_CHANNEL_NAME,
                priv,
                hostfs_rpmsg_device_created,
                NULL,
                hostfs_rpmsg_channel_created,
                hostfs_rpmsg_channel_destroyed,
                hostfs_rpmsg_channel_received);
}
