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
  int                  cpu_id;
};

struct hostfs_rpmsg_cookie_s
{
  sem_t                        sem;
  struct hostfs_rpmsg_header_s *msg;
  int                          result;
  void                         *privdata;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void hostfs_rpmsg_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_read_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_opendir_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_statfs_handler(struct rpmsg_channel *channel,
                void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_stat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static void hostfs_rpmsg_device_created(struct remote_device *rdev, void *priv_);
static void hostfs_rpmsg_channel_created(struct rpmsg_channel *channel);
static void hostfs_rpmsg_channel_destroyed(struct rpmsg_channel *channel);
static void hostfs_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);
static int  hostfs_msg_send_recv(uint32_t command, bool copy, void *privdata,
                    struct hostfs_rpmsg_header_s *msg, int len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct hostfs_rpmsg_s g_hostfs_rpmsg;

static const rpmsg_rx_cb_t g_hostfs_rpmsg_handler[] =
{
  [HOSTFS_RPMSG_OPEN]      = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_CLOSE]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_READ]      = hostfs_rpmsg_read_handler,
  [HOSTFS_RPMSG_WRITE]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_LSEEK]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_IOCTL]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_SYNC]      = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_DUP]       = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_FSTAT]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_OPENDIR]   = hostfs_rpmsg_opendir_handler,
  [HOSTFS_RPMSG_READDIR]   = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_REWINDDIR] = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_CLOSEDIR]  = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_STATFS]    = hostfs_rpmsg_statfs_handler,
  [HOSTFS_RPMSG_UNLINK]    = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_MKDIR]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_RMDIR]     = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_RENAME]    = hostfs_rpmsg_handler,
  [HOSTFS_RPMSG_STAT]      = hostfs_rpmsg_stat_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void hostfs_rpmsg_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie = (struct hostfs_rpmsg_cookie_s *)header->cookie;

  cookie->result = header->result;

  if (cookie->msg)
    {
      memcpy(cookie->msg, data, len);
    }

  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_read_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie = (struct hostfs_rpmsg_cookie_s *)header->cookie;
  struct hostfs_rpmsg_read_s *recv = data;

  cookie->result = header->result;

  if (header->result > 0)
    {
      memcpy(cookie->privdata, recv->buf, B2C(header->result));
    }

  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_opendir_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie = (struct hostfs_rpmsg_cookie_s *)header->cookie;
  struct hostfs_rpmsg_opendir_s *recv = data;

  cookie->result = header->result;

  if (header->result >= 0)
    {
      *((uint32_t *)cookie->privdata) = recv->dirp;
    }

  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_statfs_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie = (struct hostfs_rpmsg_cookie_s *)header->cookie;
  struct hostfs_rpmsg_statfs_s *recv = data;

  cookie->result = header->result;

  if (header->result >= 0)
    {
      memcpy(cookie->privdata, &recv->buf, sizeof(recv->buf));
    }

  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_stat_handler(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct hostfs_rpmsg_header_s *header = data;
  struct hostfs_rpmsg_cookie_s *cookie = (struct hostfs_rpmsg_cookie_s *)header->cookie;
  struct hostfs_rpmsg_stat_s *recv = data;

  cookie->result = header->result;

  if (header->result >= 0)
    {
      memcpy(cookie->privdata, &recv->buf, sizeof(recv->buf));
    }

  nxsem_post(&cookie->sem);
}

static void hostfs_rpmsg_device_created(struct remote_device *rdev, void *priv_)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;

  if (priv->cpu_id == rdev->proc.cpu_id)
    {
      rpmsg_create_channel(rdev, HOSTFS_RPMSG_CHANNEL_NAME);
    }
}

static void hostfs_rpmsg_channel_created(struct rpmsg_channel *channel)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct remote_device *rdev = channel->rdev;

  if (priv->cpu_id == rdev->proc.cpu_id)
    {
      priv->channel = channel;
    }
}

static void hostfs_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct remote_device *rdev = channel->rdev;

  if (priv->cpu_id == rdev->proc.cpu_id)
    {
      priv->channel = NULL;
    }
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

static int hostfs_msg_send_recv(uint32_t command, bool copy, void *privdata,
                struct hostfs_rpmsg_header_s *msg, int len)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_cookie_s cookie = {0};
  int ret;

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_setprotocol(&cookie.sem, SEM_PRIO_NONE);

  cookie.privdata = privdata;
  if (copy)
    {
      cookie.msg = msg;
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
 * Public Funtions
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

  memset(msg, 0, sizeof(*msg));

  msg->flags = flags;
  msg->mode  = mode;
  cstr2bstr(msg->pathname, pathname);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_OPEN, false, NULL,
          (struct hostfs_rpmsg_header_s *)msg, len);
}

int host_close(int fd)
{
  struct hostfs_rpmsg_close_s msg;

  msg.fd = fd;

  return hostfs_msg_send_recv(HOSTFS_RPMSG_CLOSE, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
}

ssize_t host_read(int fd, void *buf_, size_t count)
{
  struct hostfs_rpmsg_read_s msg;
  char *buf = buf_;
  size_t read = 0;
  int ret = 0;

  while (read < count)
    {
      msg.fd    = fd;
      msg.count = C2B(count - read);
      ret = hostfs_msg_send_recv(HOSTFS_RPMSG_READ, true, buf,
              (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
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
  struct hostfs_rpmsg_write_s *msg;
  size_t written = 0;
  uint32_t space;
  int ret = 0;


  while (written < count)
    {
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
      memcpy(msg->buf, (char *)buf + written, space);

      ret = hostfs_msg_send_recv(HOSTFS_RPMSG_WRITE, false, NULL,
                (struct hostfs_rpmsg_header_s *)msg, sizeof(*msg) + space);
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
  struct hostfs_rpmsg_lseek_s msg;
  int ret;

  msg.fd     = fd;
  msg.offset = C2B(offset);
  msg.whence = whence;

  ret = hostfs_msg_send_recv(HOSTFS_RPMSG_LSEEK, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));

  return ret < 0 ? ret: B2C(msg.offset);
}

int host_ioctl(int fd, int request, unsigned long arg)
{
  struct hostfs_rpmsg_ioctl_s msg;

  msg.fd      = fd;
  msg.request = request;
  msg.arg     = arg;

  return hostfs_msg_send_recv(HOSTFS_RPMSG_IOCTL, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
}

void host_sync(int fd)
{
  struct hostfs_rpmsg_sync_s msg;

  msg.fd = fd;

  hostfs_msg_send_recv(HOSTFS_RPMSG_SYNC, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
}

int host_dup(int fd)
{
  struct hostfs_rpmsg_dup_s msg;

  msg.fd = fd;

  return hostfs_msg_send_recv(HOSTFS_RPMSG_DUP, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
}

int host_fstat(int fd, struct stat *buf)
{
  struct hostfs_rpmsg_fstat_s msg;
  int ret;

  msg.fd = fd;

  ret = hostfs_msg_send_recv(HOSTFS_RPMSG_FSTAT, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
  if (ret == 0)
    {
      memcpy(buf, &msg.buf, sizeof(*buf));
    }

  return ret;
}

void *host_opendir(const char *name)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_opendir_s *msg;
  uint32_t dirp;
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

  memset(msg, 0, sizeof(*msg));

  cstr2bstr(msg->pathname, name);

  ret = hostfs_msg_send_recv(HOSTFS_RPMSG_OPENDIR, false, &dirp,
          (struct hostfs_rpmsg_header_s *)msg, len);

  return ret ? NULL : (void *)(uintptr_t)dirp;
}

int host_readdir(void *dirp, struct dirent* entry)
{
  struct hostfs_rpmsg_readdir_s msg;
  int ret;

  msg.dirp = (uintptr_t)dirp;

  ret = hostfs_msg_send_recv(HOSTFS_RPMSG_READDIR, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
  if (ret == 0)
    {
      memcpy(entry, &msg.entry, sizeof(*entry));
    }

  return ret;
}

void host_rewinddir(void *dirp)
{
  struct hostfs_rpmsg_rewinddir_s msg;

  msg.dirp = (uintptr_t)dirp;

  hostfs_msg_send_recv(HOSTFS_RPMSG_REWINDDIR, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
}

int host_closedir(void *dirp)
{
  struct hostfs_rpmsg_closedir_s msg;

  msg.dirp = (uintptr_t)dirp;

  return hostfs_msg_send_recv(HOSTFS_RPMSG_CLOSEDIR, true, NULL,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg));
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

  memset(msg, 0, sizeof(*msg));

  cstr2bstr(msg->pathname, path);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_STATFS, false, buf,
          (struct hostfs_rpmsg_header_s *)msg, len);
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

  memset(msg, 0, sizeof(*msg));

  cstr2bstr(msg->pathname, pathname);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_UNLINK, false, NULL,
          (struct hostfs_rpmsg_header_s *)msg, len);
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

  memset(msg, 0, sizeof(*msg));

  msg->mode = mode;
  cstr2bstr(msg->pathname, pathname);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_MKDIR, false, NULL,
          (struct hostfs_rpmsg_header_s *)msg, len);
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

  memset(msg, 0, sizeof(*msg));

  cstr2bstr(msg->pathname, pathname);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_RMDIR, false, NULL,
          (struct hostfs_rpmsg_header_s *)msg, len);
}

int host_rename(const char *oldpath, const char *newpath)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_rename_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(oldpath) + 1);
  len += B2C(strlen(newpath) + 1);

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

  memset(msg, 0, sizeof(*msg));

  cstr2bstr(msg->pathname, oldpath);
  cstr2bstr(msg->pathname + B2C(strlen(oldpath) + 1), newpath);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_RENAME, false, NULL,
          (struct hostfs_rpmsg_header_s *)msg, len);
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

  memset(msg, 0, sizeof(*msg));

  cstr2bstr(msg->pathname, path);

  return hostfs_msg_send_recv(HOSTFS_RPMSG_STAT, false, buf,
          (struct hostfs_rpmsg_header_s *)msg, len);
}

int hostfs_rpmsg_init(int cpu_id)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;

  priv->cpu_id = cpu_id;

  return rpmsg_register_callback(
                HOSTFS_RPMSG_CHANNEL_NAME,
                NULL,
                hostfs_rpmsg_device_created,
                NULL,
                hostfs_rpmsg_channel_created,
                hostfs_rpmsg_channel_destroyed,
                hostfs_rpmsg_channel_received);
}
