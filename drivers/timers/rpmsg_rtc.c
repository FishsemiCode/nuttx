/****************************************************************************
 * drivers/timers/rpmsg_rtc.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li <liguiding@pinecone.net>
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

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/timers/rpmsg_rtc.h>

#include <openamp/open_amp.h>

#include <errno.h>
#include <string.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_RTC_CHANNEL_NAME      "rpmsg-rtc"

#define RPMSG_RTC_SET               1
#define RPMSG_RTC_GET               2

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_rtc_header_s
{
  uint32_t command;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct rpmsg_rtc_set_s
{
  struct rpmsg_rtc_header_s header;
  int64_t                   tv_sec;
  int32_t                   tv_nsec;
} end_packed_struct;

#define rpmsg_rtc_get_s rpmsg_rtc_set_s

struct rpmsg_rtc_cookie_s
{
  struct rpmsg_rtc_header_s *msg;
  sem_t                     sem;
};

/* This is the private type for the RTC state. It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct rpmsg_rtc_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  sem_t                sem;
  struct rpmsg_channel *channel;
  const char           *cpu_name;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_rtc_device_created(struct remote_device *rdev, void *priv);
static void rpmsg_rtc_channel_created(struct rpmsg_channel *channel);
static void rpmsg_rtc_channel_destroyed(struct rpmsg_channel *channel);
static void rpmsg_rtc_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv, unsigned long src);
static int rpmsg_rtc_send_recv(struct rpmsg_rtc_lowerhalf_s *lower,
                uint32_t command, struct rpmsg_rtc_header_s *msg, int len);

static int rpmsg_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower_,
                           FAR struct rtc_time *rtctime);
static int rpmsg_rtc_settime(FAR struct rtc_lowerhalf_s *lower_,
                            FAR const struct rtc_time *rtctime);
static bool rpmsg_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_rpmsg_rtc_ops =
{
  .rdtime      = rpmsg_rtc_rdtime,
  .settime     = rpmsg_rtc_settime,
  .havesettime = rpmsg_rtc_havesettime,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rpmsg_rtc_device_created(struct remote_device *rdev, void *priv)
{
  struct rpmsg_rtc_lowerhalf_s *lower = priv;
  struct rpmsg_channel *channel;

  if (strcmp(lower->cpu_name, rdev->proc->cpu_name) == 0)
    {
      channel = rpmsg_create_channel(rdev, RPMSG_RTC_CHANNEL_NAME);
      if (channel != NULL)
        {
          rpmsg_set_privdata(channel, lower);
        }
    }
}

static void rpmsg_rtc_channel_created(struct rpmsg_channel *channel)
{
  struct rpmsg_rtc_lowerhalf_s *lower = rpmsg_get_privdata(channel);

  if (lower != NULL)
    {
      lower->channel = channel;
      nxsem_post(&lower->sem);
    }
}

static void rpmsg_rtc_channel_destroyed(struct rpmsg_channel *channel)
{
  struct rpmsg_rtc_lowerhalf_s *lower = rpmsg_get_privdata(channel);

  if (lower != NULL)
    {
      nxsem_wait(&lower->sem);
      lower->channel = NULL;
    }
}

static void rpmsg_rtc_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct rpmsg_rtc_header_s *header = data;
  struct rpmsg_rtc_cookie_s *cookie =
      (struct rpmsg_rtc_cookie_s *)(uintptr_t)header->cookie;

  if (cookie)
    {
      memcpy(cookie->msg, data, len);
      nxsem_post(&cookie->sem);
    }
}

static int rpmsg_rtc_send_recv(struct rpmsg_rtc_lowerhalf_s *lower,
                uint32_t command, struct rpmsg_rtc_header_s *msg, int len)
{
  struct rpmsg_rtc_cookie_s cookie;
  int ret;

  if (!lower->channel)
    {
      nxsem_wait(&lower->sem);
      nxsem_post(&lower->sem);
      DEBUGASSERT(lower->channel);
    }

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_setprotocol(&cookie.sem, SEM_PRIO_NONE);
  cookie.msg = msg;

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)&cookie;

  ret = rpmsg_send(lower->channel, msg, len);
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
              ret = msg->result;
            }
          break;
        }
    }

fail:
  nxsem_destroy(&cookie.sem);
  return ret;
}

static int rpmsg_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower_,
                            FAR struct rtc_time *rtctime)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower = (FAR struct rpmsg_rtc_lowerhalf_s *)lower_;
  struct rpmsg_rtc_get_s msg;
  int ret;

  ret = rpmsg_rtc_send_recv(lower, RPMSG_RTC_GET,
          (struct rpmsg_rtc_header_s *)&msg, sizeof(msg));
  if (ret == 0)
    {
      time_t time = msg.tv_sec;
      gmtime_r(&time, (FAR struct tm *)rtctime);
      rtctime->tm_nsec = msg.tv_nsec;
    }

  return ret;
}

static int rpmsg_rtc_settime(FAR struct rtc_lowerhalf_s *lower_,
                             FAR const struct rtc_time *rtctime)
{
  FAR struct rpmsg_rtc_lowerhalf_s *lower = (FAR struct rpmsg_rtc_lowerhalf_s *)lower_;
  struct rpmsg_rtc_set_s msg;

  msg.tv_sec  = mktime((FAR struct tm *)rtctime);
  msg.tv_nsec = rtctime->tm_nsec;

  return rpmsg_rtc_send_recv(lower, RPMSG_RTC_SET,
          (struct rpmsg_rtc_header_s *)&msg, sizeof(msg));
}

static bool rpmsg_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return true;
}

/****************************************************************************
 * Name: rpmsg_rtc_initialize
 *
 * Description:
 *
 *   Take remote core RTC as external RTC hardware through rpmsg.
 *
 * Input Parameters:
 *   cpu_name - current cpu name
 *   minor  - device minor number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *rpmsg_rtc_initialize(const char *cpu_name, int minor)
{
  struct rpmsg_rtc_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower != NULL)
    {
      nxsem_init(&lower->sem, 0, 0);
      nxsem_setprotocol(&lower->sem, SEM_PRIO_NONE);

      lower->ops = &g_rpmsg_rtc_ops;
      lower->cpu_name = cpu_name;

      rpmsg_register_callback(
                RPMSG_RTC_CHANNEL_NAME,
                lower,
                rpmsg_rtc_device_created,
                NULL,
                rpmsg_rtc_channel_created,
                rpmsg_rtc_channel_destroyed,
                rpmsg_rtc_channel_received);

      rtc_initialize(minor, (FAR struct rtc_lowerhalf_s *)lower);
    }

  return (FAR struct rtc_lowerhalf_s *)lower;
}
