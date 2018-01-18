/****************************************************************************
 * drivers/syslog/syslog_rpmsg_server.c
 * Syslog driver for rpmsg syslog server
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

#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include <openamp/open_amp.h>

#include "syslog.h"
#include "syslog_rpmsg.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void syslog_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void syslog_rpmsg_channel_received(struct rpmsg_channel *channel,
                    void *data, int len, void *priv_, unsigned long src)
{
  struct syslog_rpmsg_header_s *header = data;
  struct syslog_rpmsg_transfer_s *msg = data;
  struct syslog_rpmsg_header_s done;


  if (header->command == SYSLOG_RPMSG_TRANSFER)
    {
      syslog_write(msg->data, msg->count);

      memset(&done, 0, sizeof(done));
      done.command = SYSLOG_RPMSG_TRANSFER_DONE;
      done.result  = msg->count;
      rpmsg_send(channel, &done, sizeof(done));
    }
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

int syslog_rpmsg_server_init(void)
{
  return rpmsg_register_callback(
                SYSLOG_RPMSG_CHANNEL_NAME,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                syslog_rpmsg_channel_received);
}
