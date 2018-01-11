/********************************************************************************************
 * include/nuttx/rptun/rptun.h
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_RPTUN_RPTUN_H
#define __INCLUDE_NUTTX_RPTUN_RPTUN_H

#ifdef CONFIG_RPTUN

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <openamp/open_amp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPTUN_NOTIFY_START         (UINT32_MAX - 1)
#define RPTUN_NOTIFY_ALL           (UINT32_MAX - 0)

#define RPTUN_USER_IOCBASE         0x80

/* Access macros ************************************************************/

/****************************************************************************
 * Name: RPTUN_GET_CPUNAME
 *
 * Description:
 *   Get remote cpu name
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *
 * Returned Value:
 *   Cpu name on success, NULL on failure.
 *
 ****************************************************************************/

#define RPTUN_GET_CPUNAME(d) ((d)->ops->get_cpuname(d))

/****************************************************************************
 * Name: RPTUN_GET_RESOURCE
 *
 * Description:
 *   Get rptun resouce
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rsc  - Resource pointer to get
 *   role - Role to get
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_GET_RESOURCE(d,r,rl) ((d)->ops->get_resource(d,r,rl))

/****************************************************************************
 * Name: RPTUN_GET_SHAREMEM
 *
 * Description:
 *   Get rptun share memory
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   shm - Share memory pointer to get
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_GET_SHAREMEM(d,s) ((d)->ops->get_sharemem(d,s))

/****************************************************************************
 * Name: RPTUN_NOTIFY
 *
 * Description:
 *   Notify remote core there is a message to get.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   vqid - Message to notify
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_NOTIFY(d,v) ((d)->ops->notify(d,v))

/****************************************************************************
 * Name: RPTUN_REGISTER_CALLBACK
 *
 * Description:
 *   Attach to receive a callback when something is received on RPTUN
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to be called when something has been received
 *   arg      - A caller provided value to return with the callback
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_REGISTER_CALLBACK(d,c,a) ((d)->ops->register_callback(d,c,a))

/****************************************************************************
 * Name: RPTUN_UNREGISTER_CALLBACK
 *
 * Description:
 *   Detach RPTUN callback
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#define RPTUN_UNREGISTER_CALLBACK(d) ((d)->ops->register_callback(d,NULL,NULL))

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef int (*rptun_callback_t)(void *arg, uint32_t vqid);

struct rptun_dev_s;
struct rptun_ops_s
{
  const char *(*get_cpuname)(struct rptun_dev_s *dev);
  int (*get_resource)(struct rptun_dev_s *dev,
                    struct rsc_table_info *rsc, uint32_t *role);
  int (*get_sharemem)(struct rptun_dev_s *dev, struct proc_shm *shm);
  int (*notify)(struct rptun_dev_s *dev, uint32_t vqid);
  int (*register_callback)(struct rptun_dev_s *dev,
                    rptun_callback_t callback, void *arg);
  int (*ioctl)(struct rptun_dev_s *dev, int cmd, unsigned long arg);
};

struct rptun_dev_s
{
  const struct rptun_ops_s *ops;
};

struct __attribute__((aligned(8))) rptun_rsc_loadstart_s
{
  struct resource_table    rsc_tbl_hdr;
  unsigned int             offset[2];
  struct fw_rsc_trace      log_trace;
  struct fw_rsc_vdev       rpmsg_vdev;
  struct fw_rsc_vdev_vring rpmsg_vring0;
  struct fw_rsc_vdev_vring rpmsg_vring1;
  unsigned int             buf_size;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int rptun_register(struct rptun_dev_s *dev, int minor);
int rptun_unregister(int minor);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPTUN */
#endif /* __INCLUDE_NUTTX_RPTUN_RPTUN_H */
