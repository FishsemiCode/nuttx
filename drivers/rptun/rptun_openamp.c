/****************************************************************************
 * drivers/rptun/rptun_openamp.c
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

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/wqueue.h>

#include <errno.h>

#include <openamp/open_amp.h>

#ifndef MAX
#  define MAX(a,b) ((a)>(b)?(a):(b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_openamp_s
{
  struct rptun_dev_s           *dev;
  struct remote_proc           *rproc;
  struct proc_shm              shm;
  struct work_s                work_start;
  struct work_s                work_vring;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static void rptun_openamp_start_work(void *arg);
static void rptun_openamp_vring_work(void *arg);
static int  rptun_openamp_callback(void *arg, uint32_t vqid);
static int  rptun_openamp_enable_interrupt(struct proc_intr *intr);
static void rptun_openamp_notify(struct hil_proc *proc,
                            struct proc_intr *intr_info);
static int  rptun_openamp_init(struct hil_proc *proc);
static void rptun_openamp_release(struct hil_proc *proc);
static int  rptun_openamp_resource_init(
                            struct rptun_openamp_s *priv, int cpu_id);

/************************************************************************************
 * Public Data
 ************************************************************************************/

struct firmware_info fw_table[] = {};
int fw_table_size = sizeof(fw_table)/sizeof(fw_table[0]);

/************************************************************************************
 * Private Data
 ************************************************************************************/

static struct hil_platform_ops rptun_openamp_ops =
{
  .enable_interrupt = rptun_openamp_enable_interrupt,
  .notify           = rptun_openamp_notify,
  .initialize       = rptun_openamp_init,
  .release          = rptun_openamp_release,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void rptun_openamp_start_work(void *arg)
{
  struct hil_proc *proc = arg;
  struct rptun_openamp_s *priv = proc->pdata;
  int cpu_id = proc->cpu_id;

  remoteproc_resource_deinit(priv->rproc);
  rptun_openamp_resource_init(priv, cpu_id);
}

static void rptun_openamp_vring_work(void *arg)
{
  struct hil_proc *proc = arg;
  hil_notified(proc, RPTUN_NOTIFY_ALL);
}

static int rptun_openamp_callback(void *arg, uint32_t vqid)
{
  struct hil_proc *proc = arg;
  struct rptun_openamp_s *priv = proc->pdata;
  int ret;

  if (vqid == RPTUN_NOTIFY_START)
    {
      ret = work_queue(HPWORK, &priv->work_start,
                            rptun_openamp_start_work, proc, 0);
    }
  else
   {
      ret = work_queue(HPWORK, &priv->work_vring,
                            rptun_openamp_vring_work, proc, 0);
   }

  return ret;
}

static int rptun_openamp_enable_interrupt(struct proc_intr *intr)
{
  struct hil_proc *proc = intr->data;
  struct rptun_openamp_s *priv = proc->pdata;

  return RPTUN_REGISTER_CALLBACK(priv->dev,
                            rptun_openamp_callback, proc);
}

static void rptun_openamp_notify(struct hil_proc *proc,
                            struct proc_intr *intr_info)
{
  struct rptun_openamp_s *priv = proc->pdata;

  RPTUN_NOTIFY(priv->dev, RPTUN_NOTIFY_ALL);
}

static int rptun_openamp_init(struct hil_proc *proc)
{
  struct rptun_openamp_s *priv = proc->pdata;

  memcpy(&proc->sh_buff, &priv->shm, sizeof(priv->shm));

  proc->rsc_io = metal_io_get_region();
  proc->sh_buff.io = metal_io_get_region();
  proc->vdev.intr_info.data = proc;
  proc->vdev.vring_info[0].intr_info.data = proc;
  proc->vdev.vring_info[0].io = metal_io_get_region();
  proc->vdev.vring_info[1].intr_info.data = proc;
  proc->vdev.vring_info[1].io = metal_io_get_region();

  return 0;
}

static void rptun_openamp_release(struct hil_proc *proc)
{
  struct rptun_openamp_s *priv = proc->pdata;

  RPTUN_UNREGISTER_CALLBACK(priv->dev);
}

static void rptun_openamp_alloc_mem(
                            struct rptun_openamp_s *priv,
                            struct rsc_table_info *rsc_info)
{
  struct rptun_rsc_s *rsc_tab = (struct rptun_rsc_s *)rsc_info->rsc_tab;
  uint32_t vring0_size, vring1_size, tb_size, max_align;
  volatile uint32_t *check_ver;

  /* RPMSG_MASTER need wait remote core load resource table,
   * then calculating address with following order:
   *    resource table
   *    vring0
   *    vring1
   *    share mem buffer
   * Attention:
   * The buffers' total size shouldn't bigger then the original
   * size which given by user
   */

  check_ver = (volatile uint32_t *)&rsc_tab->rsc_tbl_hdr.ver;
  while (*check_ver != 1);

  vring0_size = vring_size(rsc_tab->rpmsg_vring0.num, rsc_tab->rpmsg_vring0.align);
  vring1_size = vring_size(rsc_tab->rpmsg_vring1.num, rsc_tab->rpmsg_vring1.align);

  max_align   = MAX(rsc_tab->rpmsg_vring0.align, rsc_tab->rpmsg_vring1.align);
  tb_size     = (sizeof(*rsc_tab) + max_align - 1) & ~(max_align - 1);

  rsc_tab->rpmsg_vring0.da       = (uint32_t)rsc_tab + tb_size;
  rsc_tab->rpmsg_vring0.notifyid = 1;
  rsc_tab->rpmsg_vring1.da       = rsc_tab->rpmsg_vring0.da + vring0_size;
  rsc_tab->rpmsg_vring1.notifyid = 2;

  priv->shm.start_addr           = (void *)(rsc_tab->rpmsg_vring1.da + vring1_size);
  priv->shm.size                 = rsc_tab->buf_size * (rsc_tab->rpmsg_vring0.num
                                                       + rsc_tab->rpmsg_vring1.num);
}

static int rptun_openamp_resource_init(
                            struct rptun_openamp_s *priv, int cpu_id)
{
  struct rsc_table_info rsc_info;
  struct remote_proc *rproc;
  struct hil_proc *proc;
  const char *cpu_name;
  static int id = 0;
  uint32_t role;
  int ret;

  ret = RPTUN_GET_RESOURCE(priv->dev, &rsc_info, &role);
  if (ret)
    {
      return ret;
    }

  if (role == RPMSG_MASTER)
    {
      rptun_openamp_alloc_mem(priv, &rsc_info);
    }

  if (cpu_id == -1)
    {
      cpu_id = id++;
    }

  cpu_name = RPTUN_GET_CPUNAME(priv->dev);

  proc = hil_create_proc_with_name(&rptun_openamp_ops, cpu_id, cpu_name, NULL);
  if (!proc)
    {
      return -ENOMEM;
    }

  proc->pdata = priv;

  ret = remoteproc_resource_init(&rsc_info, proc, NULL, NULL, NULL, &rproc, role);
  if (ret)
    {
      hil_delete_proc(proc);
      return ret;
    }

  priv->rproc = rproc;
  return 0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int rptun_openamp_register(struct rptun_dev_s *dev)
{
  struct metal_init_params params = METAL_INIT_DEFAULTS;
  struct rptun_openamp_s *priv;
  int ret;

  ret = metal_init(&params);
  if (ret)
    {
      return ret;
    }

  priv = kmm_zalloc(sizeof(struct rptun_openamp_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->dev = dev;

  RPTUN_NOTIFY(dev, RPTUN_NOTIFY_START);

  ret = rptun_openamp_resource_init(priv, -1);
  if (ret)
    {
      kmm_free(priv);
      return ret;
    }

  return 0;
}
