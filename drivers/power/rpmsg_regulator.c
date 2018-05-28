/****************************************************************************
 * drivers/power/rpmsg_regulator.c
 * Upper-half, common core driver for pmic regulators.
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

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/power/consumer.h>

#include <openamp/open_amp.h>

#include "internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_REGULATOR_CHANNEL_NAME "rpmsg-regulator"

#define RPMSG_REGULATOR_GET         1
#define RPMSG_REGULATOR_PUT         2
#define RPMSG_REGULATOR_ENABLE      3
#define RPMSG_REGULATOR_DISABLE     4
#define RPMSG_REGULATOR_GET_VOLTAGE 5
#define RPMSG_REGULATOR_SET_VOLTAGE 6

/* Debug ********************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_regulator_header
{
  uint32_t command : 31;
  uint32_t response : 1;
  int32_t result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct rpmsg_regulator_get
{
  struct rpmsg_regulator_header header;
  uint64_t id;
  char name[0];
} end_packed_struct;

#define rpmsg_regulator_put rpmsg_regulator_get
#define rpmsg_regulator_enable rpmsg_regulator_get
#define rpmsg_regulator_disable rpmsg_regulator_get

begin_packed_struct struct rpmsg_regulator_get_voltage
{
  struct rpmsg_regulator_header header;
  uint64_t id;
  int32_t uV;
} end_packed_struct;

begin_packed_struct struct rpmsg_regulator_set_voltage
{
  struct rpmsg_regulator_header header;
  uint64_t id;
  int32_t min_uV;
  int32_t max_uV;
} end_packed_struct;

struct rpmsg_regulator_cookie
{
  struct rpmsg_regulator_header *msg;
  sem_t sem;
};

struct rpmsg_regulator_dev
{
  struct rpmsg_channel *channel;
  struct list_node consumer_list;
  sem_t channelready;
  const char *cpu_name;
  bool server;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rpmsg_regulator_dev *g_rdev;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_regulator_device_created(struct remote_device *device, void *priv);
static void rpmsg_regulator_channel_created(struct rpmsg_channel *channel);
static void rpmsg_regulator_channel_destroyed(struct rpmsg_channel *channel);
static void rpmsg_regulator_channel_received(struct rpmsg_channel *channel,
                       void *data, int len, void *priv, unsigned long src);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rpmsg_regulator_device_created(struct remote_device *device, void *priv)
{
  struct rpmsg_regulator_dev *rdev = priv;

  if (!rdev->server && strcmp(rdev->cpu_name, device->proc->cpu_name) == 0) {
      rpmsg_create_channel(device, RPMSG_REGULATOR_CHANNEL_NAME);
  }
}

static void rpmsg_regulator_channel_created(struct rpmsg_channel *channel)
{
  struct rpmsg_regulator_dev *rdev = rpmsg_get_callback_privdata(channel->name);

  if (rdev) {
      rdev->channel = channel;
      rpmsg_set_privdata(channel, rdev);
      nxsem_post(&rdev->channelready);
  }
}

static void rpmsg_regulator_channel_destroyed(struct rpmsg_channel *channel)
{
  struct rpmsg_regulator_dev *rdev = rpmsg_get_callback_privdata(channel->name);

  if (rdev) {
      rdev->channel = NULL;
      nxsem_wait_uninterruptible(&rdev->channelready);
  }
}

static void rpmsg_regulator_channel_received(struct rpmsg_channel *channel,
                       void *data, int len, void *priv, unsigned long src)
{
#if defined(CONFIG_RPMSG_REGULATOR_CLIENT)
  struct rpmsg_regulator_header *header = data;
  struct rpmsg_regulator_cookie *cookie =
                       (struct rpmsg_regulator_cookie *)(uintptr_t)header->cookie;

  if (cookie && header->response) {
      memcpy(cookie->msg, data, len);
      nxsem_post(&cookie->sem);
      return;
  }

#else
  struct rpmsg_regulator_header *header = data;
  struct rpmsg_regulator_dev *rdev = rpmsg_get_privdata(channel);
  struct regulator *regulator;
  int ret;

  if (header->command == RPMSG_REGULATOR_GET) {
      struct rpmsg_regulator_get *msg = data;

      regulator = regulator_get(NULL, msg->name);
      if (regulator) {
          header->result = 0;

          list_initialize(&regulator->list_rpmsg);
          list_add_tail(&regulator->list_rpmsg, &rdev->consumer_list);
      } else {
          pwrerr("%s fail to get %s\n", __func__, msg->name);
          header->result = -ENXIO;
      }

      msg->id = (uintptr_t)regulator;

  } else if (header->command == RPMSG_REGULATOR_PUT) {
      struct rpmsg_regulator_put *msg = data;

      regulator = (struct regulator *)(uintptr_t)msg->id;
      DEBUGASSERT(regulator != NULL);

      list_delete(&regulator->list_rpmsg);
      regulator_put(regulator);

      header->result = 0;

  } else if (header->command == RPMSG_REGULATOR_ENABLE) {
      struct rpmsg_regulator_enable *msg = data;

      regulator = (struct regulator *)(uintptr_t)msg->id;
      DEBUGASSERT(regulator != NULL);

      ret = regulator_enable(regulator);
      if (!ret)
          header->result = 0;
      else
          header->result = -ENXIO;

  } else if (header->command == RPMSG_REGULATOR_DISABLE) {
      struct rpmsg_regulator_disable *msg = data;

      regulator = (struct regulator *)(uintptr_t)msg->id;
      DEBUGASSERT(regulator != NULL);

      ret = regulator_disable(regulator);
      if (!ret)
          header->result = 0;
      else
          header->result = -ENXIO;

  } else if (header->command == RPMSG_REGULATOR_SET_VOLTAGE) {
      struct rpmsg_regulator_set_voltage *msg = data;

      regulator = (struct regulator *)(uintptr_t)msg->id;
      DEBUGASSERT(regulator != NULL);

      ret = regulator_set_voltage(regulator, msg->min_uV, msg->max_uV);
      if (!ret)
          header->result = 0;
      else
          header->result = -ENXIO;

  } else if (header->command == RPMSG_REGULATOR_GET_VOLTAGE) {
      struct rpmsg_regulator_get_voltage *msg = data;

      regulator = (struct regulator *)(uintptr_t)msg->id;
      DEBUGASSERT(regulator != NULL);

      ret = regulator_get_voltage(regulator);
      if (ret > 0)
          header->result = 0;
      else
          header->result = -ENXIO;

      msg->uV = ret;
  }

  header->response = 1;
  rpmsg_send(channel, data, len);

  return;

#endif
}

#if defined(CONFIG_RPMSG_REGULATOR_CLIENT)
static int rpmsg_regulator_msg_send_recv(struct rpmsg_regulator_dev *rdev,
                       uint32_t command, struct rpmsg_regulator_header *msg, int len)
{
  struct rpmsg_regulator_cookie cookie = {0};
  int ret;

  if (!rdev->channel) {
      nxsem_wait_uninterruptible(&rdev->channelready);
      nxsem_post(&rdev->channelready);
  }

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_setprotocol(&cookie.sem, SEM_PRIO_NONE);

  cookie.msg = msg;

  msg->command = command;
  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;

  ret = rpmsg_send(rdev->channel, msg, len);
  if (ret < 0) {
      pwrerr("%s rpmsg send failed\n", __func__);
      goto err;
  }

  ret = nxsem_wait_uninterruptible(&cookie.sem);
  if (ret < 0) {
      pwrerr("%s nxsem wait failed %d\n", __func__, ret);
      goto err;
  }

  return msg->result;

err:
  nxsem_destroy(&cookie.sem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_RPMSG_REGULATOR_CLIENT)

/****************************************************************************
 * Name: regulator_get
 *
 * Description:
 *   Lookup and obtain a reference to a regulator.
 *
 * Input parameters:
 *   dev - The device for regulator consumer, NULL is ok.
 *   id - Supply name or the regulator ID.
 *
 * Returned value:
 *    A struct regulator pointer on success or NULL on failure
 *
 ****************************************************************************/

struct regulator *regulator_get(void *dev, const char *id)
{
  struct regulator *regulator = NULL;
  struct rpmsg_regulator_get *msg;
  uint32_t len;
  int ret;

  DEBUGASSERT(g_rdev != NULL);

  if (id == NULL) {
      pwrerr("%s get() with no identifier\n", __func__);
      return NULL;
  }

  len = sizeof(*msg) + strlen(id) + 1;
  msg = kmm_zalloc(len);
  if (!msg) {
      pwrerr("%s fail to get memory\n", __func__);
      return NULL;
  }

  strcpy(msg->name, id);

  ret = rpmsg_regulator_msg_send_recv(g_rdev, RPMSG_REGULATOR_GET,
                   (struct rpmsg_regulator_header *)msg, len);
  if (ret < 0) {
      pwrerr("%s fail to get %d\n", __func__, ret);
      goto err;
  }

  regulator = kmm_zalloc(sizeof(struct regulator));
  if (regulator == NULL) {
      pwrerr("%s failed to get memory\n", __func__);
      goto err;
  }

  regulator->id = msg->id;
  kmm_free(msg);

  return regulator;

err:
  kmm_free(msg);
  return NULL;
}

/****************************************************************************
 * Name: regulator_put
 *
 * Description:
 *   Free the regulator resource.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *
 ****************************************************************************/

void regulator_put(struct regulator *regulator)
{
  struct rpmsg_regulator_put *msg;
  uint32_t len;
  int ret;

  DEBUGASSERT(g_rdev != NULL);

  if (regulator == NULL) {
      pwrerr("%s regulator is NULL\n", __func__);
      return;
  }

  len = sizeof(*msg);
  msg = kmm_zalloc(len);
  if (!msg) {
      pwrerr("%s fail to get memory\n", __func__);
      goto out;
  }

  msg->id = regulator->id;

  ret = rpmsg_regulator_msg_send_recv(g_rdev, RPMSG_REGULATOR_PUT,
                   (struct rpmsg_regulator_header *)msg, len);
  if (ret < 0) {
      pwrerr("%s fail to put %d\n", __func__, ret);
  }

  kmm_free(msg);

out:
  kmm_free(regulator);
}

/****************************************************************************
 * Name: regulator_enable
 *
 * Description:
 *   Enable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_enable(struct regulator *regulator)
{
  struct rpmsg_regulator_enable *msg;
  uint32_t len;
  int ret = -EINVAL;

  DEBUGASSERT(g_rdev != NULL);

  if (regulator == NULL) {
      pwrerr("%s regulator is NULL\n", __func__);
      return ret;
  }

  len = sizeof(*msg);
  msg = kmm_zalloc(len);
  if (!msg) {
      pwrerr("%s fail to get memory\n", __func__);
      return ret;
  }

  msg->id = regulator->id;

  ret = rpmsg_regulator_msg_send_recv(g_rdev, RPMSG_REGULATOR_ENABLE,
                   (struct rpmsg_regulator_header *)msg, len);
  if (ret < 0) {
      pwrerr("%s fail to enable %d\n", __func__, ret);
  }

  kmm_free(msg);
  return ret;
}

/****************************************************************************
 * Name: regulator_disable
 *
 * Description:
 *   Disable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_disable(struct regulator *regulator)
{
  struct rpmsg_regulator_disable *msg;
  uint32_t len;
  int ret = -EINVAL;

  DEBUGASSERT(g_rdev != NULL);

  if (regulator == NULL) {
      pwrerr("%s regulator is NULL\n", __func__);
      return ret;
  }

  len = sizeof(*msg);
  msg = kmm_zalloc(len);
  if (!msg) {
      pwrerr("%s fail to get memory\n", __func__);
      return ret;
  }

  msg->id = regulator->id;

  ret = rpmsg_regulator_msg_send_recv(g_rdev, RPMSG_REGULATOR_DISABLE,
                   (struct rpmsg_regulator_header *)msg, len);
  if (ret < 0) {
      pwrerr("%s fail to disable %d\n", __func__, ret);
  }

  kmm_free(msg);
  return ret;
}

/****************************************************************************
 * Name: regulator_set_voltage
 *
 * Description:
 *   Set the regulator output voltage.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *   min_uV - Minimum required voltage in uV
 *   max_uV - Maximum acceptable voltage in uV
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_set_voltage(struct regulator *regulator, int min_uV, int max_uV)
{
  struct rpmsg_regulator_set_voltage *msg;
  uint32_t len;
  int ret = -EINVAL;

  DEBUGASSERT(g_rdev != NULL);

  if (regulator == NULL) {
      pwrerr("%s regulator is NULL\n", __func__);
      return ret;
  }

  len = sizeof(*msg);
  msg = kmm_zalloc(len);
  if (!msg) {
      pwrerr("%s fail to get memory\n", __func__);
      return ret;
  }

  msg->id = regulator->id;
  msg->min_uV = min_uV;
  msg->max_uV = max_uV;

  ret = rpmsg_regulator_msg_send_recv(g_rdev, RPMSG_REGULATOR_SET_VOLTAGE,
                   (struct rpmsg_regulator_header *)msg, len);
  if (ret < 0) {
      pwrerr("%s fail to set voltage %d\n", __func__, ret);
  }

  kmm_free(msg);
  return ret;
}

/****************************************************************************
 * Name: regulator_get_voltage
 *
 * Description:
 *   Obtain the regulator output voltage.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_get_voltage(struct regulator *regulator)
{
  struct rpmsg_regulator_get_voltage *msg;
  uint32_t len;
  int ret = -EINVAL;

  DEBUGASSERT(g_rdev != NULL);

  if (regulator == NULL) {
      pwrerr("%s regulator is NULL\n", __func__);
      return ret;
  }

  len = sizeof(*msg);
  msg = kmm_zalloc(len);
  if (!msg) {
      pwrerr("%s fail to get memory\n", __func__);
      return ret;
  }

  msg->id = regulator->id;

  ret = rpmsg_regulator_msg_send_recv(g_rdev, RPMSG_REGULATOR_GET_VOLTAGE,
                   (struct rpmsg_regulator_header *)msg, len);
  if (ret < 0) {
      pwrerr("%s fail to get voltage %d\n", __func__, ret);
  } else {
      ret = msg->uV;
  }

  kmm_free(msg);
  return ret;
}

#endif /* CONFIG_RPMSG_REGULATOR_CLIENT */

/****************************************************************************
 * Name: rpmsg_regulator_init
 *
 * Description:
 *
 *   Establish rpmsg channel for the operations of the remote regulator
 *
 * Input Parameters:
 *   cpu_name - current cpu name
 *   server  - client or server
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int rpmsg_regulator_init(const char *cpu_name, bool server)
{
  struct rpmsg_regulator_dev *rdev;
  int ret = -EINVAL;

  rdev = kmm_zalloc(sizeof(struct rpmsg_regulator_dev));
  if (!rdev) {
      pwrerr("%s fail to get memory\n", __func__);
      return ret;
  }

  rdev->cpu_name = cpu_name;
  rdev->server = server;
  list_initialize(&rdev->consumer_list);
  nxsem_init(&rdev->channelready, 0, 0);
  nxsem_setprotocol(&rdev->channelready, SEM_PRIO_NONE);
  g_rdev = rdev;

  ret = rpmsg_register_callback(RPMSG_REGULATOR_CHANNEL_NAME, rdev,
                        rpmsg_regulator_device_created,
                        NULL,
                        rpmsg_regulator_channel_created,
                        rpmsg_regulator_channel_destroyed,
                        rpmsg_regulator_channel_received);
  if (ret < 0) {
      pwrerr("%s fail to register callback\n", __func__);
      goto err;
  }

  return ret;

err:
  kmm_free(rdev);
  return ret;
}
