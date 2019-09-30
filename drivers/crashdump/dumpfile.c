/****************************************************************************
 * drivers/crashdump/dumpfile.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Yuan Zhang<zhangyuan@fishsemi.com>
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
#include <nuttx/version.h>
#include <nuttx/fs/fs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include <nuttx/crashdump/dumpfile.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DUMPFILE_PATH_LEN         64
#define MAX_DUMPFILE_NAME_LEN         32
#define MAX_DUMPFILE_STRING_LEN       256

#define DUMPFIE_MAGIC                 0x7f7f7f7f

#define DUMPFILE_OVERRUN_MESSAGE      "[truncated]\n"
#define DUMPFILE_OVERRUN_SIZE         13

#define DUMPFILE_STATUS_NULL          0
#define DUMPFILE_STATUS_INIT          1
#define DUMPFILE_STATUS_READY         2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dumpfile_s
{
  int             status;
  const char      *cpuname;
  char            *buffer;
  int             size;
};

struct dumpfile_head_s
{
  int             magic;
  int             position;
  char            name[MAX_DUMPFILE_NAME_LEN];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dumpfile_s g_dump_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dumpfile_clean
 ****************************************************************************/

static int dumpfile_clean(void)
{
  FAR DIR *dir;
  FAR struct dirent *file;
  char *file_abspath;
  int unlink_pos, file_pos;

  dir = opendir(CONFIG_CRASH_DUMPFILE_MOUNTPOINT);
  if (!dir)
    {
      syslog(LOG_ERR, "open dump log dir fail: %d\n", get_errno());
      return ERROR;
    }

  unlink_pos = 2;
  file_pos = 2;

  while ((file = readdir(dir)) != NULL)
    {
      if (strcmp(file->d_name, ".") == 0 || strcmp(file->d_name, "..") == 0)
        {
          continue;
        }

      file_pos++;
      if ((file_pos - unlink_pos) >= CONFIG_CRASH_DUMPFILE_NUM)
        {
          unlink_pos++;
        }
    }

  file_abspath = (char *)zalloc(MAX_DUMPFILE_PATH_LEN);
  for (file_pos = 2, seekdir(dir, 2); file_pos < unlink_pos; file_pos++)
    {
      file = readdir(dir);
      snprintf(file_abspath, MAX_DUMPFILE_PATH_LEN, "%s/%s", \
        CONFIG_CRASH_DUMPFILE_MOUNTPOINT, file->d_name);
      unlink(file_abspath);
    }

  free(file_abspath);
  closedir(dir);

  return OK;
}

/****************************************************************************
 * Name: dumpfile_open
 ****************************************************************************/

static FAR FILE *dumpfile_open(const char *file_name)
{
  FAR FILE *file = NULL;
  char *abs_name;

  abs_name = (char *)zalloc(MAX_DUMPFILE_PATH_LEN);
  if (!abs_name)
    {
      syslog(LOG_ERR, "get file name memory fail\n");
      return NULL;
    }

  strcat(abs_name, CONFIG_CRASH_DUMPFILE_MOUNTPOINT);
  strcat(abs_name, "/");
  strcat(abs_name, file_name);

  file = fopen(abs_name, "w");

  free(abs_name);

  return file;
}

/****************************************************************************
 * Name: dumpfile_ram_flush
 ****************************************************************************/

static int dumpfile_ram_flush(struct dumpfile_head_s *head)
{
  FAR FILE *file;
  char *data_ptr;

  file = dumpfile_open(head->name);
  if (!file)
    {
      syslog(LOG_ERR, "create dump log file fail\n");
      return ERROR;
    }

  data_ptr = ((char *)(head + 1));

  fwrite(data_ptr, 1, head->position, file);

  fclose(file);

  return OK;
}


/****************************************************************************
 * Name: dumpfile_ram_write
 ****************************************************************************/

static int dumpfile_ram_write(struct dumpfile_s *priv, char *data, int len)
{
  struct dumpfile_head_s *file_head;
  int used_size, left_size;
  char *data_ptr;

  file_head = (struct dumpfile_head_s *)priv->buffer;

  used_size = sizeof(struct dumpfile_head_s) + file_head->position;
  left_size = priv->size - used_size;

  data_ptr = (char*)file_head + used_size;

  if (left_size < len)
    {
      memcpy(data_ptr, data, left_size);
      memcpy(data_ptr + left_size - DUMPFILE_OVERRUN_SIZE, \
        DUMPFILE_OVERRUN_MESSAGE, DUMPFILE_OVERRUN_SIZE);
      file_head->position += left_size;
    }
  else
    {
      memcpy(data_ptr, data, len);
      file_head->position += len;
    }

  return OK;
}

/****************************************************************************
 * Name: dumpfile_ram_start
 ****************************************************************************/

static void dumpfile_ram_start(struct dumpfile_s *priv)
{
  struct dumpfile_head_s *file_head;
  time_t curr_time;
  struct tm *gm_time;
  char *string_buf;
  int len;

  file_head = (struct dumpfile_head_s *)priv->buffer;
  file_head->magic = DUMPFIE_MAGIC;

  curr_time = time(NULL);
  gm_time = gmtime(&curr_time);

  snprintf(file_head->name, MAX_DUMPFILE_NAME_LEN, \
          "%s-%d-%d-%d-%d-%d-%d.txt", priv->cpuname, \
          gm_time->tm_year + 1900, gm_time->tm_mon + 1, gm_time->tm_mday, \
          gm_time->tm_hour, gm_time->tm_min, gm_time->tm_sec);

  string_buf = (char *)zalloc(MAX_DUMPFILE_STRING_LEN);
  if (!string_buf)
    {
      syslog(LOG_ERR, "malloc string_buf fail\n");
      return;
    }

  len = snprintf(string_buf, MAX_DUMPFILE_STRING_LEN, "NuttX %s %s\n", \
                 CONFIG_VERSION_STRING, CONFIG_VERSION_BUILD);
  dumpfile_ram_write(priv, string_buf, len);
  free(string_buf);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dumpfile_initialize
 ****************************************************************************/

int dumpfile_initialize(const char *cpuname, char *buf, int size)
{
  struct dumpfile_s *priv = &g_dump_priv;
  struct dumpfile_head_s *file_head;

  priv->cpuname = cpuname;
  priv->buffer = buf;
  priv->size = size;

  file_head = (struct dumpfile_head_s *)priv->buffer;
  if (file_head->magic == DUMPFIE_MAGIC)
    {
      dumpfile_clean();
      dumpfile_ram_flush(file_head);
    }

  memset(file_head, 0x00, sizeof(struct dumpfile_head_s));

  priv->status = DUMPFILE_STATUS_INIT;

  return OK;
}

/****************************************************************************
 * Name: dumpfile_write
 ****************************************************************************/

int dumpfile_write(FAR const char *fmt, ...)
{
  struct dumpfile_s *priv = &g_dump_priv;
  char *string_buf;
  va_list ap;
  int len;

  if (priv->status == DUMPFILE_STATUS_NULL)
    {
      syslog(LOG_ERR, "dump happened before initialize, discard it\n");
      return ERROR;
    }

  if (priv->status == DUMPFILE_STATUS_INIT)
    {
      dumpfile_ram_start(priv);
      priv->status = DUMPFILE_STATUS_READY;
    }

  string_buf = (char *)zalloc(MAX_DUMPFILE_STRING_LEN);
  if (!string_buf)
    {
      syslog(LOG_ERR, "malloc string_buf fail\n");
      return ERROR;
    }

  va_start(ap, fmt);
  len = vsnprintf(string_buf, MAX_DUMPFILE_STRING_LEN, fmt, ap);
  va_end(ap);

  dumpfile_ram_write(priv, string_buf, len);

  free(string_buf);

  return OK;
}

