/****************************************************************************
 * drivers/dumpfile.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Zhang Yuan<zhangyuan@fishsemi.com>
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

#include <nuttx/drivers/dumpfile.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DUMPFILE_NAME_LEN     64

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR FILE *g_dump_stream;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dumpfile_open
 ****************************************************************************/

static FAR FILE *dumpfile_open(void)
{
  FAR FILE *file = NULL;
  char *file_name;
  time_t curr_time;
  struct tm *gm_time;

  file_name = (char *)zalloc(MAX_DUMPFILE_NAME_LEN);
  if (!file_name)
    {
      syslog(LOG_ERR, "get file name memory fail\n");
      return NULL;
    }

  strcat(file_name, CONFIG_CRASH_DUMPFILE_MOUNTPOINT);
  strcat(file_name, "/");
  strcat(file_name, CONFIG_LIB_HOSTNAME);

  curr_time = time(NULL);
  gm_time = gmtime(&curr_time);

  snprintf(file_name + strlen(file_name), MAX_DUMPFILE_NAME_LEN, \
          "_dump-%d-%d-%d-%d-%d-%d.txt", \
          gm_time->tm_year + 1900, gm_time->tm_mon + 1, gm_time->tm_mday, \
          gm_time->tm_hour, gm_time->tm_min, gm_time->tm_sec);

  file = fopen(file_name, "w");
  if (!file)
    {
      syslog(LOG_ERR, "create dump log file fail\n");
    }

  free(file_name);

  return file;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dumpfile_write
 ****************************************************************************/

int dumpfile_write(FAR const char *fmt, ...)
{
  int len;
  va_list ap;

  if (!g_dump_stream)
    {
      g_dump_stream = dumpfile_open();

      fprintf(g_dump_stream, "NuttX %s %s\n", \
        CONFIG_VERSION_STRING, CONFIG_VERSION_BUILD);
    }

  va_start(ap, fmt);
  len = vfprintf(g_dump_stream, fmt, ap);
  va_end(ap);

  return len;
}

/****************************************************************************
 * Name: board_crashdump
 ****************************************************************************/

int dumpfile_flush(void)
{
  int ret = OK;

  if (g_dump_stream)
    {
      ret = fsync(g_dump_stream->fs_fd);
    }

  return ret;
}

