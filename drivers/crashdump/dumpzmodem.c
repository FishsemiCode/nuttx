/****************************************************************************
 * drivers/crashdump/dumpzmodem.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
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
#include <nuttx/board.h>

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CRASHDUMP_BLKSIZE       (4 * 1024)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uint32_t _eronly;
extern const uint32_t _eheap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern int sz_main(int argc, FAR char **argv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int crashdump_save(const char *file)
{
  char *cur, *end, *tmp;
  ssize_t ret;
  int fd;

  fd = open(file, O_RDWR | O_CREAT);
  if (fd < 0)
    {
      return fd;
    }

  tmp = malloc(CRASHDUMP_BLKSIZE);
  if (!tmp)
    {
      close(fd);
      return -ENOMEM;
    }

  cur = (char *)&_eronly;
  end = (char *)&_eheap;
  while (cur < end)
    {
      int blk;

      blk = (end - cur) > CRASHDUMP_BLKSIZE ?
            CRASHDUMP_BLKSIZE : (end - cur);
      memcpy(tmp, cur, blk);
      ret = write(fd, tmp, blk);
      if (ret < 0)
        {
          goto end;
        }

      _alert("save addr 0x%x, len 0x%x\n", cur, blk);

      cur += ret;
    }

  _alert("save file %s done\n", file);
  ret = 0;
end:
  free(tmp);
  close(fd);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_crashdump(uintptr_t currentsp, FAR void *tcb,
                     FAR const uint8_t *filename,
                     int lineno)
{
  char *argv[5];
  int ret;

  ret = crashdump_save(CONFIG_CRASH_DUMPZMODEM_FILE);
  if (ret < 0)
    {
      _alert("crashdump save error\n");
      return;
    }

  argv[0] = "sz";
  argv[1] = "-d";
  argv[2] = CONFIG_CRASH_DUMPZMODEM_TTY;
  argv[3] = CONFIG_CRASH_DUMPZMODEM_FILE;
  argv[4] = NULL;
  ret = sz_main(4, argv);
  if (ret < 0)
    {
      _alert("crashdump sz error\n");
      return;
    }

  unlink(CONFIG_CRASH_DUMPZMODEM_FILE);

  _alert("crash file %s, zmodem transfer done\n",
          CONFIG_CRASH_DUMPZMODEM_FILE);
}
