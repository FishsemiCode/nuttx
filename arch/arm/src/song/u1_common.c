/****************************************************************************
 * arch/arm/src/song/u1_common.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li <liguiding@fishsemi.com>
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
#include <nuttx/kmalloc.h>

#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>

#include <sys/stat.h>

static inline int ls_specialdir(const char *dir)
{
  /* '.' and '..' directories are not listed like normal directories */

  return (strcmp(dir, ".")  == 0 || strcmp(dir, "..") == 0);
}

static int _up_file_sync(char *dstfile, char *srcfile, bool append, bool sync)
{
  char *buf = NULL;
  int ret = 0;
  int fdr = -1;
  int fdw = -1;
  int oflags;

  fdr = open(srcfile, O_RDONLY);
  if (fdr < 0)
    {
      if (sync && get_errno() == ENOENT)
        {
          return unlink(dstfile);
        }
      else
        {
          return fdr;
        }
    }

  if (sync)
    {
      return ret;
    }

  oflags  = append ? O_APPEND : 0;
  oflags |= O_WRONLY | O_CREAT | O_TRUNC;
  fdw = open(dstfile, oflags);
  if (fdw < 0)
    {
      goto out;
    }

  buf = kmm_malloc(1024);
  if (!buf)
    {
      goto out;
    }

  while (1)
    {
      int nr, nw, ow;

      nr = read(fdr, buf, 1024);
      if (nr == 0)
        {
          break;
        }
      else if (nr < 0)
        {
          ret = nr;
          goto out;
        }

      ow = 0;
      do
        {
          nw = write(fdw, buf + ow, nr);
          if (nw >= 0)
            {
              nr -= nw;
              ow += nw;
            }
          else
            {
              ret = nw;
              goto out;
            }
        }
      while (nr > 0);
    }

out:
  free(buf);
  if (fdr >= 0) close(fdr);
  if (fdw >= 0) close(fdw);
  return ret;
}

int up_file_copy(char *dstfile, char *srcfile)
{
  return _up_file_sync(dstfile, srcfile, false, false);
}

int up_file_copy_append(char *dstfile, char *srcfile)
{
  return _up_file_sync(dstfile, srcfile, true, false);
}

static int _up_folder_sync(char *dstdir, char *srcdir, bool skippatch, bool sync)
{
  DIR *dirp;

  if (sync)
    {
      dirp = opendir(dstdir);
    }
  else
    {
      dirp = opendir(srcdir);
    }

  if (!dirp)
    {
      return -EINVAL;
    }

  for (; ; )
    {
      FAR struct dirent *entryp = readdir(dirp);
      char dstfile[64];
      char srcfile[64];

      if (!entryp)
        {
          /* Finished with this directory */

          break;
        }

      if (skippatch && strstr(entryp->d_name, ".patch"))
        {
          continue;
        }

      snprintf(dstfile, 64, "%s/%s", dstdir, entryp->d_name);
      snprintf(srcfile, 64, "%s/%s", srcdir, entryp->d_name);

      if (DIRENT_ISDIRECTORY(entryp->d_type) &&
          ls_specialdir(entryp->d_name))
        {
          continue;
        }
      else if (DIRENT_ISDIRECTORY(entryp->d_type))
        {
          if (sync)
            {
              if (!opendir(srcfile) && get_errno() == ENOENT)
                {
                  rmdir(dstfile);
                }
              else
                {
                  _up_folder_sync(dstfile, srcfile, skippatch, sync);
                }
            }
          else
            {
              mkdir(dstfile, 0777);
              _up_folder_sync(dstfile, srcfile, skippatch, sync);
            }
        }
      else
        {
          _up_file_sync(dstfile, srcfile, false, sync);
        }
    }

  closedir(dirp);
  return 0;
}

int up_folder_copy(char *dstdir, char *srcdir)
{
  return _up_folder_sync(dstdir, srcdir, false, false);
}

int up_folder_sync(char *dstdir, char *srcdir)
{
  int iret = -1;
  iret = _up_folder_sync(dstdir, srcdir, false, false);
  iret |= _up_folder_sync(dstdir, srcdir, false, true);
  return iret;
}

int up_folder_copy_skippatch(char *dstdir, char *srcdir)
{
  return _up_folder_sync(dstdir, srcdir, true, false);
}
