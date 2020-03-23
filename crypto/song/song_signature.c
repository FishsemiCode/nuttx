/****************************************************************************
 * crypto/song/song_signature.c
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

#include <nuttx/crypto/sha256.h>
#include <nuttx/crypto/rsa.h>
#include <nuttx/kmalloc.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SHA256_BLKSIZE      (1024)
#define IMAGE_MAGIC         0x21494350      /* "PCI!" */
#define SIGNATURE_ALIGN     0x400

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vector_table
{
  uint32_t magic;
  uint32_t imgaddr;
  uint32_t imgsize;
};

/****************************************************************************
 * Static Functions
 ****************************************************************************/

static int up_sha256_hash(const char *binfile, uint8_t *digest, uint8_t *rsa)
{
  struct vector_table vector;
  char *binbuf = NULL;
  struct file binf;
  SHA256_CTX ctx;
  int rsa_off;
  int act;
  int ret;

  ret = file_open(&binf, binfile, O_RDONLY);
  if (ret < 0)
    {
      return -EINVAL;
    }

  file_seek(&binf, 0x400, SEEK_SET);
  file_read(&binf, &vector, sizeof(vector));
  if (vector.magic != IMAGE_MAGIC)
    {
      ret = -EINVAL;
      goto out;
    }

  rsa_off = (vector.imgsize + SIGNATURE_ALIGN - 1) & ~(SIGNATURE_ALIGN - 1);

  binbuf = kmm_malloc(SHA256_BLKSIZE);
  if (!binbuf)
    {
      ret = -ENOMEM;
      goto out;
    }

  SHA256_init(&ctx);

  file_seek(&binf, 0, SEEK_SET);
  while (vector.imgsize)
    {
      int try = vector.imgsize > SHA256_BLKSIZE ? SHA256_BLKSIZE : vector.imgsize;
      act = file_read(&binf, binbuf, try);
      if (act <= 0)
        {
          ret = -EINVAL;
          goto out;
        }

      SHA256_update(&ctx, binbuf, act);
      vector.imgsize -= act;
    }

  /* Get digest */

  memcpy(digest, SHA256_final(&ctx), SHA256_DIGEST_SIZE);

  /* Get rsa */

  file_seek(&binf, rsa_off, SEEK_SET);
  file_read(&binf, rsa, RSANUMBYTES);

  ret = 0;
out:
  kmm_free(binbuf);
  file_close(&binf);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int song_signature_verify(const char *binfile, const char *keyfile,
                          int secoff, int keyoff, int keylen)
{
  uint8_t digest[SHA256_DIGEST_SIZE];
  uint8_t *keybuf, *rsabuf;
  struct file keyf;
  int secure;
  int ret;

  keybuf = rsabuf = NULL;

  ret = file_open(&keyf, keyfile, O_RDONLY);
  if (ret < 0)
    {
      return -EINVAL;
    }

  file_seek(&keyf, secoff, SEEK_SET);
  file_read(&keyf, &secure, 1);

  /* Check if skip the signature verify */

  if (secure & 0x1)
    {
      ret = 0;
      goto out;
    }

  keybuf = kmm_malloc(keylen);
  if (!keybuf)
    {
      ret = -ENOMEM;
      goto out;
    }

  file_seek(&keyf, keyoff, SEEK_SET);
  file_read(&keyf, keybuf, keylen);

  rsabuf = kmm_malloc(RSANUMBYTES);
  if (!rsabuf)
    {
      ret = -ENOMEM;
      goto out;
    }

  ret = up_sha256_hash(binfile, digest, rsabuf);
  if (ret)
    {
      goto out;
    }

  ret = !RSA_verify((const RSAPublicKey *)keybuf,
                    rsabuf, RSANUMBYTES,
                    digest, SHA256_DIGEST_SIZE);

out:
  kmm_free(keybuf);
  kmm_free(rsabuf);
  file_close(&keyf);
  return ret;
}
