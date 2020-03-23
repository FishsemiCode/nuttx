/****************************************************************************
 * crypto/sha256.c
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
/* sha256.c
**
** Copyright 2013, The Android Open Source Project
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of Google Inc. nor the names of its contributors may
**       be used to endorse or promote products derived from this software
**       without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY Google Inc. ``AS IS'' AND ANY EXPRESS OR
** IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
** EVENT SHALL Google Inc. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
** PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
** OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
** WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
** OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
** ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/crypto/sha256.h>

#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DBL_INT_ADD treats two unsigned ints a and b
 * as one 64-bit integer and adds c to it
 */

#define DBL_INT_ADD(a,b,c)  do { \
                              if ((a) > 0xffffffff - (c)) ++(b); \
                                (a) += (c); \
                            } while (0)
#define ROTLEFT(a,b)        (((a) << (b)) | ((a) >> (32-(b))))
#define ROTRIGHT(a,b)       (((a) >> (b)) | ((a) << (32-(b))))

#define CH(x,y,z)           (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z)          (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x)              (ROTRIGHT(x,2) ^ ROTRIGHT(x,13) ^ ROTRIGHT(x,22))
#define EP1(x)              (ROTRIGHT(x,6) ^ ROTRIGHT(x,11) ^ ROTRIGHT(x,25))
#define SIG0(x)             (ROTRIGHT(x,7) ^ ROTRIGHT(x,18) ^ ((x) >> 3))
#define SIG1(x)             (ROTRIGHT(x,17) ^ ROTRIGHT(x,19) ^ ((x) >> 10))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t k[64] =
{
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
  0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
  0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
  0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
  0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
  0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
  0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
  0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
  0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

/****************************************************************************
 * Static Functions
 ****************************************************************************/

static void sha256_transform(SHA256_CTX *ctx)
{
  uint32_t a, b, c, d, e, f, g, h;
  uint32_t t1, t2, m[64];
  int i, j;

  for (i = 0, j = 0; i < 16; i++, j += 4)
    {
      m[i] = (ctx->buf[j] << 24) | \
             (ctx->buf[j + 1] << 16) | \
             (ctx->buf[j + 2] << 8) | \
             (ctx->buf[j + 3]);
    }

  for ( ; i < 64; i++)
    {
      m[i] = SIG1(m[i-2]) + m[i-7] + SIG0(m[i-15]) + m[i-16];
    }

  a = ctx->state[0];
  b = ctx->state[1];
  c = ctx->state[2];
  d = ctx->state[3];
  e = ctx->state[4];
  f = ctx->state[5];
  g = ctx->state[6];
  h = ctx->state[7];

  for (i = 0; i < 64; i++)
    {
      t1 = h + EP1(e) + CH(e,f,g) + k[i] + m[i];
      t2 = EP0(a) + MAJ(a,b,c);
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }

  ctx->state[0] += a;
  ctx->state[1] += b;
  ctx->state[2] += c;
  ctx->state[3] += d;
  ctx->state[4] += e;
  ctx->state[5] += f;
  ctx->state[6] += g;
  ctx->state[7] += h;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void SHA256_init(SHA256_CTX *ctx)
{
  ctx->count = 0;
  ctx->bitlen[0] = 0;
  ctx->bitlen[1] = 0;
  ctx->state[0] = 0x6a09e667;
  ctx->state[1] = 0xbb67ae85;
  ctx->state[2] = 0x3c6ef372;
  ctx->state[3] = 0xa54ff53a;
  ctx->state[4] = 0x510e527f;
  ctx->state[5] = 0x9b05688c;
  ctx->state[6] = 0x1f83d9ab;
  ctx->state[7] = 0x5be0cd19;
}

void SHA256_update(SHA256_CTX *ctx, const void *data, int len)
{
  int i;
  const uint8_t *p = (const uint8_t *)data;

  for (i = 0; i < len; i++)
    {
      ctx->buf[ctx->count++] = p[i];
      if (ctx->count == 64)
        {
          sha256_transform(ctx);
          DBL_INT_ADD(ctx->bitlen[0], ctx->bitlen[1], 512);
          ctx->count = 0;
        }
    }
}

const uint8_t *SHA256_final(SHA256_CTX *ctx)
{
  uint8_t *p = ctx->buf;
  uint32_t i;

  /* Pad whatever data is left in the buffer. */

  i = ctx->count;
  if (i < 56)
    {
      p[i++] = 0x80;
      while (i < 56)
        p[i++] = 0x00;
    }
  else
    {
      p[i++] = 0x80;
      while (i < 64)
        p[i++] = 0x00;
      sha256_transform(ctx);
      memset(p, 0, 56);
    }

  /* Append to the padding the total message's length in bits and transform */

  DBL_INT_ADD(ctx->bitlen[0], ctx->bitlen[1], ctx->count * 8);
  p[63] = ctx->bitlen[0];
  p[62] = ctx->bitlen[0] >> 8;
  p[61] = ctx->bitlen[0] >> 16;
  p[60] = ctx->bitlen[0] >> 24;
  p[59] = ctx->bitlen[1];
  p[58] = ctx->bitlen[1] >> 8;
  p[57] = ctx->bitlen[1] >> 16;
  p[56] = ctx->bitlen[1] >> 24;
  sha256_transform(ctx);

  for (i = 0; i < 8; i++)
    {
      uint32_t tmp = ctx->state[i];

      *p++ = tmp >> 24;
      *p++ = tmp >> 16;
      *p++ = tmp >> 8;
      *p++ = tmp >> 0;
    }

  return ctx->buf;
}

/* Convenience function */

const uint8_t *SHA256_hash(const void *data, int len, uint8_t *digest)
{
  SHA256_CTX ctx;
  SHA256_init(&ctx);
  SHA256_update(&ctx, data, len);
  memcpy(digest, SHA256_final(&ctx), SHA256_DIGEST_SIZE);
  return digest;
}
