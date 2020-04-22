/****************************************************************************
 * crypto/secutils.c
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Author:  Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <stdint.h>

#include <assert.h>
#include <syslog.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_CRYPTO_MANAGER_ENTROPY_DEVRANDOM
static struct file randomFile;
static bool randomInitialized = false;
#endif

/****************************************************************************
 * Name: secfail
 *
 * Description: React on a security failure. This code is a ecurity assertion,
 * it can only be called if some secure operation was affected by a CPU fault
 * that changed a critical RAM variable in our back, eg, loop counter in secure
   copy routines.
 * The user has to provide a rather strict reaction, like rebooting the CPU,
 * triggering a signal, or something like that. Reaction should be different
 * if the failure happens in a process (kill the process) or in the kernel.
 ****************************************************************************/

void secfail(const char *msg)
{
#ifdef CONFIG_CRYPTO_SECFAILURE_BOARD
  board_security_failure(msg);
#else
  syslog(LOG_EMERG, "SECFAIL: %s\n", msg);
  while(1) {assert(false); }
  //TODO generate a system reset if possible.
#endif
}

/****************************************************************************
 * Name: secrand
 *
 * Description: Provide quality randomness. Do not use rand() from
 * the libc. A Hardware RNG is preferred. If something fails, no return code
 * is provided, but the general security failure is triggered instead.
 ****************************************************************************/

void secrand(FAR uint8_t *dest, uint32_t len)
{
#ifdef CONFIG_CRYPTO_MANAGER_ENTROPY_DEVRANDOM
  int fd;

  /* Get a handle on /dev/random */

  if(!randomInitialized)
    {
      fd = open("/dev/random", O_RDONLY);
      if(fd < 0)
        {
          secfail("Cannot initialize entropy\n");
        }
      if(file_detach(fd, &randomFile) < 0)
        {
          secfail("Cannot initialize entropy\n");
        }
      randomInitialized = true;
    }

  if(file_read(&randomFile, dest, len) != len)
    {
      secfail("Cannot get entropy\n");
    }
#else
#error define a strong entropy source
#endif
}

/****************************************************************************
 * Name: secrandbyte
 *
 * Description: Provide one byte of quality randomness. Do not use rand() from
 * the libc. A Hardware RNG is preferred.
 ****************************************************************************/

uint8_t secrandbyte(void)
{
  uint8_t buf;
  secrand(&buf, 1);
  return buf;
}

/* Sequential copy routines with constant execution time */

/****************************************************************************
 * Name: secmemcpy
 ****************************************************************************/

void secmemcpy(FAR uint8_t *dest, FAR uint8_t *src, uint32_t len)
{
  uint32_t i;
  for(i=0;i<len;i++)
    {
      dest[i] = src[i];
    }
}

/****************************************************************************
 * Name: secmemset
 ****************************************************************************/

void secmemset(FAR uint8_t *dest, uint8_t val, uint32_t len)
{
  uint32_t i;
  for(i=0;i<len;i++)
    {
      dest[i] = val;
    }
}

/****************************************************************************
 * Name: secmemclr
 *
 * Description: Use this instead of value zero to avoid attacker controlled
 * parameters.
 ****************************************************************************/

void secmemclr(FAR uint8_t *dest, uint32_t len)
{
  uint32_t i;
  for(i=0;i<len;i++)
    {
      dest[i] = 0x00;
    }
}

/****************************************************************************
 * Name: secmemor
 *
 * Description: Compute the OR of all bytes in the buffer. Can be used to
 * detect that ALL bytes are zero or AT LEAST one byte is not zero.
 ****************************************************************************/

uint8_t secmemor(FAR uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint8_t comb = 0x00;
  for(i=0;i<len;i++)
    {
      comb |= buf[i];
    }
  return comb;
}

/****************************************************************************
 * Name: secmemand
 *
 * Description: Compute the AND of all bytes in the buffer. Can be used to
 * detect that ALL bytes are 0xFF or AT LEAST one byte is not 0xFF.
 ****************************************************************************/

uint8_t secmemand(FAR uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint8_t comb = 0xFF;
  for(i=0;i<len;i++)
    {
      comb &= buf[i];
    }
  return comb;
}

/****************************************************************************
 * Name: secmemorofxor
 *
 * Description: Combine two buffers using XOR and accumulate all one bits.
 * Can be used to compare two buffers for equality.
 ****************************************************************************/

uint8_t secorofxor(FAR uint8_t *bufa, FAR uint8_t *bufb, uint32_t len)
{
  uint32_t i;
  uint8_t comb = 0x00;
  for(i=0;i<len;i++)
    {
      comb |= (bufa[i] ^ bufb[i]);
    }
  return comb;
}

/* The following operations operates on fixed size buffers and operate on the
 * required bytes in random order, so that an attacker cannot correlate
 * different executions of the routines to attempt a side channel attack.
 * Performance is not worse than a dumb bytewise linear copy.
 */

/****************************************************************************
 * Name: seccpy8
 *
 * Description: Copy 8 bytes in random order.
 ****************************************************************************/

void seccpy8(FAR uint8_t *dest, FAR uint8_t *src)
{
  int start;
  int pos  =  secrandbyte() & 0x07; //There are 8 possible start positions
  int step = (secrandbyte() & 0x07) | 1; //there are 4 possible odd skips (1..7)
  start    = pos;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  dest[pos] = src[pos]; pos = (pos + step) & 0x07;
  if(pos != start) secfail("Fault attack!");
}

/****************************************************************************
 * Name: seccpy16
 *
 * Description: Copy 16 bytes in random order.
 ****************************************************************************/

void seccpy16(FAR uint8_t *dest, FAR uint8_t *src)
{
  int start;
  int pos  =  secrandbyte() & 0x0F; //There are 16 possible start positions
  int step = (secrandbyte() & 0x0F) | 1; //there are 8 possible odd skips (1..15)
  start    = pos;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = src[pos]; pos = (pos + step) & 0x0F;
  if(pos != start) secfail("Fault attack!");
}

/****************************************************************************
 * Name: secxor8
 *
 * Description: XOR two 8 bytes buffers, in random order.
 ****************************************************************************/

void secxor8(FAR uint8_t *dest, FAR uint8_t *srca, FAR uint8_t *srcb)
{
  int start;
  int pos  =  secrandbyte() & 0x07; //There are 8 possible start positions
  int step = (secrandbyte() & 0x07) | 1; //there are 4 possible odd skips (1..7)
  start    = pos;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x07;
  if(pos != start) secfail("Fault attack!");
}


/****************************************************************************
 * Name: secxor16
 *
 * Description: XOR two 16 bytes buffers, in random order.
 ****************************************************************************/

void secxor16(FAR uint8_t *dest, FAR uint8_t *srca, FAR uint8_t *srcb)
{
  int start;
  int pos  =  secrandbyte() & 0x0F; //There are 16 possible start positions
  int step = (secrandbyte() & 0x0F) | 1; //there are 8 possible odd skips (1..15)
  start    = pos;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  dest[pos] = srca[pos] ^ srcb[pos]; pos = (pos + step) & 0x0F;
  if(pos != start) secfail("Fault attack!");
}

