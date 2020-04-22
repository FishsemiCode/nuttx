/****************************************************************************
 * include/nuttx/crypto/api.h
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
#ifndef __INCLUDE_NUTTX_CRYPTO_API_H
#define __INCLUDE_NUTTX_CRYPTO_API_H

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct cryptoman_context_s;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cryptoman_libinit(void);

FAR struct cryptoman_context_s *cryptoman_sessalloc(void);

int cryptoman_sessfree(FAR struct cryptoman_context_s *ctx);

int cryptoman_modinfo(FAR struct cryptoman_context_s *ctx,
                      bool first,
                      char name[32],
                      uint32_t alg,
                      FAR uint32_t *flags);

/****************************************************************************
 * Name: cryptoman_modselect
 * Description: switch the current module. If something fails, the previous
 *   module is not lost.
 ****************************************************************************/

int cryptoman_modselect(FAR struct cryptoman_context_s *ctx,
                        char name[32]);

int cryptoman_modauth(FAR struct cryptoman_context_s *ctx,
                      uint32_t step,
                      uint32_t len,
                      FAR uint8_t *data,
                      uint32_t rsplen,
                      FAR uint8_t *response);

int cryptoman_keyinfo(FAR struct cryptoman_context_s *ctx,
                      bool first,
                      FAR uint32_t *id,
                      FAR uint32_t *length,
                      FAR uint32_t *flags);

int cryptoman_keycreate(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t flags,
                        uint32_t length,
                        FAR const uint8_t *value);

int cryptoman_keydelete(FAR struct cryptoman_context_s *ctx,
                        uint32_t id);

int cryptoman_keygen(FAR struct cryptoman_context_s *ctx,
                     uint32_t keyid,
                     uint32_t rngalgid);

int cryptoman_keyupdate(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t component,
                        uint32_t unwrapalg,
                        uint32_t buflen,
                        FAR uint8_t *buffer);

int cryptoman_keyread(FAR struct cryptoman_context_s *ctx,
                      uint32_t id,
                      uint32_t component,
                      uint32_t buflen,
                      FAR uint8_t *buffer);

int cryptoman_keycopy(FAR struct cryptoman_context_s *ctx,
                        uint32_t destid,
                        uint32_t flags,
                        uint32_t srcid);

int cryptoman_alginfo(FAR struct cryptoman_context_s *ctx,
                      uint32_t id,
                      FAR uint32_t *blocklen);

int cryptoman_alginit(FAR struct cryptoman_context_s *ctx,
                      uint32_t algid,
                      uint32_t opmode,
                      uint32_t keyid);

int cryptoman_algsetup(FAR struct cryptoman_context_s *ctx,
                       uint32_t id,
                       uint32_t length,
                       FAR uint8_t *value);

int cryptoman_algstatus(FAR struct cryptoman_context_s *ctx,
                        uint32_t id,
                        uint32_t buflen,
                        FAR uint8_t *buffer);

/****************************************************************************
 * Name: cryptoman_algupdate
 * Description: Manage bytes, with the assumtion that these are NOT the last
 * bytes to be fed to the algorithm. All the input bytes are consumed by the
 * call, and it is an error if there is not enough room in the output to manage
 * all the input bytes. If that happens the algorithm myst be reinitialized.
 * Parameters:
 * ctx       : The crypto context
 * len_in    : number of bytes to be managed, arbitrary
 * data_in   : pointer to the data to be managed
 * len_out   : length of the buffer used to receive the result
 * data_out  : buffer to receive the result
 * Return value: The number of bytes that were written in buf_out, or a negative
 * error
 ****************************************************************************/

int cryptoman_algupdate(FAR struct cryptoman_context_s *ctx,
                        uint32_t len_in,
                        FAR const uint8_t *data_in,
                        uint32_t len_out,
                        FAR uint8_t *data_out);

/****************************************************************************
 * Name: cryptoman_algupdate
 * Description: Manage all or the last bytes to be provided to algorithm. it is
 * an error if all the bytes provided to this function produces result that
 * cannot be stored in the output buffer. If that happens, the algorithm must
 * be reinitialized. Due to padding, the output may be longer than the input. For
 * decryption with padding algorithms, the output buffer MUST be big enough to
 * hold the deciphered padding, even if this padding is removed.
 * Parameters:
 * ctx       : The crypto context
 * len_in    : number of bytes to be managed, arbitrary
 * data_in   : pointer to the data to be managed
 * len_out   : length of the buffer used to receive the result
 * data_out  : buffer to receive the result
 * Return value: The number of bytes that were written in buf_out, or a negative
 * error
 ****************************************************************************/

int cryptoman_algfinish(FAR struct cryptoman_context_s *ctx,
                        uint32_t len_in,
                        FAR const uint8_t *data_in,
                        uint32_t len_out,
                        FAR uint8_t *data_out);

/****************************************************************************
 * Name: cryptomod_register
 *
 * Description:
 *   Register a cryptographic module with the manager.
 *
 ****************************************************************************/

int cryptoman_register(FAR struct cryptomodule_s *module);

#endif /* __INCLUDE_NUTTX_CRYPTO_MODULE_H */

