/****************************************************************************
 * include/nuttx/crypto/module.h
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
#ifndef __INCLUDE_NUTTX_CRYPTO_MODULE_H
#define __INCLUDE_NUTTX_CRYPTO_MODULE_H

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct cryptomodule_ops_s
{
    /* Name: sessioncreate
     * Description: Allocate memory for a new session.
     */

    CODE FAR void* (*session_create)(void);

    /* Name: sessionfree
     * Description: Destroy a session and all volatile keys.
     */

    CODE int       (*session_free)(FAR void* session);
                                   
    /* Name: authenticate
     * Description: Run authentication rounds to unlock the module.
     */

    CODE int       (*session_auth)(FAR void *session, uint32_t step,
                                   uint32_t cmdlen, FAR uint8_t *cmd,
                                   uint32_t rsplen, uint8_t *rsp);
                                   
    /* Name: keycount
     * Description: Return the number of keys in the module, including all
     *              volatile and persistent.
     */

    CODE int       (*key_count)(FAR void *session);

    /* Name: keyinfo
     * Description: Return information about the ith key in the module.
     */

    CODE int       (*key_info)(FAR void *session, int index, FAR uint32_t *id,
                               FAR uint32_t *flags, FAR uint32_t *length);

    /* Name: keycreate
     * Description: Allocate memory for a new volatile key in the current
     *              session.
     */

    CODE int       (*key_create)(FAR void *session, uint32_t id, uint32_t flags,
                                 uint32_t length, FAR uint8_t *value);
                                    
    /* Name: keydelete
     * Description: Remove the indicated key from the session.
     */

    CODE int       (*key_delete)(FAR void *session, uint32_t id);
    
    /* Name: keygen
     * Description: Update the key with random values
     */

    CODE int       (*key_gen)(FAR void *session, uint32_t id, uint32_t rngalg);

    /* Name: keyupdate
     * Description: Define the value of a key component, with optional
     *              unwrapping.
     */

    CODE int       (*key_update)(FAR void *session, uint32_t id,
                                 uint32_t component, uint32_t unwrapalg,
                                 uint32_t len, FAR uint8_t *value);
                                    
    /* Name: keyread
     * Description: Return the value of a key component. Only possible if the
     *              key was created with the proper flags.
     */

    CODE int       (*key_read)(FAR void *session, uint32_t id,
                               uint32_t component, uint32_t buflen,
                               FAR uint8_t *buffer);
    /* Name: keycopy
     * Description: Copy the value of a key in another key. This can be used to
     *              make a key persistent.
     */

    CODE int       (*key_copy)(FAR void *session, uint32_t dest, uint32_t src);

    /* Name: alg_supported
     * Description: Returns -1 only if the alg is not supported, else the block
     *              length of this alg.
     */

    CODE int       (*alg_supported)(FAR void *session, uint32_t algid);
    
    /* Name: alg_init
     * Description: Prepare the module to use this algorithm, using this key
     */

    CODE int       (*alg_init)(FAR void *session, uint32_t algid, uint32_t mode,
                               uint32_t keyid);
    
    /* Name: alg_ioctl
     * Description: Define or retrieve customized parameters for the currently
     * initialized algorithm. Used to implement alg_setup and alg_status.
     * NOTE: Padding options are not managed by crypto modules but by the
     * cryptomanager itself.
     * In case of execution error, the algorithm must be reinitialized.
     */

    CODE int       (*alg_ioctl)(FAR void *session, uint32_t param,
                                uint32_t length, FAR uint8_t *value);
    
    /* Name: alg_update
     * Description: Process one or more data blocks. For block based algorithms,
     * inlen MUST be a multiple of the algorithm block size. For RNG indata and
     * inlen are not significant. Data is only returned for
     * ciphering/deciphering/RNG operations. This function is useful if the
     * input data cannot be obtained in one part. If data to be processed is
     * entirely available in one contiguous buffer, it is expected that the user
     * will call alg_finish() with no prior use of alg_update().
     * In case of execution error, the algorithm must be reinitialized.
     * Returns the number of bytes writen in outdata.
     */

    CODE int       (*alg_update)(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata);

    /* Name: alg_finish
     * Description: Process the last data block(s) of a sequence. For block
     * based algorithms, inlen MUST be a multiple of the algorithm block size.
     * In case of execution error, the algorithm must be reinitialized.
     * Returns the number of bytes writen in outdata.
     */

    CODE int       (*alg_finish)(FAR void *session, uint32_t inlen,
                                 FAR uint8_t *indata, uint32_t outlen,
                                 FAR uint8_t *outdata);
};

struct cryptomodule_s
{
    char                       name[32]; /* Module name */
    uint32_t                   flags;    /* Moudle flags */
    struct cryptomodule_ops_s *ops;      /* Module operations */
};

/* Name: softcrypto_register
 * Description: Register the software only cryptographic module
 */

int softcrypto_register(void);

#endif /* __INCLUDE_NUTTX_CRYPTO_MODULE_H */

