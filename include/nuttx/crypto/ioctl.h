/****************************************************************************
 * include/nuttx/crypto/ioctl.h
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
#ifndef __INCLUDE_NUTTX_CRYPTO_IOCTL_H
#define __INCLUDE_NUTTX_CRYPTO_IOCTL_H

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module commands */

/* Command:     CRYPTOIOC_MODINFO
 * Description: Get information about a cryptographic module. When no module
 *              name is provided, information about the first (or next) module
 *              is returned. When a name is provided, information about this
 *              particular module is returned. When an algorithm is provided,
 *              only modules implementing this algorithm are returned.
 * Argument:    A pointer to an instance of type cryptoioc_modinfo_s
 */

#define CRYPTOIOC_MODINFO   _CRYPTOIOC( 1)

/* Command:     CRYPTOIOC_MODSELECT
 * Description: Define the currently used module
 * Argument:    A pointer to an instance of type struct cryptoioc_modselect_s
 */

#define CRYPTOIOC_MODSELECT _CRYPTOIOC( 2)

/* Command:     CRYPTOIOC_MODAUTH
 * Description: Authenticate with a module. Can be a simple PIN authentication
 *              or a multi-step mutual authentication.
 * Argument:    A pointer to an instance of type struct cryptoioc_modauth_s
 */

#define CRYPTOIOC_MODAUTH   _CRYPTOIOC( 3)

/* Key commands */

/* Command:     CRYPTOIOC_KEYINFO
 * Description: Return information about a key in the current module
 * Argument:    A pointer to an instance of type struct cryptoioc_keyinfo_s
 */

#define CRYPTOIOC_KEYINFO   _CRYPTOIOC( 4)

/* Command:     CRYPTOIOC_KEYCREATE
 * Description: Create a new key, including optional initialization
 * Argument:    A pointer to an instance of type struct cryptoioc_keycreate_s
 */

#define CRYPTOIOC_KEYCREATE _CRYPTOIOC( 5)

/* Command:     CRYPTOIOC_KEYDELETE
 * Description: Delete a persistent or volatile key.
 * Argument:    A pointer to an instance of type struct cryptoioc_keydelete_s
 */

#define CRYPTOIOC_KEYDELETE _CRYPTOIOC( 6)

/* Command:     CRYPTOIOC_KEYGENERATE
 * Description: Update an existing key with random values. Key flags are
                updated.
 * Argument:    A pointer to an instance of type struct cryptoioc_keygen_s
 */

#define CRYPTOIOC_KEYGENERATE _CRYPTOIOC( 7)

/* Command:     CRYPTOIOC_KEYUPDATE
 * Description: Set the value of one key component, possibly wrapped. Key flags
                are updated.
 * Argument:    A pointer to an instance of type struct cryptoioc_keyupdate_s
 */

#define CRYPTOIOC_KEYUPDATE _CRYPTOIOC( 8)

/* Command:     CRYPTOIOC_KEYREAD
 * Description: Return the value of a key, if allowed by the key's flags.
 * Argument:    A pointer to an instance of type struct cryptoioc_keyread_s
 */

#define CRYPTOIOC_KEYREAD   _CRYPTOIOC( 9)

/* Command:     CRYPTOIOC_KEYCOPY
 * Description: Copy the value of a key in another key. Both keys must have the
 *              same type and size. Mainly used to make a volatile key
 *              persistent. It is forbidden to copy an unreadable key into a
 *              readable one.
 * Argument:    A pointer to an instance of type struct cryptoioc_keycopy_s
 */

#define CRYPTOIOC_KEYCOPY   _CRYPTOIOC(10)

/* Algorithm commands */

/* Command:     CRYPTOIOC_ALGINIT
 * Description: Prepare the module to start using an algorithm with an optional
                key and operational mode.
 * Argument:    A pointer to an instance of type struct cryptoioc_alginit_s
 */

#define CRYPTOIOC_ALGINIT   _CRYPTOIOC(11)

/* Command:     CRYPTOIOC_ALGINFO
 * Description: Get information about an algorithm.
 * Argument:    A pointer to an instance of type struct cryptoioc_alginfo_s
 */

#define CRYPTOIOC_ALGINFO   _CRYPTOIOC(12)

/* Command:     CRYPTOIOC_ALGSETUP
 * Description: Define additional parameters required to use an algorithm. Some
 *              parameters are managed by the cryptomanager (eg padding) and
 *              some other ones (more specific) are managed by the low level
 *              crypto module.
 * Argument:    A pointer to an instance of type struct cryptoioc_algsetup_s
 */

#define CRYPTOIOC_ALGSETUP  _CRYPTOIOC(13)

/* Command:     CRYPTOIOC_ALGUPDATE
 * Description: Process some data with the current algorithm
 * Argument:    A pointer to an instance of type struct
 *              cryptoioc_algupdatefinish_s
 */

#define CRYPTOIOC_ALGUPDATE _CRYPTOIOC(14)

/* Command:     CRYPTOIOC_ALGFINISH
 * Description: Finish the current processing using the current algorithm
 * Argument:    A pointer to an instance of type struct
 *              cryptoioc_algupdatefinish_s
 */

#define CRYPTOIOC_ALGFINISH _CRYPTOIOC(15)

/* Command:     CRYPTOIOC_ALGSTATUS
 * Description: Retrieve additional parameters produced while using an algorithm
                This is mainly used to provide the computed AES GCM MAC.
 * Argument:    A pointer to an instance of type struct cryptoioc_algstatus_s
 */

#define CRYPTOIOC_ALGSTATUS _CRYPTOIOC(16)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Module commands */

struct cryptoioc_modinfo_s
{
    bool     first;    /* In - Set to TRUE for the first enumeration call */
    char     name[32]; /* In - If all zero and first, return info about the
                        *      first module
                        *      If not zero and first, return info about specific
                        *      module
                        *      If not first, return info about the next module
                        * Out - Module name
                        */
    uint32_t alg;      /* In - Provide a agorithm ID to filter only modules
                        *      implementing this algorithm
                        */
    uint32_t flags;    /* Out - Module flags */
};

struct cryptoioc_modauth_s
{
    uint32_t     step;     /* In - Authentication step, module specific */
    uint32_t     len;      /* In - Length of the authentication data */
    FAR uint8_t *data;     /* In - Authentication data */
    uint32_t     rsplen;   /* In - Length of the buffer used to store response
                            *      authentication data
                            * Out - Length of the response authentication data
                            */
    FAR uint8_t *response; /* Out - Response authentication data */
};

struct cryptoioc_modselect_s
{
    char name[32]; /* In - Name of the requested module */
};

/* Key commands */

struct cryptoioc_keyinfo_s
{
    bool     first;  /* In - Set to TRUE for the first enumeration call */
    uint32_t id;     /* In - If all zero and first, return the first key of the
                      *      module
                      *      If not zero and first, return info about a specific
                      *      key
                      *      If not first, return information about the next key
                      * Out - key identifier
                      */
    uint32_t length; /* Out - key length in bytes */
    uint32_t flags;  /* Out - key characteristics */
};

struct cryptoioc_keycreate_s
{
    uint32_t           id;    /* In - Key identifier, if zero, the key
                               *      identifier is random.
                               */
    uint32_t           flags; /* In - Flags for the key to be created. */
    uint32_t           length;/* In - Key length in bytes. For composite key,
                               *      this is the length of a single component.
                               */
    FAR const uint8_t *value; /* In - If this is not null, points to data used
                               * to initialize the main component of the key.
                               */
};

struct cryptoioc_keydelete_s
{
    uint32_t keyid; /* In - Identifier of the key to be removed */
};

struct cryptoioc_keygen_s
{
    uint32_t keyid;    /* In - Identifier of the key to be generated */
    uint32_t rngalgid; /* In - Identifier of the RNG algorithm used for
                        *      generation
                        */
};

struct cryptoioc_keyupdate_s
{
    uint32_t     id;        /* In - Identifier of the key to update */
    uint32_t     component; /* In - Index of the component to update */
    uint32_t     unwrapalg; /* In - Algorithm used for unwrapping */
    uint32_t     buflen;    /* In - length of the component */
    FAR uint8_t *buffer;    /* In - buffer containing the key value */
};

struct cryptoioc_keyread_s
{
    uint32_t     id;        /* In - Identifier of the key to update */
    uint32_t     component; /* In - Index of the component to update */
    uint32_t     buflen;    /* In - length of the buffer
                             * Out - length of the returned component
                             */
    FAR uint8_t *buffer;    /* Out - buffer containing the key value */
};

struct cryptoioc_keycopy_s
{
    uint32_t destid; /* In - Identifier of the destination key */
    uint32_t flags;  /* In - Flags for the new key(checked for security) */
    uint32_t srcid;  /* In - Identifier of the source key */
};

/* Algorithm commands */

struct cryptoioc_alginfo_s
{
    uint32_t id;       /* In - Algorithm ID */
    uint32_t blocklen; /* Out - Block length in bytes */
};

struct cryptoioc_alginit_s
{
    uint32_t algid; /* In - Algorithm ID */
    uint32_t algop; /* In - Operation mode */
    uint32_t keyid; /* In - Key to be used with this algorithm */
};

struct cryptoioc_algsetup_s
{
    uint32_t     id;     /* In - Type of parameter being defined */
    uint32_t     length; /* In - Length in bytes of the parameter value */
    FAR uint8_t *value;  /* In - Parameter value */
};

struct cryptoioc_algstatus_s
{
    uint32_t     id;     /* In - Type of parameter being requested */
    uint32_t     buflen; /* In - Length in bytes of the buffer provided to store
                          * the requested parameter */
    FAR uint8_t *buffer; /* Out - Parameter value */
};

/* Update and Finish - len_in can be arbitrary, but for block ciphers:
 * - it is more efficient to use multiple of the block size
 * - if the input size is not a multiple of the block size, the output length
 *   is likely to be shorter than the input length.
 */
struct cryptoioc_algupdatefinish_s
{
    uint32_t           len_in;   /* In - Length of the data to be processed */
    FAR const uint8_t *data_in;  /* In - Data to be processed */
    uint32_t           len_out;  /* In - Length of the buffer used for returned
                                  *      data
                                  * Out - Length of the returned data
                                  */
    FAR uint8_t       *data_out; /* Out - Buffer for the returned data */
};

#endif /* __INCLUDE_NUTTX_CRYPTO_IOCTL_H */

