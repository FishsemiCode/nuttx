/****************************************************************************
 * include/nuttx/crypto/manager.h
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
#ifndef __INCLUDE_NUTTX_CRYPTO_MANAGER_H
#define __INCLUDE_NUTTX_CRYPTO_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Overview
 *
 * The cryptography manager is a character device that manages cryptography in
 * the full system. Each time the /dev/cryptoman device is opened, a crypto
 * environment is created. Operations and objects executed in the context of an
 * environment are isolated (except for non volatile shared keys).
 * This API is inspired from both PKCS#11 and Javacard, but has been heavily
 * simplified for an embedded environment.
 *
 * Module implementation
 *
 * At boot time, one or more crypto modules can be registered with the crypto
 * manager for later use.
 * A crypto module can be a software implementation running on the CPU executing
 * NuttX, but it can also be an external hardware module, possibly providing
 * extra security.
 * An environment provides a way to enumerate modules and choose the required
 * one.
 * Multiple modules are useful because not all modules are able to provide all
 * algorithms.
 *
 * Only a single module can be active at any moment in any given environment.
 * When an environment is created, no module is active yet. A module can be made
 * active by using CRYPTOIOC_MODSELECT.
 * Note that some modules require authentication (PIN or challenge response
 * based) before being usable.
 *
 * Keys
 *
 * Cryptographic keys are sensitive material. They must be handled with care.
 * This API allows the use of keys without manipulating their value, using key
 * identifiers. Keys can be of two types :
 *   Persistent keys are retained after an environment is destroyed. They can be
 *     present before any environment is opened. They can be stored in any way,
 *     system FLASH (usually protected), external module, etc.
 *   Volatile keys can be defined in the context of an environment and they are
 *     deleted after the environment is destroyed. They can be retained in any
 *     way, system RAM, external module, etc.
 * Most keys (symmetric keys) have only one component, which is the value of the
 * key.
 * Composite keys like RSA keys have multiple components. If the key is
 * generated at creation, all components are defined at once, else they have to
 * be imported one at a time using CRYPTOIOC_KEYUPDATE. A composite key cannot
 * be used before all its components have been set.
 *
 * Algorithms
 *
 * Each environment can be setup to use a cryptographic algorithm. Once the
 * algorithm is defined in the environment, it can be used to process data.
 * Possible operations are:
 *   Ciphering and deciphering, including a padding for the last bytes
 *   Message Digests
 *   Generation and verification of Signatures
 *   Generation of random data
 */
 
/* Crypto module flags */

#define CRYPTOMOD_FLAG_AUTHREQUIRED (1<<0)
#define CRYPTOMOD_FLAG_INSECURE     (1<<1)

/* Key flags used in CRYPTOIOC_KEYINFO */

#define KEYFLAG_TYPE_SHIFT    0
#define KEYFLAG_TYPE_MASK     (255 << KEYFLAG_TYPE_SHIFT)
#define KEYFLAG_TYPE_AES      (  1 << KEYFLAG_TYPE_SHIFT)
#define KEYFLAG_TYPE_DES      (  2 << KEYFLAG_TYPE_SHIFT)
#define KEYFLAG_TYPE_RSAPUB   (  3 << KEYFLAG_TYPE_SHIFT)
#define KEYFLAG_TYPE_RSAPRIV  (  4 << KEYFLAG_TYPE_SHIFT)
#define KEYFLAG_TYPE_RSACRT   (  5 << KEYFLAG_TYPE_SHIFT)

#define KEYFLAG_VOLATILE      (1<<12) /* Key is volatile */
#define KEYFLAG_SECURE        (1<<13) /* Key is stored in a secure hw module */
#define KEYFLAG_GENERATED     (1<<14) /* Key has been generated by the module */
#define KEYFLAG_READABLE      (1<<15) /* Key can be exported */

#define KEYFLAG_NCOMP_SHIFT   8
#define KEYFLAG_NCOMP_MASK    ( 15 << KEYFLAG_NCOMP_SHIFT)

/* This mask only selects the flags that can be written in keycreate:
 *   - key type
 *   - volatility
 * The other flags are generated by the cryptomodule and returned, but cannot be set.
 */

#define KEYFLAG_MASK_WRITE    ( (255 << 8) | (1 << 12) )
/* Key component indexes for symmetric keys */

#define KEYCOMP_MAIN     0

/* Key component indexes for Public and Private RSA keys */

#define KEYCOMP_RSAEXP   0
#define KEYCOMP_RSAMOD   1

/* Key component indexes for Private RSA CRT keys */

#define KEYCOMP_CRTP     0 /* CRT component P */
#define KEYCOMP_CRTQ     1 /* CRT component Q */
#define KEYCOMP_CRTDP    2 /* CRT component dP */
#define KEYCOMP_CRTDQ    3 /* CRT component dQ */
#define KEYCOMP_CRTQI    4 /* CRT component qinv */

/* Algorithm setup */

enum algoparam_e
{

    /* These generic parameters are managed by the cryptomanager */

    ALGPARAM_PADDING,   /* Define which padding is used when the TOTAL data
                         * length is not a multiple of the algorithm block size.
                         */

    /* These parameters are forwarded to the low level crypto module */

    ALGPARAM_IV,        /* Used to define the initial vector for CBC and CTR
                         * calculations. This is generic, but for efficiency,
                         * this parameter is forwarded to the low level crypto
                         * modules, to allow them to process multiple blocks in
                         * one call.
                         */

    ALGPARAM_AESGCM_AAD,/* Additional authenticated data for AES GCM */
    ALGPARAM_AESGCM_TAG /* Authentication tag returned by AES GCM */
};

/* Padding options, used with block based algorithms to allow ciphering of
 * arbitrary sized payloads */

enum algpadding_e
{
    ALG_PADDING_NONE,     /* No padding, data must have the correct length */
    ALG_PADDING_ZERO,     /* Only zero bits, NOT REMOVABLE, avoid using this! */
    ALG_PADDING_X923,     /* Zero padding, length in last byte */
    ALG_PADDING_ISO78164, /* one "1" bit followed by zero bits */
    ALG_PADDING_PKCS5,    /* encode padding length in padding */
    ALG_PADDING_ISO10126  /* Random bytes, last one is length.
                           * This padding is  NOT DETERMINISTIC and should not
                           * be used!
                           */

};

/* Usable algorithms. They include the operating mode but the used padding
 * method is set independently. */

enum alg_e
{
    ALG_NONE, /* Not an algorithm, value 0 is reserved */
    
    /* Ciphering */
    
    ALG_AES_ECB, /* Any AES key length */
    ALG_AES_CBC,
    ALG_AES_CTR,
    ALG_AES_GCM,
    ALG_DES_ECB, /* Includes all variants of DES, single, 2keys, 3keys */
    ALG_DES_CBC,
    ALG_DES_CTR,
    ALG_SM4_ECB,
    ALG_SM4_CBC,
    ALG_SM4_CTR,
    ALG_SM4_GCM,
    
    /* Hashes */
    
    ALG_SHA1,
    ALG_SHA256,
    ALG_SHA384,
    ALG_SHA512,
    ALG_SHA3,
    ALG_BLAKE2S,
    ALG_SM3,
    ALG_IPV4HDR_CHECKSUM,
    
    /* Signature */
    
    ALG_HMAC_SHA1,
    ALG_HMAC_SHA256,
    ALG_HMAC_SHA384,
    ALG_HMAC_SHA512,
    ALG_RSA,         /* To be clarified, padding, etc */
    
    /* Key wrapping */
    ALG_UNWRAP_NONE, /* Value is just copied in the key component */
    ALG_UNWRAP_DEVICEDEPENDENT, /* Just provide a custom value with no 
                                 * official definition */
    
    /* Random generation */

    ALG_RND_BUILTIN, /* Random generator built into the module */
    ALG_RND_BLAKE2S,

    ALG_PRIVATE
};

/* Algorithm operations */

enum algop_e
{
    ALGOP_CIPHER,
    ALGOP_DECIPHER,
    ALGOP_SIGN,
    ALGOP_VERIFY
};

#endif /* __INCLUDE_NUTTX_CRYPTO_MANAGER_H */

