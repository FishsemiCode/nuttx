/****************************************************************************
 * nuttx/crypto/song_crypto.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li <liguiding@pinecone.net>
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

#ifndef __CIPHER_SONG_CIPHER_H
#define __CIPHER_SONG_CIPHER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CIPHER_INT_STATUS                   0x0000
#define CIPHER_INT_EN                       0x0004
#define CIPHER_BC_REKEY_CTRL                0x0100
#define CIPHER_BC_HKEY_CTRL                 0x0104
#define CIPHER_GHASH_MTBL_CTRL              0x0108
#define CIPHER_EN_BC_EN                     0x010C
#define CIPHER_BC_INFO                      0x0110
#define CIPHER_BC_LP_EN                     0x0114
#define CIPHER_BC_KEY_DAT0                  0x0120
#define CIPHER_BC_KEY_DAT1                  0x0124
#define CIPHER_BC_KEY_DAT2                  0x0128
#define CIPHER_BC_KEY_DAT3                  0x012C
#define CIPHER_BC_KEY_DAT4                  0x0130
#define CIPHER_BC_KEY_DAT5                  0x0134
#define CIPHER_BC_KEY_DAT6                  0x0138
#define CIPHER_BC_KEY_DAT7                  0x013C
#define CIPHER_BC_IV_DAT0                   0x0140
#define CIPHER_BC_IV_DAT1                   0x0144
#define CIPHER_BC_IV_DAT2                   0x0148
#define CIPHER_BC_IV_DAT3                   0x014C
#define CIPHER_GHASH_EY0_VLD                0x0150
#define CIPHER_GHASH_EY0_DAT0               0x0160
#define CIPHER_GHASH_EY0_DAT1               0x0164
#define CIPHER_GHASH_EY0_DAT2               0x0168
#define CIPHER_GHASH_EY0_DAT3               0x016C
#define CIPHER_GHASH_PROD_DAT0              0x0180
#define CIPHER_GHASH_PROD_DAT1              0x0184
#define CIPHER_GHASH_PROD_DAT2              0x0188
#define CIPHER_GHASH_PROD_DAT3              0x018C
#define CIPHER_GHASH_LEN_BLK_VLD            0x0190
#define CIPHER_GHASH_AAD_LEN                0x0194
#define CIPHER_GHASH_TEXT_LEN               0x0198
#define CIPHER_DMA_AAD_RDAT_ADDR            0x019C
#define CIPHER_DMA_AAD_RDAT_LEN             0x01A0
#define CIPHER_DMA_BC_RDAT_ADDR             0x01A4
#define CIPHER_DMA_BC_RDAT_LEN              0x01A8
#define CIPHER_DMA_BC_WDAT_ADDR             0x01AC
#define CIPHER_DMA_BC_WDAT_LEN              0x01B0
#define CIPHER_DMA0_RDAT_CLOSE              0x01B4
#define CIPHER_DMA0_WDAT_CLOSE              0x01B8
#define CIPHER_DMA0_RDAT_PRI                0x01BC
#define CIPHER_DMA0_WDAT_PRI                0x01C0
#define CIPHER_BC_GCM_MAC0                  0x01D0
#define CIPHER_BC_GCM_MAC1                  0x01D4
#define CIPHER_BC_GCM_MAC2                  0x01D8
#define CIPHER_BC_GCM_MAC3                  0x01DC
#define CIPHER_BC_SW_CTRL                   0x0200
#define CIPHER_BC_TEXT_IN0                  0x0210
#define CIPHER_BC_TEXT_IN1                  0x0214
#define CIPHER_BC_TEXT_IN2                  0x0218
#define CIPHER_BC_TEXT_IN3                  0x021C
#define CIPHER_BC_TEXT_OUT0                 0x0220
#define CIPHER_BC_TEXT_OUT1                 0x0224
#define CIPHER_BC_TEXT_OUT2                 0x0228
#define CIPHER_BC_TEXT_OUT3                 0x022C
#define CIPHER_EN_HC_EN                     0x0300
#define CIPHER_SHA_INFO                     0x0304
#define CIPHER_HASH_LP_EN                   0x0308
#define CIPHER_DMA_SH_RDAT_ADDR             0x0310
#define CIPHER_DMA_SH_RDAT_LEN              0x0314
#define CIPHER_DMA_SC_RDAT_ADDR             0x0318
#define CIPHER_DMA_SC_RDAT_LEN              0x031C
#define CIPHER_DMA1_RDAT_CLOSE              0x0320
#define CIPHER_DMA1_RDAT_PRI                0x0324
#define CIPHER_HASH_MAC0                    0x0330
#define CIPHER_HASH_MAC1                    0x0334
#define CIPHER_HASH_MAC2                    0x0338
#define CIPHER_HASH_MAC3                    0x033C
#define CIPHER_HASH_MAC4                    0x0340
#define CIPHER_HASH_MAC5                    0x0344
#define CIPHER_HASH_MAC6                    0x0348
#define CIPHER_HASH_MAC7                    0x034C
#define CIPHER_CHECKSUM_DATA                0x0380
#define CIPHER_WDMA0_BURST_CNT_CLR          0x03FC
#define CIPHER_RDMA_BURST_CNT_CLR           0x0400
#define CIPHER_ENDIAN_CFG                   0x0404
#define CIPHER_CFG_RDMA0_TEXT_IN            0x0408
#define CIPHER_CFG_RDMA1_TEXT_IN            0x040C
#define CIPHER_DBG_RDMA0_ERR_FLG            0x0500
#define CIPHER_DBG_RDMA0_RD0                0x0504
#define CIPHER_DBG_RDMA0_RD1                0x0508
#define CIPHER_DBG_RDMA0_WR                 0x050C
#define CIPHER_DBG_RDMA0_CH                 0x0510
#define CIPHER_DBG_RDMA1_ERR_FLG            0x0520
#define CIPHER_DBG_RDMA1_RD0                0x0524
#define CIPHER_DBG_RDMA1_RD1                0x0528
#define CIPHER_DBG_RDMA1_WR                 0x052C
#define CIPHER_DBG_RDMA1_CH                 0x0530
#define CIPHER_DBG_RDMA_AMBA_RCH            0x0540
#define CIPHER_DBG_RDMA0_BURST_FIRST_ADDR   0x0544
#define CIPHER_DBG_RDMA1_BURST_FIRST_ADDR   0x0548
#define CIPHER_DBG_WDMA0_ERR_FLG            0x0550
#define CIPHER_DBG_WDMA0_WR0                0x0554
#define CIPHER_DBG_WDMA0_WR1                0x0558
#define CIPHER_DBG_WDMA0_RD                 0x055C
#define CIPHER_DBG_WDMA0_CH                 0x0560
#define CIPHER_DBG_WDMA0_INNER_FIFO_ST      0x0564
#define CIPHER_DBG_WDMA0_AMBA_RCH           0x0568
#define CIPHER_DBG_WDMA0_BURST_FIRST_ADDR   0x0570
#define CIPHER_DBG_FIFO_STATUS              0x0580
#define CIPHER_DBG_FIFO_HISTORY             0x0584
#define CIPHER_DBG_LEN_CNT_BF_BRF           0x0590
#define CIPHER_DBG_LEN_CNT_AF_BWF           0x0594
#define CIPHER_DBG_LEN_CNT_BF_HRF           0x0598
#define CIPHER_DBG_LEN_CNT_AF_GHASH         0x059C

#define CIPHER_INT_STATUS_AES_FINISH        0x01
#define CIPHER_INT_STATUS_SHA_FINISH        0x02
#define CIPHER_INT_STATUS_CS_FINISH         0x04
#define CIPHER_INT_STATUS_SM4_FINISH        0x08
#define CIPHER_INT_STATUS_SM3_FINISH        0x10
#define CIPHER_INT_STATUS_FIFO_OVF          0x20

#define CIPHER_INT_EN_AES_FINISH            0x01
#define CIPHER_INT_EN_SHA_FINISH            0x02
#define CIPHER_INT_EN_CS_FINISH             0x04
#define CIPHER_INT_EN_SM4_FINISH            0x08
#define CIPHER_INT_EN_SM3_FINISH            0x10
#define CIPHER_INT_EN_FIFO_OVF              0x20

#define CIPHER_BC_INFO_MODE_MASK            0x0F
#define CIPHER_BC_INFO_MODE_ECB             0x00
#define CIPHER_BC_INFO_MODE_CBC             0x01
#define CIPHER_BC_INFO_MODE_CTR             0x02
#define CIPHER_BC_INFO_MODE_GCM             0x05
#define CIPHER_BC_INFO_MODE_CAL             0x06

#define CIPHER_BC_INFO_DIR_MASK             0x10
#define CIPHER_BC_INFO_DIR_ENC              0x10
#define CIPHER_BC_INFO_DIR_DEC              0x00

#define CIPHER_BC_INFO_KEY_SIZE_MASK        0x60
#define CIPHER_BC_INFO_KEY_SIZE_BIT128      0x00
#define CIPHER_BC_INFO_KEY_SIZE_BIT192      0x20
#define CIPHER_BC_INFO_KEY_SIZE_BIT256      0x40

#define CIPHER_BC_INFO_ALG_MASK             0x80
#define CIPHER_BC_INFO_ALG_AES              0x00
#define CIPHER_BC_INFO_ALG_SM4              0x80

#define CIPHER_SHA_INFO_SHA1                0x00
#define CIPHER_SHA_INFO_SHA256              0x01

#define ALG_MODE_CAL                        0xFF
#define ALGPARAM_KEYUPDATE                  0xFF

#define SONG_CRYPTO_ALGN_UP(x, n)           (((uintptr_t)(x) + (n-1)) & ~(n-1))

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)                     (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum alg_class_e
{
  ALG_CLASS_AES,
  ALG_CLASS_CS,
  ALG_CLASS_SHA,
  ALG_CLASS_SM3,
  ALG_CLASS_SM4,
};

struct alg_head_s
{
  uintptr_t base;
  uint32_t  algclass;
  uint32_t  algid;
  uint32_t  algmode;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint32_t song_crypto_read(void *alg_, uint32_t offset);
extern void song_crypto_write(void *alg_, uint32_t offset,
                              uint32_t val);
extern int song_crypto_bcinfo(void *alg_, uint32_t algid, uint32_t algmode,
                              uint32_t keylen);
extern int song_crypto_interrupt(void *alg_);

extern void *song_crypto_common_init(struct alg_head_s *head);
extern int song_crypto_common_uninit(FAR void *alg_);
extern int song_crypto_common_ioctl(FAR void *alg_, uint32_t param,
                                    uint32_t length, FAR uint8_t *value);
extern int song_crypto_common_isr(FAR void *alg_);
extern int song_crypto_common_exe(FAR void *alg_, bool first, bool last,
                                  uint32_t inlen, FAR uint8_t *indata,
                                  uint32_t outlen, FAR uint8_t *outdata,
                                  uint32_t keylen, FAR uint8_t *keydata);

extern void *song_crypto_gcm_init(struct alg_head_s *head);
extern int song_crypto_gcm_uninit(FAR void *alg_);
extern int song_crypto_gcm_ioctl(FAR void *alg_, uint32_t param,
                                uint32_t length, FAR uint8_t *value);
extern int song_crypto_gcm_isr(FAR void *alg_);
extern int song_crypto_gcm_exe(FAR void *alg_, bool first, bool last,
                               uint32_t inlen, FAR uint8_t *indata,
                               uint32_t outlen, FAR uint8_t *outdata,
                               uint32_t keylen, FAR uint8_t *keydata);

extern void *song_crypto_sha_init(struct alg_head_s *head);
extern int song_crypto_sha_uninit(FAR void *alg_);
extern int song_crypto_sha_ioctl(FAR void *alg_, uint32_t param,
                                uint32_t length, FAR uint8_t *value);
extern int song_crypto_sha_isr(FAR void *alg_);
extern int song_crypto_sha_exe(FAR void *alg_, bool first, bool last,
                               uint32_t inlen, FAR uint8_t *indata,
                               uint32_t outlen, FAR uint8_t *outdata,
                               uint32_t keylen, FAR uint8_t *keydata);

extern void *song_crypto_sm3_init(struct alg_head_s *head);
extern int song_crypto_sm3_uninit(FAR void *alg_);
extern int song_crypto_sm3_ioctl(FAR void *alg_, uint32_t param,
                                uint32_t length, FAR uint8_t *value);
extern int song_crypto_sm3_isr(FAR void *alg_);
extern int song_crypto_sm3_exe(FAR void *alg_, bool first, bool last,
                               uint32_t inlen, FAR uint8_t *indata,
                               uint32_t outlen, FAR uint8_t *outdata,
                               uint32_t keylen, FAR uint8_t *keydata);

extern void *song_crypto_cs_init(struct alg_head_s *head);
extern int song_crypto_cs_uninit(FAR void *alg_);
extern int song_crypto_cs_ioctl(FAR void *alg_, uint32_t param,
                                uint32_t length, FAR uint8_t *value);
extern int song_crypto_cs_isr(FAR void *alg_);
extern int song_crypto_cs_exe(FAR void *alg_, bool first, bool last,
                              uint32_t inlen, FAR uint8_t *indata,
                              uint32_t outlen, FAR uint8_t *outdata,
                              uint32_t keylen, FAR uint8_t *keydata);

#endif /* __CIPHER_SONG_CIPHER_H */
