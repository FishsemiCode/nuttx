/****************************************************************************
 * include/nuttx/spi/spi_dw.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Qian Wenfa <qianwenfa@pinecone.net>
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

#ifndef __INCLUDE_SPI_DW_H
#define __INCLUDE_SPI_DW_H

#ifdef CONFIG_SPI_DW

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/spi/spi.h>
#include <nuttx/ioexpander/ioexpander.h>


/****************************************************************************
 * Public Types
 ****************************************************************************/

struct dw_spi_config_s
{
  int bus;
  uintptr_t base;
  uint32_t irq;
  /* dma tx & rx channel, fill -1 if not available */
  uint32_t tx_dma;
  uint32_t rx_dma;
  uint8_t cs_num;
  uint8_t cs_gpio[CONFIG_SPI_DW_MAX_CS];
  uintptr_t mode_ctrl;
  uint32_t mode_sel_bit;
  /* HIGH bits to 32 */
  bool hbits;
  FAR const char *mclk;
  FAR const char *pclk;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

FAR struct spi_dev_s *dw_spi_initialize(FAR const struct dw_spi_config_s *config,
                                        FAR struct ioexpander_dev_s *ioe,
                                        FAR struct dma_dev_s *dma);

void dw_spi_allinitialize(FAR const struct dw_spi_config_s *config, int config_num,
                          FAR struct ioexpander_dev_s *ioe,
                          FAR struct dma_dev_s *dma,
                          FAR struct spi_dev_s **spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI_DW */
#endif /* __INCLUDE_SPI_DW_H */
