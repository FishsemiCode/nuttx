/****************************************************************************
 * arch/arm/src/song/u1_sp_recovery.c
 *
 *   Copyright (C) 2019 Fishsemi Inc. All rights reserved.
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

#if defined(CONFIG_ARCH_CHIP_U1_SP)

#include <string.h>
#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define FLASH_MAP_BASE          0x04000000
#define FLASH_PTABLE_OFFSET     (255 * 8192)
#define PTABLE_FLAG_END         (1 << 0)
#define PTABLE_NAME_LEN         16

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct ptable_entry
{
  char name[PTABLE_NAME_LEN];
  uint64_t offset;
  uint64_t length;
  uint64_t flags;
  uint64_t reserve;
};

struct ptable
{
  char   magic[8];
  char   version[8];
  struct ptable_entry entries[];
};

struct vector_table
{
  uint32_t msp;
  uintptr_t reset_handler;
  uintptr_t vector[254];

  /* offset of magic: 0x400 */

  uint32_t magic;
  uint32_t img_addr;
  uint32_t img_size;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void jump_to_image(uint32_t msp, uint32_t reset_handler)
{
  asm volatile (
      "msr msp, %0    \n"
      "bx %1          \n"
      : : "r"(msp), "r"(reset_handler) :);
}

static const struct ptable_entry *get_partition_entry(const struct ptable *ptbl, const char *name)
{
  int i;

  for (i = 0; !(ptbl->entries[i].flags & PTABLE_FLAG_END); i++)
    {
      if (!strncmp(ptbl->entries[i].name, name, sizeof(ptbl->entries[i].name)))
        {
          return &ptbl->entries[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_jump_to_recovery(void)
{
  struct ptable *ptbl = (struct ptable *)(FLASH_PTABLE_OFFSET + FLASH_MAP_BASE);
  const struct ptable_entry *partition;
  struct vector_table *vtable;

  partition = get_partition_entry(ptbl, "recovery.bin");
  if (!partition)
    {
      return;
    }

  vtable = (struct vector_table *)((uintptr_t)(partition->offset) + FLASH_MAP_BASE);
  jump_to_image(vtable->msp, vtable->reset_handler);
}

#endif /* defined(CONFIG_ARCH_CHIP_U1_SP) */
