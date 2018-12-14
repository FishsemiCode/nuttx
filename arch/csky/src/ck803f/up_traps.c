/****************************************************************************
 * arch/csky/src/ck803f/up_traps.c
 *
 *
  * Copyright (C) 2015 The YunOS Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/module.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <syslog.h>
#include <nuttx/board.h>

#include <stdlib.h>
#include <stdio.h>

extern int ckcpu_vsr_table[64];
extern void trap(void);
extern void alignment(void);
extern void hw_vsr_interrupt(void);
extern int csky_get_vec_num(void);
extern void show_registers(uint32_t *regs);
extern void alignment_c(uint32_t *regs);
extern void mm_leak_dump(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifdef CONFIG_CRASH_LOG_PATH
#define SHOWREG crashlog
#else
#define SHOWREG syslog
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_exception_init
 *
 * Description:
 *
 ****************************************************************************/

void up_exception_init(void)
{
    return;
}

/****************************************************************************
 * Name: up_trap_c
 *
 * Description:
 *          once the system crashed, it will enter this function and collect the crashlog.
 *     .
 *return:   void
 ****************************************************************************/
void up_trap_c(uint32_t *regs)
{
    int vector;

    vector = csky_get_vec_num();

    switch (vector) {
    case VEC_ALIGN:
    case VEC_ACCESS:
    case VEC_ZERODIV:
    case VEC_ILLEGAL:
    case VEC_PRIV:
    case VEC_TRACE:
    case VEC_BREAKPOINT:
    case VEC_UNRECOVER:
    case VEC_SOFTRESET:
    case VEC_FAUTOVEC:
    case VEC_HWACCEL:
    case VEC_TLBFATAL:
    case VEC_TLBMISS:
    case VEC_TLBMODIFIED:
    case VEC_TRAP0:
    case VEC_TRAP1:
    case VEC_TRAP2:
    case VEC_TRAP3:
    case VEC_TLBINVALIDL:
    case VEC_TLBINVALIDS:
    case VEC_PRFL:
    case VEC_FPE:
        SHOWREG(LOG_EMERG,"CPU Exception: No.%d\n", vector);
        show_registers(regs);

#ifdef CONFIG_MM_DETECT_ERROR
        mm_leak_dump();
#endif

        //keep system in circulation
        while(1);
    }
}


/****************************************************************************
 * Name: show_registers
 *
 * Description:
 *          this function will show the register parameters.
 *     .
 *return:   void
 ****************************************************************************/

#if 1//def CONFIG_DEBUG
void show_registers(uint32_t *regs)
{
    int      i;
    uint8_t  *tp;
    uint32_t *sp;

    SHOWREG(LOG_EMERG,"Exception context [%p]:\n", regs);
    SHOWREG(LOG_EMERG,"EPSR: %08x\n", regs[REG_PSR]);
    SHOWREG(LOG_EMERG,"EPC : %08x\n", regs[REG_PC]);
    SHOWREG(LOG_EMERG,"r14: %08x r15: %08x\n",
           regs[REG_R14], regs[REG_R15]);
    SHOWREG(LOG_EMERG,"r0 : %08x r1 : %08x r2 : %08x r3 : %08x\n",
           regs[REG_R0], regs[REG_R1], regs[REG_R2], regs[REG_R3]);
    SHOWREG(LOG_EMERG,"r4 : %08x r5 : %08x r6 : %08x r7 : %08x\n",
           regs[REG_R4], regs[REG_R5], regs[REG_R6], regs[REG_R7]);
    SHOWREG(LOG_EMERG,"r8 : %08x r9 : %08x r10: %08x r11: %08x\n",
           regs[REG_R8], regs[REG_R9], regs[REG_R10], regs[REG_R11]);
    SHOWREG(LOG_EMERG,"r12: %08x r13: %08x\n",
           regs[REG_R12], regs[REG_R13]);

#ifdef __CSKY_DSP__
    SHOWREG(LOG_EMERG,"hi: %08x lo:  %08x\n",
           regs[REG_HI], regs[REG_LO]);
#endif

    SHOWREG(LOG_EMERG, "CODE:\n");
    tp = ((uint8_t *)regs[REG_PC]) - 0x20;
    tp += ((int)tp % 4) ? 2 : 0;
    for (sp = (uint32_t *)tp, i = 0; (i < 0x40); sp += 4, i += 16) {
        SHOWREG(LOG_EMERG,"%08x: %08x %08x %08x %08x\n",
                (int)(tp + i), sp[0], sp[1], sp[2], sp[3]);
    }

    SHOWREG(LOG_EMERG, "STACK:\n");
    tp = (uint8_t *)regs[REG_SP];
    for (sp = (uint32_t *) tp, i = 0; (i < 0xc0); sp += 4, i += 16) {
        SHOWREG(LOG_EMERG,"%08x: %08x %08x %08x %08x\n",
                (int)(tp + i), sp[0], sp[1], sp[2], sp[3]);
    }
}
#else
void show_registers(uint32_t *regs) {}
#endif
