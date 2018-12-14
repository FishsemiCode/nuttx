/************************************************************************************
 * arch/csky/src/ck803f/up_decodeirq.c
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
 ************************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

extern int csky_get_vec_num(void);

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * up_decodeirq()
 *
 * Description:
 *   The INTC only support autovec interrupt,
 *       vectored interrupt and fast interrupt is not supported.
 *
 * Parameters:
 *   regs - thread's context
 *
 * Return Value:
 *   New thread's context
 *
 ********************************************************************************/

uint32_t *up_decodeirq(uint32_t *regs)
{
    /* Decode the interrupt. We have to do this by search for the lowest numbered
     * non-zero bit in the interrupt status register.
     */

    int irq = NR_IRQS;

    irq = csky_get_vec_num() - 32;
    if (irq == -1) {
        return regs;
    }

    /* Verify that the resulting IRQ number is valid */

    if (irq < NR_IRQS) {
        uint32_t *savestate;

        /* Current regs non-zero indicates that we are processing an interrupt;
         * current_regs is also used to manage interrupt level context switches.
         */

        savestate    = (uint32_t *)current_regs;
        current_regs = regs;

        /* Deliver the IRQ */

        irq_dispatch(irq, regs);

        /* Restore the previous value of current_regs.  NULL would indicate that
         * we are no longer in an interrupt handler.  It will be non-NULL if we
         * are returning from a nested interrupt.
         */

        regs = (uint32_t *)current_regs;
        current_regs = savestate;
    }

    return regs;
}
