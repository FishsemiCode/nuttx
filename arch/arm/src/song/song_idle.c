/****************************************************************************
 * arch/arm/src/song/song_idle.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>

#include "chip.h"
#include "nvic.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_cpu_idle(void);
void weak_function up_cpu_normal(void)
{
  up_cpu_idle();
}

void weak_function up_cpu_idle(void)
{
  modifyreg32(NVIC_SYSCON, NVIC_SYSCON_SLEEPDEEP, 0);
}

void weak_function up_cpu_standby(void)
{
  modifyreg32(NVIC_SYSCON, 0, NVIC_SYSCON_SLEEPDEEP);
}

void weak_function up_cpu_sleep(void)
{
  up_cpu_standby();
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_idlepm(void)
{
  enum pm_state_e newstate;
  int ret;

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_CPU_DOMAIN);

  /* Check for state changes */

  /* PM_NORMAL means can't enter any lp_state, but CPU is idle, so enter wfe */

  if (newstate == PM_NORMAL)
    {
      up_cpu_normal();
    }
  else
    {
      /* Then force the global state change */

      ret = pm_changestate(PM_CPU_DOMAIN, newstate);

      /* changestate successful and state is different, update oldstate and config */

      if (ret == 0)
        {
          /* MCU-specific power management logic */

          switch (newstate)
            {
            case PM_IDLE:
              up_cpu_idle();
              break;

            case PM_STANDBY:
              up_cpu_standby();
              break;

            case PM_SLEEP:
              up_cpu_sleep();
              break;

            default:
              break;
            }
        }
      /* changestate fail (assume backoff oldstate successfully), don't need to config (configed before), just enter wfe */
    }
}
#else
#  define up_idlepm() up_cpu_idle()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed
 *   when their is no other ready-to-run task.  This is processor
 *   idle time and will continue until some interrupt occurs to
 *   cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g.,
 *   this is where power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
  irqstate_t flags;

  /* Clear event reg, exceptions before should not prevent entering sleeping */
  __asm__ __volatile__("sev");
  __asm__ __volatile__("wfe");

  /* Perform IDLE mode power management */

  flags = enter_critical_section();
  up_idlepm();

  /* If there is interrupt in up_idlepm(), can't enter sleeping */

  __asm__ __volatile__("wfe");

  /* Quit wfe, include PM_IDLE and PM_STANDBY/PM_SLEEP interrupted, restore PM_NORMAL */

  pm_changestate(PM_CPU_DOMAIN, PM_NORMAL);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_pminitialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management subsystem.
 *   This function must be called *very* early in the initialization sequence
 *   *before* any other device drivers are initialized (since they may
 *   attempt to register with the power management subsystem).
 *
 * Input parameters:
 *   None.
 *
 * Returned value:
 *    None.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void up_pminitialize(void)
{
  /* Then initialize the power management subsystem proper */

  pm_initialize();
}
#endif
