/****************************************************************************
 * arch/csky/src/song/song_idle.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
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

#include "song_idle.h"

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

  /* Decide, which power saving level can be obtained */

  newstate = pm_checkstate(PM_IDLE_DOMAIN);

  /* Then force the global state change */

  pm_changestate(PM_IDLE_DOMAIN, newstate);

  /* The change may fail, let's get the final state from power manager */

  newstate = pm_querystate(PM_IDLE_DOMAIN);

  /* MCU-specific power management logic */

  switch (newstate)
    {
    case PM_NORMAL:
      up_cpu_doze();
      break;

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

  flags = up_irq_save();

  /* Perform IDLE mode power management */

  up_idlepm();

  /* Quit WFI, restore the power state */

  up_cpu_normal();
  pm_changestate(PM_IDLE_DOMAIN, PM_RESTORE);

  up_irq_restore(flags);
}

/****************************************************************************
 * Power callback default implementation
 ****************************************************************************/

void weak_function up_cpu_doze(void)
{
  __WAIT();
}

void weak_function up_cpu_idle(void)
{
  __DOZE();
}

void weak_function up_cpu_standby(void)
{
  __STOP();
}

void weak_function up_cpu_sleep(void)
{
  up_cpu_sleep();
}

void weak_function up_cpu_normal(void)
{
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
