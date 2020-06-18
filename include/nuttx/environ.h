/****************************************************************************
 * include/nuttx/environ.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_ENVIRON_H
#define __INCLUDE_NUTTX_ENVIRON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#ifndef CONFIG_DISABLE_ENVIRON

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback function used with env_foreach() */

typedef CODE int (*env_foreach_t)(FAR void *arg, FAR const char *pair);

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

/****************************************************************************
 * Name: env_foreach
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 *   This is an internal OS function and should not be used by applications.
 *
 * Input Parameters:
 *   group - The task group containing environment array to be searched.
 *   cb    - The callback function to be invoked for each environment
 *           variable.
 *
 * Returned Value:
 *   Zero if the all environment variables have been traversed.  A non-zero
 *   value means that the callback function requested early termination by
 *   returning a nonzero value.
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

struct task_group_s;  /* Forward reference */
int env_foreach(FAR struct task_group_s *group, env_foreach_t cb,
                FAR void *arg);

/****************************************************************************
 * Name: get_environ_ptr_global
 *
 * Description:
 *   Return a pointer to the assign thread specific environ variable.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   A pointer to the per-thread environ variable.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR char **get_environ_ptr_global(void);

/****************************************************************************
 * Name: getenv_global
 *
 * Description:
 *   The getenv_global() function searches the environment list for a string that
 *   matches the string pointed to by name.
 *
 * Input Parameters:
 *   name - The name of the variable to find.
 *
 * Returned Value:
 *   The value of the valiable (read-only) or NULL on failure
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

FAR char *getenv_global(FAR const char *name);

/****************************************************************************
 * Name: putenv_global
 *
 * Description:
 *   The putenv_global() function adds or changes the value of environment variables.
 *   The argument string is of the form name=value. If name does not already
 *   exist in  the  environment, then string is added to the environment. If
 *   name does exist, then the value of name in the environment is changed to
 *   value.
 *
 * Input Parameters:
 *   name=value string describing the environment setting to add/modify
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int putenv_global(FAR const char *string);

/****************************************************************************
 * Name: clearenv_global
 *
 * Description:
 *   The clearenv_global() function clears the environment of all name-value pairs
 *   and sets the value of the external variable environ to NULL.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int clearenv_global(void);

/****************************************************************************
 * Name: setenv_global
 *
 * Description:
 *   The setenv_global() function adds the variable name to the environment with the
 *   specified 'value' if the varialbe 'name" does not exist. If the 'name'
 *   does exist in the environment, then its value is changed to 'value' if
 *   'overwrite' is non-zero; if 'overwrite' is zero, then the value of name
 *   unaltered.
 *
 * Input Parameters:
 *   name - The name of the variable to change
 *   value - The new value of the variable
 *   overwrite - Replace any existing value if non-zero.
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int setenv_global(FAR const char *name, FAR const char *value, int overwrite);

/****************************************************************************
 * Name: unsetenv_global
 *
 * Description:
 *   The unsetenv_global() function deletes the variable name from the environment.
 *
 * Input Parameters:
 *   name - The name of the variable to delete
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int unsetenv_global(FAR const char *name);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __INCLUDE_NUTTX_ENVIRON_H */
