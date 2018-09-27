/************************************************************************************
 * arch/csky/src/ck802/up_mdelay.c
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


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds.
 *   *** NOT multi-tasking friendly ***
 *
 * ASSUMPTIONS:
 *   The setting CONFIG_BOARD_LOOPSPERMSEC has been calibrated
 *
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds)
{
    volatile unsigned int i;
    volatile unsigned int j;

    for (i = 0; i < milliseconds; i++) {
        for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++) {
        }
    }
}
