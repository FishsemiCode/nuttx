/****************************************************************************
 * configs/unicorn/src/sim.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Pinecone <pinecone@pinecone.net>
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

#include <nuttx/config.h>

#ifdef CONFIG_UNICORN_SIM

#ifdef CONFIG_BOARD_INITIALIZE
#include "sim_boot.c"
#endif

#ifdef CONFIG_LIB_BOARDCTL
#include "sim_appinit.c"
#endif

#if defined(CONFIG_BOARD_INITIALIZE) || defined(CONFIG_LIB_BOARDCTL)
#include "sim_bringup.c"
#ifdef CONFIG_LIB_ZONEINFO_ROMFS
#include "sim_zoneinfo.c"
#endif
#endif // defined(CONFIG_BOARD_INITIALIZE) || defined(CONFIG_LIB_BOARDCTL)

#if defined(CONFIG_SIM_X11FB) && defined(CONFIG_SIM_TOUCHSCREEN)
#include "sim_touchscreen.c"
#endif

#ifdef CONFIG_EXAMPLES_GPIO
#ifdef CONFIG_GPIO_LOWER_HALF
#include "sim_ioexpander.c"
#else
#include "sim_gpio.c"
#endif
#endif // CONFIG_EXAMPLES_GPIO

#endif // CONFIG_UNICORN_SIM
