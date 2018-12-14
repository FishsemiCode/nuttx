/*
 * Copyright (C) 2017 C-SKY Microsystems Co., Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * @file     pinmux.h
 * @brief    Header file for the pinmux
 * @version  V1.0
 * @date     23. August 2017
 ******************************************************************************/
#ifndef __ASSEMBLY__
#ifndef SMARTL_PINMUX_H
#define SMARTL_PINMUX_H

#include <stdint.h>
#include "pin_name.h"

int32_t drv_pinmux_config(pin_name_e pin, pin_func_e pin_func);

#endif /* HOBBIT_PINMUX_H */
#endif /* __ASSEMBLY__ */

