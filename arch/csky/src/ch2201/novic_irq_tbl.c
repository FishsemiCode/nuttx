/*
 * Copyright (C) 2016 YunOS Project. All rights reserved.
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
#if 0
extern void Default_Handler(void);
extern void CORET_IRQHandler(void);
extern void USART0_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void USART3_IRQHandler(void);

void (*g_irqvector[])(void) = {
    Default_Handler,  /* 0 */
    CORET_IRQHandler,  /* 1 */
    Default_Handler,   /* 2 */
    Default_Handler,   /* 3 */
    Default_Handler,   /* 4 */
    Default_Handler,   /* 5 */
    USART0_IRQHandler, /* 6 */
    USART1_IRQHandler, /* 7 */
    USART2_IRQHandler, /* 8 */
    Default_Handler,   /* 9 */
    Default_Handler,   /* 10 */
    Default_Handler,   /* 11 */
    Default_Handler,   /* 12 */
    Default_Handler,   /* 13 */
    Default_Handler,   /* 14 */
    Default_Handler,   /* 15 */
    Default_Handler,   /* 16 */
    Default_Handler,   /* 17 */
    Default_Handler,   /* 18 */
    Default_Handler,   /* 19 */
    Default_Handler,   /* 20 */
    Default_Handler,   /* 21 */
    Default_Handler,   /* 22 */
    Default_Handler,   /* 23 */
    Default_Handler,   /* 24 */
    Default_Handler,   /* 25 */
    Default_Handler,   /* 26 */
    Default_Handler,  /* 27 */
    Default_Handler,   /* 28 */
    Default_Handler,   /* 29 */
    Default_Handler,   /* 30 */
    Default_Handler,   /* 31 */
};

#endif
