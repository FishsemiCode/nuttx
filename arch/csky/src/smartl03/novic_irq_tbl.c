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
extern void TIM0_IRQHandler(void);
extern void TIM1_IRQHandler(void);
extern void TIM2_IRQHandler(void);
extern void TIM3_IRQHandler(void);
extern void USART_IRQHandler(void);
extern void GPIO0_IRQHandler(void);
extern void GPIO1_IRQHandler(void);
extern void GPIO2_IRQHandler(void);
extern void GPIO3_IRQHandler(void);
extern void GPIO4_IRQHandler(void);
extern void GPIO5_IRQHandler(void);
extern void GPIO6_IRQHandler(void);
extern void GPIO7_IRQHandler(void);

void (*g_irqvector[])(void) = {
    USART_IRQHandler, /* 0, default interrupt entry */
    CORET_IRQHandler,  /* 1, default interrupt entry */
    TIM0_IRQHandler, /* 2, default interrupt entry */
    TIM1_IRQHandler, /* 3, default interrupt entry */
    TIM2_IRQHandler, /* 4, default interrupt entry */
    TIM3_IRQHandler, /* 5, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 6, default interrupt entry */
    GPIO0_IRQHandler, /* 7, default interrupt entry */
    GPIO1_IRQHandler, /* 8, default interrupt entry */
    GPIO2_IRQHandler, /* 9, default interrupt entry */
    GPIO3_IRQHandler, /* 10, default interrupt entry */
    GPIO4_IRQHandler, /* 11, default interrupt entry */
    GPIO5_IRQHandler, /* 12, default interrupt entry */
    GPIO6_IRQHandler, /* 13, default interrupt entry */
    GPIO7_IRQHandler, /* 14, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 15, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 16, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 17, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 18, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 19, default interrupt entry */
    NOVIC_IRQ_Default_Handler, /* 20, default interrupt entry */
};
#endif
