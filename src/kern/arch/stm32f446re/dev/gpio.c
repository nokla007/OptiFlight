/*
 * Copyright (c) 2022
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <gpio.h>
void DRV_GPIO_INIT(GPIO_TypeDef* gpio)
{
    gpio->MODER |= 1 << 0;
}

void GPIO_ENABLE(GPIO_TypeDef* port) {
    switch ((uint32_t)port) {
    case (uint32_t)GPIOA:
        RCC->AHB1ENR |= 1;
        break;
    case (uint32_t)GPIOB:
        RCC->AHB1ENR |= 0b10;
        break;
    case (uint32_t)GPIOC:
        RCC->AHB1ENR |= 0b100;
        break;
    case (uint32_t)GPIOD:
        RCC->AHB1ENR |= 0b1000;
        break;
    case (uint32_t)GPIOE:
        RCC->AHB1ENR |= 0b10000;
        break;
    case (uint32_t)GPIOF:
        RCC->AHB1ENR |= 0b100000;
        break;
    default:
        break;
    }
}

void pinConfig(GPIO_TypeDef* port, uint8_t pin, uint32_t config) {
    // set moder to 00
    // then set mode
    port->MODER &= ~(0b11 << (pin * 2));
    port->MODER |=
        (((config & GPIO_MODER_MASK) >> GPIO_MODER_MASK_Pos) << (pin * 2));

    // set pupd  to 00
    // then set pupd
    port->PUPDR &= ~(0b11 << (pin * 2));
    port->PUPDR |= (((config >> GPIO_PUPD_MASK_Pos) & 0b11) << (pin * 2));

    // set otyper to 0
    // then set otyper
    port->OTYPER &= ~(0b1 << (pin));
    port->OTYPER |= (((config >> GPIO_OTYPE_MASK_Pos) & 0b1) << (pin));

    // set OSPEEDR to 00
    // then set OSPEEDR
    port->OSPEEDR &= ~(0b11 << (pin * 2));
    port->OSPEEDR |= (((config >> GPIO_OSPEEDR_MASK_Pos) & 0b11) << (pin * 2));

    if (pin <= 7) {
        // set AFRL to 0b0000
        // then set AFRL
        port->AFRL &= ~(0b1111 << (pin * 4));
        port->AFRL |= (((config >> GPIO_AF_MASK_Pos) & 0b1111) << (pin * 4));
    }
    else {
        // set AFRH to 0b0000
        // then set AFRH
        port->AFRH &= ~(0b1111 << ((pin - 8) * 4));
        port->AFRH |= (((config >> GPIO_AF_MASK_Pos) & 0b1111) << ((pin - 8) * 4));
    }

    if (((config >> GPIO_LOCK_PIN_Pos) & 0b1)) {
        uint32_t lcr = (1 << pin);
        lcr |= (1 << 16);
        port->LCKR = lcr;
        lcr &= ~(1 << 16);
        port->LCKR = lcr;
        lcr |= (1 << 16);
        port->LCKR = lcr;

        lcr = port->LCKR;
    }
}




void digitalWrite(GPIO_TypeDef* port, uint8_t pin, uint8_t value) {
    port->BSRR = value ? 1 << pin : (1 << pin) << 16;
}

uint8_t digitalRead(GPIO_TypeDef* port, uint8_t pin) {
    return ((port->IDR >> pin) & 0b1);
}


void pinInterruptConfig(GPIO_TypeDef* port, uint8_t pin,
    uint8_t trigger, uint8_t priority) {
    RCC->APB2ENR |= (1 << 14); // Enable SYSCNFG Clock

    uint8_t EXTICode = port == GPIOA ? 0x0
        : port == GPIOB ? 0x1
        : port == GPIOC ? 0x2
        : port == GPIOD ? 0x3
        : port == GPIOE ? 0x4
        : port == GPIOF ? 0x5
        : port == GPIOG ? 0x6
        : 0x7;

    // set EXTICR
    SYSCFG->EXTICR[(uint8_t)(pin / 4)] &= ~(0xf << (4 * (pin % 4)));
    SYSCFG->EXTICR[(uint8_t)(pin / 4)] |= (EXTICode << (4 * (pin % 4)));

    EXTI->IMR |= (1 << pin);                       // Disable mask on EXTI1
    EXTI->RTSR |= ((trigger & 0b1) << pin);        // set rising triger
    EXTI->FTSR |= (((trigger >> 1) & 0b1) << pin); // set falling triger

    IRQn_TypeDef _IRQn;

    _IRQn = pin == 0 ? EXTI0_IRQn
        : pin == 1 ? EXTI1_IRQn
        : pin == 2 ? EXTI2_IRQn
        : pin == 3 ? EXTI3_IRQn
        : pin == 4 ? EXTI4_IRQn
        : pin <= 9 ? EXTI9_5_IRQn
        : EXTI15_10_IRQn;

    if (priority > 0)
        __NVIC_SetPriority(_IRQn, priority);
    __NVIC_EnableIRQ(_IRQn);
}

uint8_t isInterruptPending(GPIO_TypeDef* port, uint8_t pin) {
    if (EXTI->PR & (1 << pin)) {
        EXTI->PR |= (1 << pin);
        return 1;
    }
    return 0;
}

uint8_t isPinLocked(GPIO_TypeDef* port, uint8_t pin) {
    uint32_t lcr = port->LCKR;
    return (lcr >> pin) & 0b1;
}