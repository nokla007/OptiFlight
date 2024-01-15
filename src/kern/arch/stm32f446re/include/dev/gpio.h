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
 * GPIO_INTERRUPT_TRIGGER_FALLING | GPIO_INTERRUPT_TRIGGER_RISING,
 */
#ifndef __GPIO_H
#define __GPIO_H
#include "../sys/stm32_peps.h"
#include "cm4.h"


typedef struct __PinWithFunction {
    GPIO_TypeDef* GPIO;
    uint8_t pin;
    uint32_t af;
} PinWithFunction;

typedef enum GPIO_CONFIG_e {
    // MODER_REG 0b 00 00 00 xx
    GPIO_INPUT = 0b00,
    GPIO_OUTPUT = 0b01,
    GPIO_ALTERNET = 0b10,
    GPIO_ANALOG = 0b11,

    GPIO_MODER_MASK = 0b11,
    GPIO_MODER_MASK_Pos = 0,

    // PUPD_REG  0b 00 00 xx 00
    GPIO_PULL_UP = 0b0100,
    GPIO_PULL_DOWN = 0b1000,

    GPIO_PUPD_MASK = 0b1100,
    GPIO_PUPD_MASK_Pos = 2,

    // OTYPE_REG 0b 00 Xx 00 00
    GPIO_PUSH_PULL = 0b00000000,
    GPIO_OPEN_DRAIN = 0b00010000,

    GPIO_OTYPE_MASK = 0b110000,
    GPIO_OTYPE_MASK_Pos = 4,

    // OSPEEDR_REG 0b xx 00 00 00
    GPIO_LOW_SPEED = 0b00000000,
    GPIO_MIDIUM_SPEED = 0b01000000,
    GPIO_FAST_SPEED = 0b10000000,
    GPIO_HIGH_SPEED = 0b11000000,

    GPIO_OSPEEDR_MASK = 0b11000000,
    GPIO_OSPEEDR_MASK_Pos = 6,

    // AFRL_REG
    GPIO_AF_AF0 = 0x000,
    GPIO_AF_AF1 = 0x100,
    GPIO_AF_AF2 = 0x200,
    GPIO_AF_AF3 = 0x300,
    GPIO_AF_AF4 = 0x400,
    GPIO_AF_AF5 = 0x500,
    GPIO_AF_AF6 = 0x600,
    GPIO_AF_AF7 = 0x700,
    GPIO_AF_AF8 = 0x800,
    GPIO_AF_AF9 = 0x900,
    GPIO_AF_AF10 = 0xA00,
    GPIO_AF_AF11 = 0xB00,
    GPIO_AF_AF12 = 0xC00,
    GPIO_AF_AF13 = 0xD00,
    GPIO_AF_AF14 = 0xE00,
    GPIO_AF_AF15 = 0xF00,

    GPIO_AF_MASK = 0b111100000000,
    GPIO_AF_MASK_Pos = 8,

    // LCKR_REG 0b 01 00 00 00 00 00 00
    GPIO_LOCK_PIN = 0b01000000000000,
    GPIO_LOCK_PIN_Pos = 12,

} GPIO_CONFIG;

typedef enum GPIO_PIN_VALUE_e {
    GPIO_PIN_HIGH = 1,
    GPIO_PIN_LOW = 0,
} GPIO_PIN_VALUE;

typedef enum GPIO_INTERRUPT_TRIGGER_e {
    GPIO_INTERRUPT_TRIGGER_RISING = 0b01,
    GPIO_INTERRUPT_TRIGGER_FALLING = 0b10,
} GPIO_INTERRUPT_TRIGGER;




void DRV_GPIO_INIT(GPIO_TypeDef*);

void GPIO_ENABLE(GPIO_TypeDef* port);
void pinConfig(GPIO_TypeDef* port, uint8_t pin, uint32_t config);
void digitalWrite(GPIO_TypeDef* port, uint8_t pin, uint8_t value);
uint8_t digitalRead(GPIO_TypeDef* port, uint8_t pin);
uint8_t isPinLocked(GPIO_TypeDef* port, uint8_t pin);
void pinInterruptConfig(GPIO_TypeDef* port, uint8_t pin,
    uint8_t trigger, uint8_t priority);
uint8_t isInterruptPending(GPIO_TypeDef* port, uint8_t pin);



#endif
