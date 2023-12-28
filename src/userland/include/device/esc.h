#ifndef __LIBS__ESC_H__
#define __LIBS__ESC_H__

//define all your function prototypes and variables under this line

#include "gpio.h"
#include "timer.h"

typedef enum PWM_CHANNEL {
    CHANNEL1,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
} PWM_CHANNEL;

typedef struct
{
    /* data */
    GPIO_TypeDef* gpio;
    uint8_t pin;
    PWM_CHANNEL channel;
} ESC;

void ESC_ConfigWithTimer2(void);
void ESC_Attach(ESC* esc);
void ESC_write(ESC* esc, uint16_t micros);

#endif