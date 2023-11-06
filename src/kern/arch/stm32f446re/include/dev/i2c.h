#ifndef __I2C_H
#define __I2C_H
#include "../sys/stm32_peps.h"
#include <stdbool.h>
#include <ktimes.h>
bool DRV_I2C_INIT(I2C_TypeDef*);
void _I2C_GPIO_Config(GPIO_TypeDef* gpio_scl, uint16_t pin_scl, GPIO_TypeDef* gpio_sda, uint16_t pin_sda);
bool _I2C_MEM_WRITE(I2C_TypeDef*, uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool _I2C_MEM_READ(I2C_TypeDef*, uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint16_t size);
void _I2C_Reset(I2C_TypeDef* i2c);
#endif
