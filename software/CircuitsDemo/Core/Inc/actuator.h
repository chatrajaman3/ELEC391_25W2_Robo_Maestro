#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Pin_t;

void Actuator_Init(int count, ...);
void Actuator_SetState(GPIO_PinState state, int count, ...);

#endif