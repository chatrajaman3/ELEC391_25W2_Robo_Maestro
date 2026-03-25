#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include "actuator.h"
#include <stdarg.h>



void Actuator_Init(int count, ...) {
    va_list args;
    va_start(args, count);
    
    for (int i = 0; i < count; i++){
        GPIO_Pin_t finger = va_arg(args, GPIO_Pin_t);
        HAL_GPIO_WritePin(finger.port, finger.pin, GPIO_PIN_RESET);
    }
    va_end(args);
}
void Actuator_SetState(GPIO_PinState state, int count, ...) {
    va_list args;
    va_start(args, count);

    for (int i = 0; i < count; i++) {
        GPIO_Pin_t finger = va_arg(args, GPIO_Pin_t);
        HAL_GPIO_WritePin(finger.port, finger.pin, state);
    }
    va_end(args);
}