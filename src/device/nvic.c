/*
 * NVIC: Nested Vector Interrupt Controller
 */
#include "stm32f1xx.h"
#include "device/i2c.h"
#include "device/nvic.h"
#include "device/usart.h"

void nvic_config(void)
{
#ifdef USART1_ENABLED
    NVIC_EnableIRQ(USART1_IRQn);
#endif
#ifdef USART2_ENABLED
    NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef USART3_ENABLED
    NVIC_EnableIRQ(USART3_IRQn);
#endif

#ifdef I2C1_ENABLED
    NVIC_EnableIRQ(I2C1_EV_IRQn);
#endif
#ifdef I2C2_ENABLED
    NVIC_EnableIRQ(I2C2_EV_IRQn);
#endif
}