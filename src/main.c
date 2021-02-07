#include "stm32f1xx.h"

#include "device/gpio.h"
#include "device/nvic.h"
#include "device/rcc.h"
#include "device/usart.h"

void system_init(void)
{
    rcc_config();
    gpio_config();
    usart_config();
    nvic_config();
}

int main()
{
    system_init();

    return 0;
}
