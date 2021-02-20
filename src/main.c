#include "stm32f1xx.h"

#include "device/gpio.h"
#include "device/i2c.h"
#include "device/nvic.h"
#include "device/rcc.h"
#include "device/usart.h"

void system_init(void)
{
    rcc_config();
    gpio_config();
    i2c_config();
    usart_config();
    nvic_config();
}

int main()
{
    system_init();

    while (1) {
        if (secflag) {
            gpio_toggle(STATUS_LED_PORT, STATUS_LED_PIN);
            secflag = 0;
        }
    }

    return 0;
}
