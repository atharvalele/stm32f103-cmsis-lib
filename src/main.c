#include "stm32f1xx.h"

#include "device/rcc.h"

void system_init(void)
{
    rcc_config();
}

int main()
{
    system_init();

    return 0;
}
