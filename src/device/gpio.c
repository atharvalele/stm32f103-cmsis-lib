#include "stm32f1xx.h"
#include "device/gpio.h"

void gpio_config()
{
    /* Enbale Clock */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);
    /* Set pin 13 as output with speed 2 MHz */
    gpio_mode_set(GPIOC, GPIO13, GPIO_OUTPUT_2MHz, GPIO_CNF_PUSH_PULL);
    /* Set output as Push pull */
    gpio_output_options_set(GPIOC, GPIO13, GPIO_CNF_PUSH_PULL);

    /* Setup for GPIO */
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    gpio_mode_set(GPIOA, GPIO3, GPIO_INPUT, GPIO_CNF_FLOATING);
    gpio_mode_set(GPIOA, GPIO2, GPIO_OUTPUT_50MHz, GPIO_CNF_PUSH_PULL);
    gpio_output_options_set(GPIOA, GPIO2, GPIO_CNF_PUSH_PULL);
}

/* Toggle All GPIO pins passed as an argument */
void gpio_toggle(GPIO_TypeDef *port, uint16_t pins)
{
    uint32_t portval;

    portval = port->ODR ;
    port->BSRR = (portval & pins) << 16 | (~portval & pins);
}

/* Set GPIO Pin Modes - I/P, O/P, Analog, AF  & Pull Up / Pull Down / Output Speed 10 MHz 
 * 2 MHz / 50 MHz
 */
void gpio_mode_set(GPIO_TypeDef *port, uint16_t pins, uint8_t mode, uint8_t pupd_sett)
{
    uint8_t i;
    uint32_t moder;
    /*
     * We only want to change the pins that are passed,
     * so keep the original values and mask the passed ones
     */

    moder =  port->CRL;

    for (i = 0; i < 8; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        moder &= ~GPIO_SETT_MASK(i);
        moder |= GPIO_SETT(i, mode);
        if (mode == 0x00) {
            moder &= ~GPIO_CNF_SETT_MASK(i);
            moder |= GPIO_SETT(i, pupd_sett);
        }

    }

    port->CRL = moder;

    moder = port->CRH;

    for (i = 8; i < 16; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        /* In case of the CRH i.e. Configuration High Register the 
         * value needs to be modded by 8 because the register is
         * 32 bits only. The macro would expand to (4 * (8) + 2)
         * for the first iteration however we need it to start from 0
         * therefore the value passed is i % 8
         */
        moder &= ~GPIO_SETT_MASK(i % 8);
        moder |= GPIO_SETT(i % 8, mode);
        if (mode == GPIO_INPUT) {
            moder &= ~GPIO_CNF_SETT_MASK(i % 8);
            moder |= GPIO_CNF_SETT(i % 8, pupd_sett);
        }
    }

    port->CRH = moder;
}

/* Set GPIO Output */
void gpio_output_options_set(GPIO_TypeDef *port, uint16_t pins, uint8_t otype_sett)
{
    uint8_t i;
    uint32_t otyper;
    
    otyper = port->CRL;

    for (i = 0; i < 8; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        otyper &= ~GPIO_CNF_SETT_MASK(i);
        otyper |= GPIO_CNF_SETT(i, otype_sett);
    }

    port->CRL = otyper;

    otyper = port->CRH;

    for (i = 8; i < 16; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }
        /* In case of the CRH i.e. Configuration High Register the 
         * value needs to be modded by 8 because the register is
         * 32 bits only. The macro would expand to (4 * (8) + 2)
         * for the first iteration however we need it to start from 0
         * therefore the value passed is i % 8
         */
        otyper &= ~GPIO_CNF_SETT_MASK(i % 8);
        otyper |= GPIO_CNF_SETT(i % 8, otype_sett);
    }

    port->CRH = otyper;

}

/* Turn ON all GPIO pins passed as an argument */
void gpio_set(GPIO_TypeDef *port, uint16_t pins)
{
    port->BSRR |= pins;
}

/* Turn OFF all GPIO pins passed as an argument */
void gpio_clear(GPIO_TypeDef *port, uint16_t pins)
{
    port->BRR |= pins;
}