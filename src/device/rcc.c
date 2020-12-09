#include "stm32f1xx.h"

#include "device/rcc.h"

volatile uint32_t rcc_ms_ticks = 0;

void rcc_config(void)
{
    /*
     * To reliably read from the flash memory, we need to add
     * some latency. Page 58 - RM0008 Rev 20
     * - 0 wait states, if 0 < SYSCLK ≤ 24 MHz
     * - 1 wait state, if 24 MHz < SYSCLK ≤ 48 MHz
     * - 2 wait states, if 48 MHz < SYSCLK ≤ 72 MHz
     */
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);

    /* Turn on HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0);

    /* Set HSE as PLL input */
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);

    /* Don't divide HSE Input */
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE_HSE);

    /*
     * STM32F103 supports 72MHz as max system frequency. Set the
     * PLL multipliers & dividers accordingly
     * 
     * HSE = 8MHz -> x9 = 72MHz
     * APB1 = max 36MHz = PLL / 2
     * APB2 = max 72MHz = PLL / 1
     */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    /* Turn on PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0);

    /* Switch System Clock to PLL */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    /* Update the SystemCoreClock global var */
    SystemCoreClockUpdate();

    /* SysTick 1ms tick enable */
    SysTick_Config(SystemCoreClock/1000);
}

void SysTick_Handler(void)
{
      /* Increment counter necessary in delay_ms()*/
      rcc_ms_ticks++;
}

/* Uses the SysTick Timer to generate an accurate time delay */
void delay_ms(uint32_t ms)
{
    uint32_t curr_ticks;

    curr_ticks = rcc_ms_ticks;

    /* Wait */
    while((rcc_ms_ticks - curr_ticks) < ms);
}