#ifndef _RCC_H_
#define _RCC_H_

/*
 * RCC: Reset & Clock Control
 */

extern volatile uint32_t rcc_ms_ticks;

void rcc_config(void);
void delay_ms(uint32_t ms);

#endif