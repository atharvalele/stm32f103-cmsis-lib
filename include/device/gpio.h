#ifndef _GPIO_H_
#define _GPIO_H_

/* GPIO Pin definitions */
#define GPIO0		(1 << 0)
#define GPIO1		(1 << 1)
#define GPIO2		(1 << 2)
#define GPIO3		(1 << 3)
#define GPIO4		(1 << 4)
#define GPIO5		(1 << 5)
#define GPIO6		(1 << 6)
#define GPIO7		(1 << 7)
#define GPIO8		(1 << 8)
#define GPIO9		(1 << 9)
#define GPIO10		(1 << 10)
#define GPIO11		(1 << 11)
#define GPIO12		(1 << 12)
#define GPIO13		(1 << 13)
#define GPIO14		(1 << 14)
#define GPIO15		(1 << 15)
#define GPIO_ALL	0xFFFF

/* Useful Port and Pin Definitions */
#define STATUS_LED_PORT GPIOC
#define STATUS_LED_PIN  GPIO13

/* GPIO Port Configuration */
/* Input Mode MODE[1:0] = 00 */
#define GPIO_CNF_ANALOG         0x00
#define GPIO_CNF_FLOATING       0x01
#define GPIO_CNF_PUPD           0x02
#define GPIO_CNF_RES            0x03

/* Output Mode MODE[1:0] > 00) */
#define GPIO_CNF_PUSH_PULL      0x00
#define GPIO_CNF_OD             0x01
#define GPIO_CNF_AF_PUSH_PULL   0x02
#define GPIO_CNF_AF_OD          0x03

/* GPIO Port Mode */
#define GPIO_INPUT              0x00
#define GPIO_OUTPUT_10MHz       0x01
#define GPIO_OUTPUT_2MHz        0x02
#define GPIO_OUTPUT_50MHz       0x03

#define GPIO_SETT(n, sett) ((sett) << (4 * (n)))
#define GPIO_SETT_MASK(n)  (0x03 << (4 * (n)))

#define GPIO_CNF_SETT(n, sett) ((sett) << ((4 * (n)) + 2))
#define GPIO_CNF_SETT_MASK(n)  (0x03 << ((4 * (n)) + 2))

void gpio_config(void);
void gpio_toggle(GPIO_TypeDef *port, uint16_t pins);
void gpio_mode_set(GPIO_TypeDef *port, uint16_t pins, uint8_t mode, uint8_t pupd_sett);
void gpio_output_options_set(GPIO_TypeDef *port, uint16_t pins, uint8_t otype_sett);
void gpio_set(GPIO_TypeDef *port, uint16_t pins);
void gpio_clear(GPIO_TypeDef *port, uint16_t pins);

#endif