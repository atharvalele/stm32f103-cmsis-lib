/*
 * I2C : Inter-Intergrated Circuit
 */

#include "stm32f1xx.h"
#include "device/gpio.h"
#include "device/i2c.h"

#include "utils/sw_fifo.h"

/* I2C declarations */
#ifdef I2C1_ENABLED
struct sw_fifo_t i2c1_tx_fifo;
char i2c1_tx_buf[I2C_BUF_SIZE];
struct sw_fifo_t i2c1_rx_fifo;
char i2c1_rx_buf[I2C_BUF_SIZE];
struct i2c_handle_t i2c1 = {
    .port = I2C1,
    .tx_fifo = &i2c1_tx_fifo,
    .tx_buf = i2c1_tx_buf,
    .rx_fifo = &i2c1_rx_fifo,
    .rx_buf = i2c1_rx_buf,
};
#endif
#ifdef I2C2_ENABLED
struct sw_fifo_t i2c2_tx_fifo;
char i2c2_tx_buf[I2C_BUF_SIZE];
struct sw_fifo_t i2c2_rx_fifo;
char i2c2_rx_buf[I2C_BUF_SIZE];
struct i2c_handle_t i2c2 = {
    .port = I2C2,
    .tx_fifo = &i2c2_tx_fifo,
    .tx_buf = i2c2_tx_buf,
    .rx_fifo = &i2c2_rx_fifo,
    .rx_buf = i2c2_rx_buf,
};
#endif
#ifdef I2C3_ENABLED
struct sw_fifo_t i2c3_tx_fifo;
char i2c3_tx_buf[I2C_BUF_SIZE];
struct sw_fifo_t i2c3_rx_fifo;
char i2c3_rx_buf[I2C_BUF_SIZE];
struct i2c_handle_t i2c3 = {
    .port = I2C3,
    .tx_fifo = &i2c3_tx_fifo,
    .tx_buf = i2c3_tx_buf,
    .rx_fifo = &i2c3_rx_fifo,
    .rx_buf = i2c3_rx_buf,
};
#endif


/*
 * Setup all enabled I2C ports.
 * I2C clock is derived from the PCLK domain clock.
 */
void i2c_config(void)
{
#ifdef I2C1_ENABLED
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
    i2c_setup(&i2c1);
#endif
#ifdef I2C2_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN);
    i2c_setup(&i2c2);
#endif
}

void i2c_setup(struct i2c_handle_t *i2c)
{
    /* PCLK1 cycle time */
    uint32_t tpclk1 = 1e9 / ( SystemCoreClock / 2) ;

    /* Initialize the FIFO */
    sw_fifo_init(i2c->tx_fifo, i2c->tx_buf, I2C_BUF_SIZE);
    sw_fifo_init(i2c->rx_fifo, i2c->rx_buf, I2C_BUF_SIZE);

    /* Disable the I2C perpiheral */
    CLEAR_BIT(i2c->port->CR1, I2C_CR1_PE);

    /*
     * GPIO config for I2C needs to be done with the peripheral disabled, hence do it
     * here instead of in gpio.c
     */
    if (i2c->port == I2C1) {
        gpio_mode_set(GPIOB, (GPIO6 | GPIO7), GPIO_OUTPUT_50MHz, GPIO_CNF_AF_OD);
        gpio_output_options_set(GPIOB, (GPIO6 | GPIO7), GPIO_CNF_AF_OD);
    } else if (i2c->port == I2C2) {
        gpio_mode_set(GPIOB, (GPIO10 | GPIO11), GPIO_OUTPUT_50MHz, GPIO_CNF_AF_OD);
        gpio_output_options_set(GPIOB, (GPIO10 | GPIO11), GPIO_CNF_AF_OD);
    }

    /* Reset */
    SET_BIT(i2c->port->CR1, I2C_CR1_SWRST);
    CLEAR_BIT(i2c->port->CR1, I2C_CR1_SWRST);

    /* Enable ACKs */
    SET_BIT(i2c->port->CR1, I2C_CR1_ACK);

    /* Select fast mode */
    SET_BIT(i2c->port->CCR, I2C_CCR_FS);

    /*
     * Set up the I2C speed configuration for I2C Fast Mode
     * Note: Check the reference manual RM0008 for more details.
     */
    MODIFY_REG(i2c->port->CR2, I2C_CR2_FREQ, 36 << I2C_CR2_FREQ_Pos);
    MODIFY_REG(i2c->port->CCR, I2C_CCR_CCR, ((((36) * 5) / 6)) << I2C_CCR_CCR_Pos);
    MODIFY_REG(i2c->port->TRISE, I2C_TRISE_TRISE, ((300 / tpclk1) + 1) << I2C_TRISE_TRISE_Pos);

    /* Fast mode duty cycle = t(low)/t(high) = 2 */
    CLEAR_BIT(i2c->port->CCR, I2C_CCR_DUTY);

    /* Enable the I2C peripheral */
    SET_BIT(i2c->port->CR1, I2C_CR1_PE);
}

void i2c_start(struct i2c_handle_t *i2c, uint8_t rdwr)
{
    /* Get the base address */
    i2c->send_addr = i2c->periph_addr;
    /* Add the R/W bit to it */
    i2c->send_addr <<= 1;
    i2c->send_addr |= rdwr;

    /* Generate a start condition */
    SET_BIT(i2c->port->CR1, I2C_CR1_START);
}

void i2c_stop(struct i2c_handle_t *i2c)
{
    SET_BIT(i2c->port->CR1, I2C_CR1_STOP);
}

void i2c_nack(struct i2c_handle_t *i2c)
{
    CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
}

void i2c_transfer(struct i2c_handle_t *i2c, char addr, uint8_t num_bytes, uint8_t i2c_mode,
                   char reg, char *data)
{
    /* Set the transfer mode */
    i2c->mode = i2c_mode;

    /* Set the peripheral register */
    i2c->reg = reg;

    /* Set the peripheral address */
    i2c->periph_addr = addr;

    /* Set the mode */
    i2c->mode = i2c_mode;
    
    /* Where are we reading/writing from? */
    sw_fifo_write(i2c->tx_fifo, &reg, 1);

    if (i2c->mode == I2C_WRITE) {
        sw_fifo_write(i2c->tx_fifo, data, num_bytes);
    } else if (i2c->mode == I2C_READ) {
        i2c->num_rx_bytes = num_bytes;
    }

    /* Enable ACKs */
    SET_BIT(i2c->port->CR1, I2C_CR1_ACK);

    /* Start transmission */
    i2c_start(i2c, I2C_WRITE);

    /* Enable TX/TC interrupts */
    SET_BIT(i2c->port->CR2, I2C_CR2_ITBUFEN);
    SET_BIT(i2c->port->CR2, I2C_CR2_ITEVTEN);
}

/* I2C1 Event IRQ Handler */
#ifdef I2C1_ENABLED
void I2C1_EV_IRQHandler(void)
{
    char outgoing_byte;
    char incoming_byte;
    uint8_t bytes;

    /* Start sequence complete, send the address */
    if (I2C1->SR1 & I2C_SR1_SB) {
        I2C1->DR = i2c1.send_addr;
    }

    /* Address sent */
    if (I2C1->SR1 & I2C_SR1_ADDR) {
        /* ACK the interrupt */
        (void)(I2C1->SR2);
        /*
         * If we're reading and expect just one byte from the
         * peripheral, we need to immediately send a NACK and
         * set up the device for a stop sequence.
         */
        if (READ_BIT(I2C1->SR2, I2C_SR2_TRA) == 0) {
            if (i2c1.num_rx_bytes == 1) {
                i2c_nack(&i2c1);
                i2c_stop(&i2c1);
                CLEAR_BIT(I2C1->CR2, I2C_CR2_ITBUFEN);
                CLEAR_BIT(I2C1->CR2, I2C_CR2_ITEVTEN);
            }
        }
    }

    /* Tx interrupt */
    if (I2C1->SR1 & I2C_SR1_TXE) {
        bytes = sw_fifo_read(&i2c1_tx_fifo, &outgoing_byte, 1);
        if (bytes != 0) {
            I2C1->DR = outgoing_byte;
        } else {
            /* Repeated start for starting a read sequence */
            if (i2c1.mode == I2C_READ) {
                i2c_start(&i2c1, I2C_READ);
                i2c1.rx_bytes_count = 0;
            } else {
                i2c_stop(&i2c1);
            }
        }
    }

    /* Rx interrupt */
    if (I2C1->SR1 & I2C_SR1_RXNE) {
        if (i2c1.rx_bytes_count == i2c1.num_rx_bytes - 1) {
            /*
             * We need to nack after the last byte, so we clear the
             * ACK bit after the second last byte.
             */
            i2c_nack(&i2c1);
        }
        if (i2c1.rx_bytes_count >= i2c1.num_rx_bytes) {
            /* we're done with the transfer, stop the interrupts */
            CLEAR_BIT(I2C1->CR2, I2C_CR2_ITBUFEN);
            CLEAR_BIT(I2C1->CR2, I2C_CR2_ITEVTEN);
        } else {
        incoming_byte = I2C1->DR;
        i2c1.rx_bytes_count++;
        sw_fifo_write(&i2c1_rx_fifo, &incoming_byte, 1);
        }
    }
}
#endif

/* I2C2 Event IRQ Handler */
#ifdef I2C2_ENABLED
void I2C2_EV_IRQHandler(void)
{
    char outgoing_byte;
    char incoming_byte;
    uint8_t bytes;

    /* Start sequence complete, send the address */
    if (I2C2->SR1 & I2C_SR1_SB) {
        I2C2->DR = i2c2.send_addr;
    }

    /* Address sent */
    if (I2C2->SR1 & I2C_SR1_ADDR) {
        /* ACK the interrupt */
        (void)(I2C2->SR2);
        /*
         * If we're reading and expect just one byte from the
         * peripheral, we need to immediately send a NACK and
         * set up the device for a stop sequence.
         */
        if (READ_BIT(I2C2->SR2, I2C_SR2_TRA) == 0) {
            if (i2c2.num_rx_bytes == 1) {
                i2c_nack(&i2c2);
                i2c_stop(&i2c2);
                CLEAR_BIT(I2C2->CR2, I2C_CR2_ITBUFEN);
                CLEAR_BIT(I2C2->CR2, I2C_CR2_ITEVTEN);
            }
        }
    }

    /* Tx interrupt */
    if (I2C2->SR1 & I2C_SR1_TXE) {
        bytes = sw_fifo_read(&i2c2_tx_fifo, &outgoing_byte, 1);
        if (bytes != 0) {
            I2C2->DR = outgoing_byte;
        } else {
            /* Repeated start for starting a read sequence */
            if (i2c2.mode == I2C_READ) {
                i2c_start(&i2c2, I2C_READ);
                i2c2.rx_bytes_count = 0;
            } else {
                i2c_stop(&i2c2);
            }
        }
    }

    /* Rx interrupt */
    if (I2C2->SR1 & I2C_SR1_RXNE) {
        if (i2c2.rx_bytes_count == i2c2.num_rx_bytes - 1) {
            /*
             * We need to nack after the last byte, so we clear the
             * ACK bit after the second last byte.
             */
            i2c_nack(&i2c2);
        }
        if (i2c2.rx_bytes_count >= i2c2.num_rx_bytes) {
            /* we're done with the transfer, stop the interrupts */
            CLEAR_BIT(I2C2->CR2, I2C_CR2_ITBUFEN);
            CLEAR_BIT(I2C2->CR2, I2C_CR2_ITEVTEN);
        } else {
        incoming_byte = I2C2->DR;
        i2c2.rx_bytes_count++;
        sw_fifo_write(&i2c2_rx_fifo, &incoming_byte, 1);
        }
    }
}
#endif
