#ifndef _I2C_H_
#define _I2C_H_
/*
 * I2C : Inter-Intergrated Circuit
 */

#define I2C_BUF_SIZE 32

/* I2C Mode Identifiers */
#define I2C_WRITE 0
#define I2C_READ  1

/* Define I2Cx_ENABLED to enable a particular I2C */
#define I2C1_ENABLED

/* I2C declarations */
#ifdef I2C1_ENABLED
extern struct sw_fifo_t i2c1_tx_fifo;
extern char i2c1_tx_buf[I2C_BUF_SIZE];
extern struct sw_fifo_t i2c1_rx_fifo;
extern char i2c1_rx_buf[I2C_BUF_SIZE];
extern struct i2c_handle_t i2c1;
#endif
#ifdef I2C2_ENABLED
extern struct sw_fifo_t i2c2_tx_fifo;
extern char i2c2_tx_buf[I2C_BUF_SIZE];
extern struct sw_fifo_t i2c2_rx_fifo;
extern char i2c2_rx_buf[I2C_BUF_SIZE];
extern struct i2c_handle_t i2c2;
#endif
#ifdef I2C3_ENABLED
extern struct sw_fifo_t i2c3_tx_fifo;
extern char i2c3_tx_buf[I2C_BUF_SIZE];
extern struct sw_fifo_t i2c3_rx_fifo;
extern char i2c3_rx_buf[I2C_BUF_SIZE];
extern struct i2c_handle_t i2c3;
#endif

/* I2C instance struct */
struct i2c_handle_t {
    I2C_TypeDef *port;
    struct sw_fifo_t *tx_fifo;
    struct sw_fifo_t *rx_fifo;
    char *tx_buf;
    char *rx_buf;
    uint8_t mode;
    char reg;
    char periph_addr;
    char send_addr;
    uint8_t rx_bytes_count;
    uint8_t tx_bytes_count;
    uint8_t num_rx_bytes;
};

/* Functions */
void i2c_config(void);
void i2c_setup(struct i2c_handle_t *i2c);
void i2c_start(struct i2c_handle_t *i2c, uint8_t rdwr);
void i2c_stop(struct i2c_handle_t *i2c);
void i2c_nack(struct i2c_handle_t *i2c);
void i2c_transfer(struct i2c_handle_t *i2c, char addr, uint8_t num_bytes, uint8_t i2c_mode,
                   char reg, char *data);

#endif