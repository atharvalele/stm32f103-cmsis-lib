#ifndef _USART_H_
#define _USART_H_
/*
 * USART: Universal Asynchronous reciever transmitter
 */

#define USART_BUF_SIZE  128

/* Define USARTx_ENABLED to have access to FIFO buffers
 * and interrupts
 */
#define USART2_ENABLED

#ifdef USART1_ENABLED
extern struct sw_fifo_t usart1_tx_fifo;
extern char usart1_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart1_rx_fifo;
extern char usart1_rx_buf[USART_BUF_SIZE];
extern struct usart_handle_t usart1;
#endif
#ifdef USART2_ENABLED
extern struct sw_fifo_t usart2_tx_fifo;
extern char usart2_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart2_rx_fifo;
extern char usart2_rx_bug[USART_BUF_SIZE];
extern struct usart_handle_t usart2;
#endif
#ifdef USART3_ENABLED
extern struct sw_fifo_t usart3_tx_fifo;
extern char usart3_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart3_rx_fifo;
extern char usart3_rx_bug[USART_BUF_SIZE];
extern struct usart_handle_t usart3;
#endif

struct usart_handle_t {
    USART_TypeDef *port;
    struct sw_fifo_t *tx_fifo;
    struct sw_fifo_t *rx_fifo;
    char *tx_buf;
    char *rx_buf;
    uint32_t baudrate;
};

void usart_config(void);
void usart_setup(struct usart_handle_t *usart);
void usart_str_send(struct usart_handle_t *usart, char *str);

#endif