#ifndef _SW_FIFO_H
#define _SW_FIFO_H

/*
 * Generic FIFO implementation
 */

/* Struct of the FIFO, an array must be passed externally to
 * initialize the FIFO. This allows for having variable
 * size arrays for different appliations
 */
struct sw_fifo_t {
    char *buf;
    uint16_t head;
    uint16_t tail;
    uint16_t size;
};

void sw_fifo_init(struct sw_fifo_t *fifo, char *buf, uint16_t size);
uint16_t sw_fifo_read(struct sw_fifo_t *fifo, char *buf, uint16_t num_bytes);
uint16_t sw_fifo_write(struct sw_fifo_t *fifo, char *buf, uint16_t num_bytes);
uint16_t sw_fifo_get_free_space(struct sw_fifo_t *fifo);

#endif