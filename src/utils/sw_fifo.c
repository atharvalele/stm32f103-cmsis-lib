/*
 * Generic FIFO implementation
 */

#include <stdint.h>

#include "utils/sw_fifo.h"

/* Initialize the FIFO with given size */
void sw_fifo_init(struct sw_fifo_t *fifo, char *buf, uint16_t size)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->size = size;
    fifo->buf = buf;
}

/* 
 * Read a number of bytes from the FIFO
 * Returns the number of bytes read
 */
uint16_t sw_fifo_read(struct sw_fifo_t *fifo, char *buf, uint16_t num_bytes)
{
    uint16_t bytes_read;
    char *temp_ptr;

    temp_ptr = buf;

    /* Start reading */
    for (bytes_read = 0; bytes_read < num_bytes; bytes_read++) {
        /* Check if data is available */
        if (fifo->tail != fifo->head) {
            *temp_ptr = fifo->buf[fifo->tail];
            /* Increment buffer pointer and FIFO tail */
            temp_ptr++;
            fifo->tail++;
        } else {
            /* No more data to read */
            return bytes_read;
        }
    }
    /* We read all bytes */
    return num_bytes;
}

/* Write a number of bytes to the FIFO */
uint16_t sw_fifo_write(struct sw_fifo_t *fifo, char *buf, uint16_t num_bytes)
{
    uint16_t bytes_written;
    char *temp_ptr;

    temp_ptr = buf;

    for (bytes_written = 0; bytes_written < num_bytes; bytes_written++) {
        /* Check if there is space in the buffer */
        if (((fifo->head + 1) == fifo->tail) ||
            (((fifo->head+1) == fifo->size) && (fifo->tail == 0))) {
                /* No more space to write data */
                return bytes_written;
        } else {
            fifo->buf[fifo->head] = *temp_ptr;
            /* Increment head and buffer ptr */
            temp_ptr++;
            fifo->head++;
        }
    }
    /* We wrote all the bytes */
    return num_bytes;
}
