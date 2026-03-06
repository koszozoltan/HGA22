/*
 * queue.h
 *
 *  Created on: Jan 30, 2026
 *      Author: zkoszo
 */

#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>

typedef struct {
	int16_t  *buf;        // külső buffer
    uint16_t size;        // buffer mérete (2 hatványa!)
    volatile uint16_t head;
    volatile uint16_t tail;
} stream_q_t;


void		sq_init		(stream_q_t *q, int16_t *buffer,uint16_t size);
uint16_t	sq_size		(const stream_q_t *q);
void 		sq_push		(stream_q_t *q,const int16_t *src, uint16_t len);
uint16_t 	sq_peek		(const stream_q_t *q,int16_t *dst, uint16_t len);
uint16_t 	sq_discard	(stream_q_t *q,uint16_t len);

/*
uint8_t rx_buf1[256];
uint8_t rx_buf2[128];

stream_q_t q_uart;
stream_q_t q_adc;

sq_init(&q_uart, rx_buf1, sizeof(rx_buf1));
sq_init(&q_adc,  rx_buf2, sizeof(rx_buf2));

sq_push(&q_uart, dma_rx, 8);
sq_push(&q_adc,  adc_data, 8);
 */



#endif
