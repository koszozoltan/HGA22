/*
 * queue.c
 *
 *  Created on: Jan 30, 2026
 *      Author: zkoszo
 */


#include "queue.h"



void sq_init(stream_q_t *q, int16_t *buffer, uint16_t size) {
    q->buf  = buffer;
    q->size = size;
    q->head = 0;
    q->tail = 0;
}


uint16_t sq_size(const stream_q_t *q) {
    if (q->head >= q->tail) {
        return q->head - q->tail;
    } else {
        return q->size - (q->tail - q->head);
    }
}

void sq_push(stream_q_t *q,const int16_t *src, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {

        uint16_t next = (q->head + 1) & (q->size - 1);

        if (next == q->tail) {
            /* overwrite → eldob 1 byte-ot */
            q->tail = (q->tail + 1) & (q->size - 1);
        }

        q->buf[q->head] = src[i];
        q->head = next;
    }
}



uint16_t sq_peek(const stream_q_t *q, int16_t *dst, uint16_t len) {
    uint16_t avail = sq_size(q);
    if (len > avail) {
        len = avail;
    }

    uint16_t idx = q->tail;

    for (uint16_t i = 0; i < len; i++) {
        dst[i] = q->buf[idx];
        idx = (idx + 1) & (q->size - 1);
    }

    return len;
}



uint16_t sq_discard(stream_q_t *q, uint16_t len)
{
    uint16_t avail = sq_size(q);
    if (len > avail) {
        len = avail;
    }

    q->tail = (q->tail + len) & (q->size - 1);
    return len;
}

