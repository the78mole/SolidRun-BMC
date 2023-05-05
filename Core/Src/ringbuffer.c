/*
 * ringbuffer.c
 *
 *  Created on: 04.05.2023
 *      Author: dagl274982
 */

#include "ringbuffer.h"

void rb_init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->isInit = true;
}

bool rb_is_full(RingBuffer *rb) {
    return (rb->head + 1) % RING_BUFFER_SIZE == rb->tail;
}

bool rb_is_empty(RingBuffer *rb) {
    return rb->head == rb->tail;
}

bool rb_avail(RingBuffer *rb) {
    return rb->head != rb->tail;
}

bool rb_push(RingBuffer *rb, uint8_t value) {
    if (rb_is_full(rb)) {
        return false;
    }

    rb->buffer[rb->head] = value;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;

    return true;
}

bool rb_pop(RingBuffer *rb, uint8_t *value) {
    if (rb_is_empty(rb)) {
        return false;
    }

    *value = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;

    return true;
}

