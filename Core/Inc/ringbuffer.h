/*
 * ringbuffer.h
 *
 *  Created on: 04.05.2023
 *      Author: dagl274982
 */

#ifndef SRC_RINGBUFFER_H_
#define SRC_RINGBUFFER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define RING_BUFFER_SIZE 32

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    int head;
    int tail;
    int isInit;
} RingBuffer;

void rb_init(RingBuffer *rb);
bool rb_is_full(RingBuffer *rb);
bool rb_is_empty(RingBuffer *rb);
bool rb_avail(RingBuffer *rb);
bool rb_push(RingBuffer *rb, uint8_t value);
bool rb_pop(RingBuffer *rb, uint8_t *value);

#endif /* SRC_RINGBUFFER_H_ */
