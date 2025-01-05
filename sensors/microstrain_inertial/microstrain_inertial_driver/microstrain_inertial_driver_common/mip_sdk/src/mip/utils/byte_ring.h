#pragma once

#include <stdint.h>
#include <stddef.h>


typedef struct byte_ring_state
{
    uint8_t* buffer;
    size_t   size;
    size_t   head;
    size_t   tail;
} byte_ring_state;

void byte_ring_init(byte_ring_state* state, uint8_t* buffer, size_t size);
void byte_ring_clear(byte_ring_state* state);

size_t byte_ring_capacity(const byte_ring_state* state);
size_t byte_ring_count(const byte_ring_state* state);
size_t byte_ring_free_space(const byte_ring_state* state);

uint8_t byte_ring_at(const byte_ring_state* state, size_t index);

size_t byte_ring_pop(byte_ring_state* state, size_t count);

size_t byte_ring_copy_to(const byte_ring_state* state, uint8_t* buffer, size_t count);
size_t byte_ring_copy_from_and_update(byte_ring_state* state, const uint8_t** bytes, size_t* count);


size_t byte_ring_get_write_ptr(byte_ring_state* state, uint8_t** ptr_out);
void byte_ring_notify_written(byte_ring_state* state, size_t count);
