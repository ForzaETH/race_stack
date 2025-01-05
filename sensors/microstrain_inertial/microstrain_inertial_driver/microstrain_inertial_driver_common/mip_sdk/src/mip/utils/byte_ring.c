
#include "byte_ring.h"

#include <assert.h>


void byte_ring_init(byte_ring_state* state, uint8_t* buffer, size_t size)
{
    assert(buffer != NULL);
    assert( ((size - 1) & size) == 0 );  // Size must be a power of 2

    state->buffer = buffer;
    state->size   = size;
    state->head   = 0;
    state->tail   = 0;
}

void byte_ring_clear(byte_ring_state* state)
{
    state->head = 0;
    state->tail = 0;
}

size_t byte_ring_capacity(const byte_ring_state* state)
{
    return state->size;
}

size_t byte_ring_count(const byte_ring_state* state)
{
    assert(state->head - state->tail <= state->size);  // Buffer hasn't been overrun

    return state->head - state->tail;
}

size_t byte_ring_free_space(const byte_ring_state* state)
{
    size_t count = byte_ring_count(state);
    size_t size  = byte_ring_capacity(state);

    assert( count <= size );

    return size - count;
}

uint8_t byte_ring_at(const byte_ring_state* state, size_t index)
{
    assert(index < (state->head - state->tail));  // Index must be in bounds (less than count)

    return state->buffer[ (state->tail + index) % state->size ];
}

size_t byte_ring_pop(byte_ring_state* state, size_t count)
{
    size_t available = byte_ring_count(state);
    if( count > available )
        count = available;

    state->tail += count;

    return count;
}

size_t byte_ring_copy_to(const byte_ring_state* state, uint8_t* buffer, size_t count)
{
    const size_t available = byte_ring_count(state);
    if( available < count )
        count = available;

    // const size_t count_A =
    for(size_t i=0; i<count; i++)
        buffer[i] = byte_ring_at(state, i);

    return count;
}

size_t byte_ring_copy_from_and_update(byte_ring_state* state, const uint8_t** const bytes, size_t* available)
{
    const size_t space = byte_ring_free_space(state);
    const size_t count = (*available < space) ? *available : space;

    const size_t capacity = byte_ring_capacity(state);

    for(size_t i=0; i<count; i++)
        state->buffer[ (state->head + i) % capacity ] = (*bytes)[i];

    state->head += count;

    *bytes += count;
    *available -= count;
    return count;
}


size_t byte_ring_get_write_ptr(byte_ring_state* state, uint8_t** const ptr_out)
{
    const size_t remainingSpace = byte_ring_free_space(state);
    const size_t capacity = byte_ring_capacity(state);

    const size_t head = state->head % capacity;
    const size_t bytesUntilWrap = capacity - head;

    *ptr_out = &state->buffer[head];

    if( remainingSpace >= bytesUntilWrap )
        return bytesUntilWrap;
    else
        return remainingSpace;
}

void byte_ring_notify_written(byte_ring_state* state, size_t count)
{
    assert( count <= byte_ring_free_space(state) );

    state->head += count;
}
