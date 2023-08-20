#include "serial_queue.h"

#include <string.h>
#include <zephyr/kernel.h>

#define TEST_POWER_OF_TWO(x) (((x) & ((x)-1)) == 0)

BUILD_ASSERT(TEST_POWER_OF_TWO(SERIAL_QUEUE_ELEMENTS), "Must have a power of two elements in serial queue");

#define SERIAL_QUEUE_INDEX_MASK ((SERIAL_QUEUE_ELEMENTS)-1)

void serial_queue_init(struct serial_queue *queue)
{
    atomic_set(&queue->write_index, 0);
    atomic_set(&queue->read_index, 0);

    k_msgq_init(&queue->message_queue, queue->msg_queue_buffer, sizeof(uint32_t), MESSAGE_QUEUE_ELEMENTS);
}

static uint32_t serial_queue_space_used(const struct serial_queue *queue)
{
    uint32_t write_index = atomic_get(&queue->write_index);
    uint32_t read_index = atomic_get(&queue->read_index);

    return (write_index - read_index) & SERIAL_QUEUE_INDEX_MASK;
}

static uint32_t serial_queue_space_left(const struct serial_queue *queue)
{
    // Because read_index == write_index is considered as empty, we can never use the full
    // SERIAL_QUEUE_ELEMENTS number of bytes, hence the -1
    return SERIAL_QUEUE_ELEMENTS - 1 - serial_queue_space_used(queue);
}

// Reads data from the serial queue, waiting a timeout for data to arrive if non already present
// Returns the amount of data read, or < 0 if an error occurred
int serial_queue_read(struct serial_queue *queue, void *buffer, size_t length, k_timeout_t timeout)
{
    return 0;
}

// Writes data to the queue, returning number of bytes written, or < 0 for an error
int serial_queue_write(struct serial_queue *queue, const void *buffer, size_t length)
{
    uint32_t space_left = serial_queue_space_left(queue);

    if (length > space_left)
    {
        length = space_left;
    }

    if (length == 0)
    {
        return 0;
    }

    uint32_t write_index = atomic_get(&queue->write_index);
    uint32_t encoded_message = (write_index & 0xFFFF) | (length << 16);

    uint32_t maximum_contiguous_write = SERIAL_QUEUE_ELEMENTS - write_index;
    uint32_t initial_write = length;
    uint32_t leftover_write = 0;

    if (initial_write > maximum_contiguous_write)
    {
        leftover_write = length - maximum_contiguous_write;
        initial_write = maximum_contiguous_write;
    }

    memcpy(queue->serial_buffer + write_index, buffer, initial_write);

    if (leftover_write > 0)
    {
        memcpy(queue->serial_buffer, ((const uint8_t *)buffer) + initial_write, leftover_write);
    }

    write_index += length;
    write_index &= SERIAL_QUEUE_INDEX_MASK;

    atomic_set(&queue->write_index, write_index);

    int result = k_msgq_put(&queue->message_queue, &encoded_message, K_NO_WAIT);

    return (result == 0) ? length : result;
}