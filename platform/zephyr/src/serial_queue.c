#include "serial_queue.h"

#include <string.h>
#include <zephyr/kernel.h>

#define TEST_POWER_OF_TWO(x) (((x) & ((x)-1)) == 0)

BUILD_ASSERT(TEST_POWER_OF_TWO(SERIAL_QUEUE_ELEMENTS), "Must have a power of two elements in serial queue");

#define SERIAL_QUEUE_INDEX_MASK ((SERIAL_QUEUE_ELEMENTS)-1)

#define EVENT_UNREAD_DATA_MASK 0x0001

void serial_queue_init(struct serial_queue *queue)
{
    atomic_set(&queue->write_index, 0);
    atomic_set(&queue->read_index, 0);

    k_event_init(&queue->event_unread_data);
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

static uint32_t serial_queue_wait_for_write(struct serial_queue *queue, k_timeout_t timeout)
{
    // Immediately clear first, and then check read/write index, because the write function
    // updates read/write index, and then sets the event (and thus this must do the opposite)
    k_event_clear(&queue->event_unread_data, EVENT_UNREAD_DATA_MASK);

    // Handles race condition where a write *just* happened immediately after this clear

    uint32_t bytes_in_queue = serial_queue_space_used(queue);

    if (bytes_in_queue == 0)
    {
        if (k_event_wait(&queue->event_unread_data, EVENT_UNREAD_DATA_MASK, false, timeout) != 0)
        {
            bytes_in_queue = serial_queue_space_used(queue);
        }
    }

    return bytes_in_queue;
}

static void serial_queue_read_to_buffer(struct serial_queue *queue, void *buffer, size_t length)
{
    // TODO: This.  Handle the case of wraparound, as well.

    // Note: No need to manually compare read_index and write_index and set the event,
    // because that will be done at the start of the next call to serial_queue_read()
}

// Reads data from the serial queue, waiting a timeout for data to arrive if non already present
// Returns the amount of data read, or < 0 if an error occurred
int serial_queue_read(struct serial_queue *queue, void *buffer, size_t length, k_timeout_t timeout)
{
    uint32_t bytes_in_queue = serial_queue_space_used(queue);

    if ((bytes_in_queue == 0) && !K_TIMEOUT_EQ(timeout, K_NO_WAIT))
    {
        bytes_in_queue = serial_queue_wait_for_write(queue, timeout);
    }

    if (bytes_in_queue != 0)
    {
        if (bytes_in_queue > length)
        {
            bytes_in_queue = length;
        }
        serial_queue_read_to_buffer(queue, buffer, bytes_in_queue);
    }

    return bytes_in_queue;
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

    k_event_set(&queue->event_unread_data, EVENT_UNREAD_DATA_MASK);

    return length;
}