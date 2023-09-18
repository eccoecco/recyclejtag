/*!
    serialqueue.h - Simple ISR/thread safe serial data queue for Zephyr

    Restrictions:
        * Only allows single writer, single reader.
        * All queues are a fixed size
*/

#pragma once

#include <zephyr/kernel.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SERIAL_QUEUE_ELEMENTS 1024
#define MESSAGE_QUEUE_ELEMENTS 32

struct serial_queue
{
    uint8_t serial_buffer[SERIAL_QUEUE_ELEMENTS]; //!< Buffer for the serial data

    atomic_t write_index;
    atomic_t read_index;

    struct k_event event_unread_data; //!< Set when there's unread data
};

// Initialises a serial queue
void serial_queue_init(struct serial_queue *queue);

// Reads data from the serial queue, waiting a timeout for data to arrive if non already present
// Returns the amount of data read, or < 0 if an error occurred
int serial_queue_read(struct serial_queue *queue, void *buffer, size_t length, k_timeout_t timeout);

// Writes data to the queue, returning number of bytes written, or < 0 for an error
int serial_queue_write(struct serial_queue *queue, const void *buffer, size_t length);

#ifdef __cplusplus
}
#endif
