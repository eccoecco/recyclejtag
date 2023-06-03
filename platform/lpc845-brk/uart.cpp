#include <array>
#include <cstddef>

#include <LPC845.h>

#include "uart.h"

namespace
{

constexpr size_t ReceiveBufferSize = 1024; //!< Can receive a lot of data from a computer

/// @brief No need for a big transmit buffer, since the transmit is constrained by the receive
constexpr size_t TransmitBufferSize = 256;

template <size_t bufferSize> struct CircularBuffer
{
    static constexpr uint32_t bufferIndexMask = bufferSize - 1;
    static_assert((bufferIndexMask & bufferSize) == 0, "Buffer size must be a power of 2");

    std::array<uint8_t, bufferSize> Buffer;

    // Cortex M0 guarantees atomic read/writes to aligned 32-bit variables
    uint32_t ReadIndex = 0;
    uint32_t WriteIndex = 0;

    inline uint32_t IncrementIndex(uint32_t value) const
    {
        return (value + 1) & bufferIndexMask;
    }

    bool Full() const
    {
        return IncrementIndex(WriteIndex) == ReadIndex;
    }

    bool Empty() const
    {
        return (ReadIndex == WriteIndex);
    }

    inline void Write(uint8_t value)
    {
        Buffer[WriteIndex] = value;
        WriteIndex = IncrementIndex(WriteIndex);
    }

    inline uint8_t Read()
    {
        uint8_t value = Buffer[ReadIndex];
        ReadIndex = IncrementIndex(ReadIndex);
        return value;
    }
};

namespace Internal
{

CircularBuffer<ReceiveBufferSize> ReceiveBuffer;
CircularBuffer<TransmitBufferSize> TransmitBuffer;

} // namespace Internal
} // namespace

namespace Uart
{

void WriteCharacter(uint8_t value)
{
    Internal::TransmitBuffer.Write(value);

    USART0->INTENSET = USART_INTENSET_TXRDYEN(1);
}

int ReadCharacter(void)
{
    if (Internal::ReceiveBuffer.Empty())
    {
        return -1;
    }

    return Internal::ReceiveBuffer.Read();
}

} // namespace Uart

// Need to declare this as C, otherwise name mangling means that this doesn't get registered
// as the proper interrupt handler
extern "C"
{

void USART0_IRQHandler(void)
{
    uint32_t usart0Status = USART0->STAT;

    if ((usart0Status & USART_STAT_RXRDY(1)) != 0)
    {
        uint8_t receivedByte = USART0->RXDAT;

        if (!Internal::ReceiveBuffer.Full())
        {
            Internal::ReceiveBuffer.Write(receivedByte);
        }
    }

    if ((usart0Status & USART_STAT_TXRDY(1)) != 0)
    {
        if (Internal::TransmitBuffer.Empty())
        {
            USART0->INTENCLR = USART_INTENCLR_TXRDYCLR(1);
        }
        else
        {
            uint8_t value = Internal::TransmitBuffer.Read();

            USART0->TXDAT = value;
        }
    }
}
}