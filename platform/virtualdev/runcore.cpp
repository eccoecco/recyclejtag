#include <array>
#include <ctime>
#include <iostream>
#include <memory>

#include <signal.h>
#include <sys/epoll.h>
#include <unistd.h>

#include "posixfile.h"
#include "runcore.h"

#include <rjcore/rjcore.h>

struct PlatformData
{
    int fd = -1;
    int currentSerialBaud = 0;
    struct
    {
        int totalBitsToShift = 0;
        int bitsShifted = 0;
    } packet;
};

namespace
{

// To examine various operation modes, select one of:
//   RJCoreTapShiftMode_GPIO
//   RJCoreTapShiftMode_Packet
//   RJCoreTapShiftMode_Custom
constexpr enum RJCoreTapShiftMode selectedTapShiftMode = RJCoreTapShiftMode_Custom;

uint32_t PlatformImpl_CurrentUptime(void *)
{
    struct timespec systemTime;
    clock_gettime(CLOCK_MONOTONIC, &systemTime);

    return systemTime.tv_sec + systemTime.tv_nsec / 1000000;
}

void PlatformImpl_TransmitData(void *privateData, const void *buffer, size_t length)
{
    auto platformData = static_cast<PlatformData *>(privateData);

    write(platformData->fd, buffer, length);
}

bool PlatformImpl_SetSerialMode(void *privateData, enum RJCoreSerialMode mode)
{
    auto platformData = static_cast<PlatformData *>(privateData);

    int baud = (mode == 0) ? 115200 : (mode == 1) ? 1000000 : -1;

    if (baud < 0)
    {
        std::cout << "Unknown serial mode: " << mode << '\n';
        return false;
    }

    if (platformData->currentSerialBaud != baud)
    {
        platformData->currentSerialBaud = baud;

        // Yeah, sure, we support all sorts of baud rates.
        std::cout << "Serial port baud request to " << baud << " baud\n";
    }

    return true;
}

bool PlatformImpl_SetPortMode(void *, enum RJCorePortMode mode)
{
    switch (mode)
    {
    case RJCorePortMode_HighImpedance:
        std::cout << "Set port to high impedance\n";
        break;
    case RJCorePortMode_PushPull:
        std::cout << "Set port to push pull\n";
        break;
    case RJCorePortMode_OpenDrain:
        std::cout << "Set port to open drain\n";
        break;
    default:
        std::cout << "Unknown port mode: " << mode << '\n';
        return false;
    }

    return true;
}

bool PlatformImpl_SetFeature(void *, enum RJCoreFeature feature, int action)
{
    std::string description{(action != 0) ? "on" : "off"};

    if ((feature & RJCoreFeature_LED) != 0)
    {
        std::cout << "Feature LED: " << description << '\n';
    }
    if ((feature & RJCoreFeature_VReg) != 0)
    {
        std::cout << "Feature Vreg: " << description << '\n';
    }
    if ((feature & RJCoreFeature_TRst) != 0)
    {
        std::cout << "Feature Trst: " << description << '\n';
    }
    if ((feature & RJCoreFeature_SRst) != 0)
    {
        std::cout << "Feature Srst: " << description << '\n';
    }
    if ((feature & RJCoreFeature_Pullup) != 0)
    {
        std::cout << "Feature Pullup: " << description << '\n';
    }

    return true;
}

void PlatformImpl_ReadVoltages(void *, uint16_t *values)
{
    for (int i = 0; i < 4; ++i)
    {
        values[i] = 0;
    }
}

void PlatformImpl_NewTapShift(void *privateData, int totalBitsToShift)
{
    auto platformData = static_cast<PlatformData *>(privateData);
    platformData->packet.bitsShifted = 0;
    platformData->packet.totalBitsToShift = totalBitsToShift;

    std::cout << "New tap shift of " << totalBitsToShift << " bits\n";
}

int PlatformImpl_TapShiftGPIOClock(void *, int tdi, int tms)
{
    (void)tdi;
    (void)tms;
    return 0;
}

inline void TapShiftPacket_ProcessByte(PlatformData *platformData, int bitsToProcess, uint8_t tdi, uint8_t tms)
{
    (void)bitsToProcess;
    (void)tdi;
    (void)tms;

    uint8_t reply = 0;

    write(platformData->fd, &reply, 1);

    platformData->packet.bitsShifted += bitsToProcess;
}

// In partial control, this lets the state machine still get notified by uart events, and handles
// buffering and passing pointer data over.  Multiple calls to this are expected, as packets of data
// come in.
int PlatformImpl_TapShiftPacket(void *privateData, const uint8_t *buffer, int bitsToShift)
{
    auto platformData = static_cast<PlatformData *>(privateData);

    std::cout << "Packet to process has bits to shift: " << bitsToShift << '\n';

    for (; bitsToShift > 0; bitsToShift -= 8, buffer += 2)
    {
        int bitsValid = 8;
        if (bitsValid > bitsToShift)
        {
            bitsValid = bitsToShift;
        }

        TapShiftPacket_ProcessByte(platformData, bitsValid, buffer[0], buffer[1]);
    }

    if (platformData->packet.bitsShifted == platformData->packet.totalBitsToShift)
    {
        std::cout << "Finished processing an entire tap shift\n";
    }
    else if (platformData->packet.bitsShifted > platformData->packet.totalBitsToShift)
    {
        std::cout << "INTERNAL ERROR: Shifted more bits than expected!?\n";
    }

    return 0;
}

int PlatformImpl_TapShiftCustom(void *privateData, const uint8_t *coreBuffer, int coreBufferLength,
                                int totalBitsToShift)
{
    std::array<uint8_t, 32> localBuffer;
    int bytesInBuffer = 0;

    auto platformData = static_cast<PlatformData *>(privateData);

    if ((coreBufferLength & 0x1) != 0)
    {
        // An odd number of bytes were presented from the core
        // Since tdi/tms are processed in even pairs, we tuck away that odd byte
        // at the end in our local buffer, to ensure that coreBufferLength is even
        bytesInBuffer = 1;
        localBuffer[0] = coreBuffer[coreBufferLength - 1];
        --coreBufferLength;
    }

    if (coreBufferLength > 0)
    {
        // Process pairs of existing data
        int bitsAvailable = coreBufferLength << 2;
        if (bitsAvailable > totalBitsToShift)
        {
            bitsAvailable = totalBitsToShift;
        }

        // Just call this as an example of processing data
        if (PlatformImpl_TapShiftPacket(privateData, coreBuffer, bitsAvailable) < 0)
        {
            return -1;
        }
    }

    while (platformData->packet.bitsShifted < platformData->packet.totalBitsToShift)
    {
        // Need to ensure that we only read as many bytes as we need to for the tap shift
        // No more, because that would mean that we accidentally get data for the next
        // command (if it was already sent).
        int bitsLeftToShift = platformData->packet.totalBitsToShift - platformData->packet.bitsShifted;
        int bytesRemaining = ((bitsLeftToShift + 7) >> 3) * 2;
        if (bytesRemaining > static_cast<int>(localBuffer.size()))
        {
            bytesRemaining = static_cast<int>(localBuffer.size());
        }
        bytesRemaining -= bytesInBuffer;

        int bytesRead = read(platformData->fd, localBuffer.data() + bytesInBuffer, bytesRemaining);

        if (bytesRead <= 0)
        {
            // Shouldn't happen, if it does, no recovery
            return -1;
        }

        bytesInBuffer += bytesRead;

        if (bytesInBuffer > 1)
        {
            // Only process even numbers of bytes
            int bitsInBuffer = (bytesInBuffer & ~0x1) << 2;
            if (bitsInBuffer > bitsLeftToShift)
            {
                bitsInBuffer = bitsLeftToShift;
            }
            if (PlatformImpl_TapShiftPacket(privateData, localBuffer.data(), bitsInBuffer) < 0)
            {
                return -1;
            }
        }

        if ((bytesInBuffer & 1) != 0)
        {
            // Odd bytes leave 1 byte in the buffer
            localBuffer[0] = localBuffer[bytesInBuffer - 1];
            bytesInBuffer = 1;
        }
        else
        {
            // All even bytes should have been processed
            bytesInBuffer = 0;
        }
    }

    return 0;
}

std::unique_ptr<PosixFile> CreateEpollInstance(int fd)
{
    int epfd = epoll_create(1);
    if (epfd < 0)
    {
        return nullptr;
    }

    auto ep = std::make_unique<PosixFile>(epfd);

    struct epoll_event event
    {
        .events = EPOLLIN, .data = {},
    };
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &event) < 0)
    {
        return nullptr;
    }

    return ep;
}

constexpr int epollTimeout_ms = 100;

void ignoreSighup()
{
    // Mask out SIGHUP, so that the remote pseudo-terminal closing doesn't interrupt this
    sigset_t signals;
    sigemptyset(&signals);
    sigaddset(&signals, SIGHUP);
    sigprocmask(SIG_BLOCK, &signals, NULL);
}

} // namespace

void startRjCore(int fd)
{
    ignoreSighup();

    auto ep = CreateEpollInstance(fd);
    if (!ep)
    {
        std::cout << "Failed to create epoll instance\n";
        return;
    }

    RJCoreHandle rjcoreHandle;
    RJCorePlatform platform{
        .tapShiftMode = selectedTapShiftMode,
        .currentUptime = PlatformImpl_CurrentUptime,
        .transmitData = PlatformImpl_TransmitData,
        .setSerialMode = PlatformImpl_SetSerialMode,
        .setPortMode = PlatformImpl_SetPortMode,
        .setFeature = PlatformImpl_SetFeature,
        .readVoltages = PlatformImpl_ReadVoltages,
        .newTapShift = PlatformImpl_NewTapShift,
        .tapShiftGPIO = PlatformImpl_TapShiftGPIOClock,
        .tapShiftPacket = PlatformImpl_TapShiftPacket,
        .tapShiftCustom = PlatformImpl_TapShiftCustom,
    };
    PlatformData platformData{
        .fd = fd,
        .packet = {},
    };

    RJCore_Init(&rjcoreHandle, &platform, &platformData);

    while (true)
    {
        struct epoll_event event;
        int descriptorsReady;
        std::array<uint8_t, 64> buffer;
        int bytesRead;

        descriptorsReady = epoll_wait(ep->Descriptor(), &event, 1, epollTimeout_ms);

        if (descriptorsReady < 0)
        {
            std::cout << "Epoll error\n";
            break;
        }

        if (descriptorsReady == 0)
        {
            bytesRead = 0;
        }
        else
        {
            bytesRead = read(fd, buffer.data(), buffer.size());
            if (bytesRead == 0)
            {
                // SIGHUP - other end has closed the connection
                std::cout << "Remote end closed connection\n";
                break;
            }
        }

        if (bytesRead < 0)
        {
            std::cout << "Error while reading\n";
            break;
        }

        RJCore_NotifyDataReceived(&rjcoreHandle, buffer.data(), bytesRead);
    }
}
