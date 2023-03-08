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
    int fd;
    struct
    {
        int bitsToShift;
        int tdi;
    } packet; // Used in tap shift packet mode
};

namespace
{

uint32_t PlatformImpl_CurrentUptime(void *)
{
    struct timespec systemTime;
    clock_gettime(CLOCK_MONOTONIC, &systemTime);

    return systemTime.tv_sec + systemTime.tv_nsec / 1000000;
}

void PlatformImpl_TransmitData(void *platformData, const void *buffer, size_t length)
{
    auto data = static_cast<PlatformData *>(platformData);

    write(data->fd, buffer, length);
}

bool PlatformImpl_SetSerialMode(void *platformData, enum RJCoreSerialMode mode)
{
    (void)platformData;

    int baud = (mode == 0) ? 115200 : (mode == 1) ? 1000000 : -1;

    if (baud < 0)
    {
        std::cout << "Unknown serial mode: " << mode << '\n';
        return false;
    }

    // Yeah, sure, we support all sorts of baud rates.
    std::cout << "Serial port baud request to " << baud << " baud\n";
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

constexpr enum RJCoreTapShiftMode selectedTapShiftMode = RJCoreTapShiftMode_Packet;
constexpr bool selectedTapShiftFullControl = false;

void PlatformImpl_NewTapShift(void *privateData, int totalBitsToShift)
{
    auto platformData = static_cast<PlatformData *>(privateData);
    platformData->packet.bitsToShift = totalBitsToShift;
    platformData->packet.tdi = -1;

    std::cout << "New tap shift of " << totalBitsToShift << " bits\n";
}

int PlatformImpl_TapShiftGPIOClock(void *, int tdi, int tms)
{
    (void)tdi;
    (void)tms;
    return 0;
}

inline void TapShiftPacket_ProcessByte(PlatformData *platformData, uint8_t tdi, uint8_t tms)
{
    int bitsToProcess = 8;
    if (bitsToProcess > platformData->packet.bitsToShift)
    {
        bitsToProcess = platformData->packet.bitsToShift;
    }

    (void)tdi;
    (void)tms;

    uint8_t reply = 0;

    write(platformData->fd, &reply, 1);

    platformData->packet.bitsToShift -= bitsToProcess;
}

// In partial control, this lets the state machine still get notified by uart events, and handles
// buffering and passing pointer data over.  Multiple calls to this are expected, as packets of data
// come in.
int PlatformImpl_TapShiftPacket_PartialControl(void *privateData, const uint8_t *buffer, int bufferLength)
{
    auto platformData = static_cast<PlatformData *>(privateData);
    int bytesProcessed = 0;

    std::cout << "Packet to process of buffer length: " << bufferLength << '\n';

    if (bufferLength == 0)
    {
        return 0;
    }

    // Handle the case where there's a leftover tdi from the previous packet
    if (platformData->packet.tdi >= 0)
    {
        TapShiftPacket_ProcessByte(platformData, platformData->packet.tdi, *buffer);
        platformData->packet.tdi = -1;

        ++buffer;
        ++bytesProcessed;
        --bufferLength;
    }

    for (; (bufferLength > 1) && (platformData->packet.bitsToShift > 0); bufferLength -= 2, buffer += 2)
    {
        TapShiftPacket_ProcessByte(platformData, buffer[0], buffer[1]);
        bytesProcessed += 2;
    }

    if ((bufferLength == 1) && (platformData->packet.bitsToShift > 0))
    {
        // Oh dear, leftover tdi, and still more data to shift
        platformData->packet.tdi = *buffer;
        ++bytesProcessed;
    }

    if (platformData->packet.bitsToShift == 0)
    {
        std::cout << "Finished shifting a packet\n";
    }

    return (platformData->packet.bitsToShift == 0) ? bytesProcessed : 0;
}

int PlatformImpl_TapShiftPacket_FullControl(void *privateData, const uint8_t *buffer, int bufferLength)
{
    // If you want the callback to assume full control, then:
    // 1. Process the buffer that's passed in (and make sure that if there's more data than required,
    //    only the bytes used are returned)
    // 2. If more bytes are required, then access the serial port to grab more data directly, rather than
    //    going through the state machine.  For slower processors, removing this level of indirection is
    //    critical for best performance.
    // 3. Return the total bytes used from this buffer

    // TODO: Provide example

    (void)privateData;
    (void)buffer;
    (void)bufferLength;

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
        .tapShiftPacket = selectedTapShiftFullControl ? PlatformImpl_TapShiftPacket_FullControl
                                                      : PlatformImpl_TapShiftPacket_PartialControl,
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
