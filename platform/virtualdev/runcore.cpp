#include <array>
#include <ctime>
#include <iostream>

#include <unistd.h>

#include "runcore.h"
#include <rjcore/rjcore.h>

struct PlatformData
{
    int fd;
};

static uint32_t PlatformImpl_CurrentUptime(void *)
{
    struct timespec systemTime;
    clock_gettime(CLOCK_MONOTONIC, &systemTime);

    return systemTime.tv_sec + systemTime.tv_nsec / 1000000;
}

static void PlatformImpl_TransmitData(void *platformData, const char *buffer, size_t length)
{
    auto data = static_cast<PlatformData *>(platformData);

    write(data->fd, buffer, length);
}

void startRjCore(int fd)
{
    std::array<char, 64> buffer;
    int bytesRead;

    RJCoreState rjcore;
    RJCorePlatform platform{
        .currentUptime = PlatformImpl_CurrentUptime,
        .transmitData = PlatformImpl_TransmitData,
    };
    PlatformData platformData{
        .fd = fd,
    };

    RJCore_Init(&rjcore, &platform, &platformData);

    // TODO: Swap to poll, and implement timeouts
    while ((bytesRead = read(fd, buffer.data(), buffer.size())) > 0)
    {
        if (bytesRead > 0)
        {
            RJCore_NotifyDataReceived(&rjcore, buffer.data(), bytesRead);
        }
    }

    if (bytesRead < 0)
    {
        std::cout << "Error while reading\n";
    }
}
