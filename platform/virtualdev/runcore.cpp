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
};

namespace
{

uint32_t PlatformImpl_CurrentUptime(void *)
{
    struct timespec systemTime;
    clock_gettime(CLOCK_MONOTONIC, &systemTime);

    return systemTime.tv_sec + systemTime.tv_nsec / 1000000;
}

void PlatformImpl_TransmitData(void *platformData, const char *buffer, size_t length)
{
    auto data = static_cast<PlatformData *>(platformData);

    write(data->fd, buffer, length);
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

    RJCoreState rjcore;
    RJCorePlatform platform{
        .currentUptime = PlatformImpl_CurrentUptime,
        .transmitData = PlatformImpl_TransmitData,
    };
    PlatformData platformData{
        .fd = fd,
    };

    RJCore_Init(&rjcore, &platform, &platformData);

    while (true)
    {
        struct epoll_event event;
        int descriptorsReady;
        std::array<char, 64> buffer;
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

        RJCore_NotifyDataReceived(&rjcore, buffer.data(), bytesRead);
    }
}
