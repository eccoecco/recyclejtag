#include <array>
#include <iostream>

#include <unistd.h>

#include "runcore.h"

void startRjCore(int fd)
{
    std::array<char, 64> buffer;
    int bytesRead;

    while ((bytesRead = read(fd, buffer.data(), buffer.size())) > 0)
    {
        // Just do an echo server for proof of concept
        write(fd, buffer.data(), bytesRead);
    }

    if (bytesRead < 0)
    {
        std::cout << "Error while reading\n";
    }
}
