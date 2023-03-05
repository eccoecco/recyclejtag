/*!
    @file posixfile.h
    @brief Helper for RAII file closure
*/

#pragma once

#include <memory>

#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

/// @brief Basically a glorified way of making sure that a file descriptor is closed on scope exit
class PosixFile
{
  public:
    PosixFile() = delete;
    PosixFile(const PosixFile &) = delete;
    PosixFile &operator=(const PosixFile &) = delete;

    PosixFile(int fd) : m_fd{fd} {};
    ~PosixFile()
    {
        if (m_fd >= 0)
        {
            close(m_fd);
        }
    }

    PosixFile(PosixFile &&other) : m_fd{other.m_fd}
    {
        other.m_fd = -1;
    }
    PosixFile &operator=(PosixFile &&other)
    {
        // Temp used to handle edge case where *this = std::move(*this);
        int temp = other.m_fd;
        other.m_fd = -1;
        m_fd = temp;
        return *this;
    }

    int Descriptor() const
    {
        return m_fd;
    }

    static std::unique_ptr<PosixFile> Open(const char *remotePath)
    {
        int fd = open(remotePath, O_RDWR);
        if (fd == -1)
        {
            return nullptr;
        }

        return std::make_unique<PosixFile>(fd);
    }

  protected:
  private:
    int m_fd;
};