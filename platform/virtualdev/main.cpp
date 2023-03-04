#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

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

/// @brief Creates a pseudo-terminal that openocd can connect to
class PseudoTerminal
{
  public:
    PseudoTerminal() = delete;
    PseudoTerminal(const PseudoTerminal &) = delete;
    PseudoTerminal(PseudoTerminal &&) = delete;
    PseudoTerminal &operator=(const PseudoTerminal &) = delete;
    PseudoTerminal &operator=(PseudoTerminal &&) = delete;

    PseudoTerminal(PosixFile &&terminalDescriptor) : m_LocalEnd{std::move(terminalDescriptor)}
    {
    }

    std::string GetPtsName() const
    {
        return ptsname(m_LocalEnd.Descriptor());
    }

    const PosixFile &Terminal() const
    {
        return m_LocalEnd;
    }

    static std::unique_ptr<PseudoTerminal> Create()
    {
        int fdm = posix_openpt(O_RDWR);
        if (fdm == -1)
        {
            return nullptr;
        }

        PosixFile terminal{fdm};

        struct termios term;
        if (tcgetattr(fdm, &term))
        {
            return nullptr;
        }
        if ((term.c_lflag & ECHO) != 0)
        {
            // Remove local echo
            term.c_lflag &= ~ECHO;
            if (tcsetattr(fdm, TCSANOW, &term))
            {
                return nullptr;
            }
        }

        if (grantpt(fdm))
        {
            return nullptr;
        }
        if (unlockpt(fdm))
        {
            return nullptr;
        }

        return std::make_unique<PseudoTerminal>(std::move(terminal));
    }

  protected:
  private:
    PosixFile m_LocalEnd;
};

static void startRjCore(int fd);

int main(int argc, char *argv[])
{
    std::unique_ptr<PseudoTerminal> pt;
    std::unique_ptr<PosixFile> remoteFile;
    int remoteFd = -1;

    if (argc == 1)
    {
        pt = PseudoTerminal::Create();
        if (!pt)
        {
            std::cout << "Failed to create pseudo-terminal\n";
            return -1;
        }

        std::cout << "Connect OpenOCD to: '" << pt->GetPtsName() << "'\n";

        remoteFd = pt->Terminal().Descriptor();
    }
    else
    {
        std::cout << "Attempting to open: '" << argv[1] << "'\n";
        remoteFile = PosixFile::Open(argv[1]);
        if (!remoteFile)
        {
            std::cout << "Failed to open.\n";
            return -1;
        }

        remoteFd = remoteFile->Descriptor();
    }

    if (remoteFd < 0)
    {
        std::cout << "Failed to acquire descriptor\n";
        return -1;
    }

    startRjCore(remoteFd);

    return 0;
}

static void startRjCore(int fd)
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
