/*!
    @file pseudoterminal.h
    @brief Opens a pseudo-terminal under Linux
*/

#pragma once

#include <memory>
#include <string>

#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include "posixfile.h"

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
