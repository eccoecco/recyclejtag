#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

#include "posixfile.h"
#include "pseudoterminal.h"
#include "runcore.h"

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
