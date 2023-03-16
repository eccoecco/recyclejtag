# Recycle JTAG: LPC845-BRK

The LPC845-BRK is a very cheap, but capable, development board for the NXP LPC845 microcontroller.

An advantage of the LPC845 is the onboard CMSIS-DAP debug probe, which also doubles as a virtual serial port.  Now, because the virtual serial port is hooked up to the LPC845's UART, as opposed to being an on-chip USB peripheral, this port only supports the "normal" baud rate of 115,200, as opposed to the fast serial port of 1Mbit/s.

In this application, another advantage of the LPC845 itself is that its SPI port allows transfers from 1-16 bits, which is very useful since Jtag is a bit oriented protocol, as opposed to being byte oriented.

## Initial Setup

### Obtain arm-none-eabi toolchain

Grab a recent version of the toolchain - I get it from [ARM](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).  I believe that some Linux distributions also have it available.  The toolchain from ARM tends to be one of the most recent; sometimes distribution provided toolchains are a bit dated.

Note that the toolchain does not have to be in the path.

I personally installed the toolchain so that `arm-none-eabi-gcc` can be found in `${HOME}/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc`

Since that is not in the path, you will need to set the `ARMGCC_DIR` environment variable to `${HOME}/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/` when configuring the project with CMake.

### Grab NXP's MCUXpresso SDK

You'll need to grab [NXP's MCUXpresso SDK](https://mcuxpresso.nxp.com/en/welcome).  NXP requires you to configure what you want to download, and here are the settings that I use:

* OS: Linux or Windows
* Middleware: None
* Toolchain: GCC ARM Embedded 10-2021.10 (even though the version of gcc we're using is newer, that should be okay)

In this SDK are useful files, such as LPC845 header files, startup scripts, and linker scripts.  It also integrates nicely with CMake.

I have installed this so that the SDK manifest can be found in `${HOME}/mcuxpresso/sdk-2.13-lpc845/LPC845_manifest_v3_10.xml`

Set `LPC845_SDK_DIR` to this path.

### Grab `pyocd` for Programming and Debugging

Install [pyOCD](https://pyocd.io/) to allow easy access to the CMSIS-DAP debug probe on the LPC845-BRK development board. To install, you need to have a recent version of Python 3. The simplest way which makes it available to the current user is to simply run:
```
python3 -m pip install -U pyocd
```

If on Linux, you may see warnings like `WARNING: The script natsort is installed in '/home/username/.local/bin' which is not on PATH.` in which case, add `.local/bin` to the `PATH`.

Once installed, you should be able to run it from the command prompt, and get a result like:

```
pyocd list
  #   Probe/Board                                    Unique ID   Target
-------------------------------------------------------------------------
  0   NXP Semiconductors LPC11U3x CMSIS-DAP v1.0.7   17035034    n/a
```

If you're running WSL2, you may use to investigate using `usbipd` to forward USB connections from a Windows host to the WSL instance, and then setting up udev rules to allow access.

Something like in a file in `/etc/udev/rules.d/50-cmsis-dap.rules` (copied and modified from main `pyOCD` distribution):
```
# 1fc9:0143 NXP LPC845 Breakout
SUBSYSTEM=="usb", ATTR{idVendor}=="1fc9", ATTR{idProduct}=="0132", MODE:="666"
```

You may need to run `sudo service udev restart` every time your WSL instance starts, because daemons are not kept running in WSL, and this is needed to get `udev` running.

### Manually accessing virtual COM port

During development, it is entirely possible that you're doing `cat /dev/ttyACM0` on one terminal, and echoing data on another.

In this case, you'll need to put the terminal into raw mode, disable echo, and set the correct baud rate:

```
stty -F /dev/ttyACM0 raw 115200 -echo
```

This disables a lot of processing, and also, more importantly, disables echoing, otherwise data from the LPC845 gets echoed back to itself, causing an awful infinite loop of data.

You can check to make sure that the serial port has echo disabled by querying it:

```
$ stty -F /dev/ttyACM0
speed 115200 baud; line = 0;
min = 1; time = 0;
-brkint -icrnl -imaxbel
-opost
-isig -icanon -echo
```

### Example `settings.json` for VSCode

In `platform/lpc845-brk/.vscode`, you can update `settings.json` with something like:

```
{
    "cmake.configureSettings": {
        "CMAKE_TOOLCHAIN_FILE": "toolchain/arm-none-eabi.cmake"
    },
    "cmake.environment": {
        "ARMGCC_DIR": "${userHome}/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi",
        "LPC845_SDK_DIR": "${userHome}/mcuxpresso/sdk-2.13-lpc845"
    }
}
```

This will point CMake to use the correct toolchain file, and set the CMake environment variables to where `arm-none-eabi` and the MCUXpresso SDK is stored.

Be sure to set the kit to "No active kit" or "\[Unspecified\]", otherwise CMake will attempt to configure the project with the kit while having a toolchain file that's trying to overwrite the compilers that the kit is trying to specify.  It's a mess.  Don't do it.

### Example `launch.json` for VSCode

In `platform/lpc845-brk/.vscode`, you can add a launch entry into `launch.json` like so:

```
        {
            "name": "Cortex Debug",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/rjtag.elf",
            "gdbPath": "gdb-multiarch",
            "request": "launch",
            "type": "cortex-debug",
            "runToEntryPoint": "main",
            "servertype": "pyocd",
            "armToolchainPath": "${userHome}/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin",
            "targetId": "lpc845"
        }
```

You must have installed the Cortex Debug extension, which will handle invoking `pyocd` for you.

`targetId` is required, otherwise flashing the device will fail.

Note that `gdbPath` is set to `gdb-multiarch` provided by Ubuntu (`sudo apt install gdb-multiarch`) as opposed to the version of gdb provided by ARM, because it's missing a few dependencies and I can't be bothered fixing it when a workable alternative is present.  Trying to run `arm-none-eabi-gdb` gives:

```
$ ~/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb
~/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb: error while loading shared libraries: libncursesw.so.5: cannot open shared object file: No such file or directory
```

and once that is installed, some error about Python.