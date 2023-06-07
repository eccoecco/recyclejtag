# Recycle JTAG: Zephyr RTOS

Zephyr RTOS is a modern RTOS, that deals with a lot of the low level drivers and initialisation necessary to bring a board up.

## Setting Up

Rather than the vanilla Zephyr distribution, I use Nordic's variant because:
1. It integrates nicely with the [nRF Connect Extension for VSCode](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-VS-Code) extension.
2. I'm not required to add a whole bunch of executables (useful though that they are) to my user PATH, accidentally creating a version dependency because some tool requires a specific version, but another tool requires a newer version etc etc.

Use [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop) to download their `Toolchain Manager`.  Using the `Toolchain Manager`, further download a recent version of the `nRF Connect SDK`.  At time of writing, I am using v2.4.0.  This will download Nordic's variant on Zephyr, toolchains, etc.

Note: Since Nordic's Zephyr distribution is focussed on their MCUs, if using another manufacturer's MCU, you might need to modify the manifest.  For example, to compile for STM32s, you will need to go into `${WHERE_YOU_INSTALLED_THE_SDK}/v${SDK_VERSION}/nrf/west.yml`, and edit it to include `hal_stm32`:
```
        name-allowlist:
          - TraceRecorderSource
          - canopennode
          - chre
          - cmsis
          - edtt
          - fatfs
          - hal_nordic
          - hal_st # required for ST sensors (unrelated to STM32 MCUs)
          - hal_stm32 # ---- Add this line to download STM32 headers ----
          - hal_wurthelektronik
          - liblc3
          - libmetal
```

If you do not add that line, your STM32 builds will complain about missing `*pinctrl.dtsi` files when parsing the devicetree.  After doing this, go back into the toolchain manager, and selecting the drop down for the matching version of the SDK you just updated, select "Open Command Prompt", and run `west update` to download the new submodule.

You will also want to download and install the [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download), as the `nRF Connect Extension` requires it to function correctly.

Once everything is installed, set up the Zephyr project by:
1. Opening the `nRF Connect` extension in VSCode
2. Under the `Welcome` panel, click `Open an existing application`
3. Select the `recyclejtag/platform/zephyr` directory
4. Under the `Applications` panel, the `zephyr` directory should now be added
5. Click `No build configuration - Click to create one` to create a build configuration
6. Select the board that you are using (e.g. the `blackpill_f411ce` for an STM32 development stick)
7. I also tend to `Enable debug options`, for debugging.  Then click `Build Configuration`