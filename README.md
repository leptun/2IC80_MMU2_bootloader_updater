# 2IC80_MMU2_bootloader_updater

This repository contains the code necessary to perform a self-update of the Original Prusa MMU2 bootloader using exploits in the original bootloader.

The source code is organized into multiple parts:

## 1. Caterina-Bootloader

This code is a fork of prusa3d-Caterina bootloader code that is used in the MMU2 from the factory. It is based on the repository which can be forund [here](https://github.com/prusa3d/caterina).

Improvements over the original code:
- MiniBootloader: Also known as BootloaderAPI, this is a small piece of code that is needed for programming the microcontroller flash. It is used both by the bootloader itself and also by the bootloader updater code. It is necessary so that hacks can be avoided in newer bootloader revision when updating.
- UART support: So that the firmware and bootloader can be uploaded from the printer, not only from USB.
- `74HC595` (SHR16) support for an LED animation and for disabling the Stepper motors on startup (as a safety feature).

## 2. MMU2_bootloader_update

This is the source for the tool we wrote which is responsible for updating the bootloader (using the existing bootloader code) using exploits in the original code and other low level AVR tricks.

This tool can be used to update from the original bootloader to the new bootloader. Future updates are also supported.

## 3. Misc

In this folder we keep some useful files and pieces of documentation from manufacturers.

## 4. lufa

This is a submodule of the LUFA USB library which is used by the Caterina bootloader code. It is needed for compiling the bootloader.

# Build instructions

In order to build this project, you will need the following packages:
- avr-gcc 5.4.0 (or gcc-avr depending on the distro)
- avr-libc 2.0.0
- make
- binutils
- bc

Also, make sure that the project also has submodules initialized in case you build the bootloader code. Otherwise LUFA will be missing.

The project is built using the `make` system.

In order to build the bootloader, run `make` in the `Caterina-Bootloader` directory. The compiled output should be placed in an `out` directory. If changes are made to the bootloader, then the new bootloader code must be updated in `MMU2_bootloader_update/binFiles.c`.

In order to build the bootloader updater code, run simply run `make` in the `MMU2_bootloader_update` directory. The compiled program hex file will be placed in the root of that directory. The file has a custom header, so it will be properly detected as a MMU2 firmware update by the firmware upload software (PrusaSlicer).
