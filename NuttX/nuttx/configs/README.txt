Board-Specific Configurations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This directory contains the board support for logic for all boards supported
by NuttX.  This directory is retained in a separate repository and is a Sub-
Module of NuttX and will appear as nuttx/configs when the NuttX repository
is cloned.

The nuttx/configs directory is a part of the internal OS.  It there should
contain only OS bring-up logic and driver initialization logic.  THERE SHOULD
BE NO APPLICATION CALLABLE LOGIC IN THIS DIRECTORY.

If you have board-specific, application callable logic, that logic should not
go here.  Please consider using a sub-directory under apps/platform instead.

Table of Contents
^^^^^^^^^^^^^^^^^

  o Board-Specific Configurations
  o Summary of Files
  o Configuration Variables
  o Supported Boards
  o Configuring NuttX
  o Building Symbol Tables

Board-Specific Configurations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The NuttX configuration consists of:

o Processor architecture specific files.  These are the files contained
  in the arch/<arch-name>/ directory.

o Chip/SoC specific files.  Each processor processor architecture
  is embedded in chip or System-on-a-Chip (SoC) architecture.  The
  full chip architecture includes the processor architecture plus
  chip-specific interrupt logic, general purpose I/O (GIO) logic, and
  specialized, internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the arch/<arch-name>/ directory and are selected
  via the CONFIG_ARCH_name selection

o Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as
  peripheral LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  configs/<board-name>/ sub-directories and are discussed in this
  README.  Additional configuration information maybe available in
  board-specific configs/<board-name>/README.txt files.

The configs/ subdirectory contains configuration data for each board.  These
board-specific configurations plus the architecture-specific configurations in
the arch/ subdirectory completely define a customized port of NuttX.

Directory Structure
^^^^^^^^^^^^^^^^^^^

The configs directory contains board specific configurationlogic.  Each
board must provide a subdirectory <board-name> under configs/ with the
following characteristics:


  <board-name>
  |-- README.txt
  |-- include/
  |   `-- (board-specific header files)
  |-- src/
  |   |-- Makefile
  |   `-- (board-specific source files)
  |-- <config1-dir>
  |   |-- Make.defs
  |   |-- defconfig
  |   `-- setenv.sh
  |-- <config2-dir>
  |   |-- Make.defs
  |   |-- defconfig
  |   `-- setenv.sh
  ...

Summary of Files
^^^^^^^^^^^^^^^^

README.txt -- This text file provides additional information unique to
  each board configuration sub-directory.

include/ -- This directory contains board specific header files.  This
  directory will be linked as include/arch/board at configuration time and
  can be included via '#include <arch/board/header.h>'.  These header file
  can only be included by files in arch/<arch-name>include/ and
  arch/<arch-name>/src

src/ -- This directory contains board specific drivers.  This
  directory will be linked as arch/<arch-name>/src/board at configuration
  time and will be integrated into the build system.

src/Makefile -- This makefile will be invoked to build the board specific
  drivers.  It must support the following targets:  libext$(LIBEXT), clean,
  and distclean.

A board may have various different configurations using these common source
files.  Each board configuration is described by three files:  Make.defs,
defconfig, and setenv.sh.  Typically, each set of configuration files is
retained in a separate configuration sub-directory (<config1-dir>,
<config2-dir>, .. in the above diagram).

Make.defs -- This makefile fragment provides architecture and
  tool-specific build options.  It will be included by all other
  makefiles in the build (once it is installed).  This make fragment
  should define:

    Tools: CC, LD, AR, NM, OBJCOPY, OBJDUMP
    Tool options: CFLAGS, LDFLAGS

  When this makefile fragment runs, it will be passed TOPDIR which
  is the path to the root directory of the build.  This makefile
  fragment should include:

    $(TOPDIR)/.config          : Nuttx configuration
    $(TOPDIR)/tools/Config.mk  : Common definitions

  Definitions in the Make.defs file probably depend on some of the
  settings in the .config file.  For example, the CFLAGS will most likely be
  different if CONFIG_DEBUG_FEATURES=y.

  The included tools/Config.mk file contains additional definitions that may
  be overriden in the architecture-specific Make.defs file as necessary:

    COMPILE, ASSEMBLE, ARCHIVE, CLEAN, and MKDEP macros

defconfig -- This is a configuration file similar to the Linux
  configuration file.  In contains variable/value pairs like:

  CONFIG_VARIABLE=value

  This configuration file will be used at build time:

    (1) as a makefile fragment included in other makefiles, and
    (2) to generate include/nuttx/config.h which is included by
        most C files in the system.

setenv.sh -- This is a script that you can include that will be installed at
  the toplevel of the directory structure and can be sourced to set any
  necessary environment variables.  You will most likely have to customize the
  default setenv.sh script in order for it to work correctly in your
  environment.

Configuration Variables
^^^^^^^^^^^^^^^^^^^^^^^

At one time, this section provided a list of all NuttX configuration
variables. However, NuttX has since converted to use the kconfig-frontends
tools (See http://ymorin.is-a-geek.org/projects/kconfig-frontends).  Now,
the NuttX configuration is determined by a self-documenting set of Kconfig
files.

The current NuttX configuration variables are also documented in separate,
auto-generated configuration variable document.  That configuration variable
document is generated using the kconfig2html tool that can be found in the
nuttx/tools directory. That tool analyzes the NuttX Kconfig files and
generates an excruciatingly boring HTML document.

The latest boring configuration variable documentation can be regenerated at
any time using that tool or, more appropriately, the wrapper script at
nuttx/tools/mkconfigvars.sh.  That script will generate the file
nuttx/Documentation/NuttXConfigVariables.html.

The version of NuttXConfigVariables.html for the last released version of
NuttX can also be found online at:
http://nuttx.org/Documentation/NuttXConfigVariables.html.

Supported Boards
^^^^^^^^^^^^^^^^

configs/amber
  This is placeholder for the SoC Robotics Amber Web Server that is based
  on the Atmel AVR ATMega128 MCU.  There is not much there yet and what is
  there is untested due to tool-related issues.

configs/arduino-due
  This sub-directory holds board support for the Arduino DUE board featuring
  the Atmel ATSAM3X8E MCU running at 84 MHz.

configs/avr32dev1
  This is a port of NuttX to the Atmel AVR32DEV1 board.  That board is
  based on the Atmel AT32UC3B0256 MCU and uses a specially patched
  version of the GNU toolchain:  The patches provide support for the
  AVR32 family.  That patched GNU toolchain is available only from the
  Atmel website.  STATUS: This port is functional but very basic.  There
  are configurations for NSH and the OS test.

configs/bambino-200e
  Micromint Bambino board. This board is based on the LPC4330FBD144.

configs/c5471evm
  This is a port to the Spectrum Digital C5471 evaluation board.  The
  TMS320C5471 is a dual core processor from TI with an ARM7TDMI general
  purpose processor and a c54 DSP.  It is also known as TMS320DA180 or just DA180.
  NuttX runs on the ARM core and is built with a GNU arm-nuttx-elf toolchain*.
  This port is complete and verified.

configs/cloudctrl
  Darcy's CloudController board.  This is a small network relay development
  board. Based on the Shenzhou IV development board design.  It is based on
  the STM32F107VC MCU.

configs/compal_e86, compal_e88 and compal_e99
  These directories contain the board support for compal e86, e88 and e99 phones.
  These ports are based on patches contributed by Denis Carikli for both the
  compal e99 and e88.  The patches were made by Alan Carvalho de Assis and
  Denis Carikli using the Stefan Richter's Osmocom-bb patches.  The variant
  for the e86 was submitted by Craig Comstock.

configs/demo9s12ne64
  Freescale DMO9S12NE64 board based on the MC9S12NE64 hcs12 cpu.  This
  port uses the m9s12x GCC toolchain.  STATUS:  (Still) under development; it
  is code complete but has not yet been verified.

configs/dk-tm4c129x
  This is the port of NuttX to the Tiva� DK-TM4C129x Connected Development Kit.  The
  Tiva� DK-TM4C129x features the TM4C129XNCZAD MCU.

configs/ea3131
  Embedded Artists EA3131 Development board.  This board is based on the
  an NXP LPC3131 MCU. This OS is built with the arm-nuttx-elf toolchain*.
  STATUS:  This port is complete and mature.

configs/ea3152
  Embedded Artists EA3152 Development board.  This board is based on the
  an NXP LPC3152 MCU. This OS is built with the arm-nuttx-elf toolchain*.
  STATUS:  This port is has not be exercised well, but since it is
  a simple derivative of the ea3131, it should be fully functional.

configs/eagle100
  Micromint Eagle-100 Development board.  This board is based on the
  an ARM Cortex-M3 MCU, the Luminary LM3S6918. This OS is built with the
  arm-nuttx-elf toolchain*.  STATUS:  This port is complete and mature.

configs/efm32-g8xx-stk
  The port of NuttX to the the EFM32 Gecko Starter Kit (EFM32-G8XX-STK).

configs/efm32gg-stk3700
  The port of NuttX to the the EFM32 Giant Gecko Starter Kit
  (EFM32GG-STK3700).

configs/ekk-lm3s9b96
  TI/Stellaris EKK-LM3S9B96 board.  This board is based on the
  an EKK-LM3S9B96 which is a Cortex-M3.

configs/esp-core
  The ESP32 is a dual-core system from Expressif with two Harvard
  architecture Xtensa LX6 CPUs. All embedded memory, external memory and
  nd peripherals are located on the data bus and/or the instruction bus of
  bus of these CPUs. With some minor exceptions, the address mapping of two
  CPUs is symmetric, meaning they use the same addresses to access the same
  memory.

configs/ez80f0910200kitg
  ez80Acclaim! Microcontroller.  This port use the Zilog ez80f0910200kitg
  development kit, eZ80F091 part, and the Zilog ZDS-II Windows command line
  tools.  The development environment is Cygwin under WinXP.

configs/ez80f0910200zco
  ez80Acclaim! Microcontroller.  This port use the Zilog ez80f0910200zco
  development kit, eZ80F091 part, and the Zilog ZDS-II Windows command line
  tools.  The development environment is Cygwin under WinXP.

configs/fire-stm32v2
  A configuration for the M3 Wildfire STM32 board.  This board is based on the
  STM32F103VET6 chip.  See http://firestm32.taobao.com .  Version 2 and 3 of
  the boards are supported but only version 2 has been tested.

configs/freedom-k64f
  This port uses the FreeScale FREEDOM-K64F development board. This board
  uses the Kinetis K64 MK64FN1M0VLL12 Cortex-M4 MCU.

configs/freedom-kl25z
configs/freedom-kl26z
  These configurations are for the Freescale Freedom KL25Z and very similar
  KL26Z board.  The Freedom-KL25Z features the K25Z120LE3AN chip; the
  Freedom-KL26Z has the K26Z128VLH4 chip.  These are separate configurations
  because of minor differences in the on-board logic.  Both include a
  built-in SDA debugger.

configs/hymini-stm32v
  A configuration for the HY-Mini STM32v board.  This board is based on the
  STM32F103VCT chip.

configs/kwikstik-k40.
  Kinetis K40 Cortex-M4 MCU.  This port uses the FreeScale KwikStik-K40
  development board.

configs/launchxl-tms57004
  TI Hercules TMS570LS04x/03x LaunchPad Evaluation Kit (LAUNCHXL-TMS57004)
  featuring the Hercules TMS570LS0432PZ chip.

configs/lincoln60
   NuttX port to the Micromint Lincoln 60 board.

configs/lm3s6432-s2e
  Stellaris RDK-S2E Reference Design Kit and the MDL-S2E Ethernet to
  Serial module.

configs/lm3s6965-ek
  Stellaris LM3S6965 Evaluation Kit.  This board is based on the
  an ARM Cortex-M3 MCU, the Luminary/TI LM3S6965. This OS is built with the
  arm-nuttx-elf toolchain*.  STATUS:  This port is complete and mature.

configs/lm3s8962-ek
  Stellaris LMS38962 Evaluation Kit.

configs/lm4f120-launchpad
  This is the port of NuttX to the Stellaris LM4F120 LaunchPad.  The
  Stellaris� LM4F120 LaunchPad Evaluation Board is a low-cost evaluation
  platform for ARM� Cortex�-M4F-based microcontrollers from Texas
  Instruments.

configs/lpcxpresso-lpc1115
  NuttX board support for the Embedded Artists LPCXpresso LPC1115 board
  featuring the NXP LPC1115 MCU.

configs/lpcxpresso-lpc1768
  Embedded Artists base board with NXP LPCExpresso LPC1768.  This board
  is based on the NXP LPC1768.  The Code Red toolchain is used by default.
  STATUS:  Under development.

configs/lpc4330-xplorer
  NuttX port to the LPC4330-Xplorer board from NGX Technologies featuring
  the NXP LPC4330FET100 MCU

configs/lpc4337-ws
  NuttX port to the WaveShare LPC4337-ws board featuring the NXP LPC4337JBD144
  MCU.

configs/lpc4357-evb
  NuttX port to the LPC4357-EVB board from Embest featuring the NXP
  LPC4357FET256 MCU.  Based on the LPC4300 Xplorer port and provided by
  Toby Duckworth.

configs/lpc4370-link2
  NuttX port to the NXP LPC4370-Link2 development board featuring the NXP
  LPC4370FET100 MCU.  Based on the LPC4300 Xplorer port and provided by
  Lok Tep.

configs/maple
  NuttX support for the LeafLab's Maple and Maple Mini boards. These boards
  are based on the STM32F103RBT6 chip for the standard version and on the
  STM32F103CBT6 for the mini version (See http://leaflabs.com/docs/hardware/maple.html)

configs/mbed
  The configurations in this directory support the mbed board (http://mbed.org)
  that features the NXP LPC1768 microcontroller. This OS is also built
  with the arm-nuttx-elf toolchain*.  STATUS:  Contributed.

configs/mikroe-stm32f4
  This is the port of NuttX to the MikroElektronika Mikromedia for STM32F4
  development board.  Contributed by Ken Petit.

configs/mcu123-lpc214x
  This port is for the NXP LPC2148 as provided on the mcu123.com
  lpc214x development board. This OS is also built with the arm-nuttx-elf
  toolchain*.  The port supports serial, timer0, spi, and usb.

configs/micropendous3
  This is a port to the Opendous Micropendous 3 board. This board may
  be populated with either an AVR AT90USB646, 647, 1286, or 1287 MCU.
  Support is configured for the AT90USB647.

configs/mirtoo
  This is the port to the DTX1-4000L "Mirtoo" module.  This module uses MicroChip
  PIC32MX250F128D.  See http://www.dimitech.com/ for further information.

configs/moteino-mega
  This is placeholder for the LowPowerLab MoteinoMEGA that is based
  on the Atmel AVR ATMega1284P MCU.  There is not much there yet and what is
  there is untested due to tool-related issues.

configs/moxa
  Moxa NP51x0 series of 2-port advanced RS-232/422/485 serial device servers.

configs/mx1ads
  This is a port to the Motorola MX1ADS development board.  That board
  is based on the Freescale i.MX1 processor.  The i.MX1 is an ARM920T.
  STATUS:  This port is nearly code complete but was never fully
  integrated due to tool-related issues.

configs/ne64badge
  Future Electronics Group NE64 /PoE Badge board based on the
  MC9S12NE64 hcs12 cpu.  This port uses the m9s12x GCC toolchain.
  STATUS:  Under development.  The port is code-complete but has
  not yet been fully tested.

configs/ntosd-dm320
  This port uses the Neuros OSD v1.0 Dev Board with a GNU arm-nuttx-elf
  toolchain*: see

    http://wiki.neurostechnology.com/index.php/OSD_1.0_Developer_Home

  There are some differences between the Dev Board and the currently
  available commercial v1.0 Boards.  See

    http://wiki.neurostechnology.com/index.php/OSD_Developer_Board_v1

  NuttX operates on the ARM9EJS of this dual core processor.
  STATUS: This port is code complete, verified, and included in the
  NuttX 0.2.1 release.

configs/nucleo-144
  STMicro Nucleo-144 development board family.  Included support for (1) the
  Nucleo-F767ZG board featuring the STM32F746ZGT6U MCU. The STM32F746ZGT6U
  is a 216MHz Cortex-M7 operation with 1024Kb Flash memory and 320Kb SRAM.
  And (2) the Nucleo-F746ZG board featuring the STM32F767ZIT6 MCU. The
  STM32F767ZIT6 is a 216MHz Cortex-M7 operation with 2048Kb Flash memory
  and 512Kb SRAM.

configs/nucleo-f4x1re
  STMicro ST Nucleo F401RE and F411RE boards.  See
  http://mbed.org/platforms/ST-Nucleo-F401RE and
  http://developer.mbed.org/platforms/ST-Nucleo-F411RE for more
  information about these boards.

configs/nucleo-f303re
  STMicro ST Nucleo F303RE board.  Contributed by Paul Alexander Patience.

configs/nutiny-nuc120
  This is the port of NuttX to the NuvoTon NuTiny-SDK-NUC120 board.  This
  board has the NUC120LE3AN chip with a built-in NuLink debugger.

configs/olimex-efm32g880f128-stk
  This is the port of NuttX to the Olimex EFM32G880F128-STK development
  board.

configs/olimex-lpc1766stk
  This port uses the Olimex LPC1766-STK board and a GNU GCC toolchain* under
  Linux or Cygwin.  STATUS: Complete and mature.

configs/olimex-lpc2378
  This port uses the Olimex-lpc2378 board and a GNU arm-nuttx-elf toolchain* under
  Linux or Cygwin.  STATUS: ostest and NSH configurations available.
  This port for the NXP LPC2378 was contributed by Rommel Marcelo.

configs/olimex-stm32-h405
  This port uses the Olimex STM32 H405 board and a GNU arm-nuttx-elf
  toolchain* under Linux or Cygwin. See the http://www.olimex.com for
  further information. This board features the STMicro STM32F405RGT6 MCU.
  Contributed by Martin Lederhilger.

configs/olimex-stm32-h407
  This port uses the Olimex STM32 H407 board and a GNU arm-nuttx-elf
  toolchain* under Linux or Cygwin. See the http://www.olimex.com for
  further information. This board features the STMicro STM32F407ZGT6 (144
  pins). Contributed by Neil Hancock.

configs/olimex-stm32-e407
  Olimex STM32 E407 board based on the STMicro STM32F407ZGT6 (144pins).
  Contributed by Mateusz Szafoni.

configs/olimex-stm32-p107
  This port uses the Olimex STM32-P107 board (STM32F107VC) and a GNU arm-nuttx-elf
  toolchain* under Linux or Cygwin. See the https://www.olimex.com/dev/stm32-p107.html
  for further information.  Contributed by Max Holtzberg.

configs/olimex-stm32-p207
  This port uses the Olimex STM32-P207 board (STM32F207ZE) and a GNU arm-nuttx-elf
  toolchain under Linux or Cygwin. See the https://www.olimex.com/dev/stm32-p207.html
  for further information.  Contributed by Martin Lederhilger.

configs/olimexino-stm32
  This port uses the Olimexino STM32 board (STM32F103RBT6) and a GNU arm-nuttx-elf
  toolchain* under Linux or Cygwin. See the http://www.olimex.com for further\
  information.  Contribued by David Sidrane.

configs/olimex-strp711
  This port uses the Olimex STR-P711 board and a GNU arm-nuttx-elf toolchain* under
  Linux or Cygwin. See the http://www.olimex.com/dev/str-p711.html" for
  further information.  STATUS: Configurations for the basic OS test and NSH
  are complete and verified.

configs/pcblogic-pic32mx
  This is the port of NuttX to the PIC32MX board from PCB Logic Design Co.
  This board features the MicroChip PIC32MX460F512L.
  The board is a very simple -- little more than a carrier for the PIC32
  MCU plus voltage regulation, debug interface, and an OTG connector.
  STATUS:  Code complete but testing has been stalled due to tool related problems
  (PICkit 2 does not work with the PIC32).

configs/pcduino-a10
  This directory contains the port of NuttX to the pcDuino v1 board
  See http://www.pcduino.com/ for information about pcDuino Lite, v1,
  and v2.  These boards are based around the Allwinner A10 Cortex-A8 CPU.
  I have not compared these boards in detail, but I believe that the
  differences are cosmetic.  This port was developed on the v1 board, but
  the others may be compatible.

configs/p112
  The P112 is notable because it was the first of the hobbyist single board
  computers to reach the production stage. The P112 hobbyist computers
  were relatively widespread and inspired other hobbyist centered home brew
  computing projects such as N8VEM home brew computing project. The P112
  project still maintains many devoted enthusiasts and has an online
  repository of software and other information.

  The P112 computer originated as a commercial product of "D-X Designs Pty
  Ltd" of Australia. They describe the computer as "The P112 is a stand-alone
  8-bit CPU board. Typically running CP/M (tm) or a similar operating system,
  it provides a Z80182 (Z-80 upgrade) CPU with up to 1MB of memory, serial,
  parallel and diskette IO, and realtime clock, in a 3.5-inch drive form factor.
  Powered solely from 5V, it draws 150mA (nominal: not including disk drives)
  with a 16MHz CPU clock. Clock speeds up to 24.576MHz are possible."

  The P112 board was last available new in 1996 by Dave Brooks. In late 2004
  on the Usenet Newsgroup comp.os.cpm, talk about making another run of P112
  boards was discussed. David Griffith decided to produce additional P112 kits
  with Dave Brooks blessing and the assistance of others. In addition Terry
  Gulczynski makes additional P112 derivative hobbyist home brew computers.
  Hal Bower was very active in the mid 1990's on the P112 project and ported
  the "Banked/Portable BIOS".

  Dave Brooks was successfully funded through Kickstarter for and another
  run of P112 boards in November of 2012.

configs/pic32mx-starterkit
  This directory contains the port of NuttX to the Microchip PIC32 Ethernet
  Starter Kit (DM320004) with the Multimedia Expansion Board (MEB, DM320005).
  See www.microchip.com for further information.

configs/pic32mx7mmb
  This directory will (eventually) contain the port of NuttX to the
  Mikroelektronika PIC32MX7 Multimedia Board (MMB).  See
  http://www.mikroe.com/ for further information.

configs/pirelli_dpl10
configs/pic32mz-starterkit
  This directory contains the port of NuttX to the Microchip PIC32MZ
  Embedded Connectivity (EC) Starter Kit.  There are two configurations of
  the starter kit:

    1) The PIC32MZ Embedded Connectivity Starter Kit based on the
       PIC32MZ2048ECH144-I/PH chip (DM320006), and
    2) The PIC32MZ Embedded Connectivity Starter Kit based on the
       PIC32MZ2048ECM144-I/PH w/Crypto Engine (DM320006-C)

  See www.microchip.com for further information.

  This directory contains the board support for Pirelli "Discus" DP-L10 phones.
  It is a variant of the compal_e88 config with the small changes for the
  differences in the board.

configs/qemu-i486
  Port of NuttX to QEMU in i486 mode.  This port will also run on real i486
  hardwared (Google the Bifferboard).

configs/nr5m100-nexys4
  Port of NuttX to RISC-V platform on IQ-Analog NR5M100 RISC-V FPGA platform.

configs/sabre-6quad
  This directory holds a port of NuttX to the NXP/Freescale Sabre board
  featuring the iMX 6Quad CPU.

configs/sama5d2-xult
  This is the  port of NuttX to the Atmel SAMA5D2-Xplained Ultra development
  board.  This board features the Atmel SAMA5D27.  See http://www.atmel.com.

configs/sama5d3x-ek
  This is the  port of NuttX to the Atmel SAMA5D3x-EK development boards
  (where x=1,3,4, or 5).  These boards feature the Atmel SAMA5D3
  microprocessors.  Four different SAMA5D3x-EK kits are available

    - SAMA5D31-EK with the ATSAMA5D1 (http://www.atmel.com/devices/sama5d31.aspx)
    - SAMA5D33-EK with the ATSAMA5D3 (http://www.atmel.com/devices/sama5d33.aspx)
    - SAMA5D34-EK with the ATSAMA5D4 (http://www.atmel.com/devices/sama5d34.aspx)
    - SAMA5D35-EK with the ATSAMA5D5 (http://www.atmel.com/devices/sama5d35.aspx)

  The each consist of an identical base board with different plug-in modules
  for each CPU.  An option 7 inch LCD is also available.  All four boards
  are supported by NuttX with a simple reconfiguration of the processor
  type.

configs/sama5d3-xplained
  This is the  port of NuttX to the Atmel SAMA5D3x-Xplained development board.
  This board features the Atmel SAMA5D36.  See
  http://www.atmel.com/devices/sama5d36.aspx.

configs/sama5d4-ek
  This is the  port of NuttX to the Atmel SAMA5D4-EK development board.
  This board features the Atmel SAMA5D44.  See http://www.atmel.com.

configs/samd20-xplained
  The port of NuttX to the Atmel SAMD20-Xplained Pro development board.  This
  board features the ATSAMD20J18A MCU (Cortex-M0+ with 256KB of FLASH and
  32KB of SRAM).

configs/samd21-xplained
  The port of NuttX to the Atmel SAMD21-Xplained Pro development board.  This
  board features the ATSAMD21J18A MCU (Cortex-M0+ with 256KB of FLASH and
  32KB of SRAM).

configs/sam3u-ek
  The port of NuttX to the Atmel SAM3U-EK development board.

configs/sam4cmp-db
  The port of NuttX to the Atmel SAM4CMP-DB development board.

configs/sam4e-ek
  The port of NuttX to the Atmel SAM4E-EK development board.  This board
  features the SAM4E16 MCU running at up to 120MHz.

configs/sam4l-xplained
  The port of NuttX to the Atmel SAM4L-Xplained development board.

configs/sam4s-xplained
  The port of NuttX to the Atmel SAM4S-Xplained development board.

configs/sam4s-xplained-pro
  The port of NuttX to the Atmel SAM4S-Xplained Pro development board.

configs/same70-xplained
  The port of NuttX to the Atmel SAME70 Xplained evaluation board.

configs/samv71-xult
  The port of NuttX to the Atmel SAMV71 Xplained Ultra evaluation board.

configs/sim
  A user-mode port of NuttX to the x86 Linux platform is available.
  The purpose of this port is primarily to support OS feature development.
  This port does not support interrupts or a real timer (and hence no
  round robin scheduler)  Otherwise, it is complete.

configs/shenzhou
  This is the port of NuttX to the Shenzhou development board from
  www.armjishu.com. This board features the STMicro STM32F107VCT MCU.

configs/spark
  Board support for the Spark Core from Spark Devices http://www.sparkdevices.com).
  This board features the STM32103CBT6 MCU from STMicro.

configs/skp16c26
  Renesas M16C processor on the Renesas SKP16C26 StarterKit.  This port
  uses the GNU m32c toolchain.  STATUS:  The port is complete but untested
  due to issues with compiler internal errors.

configs/stm3210e-eval
  STMicro STM3210E-EVAL development board based on the STMicro STM32F103ZET6
  microcontroller (ARM Cortex-M3).  This port uses the GNU Cortex-M3
  toolchain.

configs/stm3220g-eval
  STMicro STM3220G-EVAL development board based on the STMicro STM32F407IG
  microcontroller (ARM Cortex-M3).

configs/stm3240g-eval
  STMicro STM3240G-EVAL development board based on the STMicro STM32F103ZET6
  microcontroller (ARM Cortex-M4 with FPU).  This port uses a GNU Cortex-M4
  toolchain (such as CodeSourcery).

configs/stm32butterfly2
  Kamami stm32butterfly2 development board with optional ETH phy.  See
  https://kamami.pl/zestawy-uruchomieniowe-stm32/178507-stm32butterfly2.html

configs/stm32f103-minimum
  Generic STM32F103C8T6 Minimum ARM Development Board.

configs/stm32f4discovery
  STMicro STM32F4-Discovery board based on the STMIcro STM32F407VGT6 MCU.

configs/stm32f411e-disco
  This is a minimal configuration that supports low-level test of the
  STMicro STM32F411E-Discovery Board.

configs/stm32f429i-disco
  STMicro STM32F429I-Discovery board based on the STMicro STM32F429ZIT6 MCU.

configs/stm32f746g-disco
  STMicro STM32F746G-DISCO development board featuring the STM32F746NGH6
  MCU. The STM32F746NGH6  is a 216MHz Cortex-M7 operation with 1024Kb Flash
  memory and 300Kb SRAM.

configs/stm32f746g-ws
  Waveshare STM32F746 development board featuring the STM32F746IG MCU.

configs/stm32l476-mdk
  Motorola Mods Development Board (MDK) features  STM32L476ME MCU.
  The STM32L476ME  is a Cortex-M4 optimised for low-power operation
  at up to 80MHz operation with 1024Kb Flash memory and 96+32Kb SRAM.

configs/stm32l476vg-disco
  STMicro STM32L476VG_DISCO development board featuring the STM32L476VG
  MCU. The STM32L476VG  is a Cortex-M4 optimised for low-power operation
  at up to 80MHz operation with 1024Kb Flash memory and 96+32Kb SRAM.

configs/stm32ldiscovery
  STMicro STM32L-Discovery board based on the STMicro STM32L152RB MCU.

configs/stm32vldiscovery
  STMicro STM32VL-Discovery board based on the STMicro STM32F100RB MCU.

configs/sure-pic32mx
  The "Advanced USB Storage Demo Board," Model DB-DP11215, from Sure
  Electronics (http://www.sureelectronics.net/).  This board features
  the MicroChip PIC32MX440F512H.  See also
  http://www.sureelectronics.net/goods.php?id=1168 for further
  information about the Sure DB-DP11215 board.

configs/teensy-2.0
  This is the port of NuttX to the PJRC Teensy++ 2.0 board.  This board is
  developed by http://pjrc.com/teensy/.  The Teensy++ 2.0 is based
  on an Atmel AT90USB1286 MCU.

configs/teensy-3.x
  This is the port of NuttX to the Teensy-3.1 from PJRC
  (https://www.pjrc.com/).  The Teensy-3.1 features the Freescale
  MK30DX256VLH7 chip (now NXP).  The MK30DX256VLH7 is a 64-pin Cortex-M4
  running at 72MHz.  It has 256KiB of program FLASH memory and 64KiB of
  SRAM.  For more information about the Teensy 3.1, see

    https://www.pjrc.com/teensy/teensy31.html
    https://www.pjrc.com/store/teensy31.html

  This board configuration can also be used with the older Teensy-3.0.  The
  Teensy-3.0 has the same schematic (although some pins are not used on the
  Teensy-3.0).  the primary difference is that the Teensy 3.0 has a
  MK30DX128VLH5 with slightly less capability.

configs/teensy-lc
  This is the port of nuttx for the Teensy LC board.  The Teensy LC
  is a DIP style breakout board for the MKL25Z64 and comes with a USB
  based bootloader.

configs/tm4c123g-launchpad
  This is the port of NuttX to the Tiva� TM4C123G LaunchPad.  The
  Tiva� TM4C123G LaunchPad Evaluation Board is a low-cost evaluation
  platform for ARM� Cortex�-M4F-based microcontrollers from Texas
  Instruments.

configs/tm4c1294-launchpad
  This is the port of NuttX to the Tiva� TM4C1294 LaunchPad.  The
  Tiva� TM4C123G LaunchPad Evaluation Board is a low-cost evaluation
  platform for ARM� Cortex�-M4F-based microcontrollers from Texas
  Instruments.

configs/twr-k60n512
  Kinetis K60 Cortex-M4 MCU.  This port uses the FreeScale TWR-K60N512
  development board.

configs/ubw32

  This is the port to the Sparkfun UBW32 board.  This port uses the original v2.4
  board which is based on the MicroChip PIC32MX460F512L.  See
  http://www.sparkfun.com/products/8971.  This older version has been replaced
  with this board http://www.sparkfun.com/products/9713. See also
  http://www.schmalzhaus.com/UBW32/.

configs/us7032evb1
  This is a port of the Hitachi SH-1 on the Hitachi SH-1/US7032EVB1 board.
  STATUS:  Work has just began on this port.

configs/viewtool-stm32f107
  NuttX configurations for the ViewTool STM32F103/F107 V1.2 board.  This
  board may be fitted with either: (1) STM32F107VCT6 or (2) STM32F103VCT6.
  See http://www.viewtool.com/ for further information.

configs/xtrs
  TRS80 Model 3.  This port uses a vintage computer based on the Z80.
  An emulator for this computer is available to run TRS80 programs on a
  linux platform (http://www.tim-mann.org/xtrs.html).

configs/z16f2800100zcog
  z16f Microcontroller.  This port use the Zilog z16f2800100zcog
  development kit and the Zilog ZDS-II Windows command line tools.  The
  development environment is Cygwin under WinXP.

configs/z80sim
  z80 Microcontroller.  This port uses a Z80 instruction set simulator
  called z80sim.  This port also uses the SDCC toolchain
  (http://sdcc.sourceforge.net/") (verified with version 2.6.0).

configs/z8encore000zco
  z8Encore! Microcontroller.  This port use the Zilog z8encore000zco
  development kit, Z8F6403 part, and the Zilog ZDS-II Windows command line
  tools.  The development environment is Cygwin under WinXP.

configs/z8f64200100kit
  z8Encore! Microcontroller.  This port use the Zilog z8f64200100kit
  development kit, Z8F6423 part, and the Zilog ZDS-II Windows command line
  tools.  The development environment is Cygwin under WinXP.

configs/zp214xpa
  This port is for the NXP LPC2148 as provided on the The0.net
  ZPA213X/4XPA development board. Includes support for the
  UG-2864AMBAG01 OLED also from The0.net

configs/zkit-arm-1769
  Zilogic System's ARM development Kit, ZKIT-ARM-1769.  This board is based
  on the NXP LPC1769.  The Nuttx Buildroot toolchain is used by default.

Configuring NuttX
^^^^^^^^^^^^^^^^^

Configuring NuttX requires only copying

  configs/<board-name>/<config-dir>/Make.def to ${TOPDIR}/Make.defs
  configs/<board-name>/<config-dir>/setenv.sh to ${TOPDIR}/setenv.sh
  configs/<board-name>/<config-dir>/defconfig to ${TOPDIR}/.config

tools/configure.sh
  There is a script that automates these steps.  The following steps will
  accomplish the same configuration:

    cd tools
   ./configure.sh <board-name>/<config-dir>

  There is an alternative Windows batch file that can be used in the
  windows native enironment like:

    cd ${TOPDIR}\tools
    configure.bat <board-name>\<config-dir>

  See tools/README.txt for more information about these scripts.

  And if your application directory is not in the standard loction (../apps
  or ../apps-<version>), then you should also specify the location of the
  application directory on the command line like:

    cd tools
    ./configure.sh -a <app-dir> <board-name>/<config-dir>

Building Symbol Tables
^^^^^^^^^^^^^^^^^^^^^^

Symbol tables are needed at several of the binfmt interfaces in order to bind
a module to the base code.  These symbol tables can be tricky to create and
will probably have to be tailored for any specific application, balancing
the number of symbols and the size of the symbol table against the symbols
required by the applications.

The top-level System.map file is one good source of symbol information
(which, or course, was just generated from the top-level nuttx file
using the GNU 'nm' tool).

There are also common-separated value (CSV) values in the source try that
provide information about symbols.  In particular:

  nuttx/syscall/syscall.csv - Describes the NuttX RTOS interface, and
  nuttx/lib/libc.csv        - Describes the NuttX C library interface.

There is a tool at nuttx/tools/mksymtab that will use these CSV files as
input to generate a generic symbol table.  See nuttx/tools/README.txt for
more information about using the mksymtab tool.
