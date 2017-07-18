README
^^^^^^

README for NuttX port to the Embedded Artists LPCXpresso LPC1115 board
featuring the NXP LPC1115 MCU.

Contents
^^^^^^^^

  LCPXpresso LPC1115 Board
  Development Environment
  GNU Toolchain Options
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  Code Red IDE
  Using OpenOCD
  LEDs
  LPCXpresso Configuration Options
  Configurations

LCPXpresso LPC1115 Board
^^^^^^^^^^^^^^^^^^^^^^^^

  Pin Description                  Connector
  -------------------------------- ---------

  P0[0]/RD1/TXD3/SDA1               J6-9
  P0[1]/TD1/RXD3/SCL                J6-10
  P0[2]/TXD0/AD0[7]                 J6-21
  P0[3]/RXD0/AD0[6]                 J6-22
  P0[4]/I2SRX-CLK/RD2/CAP2.0        J6-38
  P0[5]/I2SRX-WS/TD2/CAP2.1         J6-39
  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     J6-8
  P0[7]/I2STX_CLK/SCK1/MAT2[1]      J6-7
  P0[8]/I2STX_WS/MISO1/MAT2[2]      J6-6
  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     J6-5
  P0[10]                            J6-40
  P0[11]                            J6-41

  P1[0]/ENET-TXD0                   J6-34?
  P1[1]/ENET_TXD1                   J6-35?
  P1[4]/ENET_TX_EN
  P1[8]/ENET_CRS
  P1[9]/ENET_RXD0
  P1[10]/ENET_RXD1

  P2[0]/PWM1.1/TXD1
  P2[1]/PWM1.2/RXD1                 J6-43
  P2[2]/PWM1.3/CTS1/TRACEDATA[3]    J6-44
  P2[3]/PWM1.4/DCD1/TRACEDATA[2]    J6-45
  P2[4]/PWM1.5/DSR1/TRACEDATA[1]    J6-46
  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   J6-47
  P2[6]/PCAP1[0]/RI1/TRACECLK       J6-48
  P2[7]/RD2/RTS1                    J6-49
  P2[8]/TD2/TXD2                    J6-50
  P2[9]/USB_CONNECT/RXD2            PAD19
  P2[10]/EINT0/NMI                  J6-51

  P3[25]/MAT0.0/PWM1.2              PAD13
  P3[26]/STCLK/MAT0.1/PWM1.3        PAD14

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The Code Red GNU toolchain
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain,
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the Code Red toolchain and the
  make system is setup to default to use the Code Red Linux toolchain.  To use
  the other toolchain, you simply need add one of the following configuration
  options to your .config (or defconfig) file:

    CONFIG_ARMV6M_TOOLCHAIN_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_ARMV6M_TOOLCHAIN_CODESOURCERYL=y   : CodeSourcery under Linux
    CONFIG_ARMV6M_TOOLCHAIN_DEVKITARM=y       : devkitARM under Windows
    CONFIG_ARMV6M_TOOLCHAIN_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV6M_TOOLCHAIN_CODEREDW=n        : Code Red toolchain under Windows
    CONFIG_ARMV6M_TOOLCHAIN_CODEREDL=y        : Code Red toolchain under Linux

  You may also have to modify the PATH in the setenv.h file if your make cannot
  find the tools.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Code Red (for Windoes)
  are Windows native toolchains.  The CodeSourcey (for Linux), Code Red (for Linux)
  and NuttX buildroot toolchains are Cygwin and/or Linux native toolchains. There
  are several limitations to using a Windows based toolchain in a Cygwin
  environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

Code Red IDE
^^^^^^^^^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.

  Makefile Build
  --------------
  Under Linux Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc11xx,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc11x/lpc11_vectors.S.

  Using Code Red GNU Tools from Cygwin
  ------------------------------------

  Under Cygwin, the Code Red command line tools (e.g., arm-non-eabi-gcc) cannot
  be executed because the they only have execut privileges for Administrators.  I
  worked around this by:

  Opening a native Cygwin RXVT as Administrator (Right click, "Run as administrator"),
  then executing 'chmod 755 *.exe' in the following directories:

  /cygdrive/c/nxp/lpcxpreeso_3.6/bin, and
  /cygdrive/c/nxp/lpcxpreeso_3.6/Tools/bin

  Command Line Flash Programming
  ------------------------------

  During the port development was used a STLink-v2 SWD programmer with OpenOCD to
  write the firmware in the flash and GDB to debug NuttX initialization.

  If using LPCLink as your debug connection, first of all boot the LPC-Link using
  the script:

    bin\Scripts\bootLPCXpresso type

  where type = winusb for Windows XP, or type = hid for Windows Vista / 7.

  Now run the flash programming utility with the following options

    flash_utility wire -ptarget -flash-load[-exec]=filename [-load-base=base_address]

  Where flash_utility is one of:

    crt_emu_lpc11_13 (for LPC11xx or LPC13xx parts)
    crt_emu_cm3_nxp (for LPC11xx parts)
    crt_emu_a7_nxp (for LPC21/22/23/24 parts)
    crt_emu_a9_nxp (for LPC31/32 and LPC29xx parts)
    crt_emu_cm3_lmi (for TI Stellaris parts)

  wire is one of:

    (empty) (for Red Probe+, Red Probe, RDB1768v1, or TI Stellaris evaluation boards)
    -wire=hid (for RDB1768v2 without upgraded firmware)
    -wire=winusb (for RDB1768v2 with upgraded firmware)
    -wire=winusb (for LPC-Link on Windows XP)
    -wire=hid (for LPC-Link on Windows Vista/ Windows 7)

  target is the target chip name. For example LPC1343, LPC1114/301, LPC1115 etc.

  filename is the file to flash program. It may be an executable (axf) or a binary
  (bin) file. If using a binary file, the base_address must be specified.

  base_address is the base load address when flash programming a binary file. It
  should be specified as a hex value with a leading 0x.

  Note:
  - flash-load will leave the processor in a stopped state
  - flash-load-exec will start execution of application as soon as download has
    completed.

  Examples
    To load the executable file app.axf and start it executing on an LPC1158
    target using Red Probe, use the following command line:

      crt_emu_cm3_nxp -pLPC1158 -flash-load-exec=app.axf

    To load the binary file binary.bin to address 0x1000 to an LPC1343 target
    using LPC-Link on Windows XP, use the following command line:

      crt_emu_lpc11_13_nxp -wire=hid -pLPC1343 -flash-load=binary.bin -load-base=0x1000

  tools/flash.sh
  --------------

  All of the above steps are automated in the bash script flash.sh that can
  be found in the configs/lpcxpresso/tools directory.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M0 toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/nuttx/downloads/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1115/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm0-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you
  are building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv6-m -mfloat-abi=soft
    +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
    -CROSSDEV = arm-nuttx-eabi-
    -ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections

NXFLAT Toolchain
^^^^^^^^^^^^^^^^

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX Bitbucket download site
  (https://bitbucket.org/nuttx/nuttx/downloads/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1115/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm0-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

Using OpenOCD
^^^^^^^^^^^^^

  https://acassis.wordpress.com/2015/03/29/using-openocd-to-program-the-lpc1115-lpcxpresso-board/

  Using OpenOCD to program the LPC1115 LPCXpresso board
  March 29, 2015 by acassis

  Unfortunately NXP uses a built-in programmer in the LPCXpresso board
  called LPCLink that is not supported by OpenOCD and there is not (AFAIK)
  an option to replace its firmware.

  Then I decided to cut the board to separate the “LPCXpresso LPC1115 REV A”
  from the LPCLink programmer.

  So I used a simple and low cost STLink-v2 programmer board that is
  supported by OpenOCD. In order to use OpenOCD to reprogram the LPC1115
  board we need to connect four wires from STLink-v2 to LPC1115 board:

    STLink-v2    |   LPC1115 Board
    ------------------------------
    GND              GND
    3V3              3V3
    IO               AD4
    CLK              P0.10


  Also we need to instruct OpenOCD to use SWD protocol. You can do it
  creating the following config openocd.cfg file:

    # LPC1115 LPCXpresso Target

    # Using stlink as SWD programmer
    source [find interface/stlink-v2.cfg]

    # SWD as transport
    transport select hla_swd

    # Use LPC1115 target
    set WORKAREASIZE 0x4000
    source [find target/lpc11xx.cfg]

  Now execute OpenOCD using the created config file:

    $ sudo openocd -f openocd.cfg
    Open On-Chip Debugger 0.9.0-dev-00251-g1fa4c72 (2015-01-28-20:08)
    Licensed under GNU GPL v2
    For bug reports, read
        http://openocd.sourceforge.net/doc/doxygen/bugs.html
    Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
    adapter speed: 10 kHz
    adapter_nsrst_delay: 200
    Info : Unable to match requested speed 10 kHz, using 5 kHz
    Info : Unable to match requested speed 10 kHz, using 5 kHz
    Info : clock speed 5 kHz
    Info : STLINK v2 JTAG v17 API v2 SWIM v4 VID 0x0483 PID 0x3748
    Info : using stlink api v2
    Info : Target voltage: 3.137636
    Info : lpc11xx.cpu: hardware has 4 breakpoints, 2 watchpoints

  Connect to OpenOCD server:

    $ telnet 127.0.0.1 4444

    Reset the CPU and flash the lpc1115_blink.bin file:

      > reset halt
    target state: halted
    target halted due to debug-request, current mode: Thread
    xPSR: 0xc1000000 pc: 0x1fff0040 msp: 0x10000ffc

    > flash probe 0
    flash 'lpc2000' found at 0x00000000

    > flash write_image erase blink_lpc1115.bin 0x00000000
    auto erase enabled
    target state: halted
    target halted due to breakpoint, current mode: Thread
    xPSR: 0x01000000 pc: 0x10000108 msp: 0x100001b8
    Verification will fail since checksum in image (0x00000000) to be written to flash is different from calculated vector checksum (0xefffebe9).
    To remove this warning modify build tools on developer PC to inject correct LPC vector checksum.
    wrote 4096 bytes from file blink_lpc1115.bin in 0.592621s (6.750 KiB/s)

    > reset run

  The checksum warning message could be removed if you add the checksum to
  binary, read this post:

    http://sigalrm.blogspot.com.br/2011/10/cortex-m3-exception-vector-checksum.html.

  The blink LED sample I got from Frank Duignan’s page:

    http://eleceng.dit.ie/frank/arm/BareMetalLPC1114/index.html

  Edit Makefile and configure LIBSPEC to point out to the right path:

     LIBSPEC=-L /usr/lib/gcc/arm-none-eabi/4.8/armv6-m

  $ make

  To generate the final binary I used objcopy:

  $ arm-none-eabi-objcopy -O binary main.elf blink_lpc1115.bin

  https://acassis.wordpress.com/2015/05/22/using-openocd-and-gdb-to-debug-my-nuttx-port-to-lpc11xx/

  Using OpenOCD and gdb to debug my NuttX port to LPC11xx
  May 22, 2015 by acassis

  I’m porting NuttX to LPC11xx (using the LPCXpresso LPC1115 board) and
  these are the steps I used to get OpenOCD and GDB working to debug my firmware:

  The openocd.cfg to use with STLink-v2 SWD programmer:

    # LPC1115 LPCXpresso Target

    # Using stlink as SWD programmer
    source [find interface/stlink-v2.cfg]

    # SWD as transport
    transport select hla_swd

    # Use LPC1115 target
    set WORKAREASIZE 0x4000
    source [find target/lpc11xx.cfg]

  You need to execute “reset halt” from OpenOCD telnet server to get
  “monitor reset halt” working on gdb:

    $ telnet 127.0.0.1 4444Trying 127.0.0.1...
    Connected to 127.0.0.1.
    Escape character is '^]'.
    Open On-Chip Debugger

    > reset halt
    target state: halted
    target halted due to debug-request, current mode: Thread
    xPSR: 0xc1000000 pc: 0x1fff0040 msp: 0x10000ffc

    > exit

  Now execute the command arm-none-eabi-gdb (from Debian/Ubuntu package
  “gdb-arm-none-eabi”) passing the nuttx ELF file:

    $ arm-none-eabi-gdb nuttx
    GNU gdb (7.7.1+dfsg-1+6) 7.7.1
    Reading symbols from nuttx...done.

    (gdb) target remote localhost:3333
    Remote debugging using localhost:3333
    0x1fff0040 in ?? ()

    (gdb) monitor reset halt
    target state: halted
    target halted due to debug-request, current mode: Thread
    xPSR: 0xc1000000 pc: 0x1fff0040 msp: 0x10000ffc

    (gdb) load
    Loading section .vectors, size 0xc0 lma 0x0
    Loading section .text, size 0x9197 lma 0x410
    Loading section .ARM.exidx, size 0x8 lma 0x95a8
    Loading section .data, size 0x48 lma 0x95b0
    Start address 0x410, load size 37543
    Transfer rate: 9 KB/sec, 6257 bytes/write.

    (gdb) b __start
    Breakpoint 1 at 0x410: file chip/lpc11_start.c, line 109.

    (gdb) step

    Note: automatically using hardware breakpoints for read-only addresses.

    Breakpoint 1, __start () at chip/lpc11_start.c:109
    109	{

    (gdb)
    115	  lpc11_clockconfig();

    (gdb)
    lpc11_clockconfig () at chip/lpc11_clockconfig.c:93
    93	  putreg32(SYSCON_SYSPLLCLKSEL_IRCOSC, LPC11_SYSCON_SYSPLLCLKSEL);

    (gdb)
    96	  putreg32((SYSCON_SYSPLLCTRL_MSEL_DIV(4) | SYSCON_SYSPLLCTRL_PSEL_DIV2), LPC11_SYSCON_SYSPLLCTRL);

    (gdb) p /x *0x40048008      <--- this is the LPC11_SYSCON_SYSPLLCTRL register address
    $2 = 0x23
    (gdb)

    You can use breakpoints, steps and many other GDB features.

    That is it!

LEDs
^^^^

  If CONFIG_ARCH_LEDS is defined, then support for the LPCXpresso LEDs will be
  included in the build.  See:

  - configs/lpcxpresso-lpc1115/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/lpcxpresso-lpc1115/src/lpcxpresso-lpc1115.h - GPIO settings for the LEDs.

  - configs/lpcxpresso-lpc1115/src/up_leds.c - LED control logic.

  The LPCXpresso LPC1115 has a single LEDs.  Usage this single LED by NuttX
  is as follows:

  - The LED is not illuminated until the LPCXpresso completes initialization.

    If the LED is stuck in the OFF state, this means that the LPCXpresso did not
    complete initializeation.

  - Each time the OS enters an interrupt (or a signal) it will turn the LED OFF and
    restores its previous stated upon return from the interrupt (or signal).

    The normal state, after initialization will be a dull glow.  The brightness of
    the glow will be inversely related to the proportion of time spent within interrupt
    handling logic.  The glow may decrease in brightness when the system is very
    busy handling device interrupts and increase in brightness as the system becomes
    idle.

    Stuck in the OFF state suggests that that the system never completed
    initialization;  Stuck in the ON state would indicated that the system
    intialialized, but is not takint interrupts.

  - If a fatal assertion or a fatal unhandled exception occurs, the LED will flash
    strongly as a slow, 2Hz rate.

LPCXpresso Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  General Architecture Settings:

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
     be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM0=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc11xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC1115=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lpcxpresso-lpc1115

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPCEXPRESSO=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(8*1024) (8Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x10000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
       used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

    Individual subsystems can be enabled:
      CONFIG_LPC11_MAINOSC=y
      CONFIG_LPC11_PLL0=y
      CONFIG_LPC11_UART0=y
      CONFIG_LPC11_CAN1=n
      CONFIG_LPC11_SPI=n
      CONFIG_LPC11_SSP0=n
      CONFIG_LPC11_SSP1=n
      CONFIG_LPC11_I2C0=n
      CONFIG_LPC11_I2S=n
      CONFIG_LPC11_TMR0=n
      CONFIG_LPC11_TMR1=n
      CONFIG_LPC11_PWM0=n
      CONFIG_LPC11_ADC=n
      CONFIG_LPC11_FLASH=n

  LPC11xx specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

  LPC11xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC11_CAN1 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6

Configurations
^^^^^^^^^^^^^^

Each LPCXpresso configuration is maintained in a sub-directory and can be
selected as follow:

    cd tools
    ./configure.sh lpcxpresso-lpc1115/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  minnsh:
  ------

    This is a experiment to see just how small we can get a usable NSH
    configuration.  This configuration has far fewer features than the nsh
    configuration but is also a fraction of the size.

    This minnsh configuration is a "proof-of-concept" and not very usable in
    its current state.  This configuration was created by disabling
    everything possible INCLUDING file system support.  Without file system
    support, NuttX is pretty much crippled.  Here are some of the
    consequences of disabling the file system:

    - All features that depend on the file system are lost:  device drivers,
      mountpoints, message queues, named semaphores.

    - Without device drivers, you cannot interact with the RTOS using POSIX
      interfaces.  You would have to work with NuttX as with those other
      tiny RTOSs:  As a scheduler and a callable hardare abstraction layer
      (HAL).

    - You cannot use any of the NuttX upper half device drivers since they
      depend on the pseudo-file system and device nodes.  You can, of
      course, continue to use the lower half drivers either directly.  Or,
      perhaps, you could write some custom minnsh upper half drivers that
      do not depend on a file system and expose a HAL interface.

    There is a special version of readline() the NSH uses when there is no
    file system.  It uses a special up_putc() to write data to the console
    and a special function up_getc() to read data from the console.

    - The current up_getc() implementationsa are a kludge.  They are
      analogous to the up_putc() implementations:  They directly poll the
      hardware for serial availability, locking up all lower priority tasks
      in the entire system while they poll.  So a version of NSH that uses
      up_getc() essentially blocks the system until a character is received.

    This, of course, could be fixed by creating a special, upper half
    implementation of the interrupt-driven serial lower half (like
    stm32_serial) that just supports single character console I/O
    (perhaps called up_putc and up_getc?).  The NSH could wait for serial
    input without blocking the system.  But then that would increase the
    footprint too.

    So although the minnsh configurations are a good starting point for
    making things small, they not are really very practical.  Why might
    you want a NuttX minnsh solution?  Perhaps you have software that runs
    on a family of chips including some very tiny MCUs.  Then perhaps having
    the RTOS compatibility would justify the loss of functionality?

    You can re-enable the file system and (true) serial console with
    these settings:

      Enable the file system:
        CONFIG_NFILE_DESCRIPTORS=5
        CONFIG_NFILE_STREAMS=5

      Enable the console device:
        CONFIG_DEV_CONSOLE=y

      Disable most new NSH commands.  Some like 'ls' are really mandatory
      with a file system:
        CONFIG_NSH_DISABLE_xxx=y

      Enable the upper half serial driver:
        CONFIG_SERIAL=y
        CONFIG_STANDARD_SERIAL=y

      Enable the USART1 serial driver:
        CONFIG_STM32_USART1=y
        CONFIG_STM32_USART1_SERIALDRIVER=y
        CONFIG_USART1_SERIAL_CONSOLE=y

        CONFIG_USART1_2STOP=0
        CONFIG_USART1_BAUD=115200
        CONFIG_USART1_BITS=8
        CONFIG_USART1_PARITY=0
        CONFIG_USART1_RXBUFSIZE=16
        CONFIG_USART1_TXBUFSIZE=16

    With these changes, NSH should behave better  and we preserve the device
    driver interface.  But this result in a total size increase of about
    7KB:  That is about 5KB of additional OS support for the file system and
    serial console PLUS about 2KB for the 'ls' command logic (including OS
    support for opendir(), readdir(), closedir(), stat(), and probably other
    things).

    STATUS:
    2015-6-10
      The nuttx.bin minnsh firmware file size:

        $ ls -l nuttx.bin
        -rwxr-xr-x 1 alan alan 13859 Jun 10 08:54 nuttx.bin

        $ arm-none-eabi-size nuttx
        text   data   bss    dec    hex    filename
        12818  193    704    13715  3593   nuttx

        This is serial console output (and input) :

        NuttShell (NSH)
        nsh> ls /dev
        nsh: ls: command not found

        No filesystem, no "ls" command :-)

        nsh> ?
        help usage: help [-v] [<cmd>]

        ? exec free mb mw xd
        echo exit help mh ps
        nsh> free
        total used free largest
        Mem: 6464 1816 4648 4648

        nsh> echo "NuttX is magic!"
        NuttX is magic!
        nsh>

        Replace NSH with apps/examples/hello:

        $ ls -l nuttx.bin
        -rwxr-xr-x 1 alan alan 9318 Jun 10 09:02 nuttx.bin

        $ arm-none-eabi-size nuttx
        text   data   bss    dec    hex    filename
        8277   193    704    9174   23d6   nuttx

      Some additional commits from Alan reduce this FLASH size by
      about another kilobyte.  That changes:  (1) disable stack
      dumping on assertions,and (2) make some FLASH data structures
      smaller.

      Almost 2Kb of the remaining size was due to some arithmetic
      "long long" (64 bits) operations drawn from libgcc.a.
      Alan changed vsprintf to make "long long" support optional.
      This change reduced the NuttX kernel to less than 8KiB!

  nsh:
  ---

    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. This configuration has been used for testing the microSD card.
       This support is, however, disabled in the base configuration.

       At last attempt, the SPI-based mircroSD does not work at
       higher fequencies.  Setting the SPI frequency to 400000
       removes the problem.   There must be some more optimal
       value that could be determined with additional experimetnation.

       Jumpers: J55 must be set to provide chip select PIO1_11 signal as
       the SD slot chip select.
