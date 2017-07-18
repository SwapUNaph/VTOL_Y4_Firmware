README
======

README for NuttX port to the "Bambino 200E" board from Micromint USA
featuring the NXP LPC4330FBD144 MCU

Contents
========

  - Bambino 200E board
  - Status
  - Serial Console
  - FPU
  - Bambino-200e Configuration Options
  - Configurations

Bambino 200E board
=====================

  Memory Map
  ----------

  Block                 Start      Length
  Name                  Address
  --------------------- ---------- ------
  RAM                   0x10000000   128K
  RAM2                  0x10080000    72K
  RAMAHB                0x20000000    32K
  RAMAHB2               0x20008000    16K
  RAMAHB3               0x2000c000    16K
  SPIFI flash           0x1e000000  4096K

  GPIO Usage:
  -----------

  GPIO                              PIN     SIGNAL NAME
  -------------------------------- ------- --------------
  gpio3[7]  - LED1                  101     GPIO3[7]
  gpio5[5]  - LED2                  91      GPIO5[5]
  gpio0[7]  - BTN1                  96      GPIO0[7]

  Console
  -------

  The Bambino 200E default console is the UART1 on Gadgeteer Sockets 5 (U).

Status
======

  Many drivers are working (USB0 Device, Ethernet, etc), but many drivers are
  missing.

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

Serial Console
==============

The LPC4330 Xplorer does not have RS-232 drivers or serial connectors on board.
USART0 and UART1 are available on J8 as follows:

  ------ ------ -----------------------
  SIGNAL J8 PIN   LPC4330FET100 PIN
                  (TFBGA100 package)
  ------ ------ -----------------------
  U0_TXD pin  9  F6  P6_4  U0_TXD=Alt 2
  U0_RXD pin 10  F9  P6_5  U0_RXD=Alt 2
  U1_TXD pin 13  H8  P1_13 U1_TXD=Alt 1
  U1_RXD pin 14  J8  P1_14 U1_RXD=Alt 1
  ------ ------ -----------------------

  GND  is available on J8 pin 1
  5V   is available on J8 pin 2
  VBAT is available on J8 pin 3

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the most NuttX Cortex-M4
ports.  The current LPC43xx port support only one of these options, the "Non-
Lazy Floating Point Register Save".  As a consequence, CONFIG_ARMV7M_CMNVECTOR
must be defined in *all* LPC43xx configuration files.

1. Lazy Floating Point Register Save.

   This is an untested implementation that saves and restores FPU registers
   only on context switches.  This means: (1) floating point registers are
   not stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file:

   CONFIG_ARCH_FPU=y

2. Non-Lazy Floating Point Register Save

   Mike Smith has contributed an extensive re-write of the ARMv7-M exception
   handling logic. This includes verified support for the FPU.  These changes
   have not yet been incorporated into the mainline and are still considered
   experimental.  These FPU logic can be enabled with:

   CONFIG_ARCH_FPU=y
   CONFIG_ARMV7M_CMNVECTOR=y

   You will probably also changes to the ld.script in if this option is selected.
   This should work:

   -ENTRY(_stext)
   +ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
   +EXTERN(_vectors)       /* Force the vectors to be included in the output */

CFLAGS
------

Only the recent toolchains have built-in support for the Cortex-M4 FPU.  You will see
the following lines in each Make.defs file:

  ifeq ($(CONFIG_ARCH_FPU),y)
    ARCHCPUFLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  else
    ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
  endif

Configuration Changes
---------------------

Below are all of the configuration changes that I had to make to configs/stm3240g-eval/nsh2
in order to successfully build NuttX using the Atollic toolchain WITH FPU support:

  -CONFIG_ARCH_FPU=n                       : Enable FPU support
  +CONFIG_ARCH_FPU=y

  -CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : Disable the CodeSourcery toolchain
  +CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=n

  -CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=n       : Enable the Atollic toolchains
  +CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y       :

  -CONFIG_INTELHEX_BINARY=y                : Suppress generation FLASH download formats
  +CONFIG_INTELHEX_BINARY=n                : (Only necessary with the "Lite" version)

  -CONFIG_HAVE_CXX=y                       : Suppress generation of C++ code
  +CONFIG_HAVE_CXX=n                       : (Only necessary with the "Lite" version)

See the section above on Toolchains, NOTE 2, for explanations for some of
the configuration settings.  Some of the usual settings are just not supported
by the "Lite" version of the Atollic toolchain.

Bambino-200e Configuration Options
==================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc43xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC4330=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=bambino-200e (for the Bambino-200e board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_BAMBINO_200E=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x10000000

    CONFIG_ARCH_FPU - The LPC43xxx supports a floating point unit (FPU)

       CONFIG_ARCH_FPU=y

    CONFIG_LPC43_BOOT_xxx - The startup code needs to know if the code is running
       from internal FLASH, external FLASH, SPIFI, or SRAM in order to
       initialize properly.  Note that a boot device is not specified for
       cases where the code is copied into SRAM; those cases are all covered
       by CONFIG_LPC43_BOOT_SRAM.

       CONFIG_LPC43_BOOT_SRAM=y      : Running from SRAM             (0x1000:0000)
       CONFIG_LPC43_BOOT_SPIFI=y     : Running from QuadFLASH        (0x1400:0000)
       CONFIG_LPC43_BOOT_FLASHA=y    : Running in internal FLASHA    (0x1a00:0000)
       CONFIG_LPC43_BOOT_FLASHB=y    : Running in internal FLASHA    (0x1b00:0000)
       CONFIG_LPC43_BOOT_CS0FLASH=y  : Running in external FLASH CS0 (0x1c00:0000)
       CONFIG_LPC43_BOOT_CS1FLASH=y  : Running in external FLASH CS1 (0x1d00:0000)
       CONFIG_LPC43_BOOT_CS2FLASH=y  : Running in external FLASH CS2 (0x1e00:0000)
       CONFIG_LPC43_BOOT_CS3FLASH=y  : Running in external FLASH CS3 (0x1f00:0000)

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

      CONFIG_LPC43_ADC0=y
      CONFIG_LPC43_ADC1=y
      CONFIG_LPC43_ATIMER=y
      CONFIG_LPC43_CAN1=y
      CONFIG_LPC43_CAN2=y
      CONFIG_LPC43_DAC=y
      CONFIG_LPC43_EMC=y
      CONFIG_LPC43_ETHERNET=y
      CONFIG_LPC43_EVNTMNTR=y
      CONFIG_LPC43_GPDMA=y
      CONFIG_LPC43_I2C0=y
      CONFIG_LPC43_I2C1=y
      CONFIG_LPC43_I2S0=y
      CONFIG_LPC43_I2S1=y
      CONFIG_LPC43_LCD=y
      CONFIG_LPC43_MCPWM=y
      CONFIG_LPC43_QEI=y
      CONFIG_LPC43_RIT=y
      CONFIG_LPC43_RTC=y
      CONFIG_LPC43_SCT=y
      CONFIG_LPC43_SDMMC=y
      CONFIG_LPC43_SPI=y
      CONFIG_LPC43_SPIFI=y
      CONFIG_LPC43_SSP0=y
      CONFIG_LPC43_SSP1=y
      CONFIG_LPC43_TMR0=y
      CONFIG_LPC43_TMR1=y
      CONFIG_LPC43_TMR2=y
      CONFIG_LPC43_TMR3=y
      CONFIG_LPC43_USART0=y
      CONFIG_LPC43_UART1=y
      CONFIG_LPC43_USART2=y
      CONFIG_LPC43_USART3=y
      CONFIG_LPC43_USB0=y
      CONFIG_LPC43_USB1=y
      CONFIG_LPC43_USB1_ULPI=y
      CONFIG_LPC43_WWDT=y

  LPC43xx specific U[S]ART device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the USART0).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

    CONFIG_USARTn_RS485MODE - Support LPC43xx USART0,2,3 RS485 mode
      ioctls (TIOCSRS485 and TIOCGRS485) to enable and disable
      RS-485 mode.

  LPC43xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC43_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC43_CAN2 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 = the number of CAN time quanta in segment 2. Default: 7

  LPC43xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC43_ETHERNET.

    CONFIG_ETH0_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_PHY_AUTONEG - Enable auto-negotion
    CONFIG_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_PHY_FDUPLEX - Select full (vs. half) duplex

    CONFIG_NET_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_NET_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_NET_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_NET_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG_FEATURES.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG_FEATURES.
    CONFIG_NET_HASH - Enable receipt of near-perfect match frames.

  LPC43xx USB Device Configuration

    CONFIG_LPC43_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events.
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC43_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC43_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC43_USBDEV_DMA
      Enable lpc17xx-specific DMA support
    CONFIG_LPC43_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC43_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC43xx USB Host Configuration

    CONFIG_USBHOST_OHCIRAM_SIZE
      Total size of OHCI RAM (in AHB SRAM Bank 1)
    CONFIG_USBHOST_NEDS
      Number of endpoint descriptors
    CONFIG_USBHOST_NTDS
      Number of transfer descriptors
    CONFIG_USBHOST_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_USBHOST_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_USBHOST_IOBUFSIZE
      Size of one end-user I/O buffer.  This can be zero if the
      application can guarantee that all end-user I/O buffers
      reside in AHB SRAM.

Configurations
==============

Each Bambino-200e configuration is maintained in a sub-directory and can be selected
as follow:

    cd tools
    ./configure.sh bambino-200e/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
  ----
    This configuration is the NuttShell (NSH) example at examples/nsh/.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this project assumes that you are executing directly from
       SRAM.

         CONFIG_LPC43_BOOT_SRAM=y           : Executing in SRAM
         CONFIG_ARMV7M_TOOLCHAIN_CODEREDW=y : Code Red under Windows

    3. To execute from SPIFI, you would need to set:

         CONFIG_LPC43_BOOT_SPIFI=y      : Executing from SPIFI
         CONFIG_RAM_SIZE=(128*1024)     : SRAM Bank0 size
         CONFIG_RAM_START=0x10000000    : SRAM Bank0 base address
         CONFIG_SPIFI_OFFSET=(512*1024) : SPIFI file system offset

       CONFIG_MM_REGIONS should also be increased if you want to other SRAM banks
       to the memory pool.

    4. This configuration an also be used create a block device on the SPIFI
       FLASH.  CONFIG_LPC43_SPIFI=y must also be defined to enable SPIFI setup
       support:

       SPIFI device geometry:

         CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
           bytes into the device address space.  This offset must be an exact
           multiple of the erase block size (CONFIG_SPIFI_BLKSIZE). Default 0.
         CONFIG_SPIFI_BLKSIZE - The size of one device erase block.  If not defined
           then the driver will try to determine the correct erase block size by
           examining that data returned from spifi_initialize (which sometimes
           seems bad).

       Other SPIFI options

         CONFIG_SPIFI_SECTOR512 - If defined, then the driver will report a more
           FAT friendly 512 byte sector size and will manage the read-modify-write
           operations on the larger erase block.
         CONFIG_SPIFI_READONLY - Define to support only read-only operations.
         CONFIG_SPIFI_LIBRARY - Don't use the LPC43xx ROM routines but, instead,
           use an external library implementation of the SPIFI interface.
         CONFIG_SPIFI_VERIFY - Verify all spifi_program() operations by reading
           from the SPI address space after each write.
         CONFIG_DEBUG_SPIFI_DUMP - Debug option to dump read/write buffers.  You
           probably do not want to enable this unless you want to dig through a
           *lot* of debug output!  Also required CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO,
           and CONFIG_DEBUG_FS,

    5. In my experience, there were some missing function pointers in the LPC43xx
       SPIFI ROM routines and the SPIFI configuration could only be built with
       CONFIG_SPIFI_LIBRARY=y.  The SPIFI library is proprietary and cannot be
       provided within NuttX open source repository; SPIFI library binaries can
       be found on the lpcware.com website.  In this build sceneario, you must
       also provide the patch to the external SPIFI library be defining the make
       variable EXTRA_LIBS in the top-level Make.defs file.  Good luck!
