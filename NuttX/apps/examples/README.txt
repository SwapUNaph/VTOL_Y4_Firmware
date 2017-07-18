examples
^^^^^^^^

  Selecting examples:

    The examples directory contains several sample applications that
    can be linked with NuttX.  The specific example is selected in the
    configs/<board-name>/defconfig file via the CONFIG_EXAMPLES_xyz
    setting where xyz is the name of the example. For example,

      CONFIG_EXAMPLES_OSTEST=y

    Selects the examples/ostest example.

  Built-In functions

    Some of the examples may be built as "built-in" functions that
    can be executed at run time (rather than as NuttX "main" programs).
    These "built-in" examples can be also be executed from the NuttShell
    (NSH) command line.  In order to configure these built-in  NSH
    functions, you have to set up the following:

    - CONFIG_NSH_BUILTIN_APPS - Enable support for external registered,
      "named" applications that can be executed from the NSH
      command line (see apps/README.txt for more information).

examples/adc
^^^^^^^^^^^^

  A mindlessly simple test of an ADC devices.  It simply reads from the
  ADC device and dumps the data to the console forever.

  This test depends on these specific ADC/NSH configurations settings (your
  specific ADC settings might require additional settings).

    CONFIG_ADC - Enabled ADC support
    CONFIG_NSH_BUILTIN_APPS - Build the ADC test as an NSH built-in function.
      Default: Built as a standalone problem

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_ADC_DEVPATH - The default path to the ADC device. Default: /dev/adc0
    CONFIG_EXAMPLES_ADC_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
      is defined, then the number of samples is provided on the command line
      and this value is ignored.  Otherwise, this number of samples is
      collected and the program terminates.  Default:  Samples are collected
      indefinitely.
    CONFIG_EXAMPLES_ADC_GROUPSIZE - The number of samples to read at once.
      Default: 4

examples/adjoystick
^^^^^^^^^^^^^^^^^^

  This is a simple test of the analog joystick driver.  See details about
  this driver in nuttx/include/nuttx/input/adjoystick.h.

  Configuration Pre-requisites:

    CONFIG_DISABLE_SIGNALS - Must *NOT* be selected
    CONFIG_AJOYSTICK - The analog joystick driver

  Example Configuration:
    CONFIG_EXAMPLES_AJOYSTICK - Enabled the analog joystick example
    CONFIG_EXAMPLES_AJOYSTICK_DEVNAME - Joystick device name.  Default
      "/dev/adjoy0"
    CONFIG_EXAMPLES_AJOYSTICK_SIGNO - Signal used to signal the test
      application.  Default 13.

examples/alarm
^^^^^^^^^^^^^^
  A simple example that tests the alarm IOCTLs of the RTC driver.

  Dependencies:

    CONFIG_RTC_DRIVER - RTC driver must be initialized to allow user space
      access to the RTC.
    CONFIG_RTC_ALARM - Support for RTC alarms must be enabled.

  Configuration:

    CONFIG_EXAMPLES_ALARM - Enable the RTC driver alarm test
    CONFIG_EXAMPLES_ALARM_PROGNAME - If CONFIG_BUILD_KERNEL=y, then this is
      the name of the program that will be use when the NSH ELF program is
      installed.
    CONFIG_EXAMPLES_ALARM_PRIORITY - Alarm daemon priority
    CONFIG_EXAMPLES_ALARM_STACKSIZE - Alarm daemon stack size
    CONFIG_EXAMPLES_ALARM_DEVPATH - RTC device path (/dev/rtc0)
    ONFIG_EXAMPLES_ALARM_SIGNO - Alarm signal

examples/bastest
^^^^^^^^^^^^^^^^
  This directory contains a small program that will mount a ROMFS file system
  containing the BASIC test files extracted from the BAS 2.4 release.  See
  examples/bastest/README.txt for licensing and usage information.

    CONFIG_EXAMPLES_BASTEST_DEVMINOR - The minor device number of the ROMFS block
      driver. For example, the N in /dev/ramN. Used for registering the RAM
      block driver that will hold the ROMFS file system containing the BASIC
      files to be tested.  Default: 0

    CONFIG_EXAMPLES_BASTEST_DEVPATH - The path to the ROMFS block driver device.  This
      must match EXAMPLES_BASTEST_DEVMINOR. Used for registering the RAM block driver
      that will hold the ROMFS file system containing the BASIC files to be
      tested.  Default: "/dev/ram0"

examples/bridge
^^^^^^^^^^^^^^^

  A simple test of a system with multiple networks.  It simply echoes all UDP
  packets received on network 1 and network 2 to network 2 and network 1,
  respectively.  Interface 1 and interface may or may not lie on the same
  network.

    CONFIG_EXAMPLES_BRIDGE - Enables the simple UDP bridge test

  There identical configurations for each of the two networks, NETn where n
  refers to the network being configured n={1,2}.  Let 'm' refer to the
  other network.

    CONFIG_EXAMPLES_BRIDGE_NETn_IFNAME - The register name of the network n
      device.  Must match the previously registered driver name and must
      not be the same as other network device name,
      CONFIG_EXAMPLES_BRIDGE_NETm_IFNAME
    CONFIG_EXAMPLES_BRIDGE_NETn_RECVPORT - Network n listen port number
    CONFIG_EXAMPLES_BRIDGE_NETn_SNDPORT - Network 2 send port number
    CONFIG_EXAMPLES_BRIDGE_NETn_IOBUFIZE - Size of the network n UDP
      send/receive I/O buffer
    CONFIG_EXAMPLES_BRIDGE_NETn_STACKSIZE - Network n daemon stacksize
    CONFIG_EXAMPLES_BRIDGE_NETn_PRIORITY - Network n daemon task priority

  If used as a NSH add-on, then it is assumed that initialization of both
  networks was performed externally prior to the time that this test was
  started.  Otherwise, the following options are available:

    CONFIG_EXAMPLES_BRIDGE_NETn_NOMAC - Select of the the network n hardware
      does not have a built-in MAC address.  If selected, the MAC address
      provided by CONFIG_EXAMPLES_BRIDGE_NETn_MACADDR will be used to assign
      the MAC address to the network n device.
    CONFIG_EXAMPLES_BRIDGE_NETn_DHCPC - Use DHCP Client to get the network n
      IP address.
    CONFIG_EXAMPLES_BRIDGE_NETn_IPADDR -- If CONFIG_EXAMPLES_BRIDGE_NETn_DHCPC
      is not selected, then this is the fixed IP address for network n.
    CONFIG_EXAMPLES_BRIDGE_NETn_DRIPADDR - Netweork n default router IP
      address (Gateway)
    CONFIG_EXAMPLES_BRIDGE_NETn_NETMASK - Network n mask.

examples/buttons
^^^^^^^^^^^^^^^^

  To be provided

examples/can
^^^^^^^^^^^^

  If the CAN device is configured in loopback mode, then this example can
  be used to test the CAN device in loop back mode.  It simple sinces a
  sequence of CAN messages and verifies that those messages are returned
  exactly as sent.

  This test depends on these specific CAN/NSH configurations settings (your
  specific CAN settings might require additional settings).

    CONFIG_CAN - Enables CAN support.
    CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
      mode for testing. The STM32 CAN driver does support loopback mode.
    CONFIG_NSH_BUILTIN_APPS - Build the CAN test as an NSH built-in function.
      Default: Built as a standalone problem

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_CAN_DEVPATH - The path to the CAN device. Default: /dev/can0
    CONFIG_EXAMPLES_CAN_NMSGS - If CONFIG_NSH_BUILTIN_APPS
      is defined, then the number of loops is provided on the command line
      and this value is ignored.  Otherwise, this number of CAN message is
      collected and the program terminates.  Default:  If built as an NSH
      built-in, the default is 32.  Otherwise messages are sent and received
      indefinitely.

  The default behavior assumes loopback mode.  Messages are sent, then read
  and verified.  The behavior can be altered for other kinds of testing where
  the test only sends or received (but does not verify) can messages.

   CONFIG_EXAMPLES_CAN_READONLY - Only receive messages
   CONFIG_EXAMPLES_CAN_WRITEONLY - Only send messages

examples/canard
^^^^^^^^^^^^^^^

  Example application for canutils/libcarnard.

examples/cc3000
^^^^^^^^^^^^^^^

  This is a test for the TI CC3000 wireless networking module.

examples/cctype
^^^^^^^^^^^^^^^

  Verifies all possible inputs for all functions defined in the header file
  cctype.

examples/chat
^^^^^^^^^^^^^

  Demonstrates AT chat functionality over a TTY device. This is useful with AT
  modems, for example, to establish a pppd connection (see the related pppd
  example). Moreover, some AT modems - such as ones made by u-blox - have an
  internal TCP/IP stack, often with an implementation of TLS/SSL. In such cases
  the chat utility can be used to configure the internal TCP/IP stack, establish
  socket connections, set up security (e.g., download base64-encoded
  certificates to the modem), and perform data exchange through sockets over the
  TTY device.

  Useful configuration parameters:
    CONFIG_EXAMPLES_CHAT_PRESET[0..3]    - preset chat scripts
    CONFIG_EXAMPLES_CHAT_TTY_DEVNODE     - TTY device node name
    CONFIG_EXAMPLES_CHAT_TIMEOUT_SECONDS - default receive timeout

examples/configdata
^^^^^^^^^^^^^^^^^^^

  This is a Unit Test for the MTD configuration data driver

examples/cpuhog
^^^^^^^^^^^^^^^

  Attempts to keep the system busy by passing data through a pipe in loop
  back mode.  This may be useful if you are trying run down other problems
  that you think might only occur when the system is very busy.

examples/cxxtest
^^^^^^^^^^^^^^^^

  This is a test of the C++ standard library.  At present a port of the uClibc++
  C++ library is available.  Due to licensing issues, the uClibc++ C++ library
  is not included in the NuttX source tree by default, but must be installed
  (see the README.txt file in the uClibc++ download package for installation).

  The uClibc++ test includes simple test of:

    - iostreams,
    - STL,
    - RTTI, and
    - Exceptions

  Example Configuration Options
  -----------------------------
    CONFIG_EXAMPLES_CXXTEST=y - Eanbles the example
    CONFIG_EXAMPLES_CXXTEST_CXXINITIALIZE=y - By default, if CONFIG_HAVE_CXX
      and CONFIG_HAVE_CXXINITIALIZE are defined, then this example
      will call the NuttX function to initialize static C++ constructors.
      This option may be disabled, however, if that static initialization
      was performed elsewhere.

  Other Required Configuration Settings
  -------------------------------------
  Other NuttX setting that are required include:

    CONFIG_HAVE_CXX=y
    CONFIG_HAVE_CXXINITIALIZE=y
    CONFIG_UCLIBCXX=y

  Additional uClibc++ settings may be required in your build environment.

examples/dhcpd
^^^^^^^^^^^^^^

  This examples builds a tiny DCHP server for the target system.

  NOTE: For test purposes, this example can be built as a
  host-based DHCPD server.  This can be built as follows:

    cd examples/dhcpd
    make -f Makefile.host TOPDIR=<nuttx-directory>

  NuttX configuration settings:

    CONFIG_NET=y                   - Of course
    CONFIG_NSOCKET_DESCRIPTORS     - And, of course, you must allocate some
                                     socket descriptors.
    CONFIG_NET_UDP=y               - UDP support is required for DHCP
                                     (as well as various other UDP-related
                                     configuration settings)
    CONFIG_NET_BROADCAST=y         - UDP broadcast support is needed.
    CONFIG_NETUTILS_NETLIB=y       - The networking library is needed

    CONFIG_EXAMPLES_DHCPD_NOMAC     - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLES_DHCPD_IPADDR    - Target IP address
    CONFIG_EXAMPLES_DHCPD_DRIPADDR  - Default router IP addess
    CONFIG_EXAMPLES_DHCPD_NETMASK   - Network mask

  See also CONFIG_NETUTILS_DHCPD_* settings described elsewhere
  and used in netutils/dhcpd/dhcpd.c. These settings are required
  to described the behavior of the daemon.

examples/discover
^^^^^^^^^^^^^^^^^

  This example exercises netutils/discover utility.  This example initializes
  and starts the UDP discover daemon. This daemon is useful for discovering
  devices in local networks, especially with DHCP configured devices.  It
  listens for UDP broadcasts which also can include a device class so that
  groups of devices can be discovered. It is also possible to address all
  classes with a kind of broadcast discover.

  This example will automatically be built as an NSH built-in if
  CONFIG_NSH_BUILTIN_APPS is selected.  Otherwise, it will be a standalone
  program with entry point "discover_main".

  NuttX configuration settings:

    CONFIG_EXAMPLES_DISCOVER_DHCPC - DHCP Client
    CONFIG_EXAMPLES_DISCOVER_NOMAC - Use canned MAC address
    CONFIG_EXAMPLES_DISCOVER_IPADDR - Target IP address
    CONFIG_EXAMPLES_DISCOVER_DRIPADDR - Router IP address
    CONFIG_EXAMPLES_DISCOVER_NETMASK - Network Mask

examples/adjoystick
^^^^^^^^^^^^^^^^^^

  This is a simple test of the discrete joystick driver.  See details about
  this driver in nuttx/include/nuttx/input/djoystick.h.

  Configuration Pre-requisites:

    CONFIG_DISABLE_SIGNALS - Must *NOT* be selected
    CONFIG_DJOYSTICK - The discrete joystick driver

  Example Configuration:
    CONFIG_EXAMPLES_DJOYSTICK - Enabled the discrete joystick example
    CONFIG_EXAMPLES_DJOYSTICK_DEVNAME - Joystick device name.  Default
      "/dev/djoy0"
    CONFIG_EXAMPLES_DJOYSTICK_SIGNO - Signal used to signal the test
      application.  Default 13.

examples/elf
^^^^^^^^^^^^

  This example builds a small ELF loader test case.  This includes several
  test programs under examples/elf tests.  These tests are build using
  the relocatable ELF format and installed in a ROMFS file system.  At run time,
  each program in the ROMFS file system is executed.  Requires CONFIG_ELF.
  Other configuration options:

    CONFIG_EXAMPLES_ELF_DEVMINOR - The minor device number of the ROMFS block
      driver. For example, the N in /dev/ramN. Used for registering the RAM
      block driver that will hold the ROMFS file system containing the ELF
      executables to be tested.  Default: 0

    CONFIG_EXAMPLES_ELF_DEVPATH - The path to the ROMFS block driver device.  This
      must match EXAMPLES_ELF_DEVMINOR. Used for registering the RAM block driver
      that will hold the ROMFS file system containing the ELF executables to be
      tested.  Default: "/dev/ram0"

  NOTES:

  1. CFLAGS should be provided in CELFFLAGS.  RAM and FLASH memory regions
     may require long allcs.  For ARM, this might be:

       CELFFLAGS = $(CFLAGS) -mlong-calls

     Similarly for C++ flags which must be provided in CXXELFFLAGS.

  2. Your top-level nuttx/Make.defs file must also include an approproate definition,
     LDELFFLAGS, to generate a relocatable ELF object.  With GNU LD, this should
     include '-r' and '-e main' (or _main on some platforms).

       LDELFFLAGS = -r -e main

     If you use GCC to link, you make also need to include '-nostdlib' or
     '-nostartfiles' and '-nodefaultlibs'.

  3. This example also requires genromfs.  genromfs can be build as part of the
     nuttx toolchain.  Or can built from the genromfs sources that can be found
     in the NuttX tools repository (genromfs-0.5.2.tar.gz).  In any event, the
     PATH variable must include the path to the genromfs executable.

  4. ELF size:  The ELF files in this example are, be default, quite large
     because they include a lot of "build garbage".  You can greatly reduce the
     size of the ELF binaries are using the 'objcopy --strip-unneeded' command to
     remove un-necessary information from the ELF files.

  5. Simulator.  You cannot use this example with the the NuttX simulator on
     Cygwin.  That is because the Cygwin GCC does not generate ELF file but
     rather some Windows-native binary format.

     If you really want to do this, you can create a NuttX x86 buildroot toolchain
     and use that be build the ELF executables for the ROMFS file system.

  6. Linker scripts.  You might also want to use a linker scripts to combine
     sections better.  An example linker script is at nuttx/binfmt/libelf/gnu-elf.ld.
     That example might have to be tuned for your particular linker output to
     position additional sections correctly.  The GNU LD LDELFFLAGS then might
     be:

       LDELFFLAGS = -r -e main -T$(TOPDIR)/binfmt/libelf/gnu-elf.ld

examples/flash_test
^^^^^^^^^^^^^^^^^^^

  This example performs a SMART flash block device test.  This test performs
  a sector allocate, read, write, free and garbage collection test on a SMART
  MTD block device.

    * CONFIG_EXAMPLES_FLASH_TEST=y - Enables the FLASH Test

  Dependencies:

    * CONFIG_MTD_SMART=y - SMART block driver support
    * CONFIG_NSH_BUILTIN_APPS=y - This example can only be built as an NSH
      command
    * CONFIG_BUILD_PROTECTED=n and CONFIG_BUILD_KERNEL=n- This test uses
      internal OS interfaces and so is not available in the NUTTX kernel
      builds

examples/fstest
^^^^^^^^^^^^^^

  This is a generic file system test that derives from examples/nxffs.  It
  was created to test the tmpfs file system, but should work with any file
  system provided that all initialization has already been performed prior
  to starting the test.

  * CONFIG_EXAMPLES_FSTEST: Enable the file system example
  * CONFIG_EXAMPLES_FSTEST_MAXNAME: Determines the maximum size of names used
    in the filesystem
  * CONFIG_EXAMPLES_FSTEST_MAXFILE: Determines the maximum size of a file
  * CONFIG_EXAMPLES_FSTEST_MAXIO: Max I/O, default 347.
  * CONFIG_EXAMPLES_FSTEST_MAXOPEN: Max open files.
  * CONFIG_EXAMPLES_FSTEST_MOUNTPT: Path where the file system is mounted.
  * CONFIG_EXAMPLES_FSTEST_NLOOPS: Number of test loops. default 100
  * CONFIG_EXAMPLES_FSTEST_VERBOSE: Verbose output

examples/ftpc
^^^^^^^^^^^^^

  This is a simple FTP client shell used to exercise the capabilities
  of the FTPC library (apps/netutils/ftpc).  This example is configured
  to that it will only work as a "built-in" program that can be run from
  NSH when CONFIG_NSH_BUILTIN_APPS is defined.

  From NSH, the startup command sequence is as follows.  This is only
  an example, your configration could have different mass storage devices,
  mount paths, and FTP directories:

    nsh> mount -t vfat /dev/mmcsd0 /tmp # Mount the SD card at /tmp
    nsh> cd /tmp                        # cd into the /tmp directory
    nsh> ftpc xx.xx.xx.xx[:pp]          # Start the FTP client
    nfc> login <name> <password>        # Log into the FTP server
    nfc> help                           # See a list of FTP commands

  where xx.xx.xx.xx is the IP address of the FTP server and pp is an
  optional port number.

  NOTE:  By default, FTPC uses readline to get data from stdin.  So your
  defconfig file must have the following build path:

    CONFIG_SYSTEM_READLINE=y

  NOTE: If you use the ftpc task over a telnet NSH connection, then you
  should set the following configuration item:

    CONFIG_EXAMPLES_FTPC_FGETS=y

  By default, the FTPC client will use readline() to get characters from
  the console.  Readline includes and command-line editor and echos
  characters received in stdin back through stdout.  Neither of these
  behaviors are desire-able if Telnet is used.

  You may also want to define the following in your configuration file.
  Otherwise, you will have not feeback about what is going on:

    CONFIG_DEBUG_FEATURES=y
    CONFIG_DEBUG_INFO=y
    CONFIG_DEBUG_FTPC=y

examples/ftpd
^^^^^^^^^^^^^^

  This example exercises the FTPD daemon at apps/netuils/ftpd.  Below are
  configurations specific to the FTPD example (the FTPD daemon itself may
  require other configuration options as well).

   CONFIG_EXAMPLES_FTPD - Enable the FTPD example.
   CONFIG_EXAMPLES_FTPD_PRIO - Priority of the FTP daemon.
     Default: SCHED_PRIORITY_DEFAULT
   CONFIG_EXAMPLES_FTPD_STACKSIZE - Stack size allocated for the
     FTP daemon. Default: 2048
   CONFIG_EXAMPLES_FTPD_NONETINIT - Define to suppress configuration of the
     network by apps/examples/ftpd.  You would need to suppress network
     configuration if the network is configuration prior to running the
     example.

  NSH always initializes the network so if CONFIG_NSH_BUILTIN_APPS is
  defined, so is CONFIG_EXAMPLES_FTPD_NONETINIT (se it does not explicitly
  need to be defined in that case):

    CONFIG_NSH_BUILTIN_APPS - Build the FTPD daemon example test as an
      NSH built-in function.  By default the FTPD daemon will be built
      as a standalone application.

  If CONFIG_EXAMPLES_FTPD_NONETINIT is not defined, then the following may
  be specified to customized the network configuration:

    CONFIG_EXAMPLES_FTPD_NOMAC - If the hardware has no MAC address of its
      own, define this =y to provide a bogus address for testing.
    CONFIG_EXAMPLES_FTPD_IPADDR - The target IP address.  Default 10.0.0.2
    CONFIG_EXAMPLES_FTPD_DRIPADDR - The default router address. Default
      10.0.0.1
    CONFIG_EXAMPLES_FTPD_NETMASK - The network mask.  Default: 255.255.255.0

  Other required configuration settings:  Of course TCP networking support
  is required.  But here are a couple that are less obvious:

    CONFIG_DISABLE_PTHREAD - pthread support is required
    CONFIG_DISABLE_POLL - poll() support is required

  Other FTPD configuration options thay may be of interest:

    CONFIG_FTPD_VENDORID - The vendor name to use in FTP communications.
      Default: "NuttX"
    CONFIG_FTPD_SERVERID - The server name to use in FTP communications.
      Default: "NuttX FTP Server"
    CONFIG_FTPD_CMDBUFFERSIZE - The maximum size of one command.  Default:
      512 bytes.
    CONFIG_FTPD_DATABUFFERSIZE - The size of the I/O buffer for data
      transfers.  Default: 2048 bytes.
    CONFIG_FTPD_WORKERSTACKSIZE - The stacksize to allocate for each
      FTP daemon worker thread.  Default:  2048 bytes.

  The following netutils libraries should be enabled in your defconfig
  file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_TELNED=y

examples/gpio
^^^^^^^^^^^^

  A simple test/example of the NuttX GPIO driver.

examples/hello
^^^^^^^^^^^^^^

  This is the mandatory, "Hello, World!!" example.  It is little more
  than examples/null with a single printf statement.  Really useful only
  for bringing up new NuttX architectures.

  * CONFIG_NSH_BUILTIN_APPS
    Build the "Hello, World" example as an NSH built-in application.

examples/helloxx
^^^^^^^^^^^^^^^^

  This is C++ version of the "Hello, World!!" example.  It is intended
  only to verify that the C++ compiler is functional, that basic C++
  library suupport is available, and that class are instantiated
  correctly.

  NuttX configuration prerequisites:

    CONFIG_HAVE_CXX -- Enable C++ Support

  Optional NuttX configuration settings:

    CONFIG_HAVE_CXXINITIALIZE -- Enable support for static constructors
      (may not be available on all platforms).

  NuttX configuration settings specific to this examp;le:

    CONFIG_NSH_BUILTIN_APPS -- Build the helloxx example as a
      "built-in"  that can be executed from the NSH command line.
    CONFIG_EXAMPLES_HELLOXX_NOSTACKCONST - Set if the system does not
      support construction of objects on the stack.
    CONFIG_EXAMPLES_HELLOXX_CXXINITIALIZE - By default, if CONFIG_HAVE_CXX
      and CONFIG_HAVE_CXXINITIALIZE are defined, then this example
      will call the NuttX function to initialize static C++ constructors.
      This option may be disabled, however, if that static initialization
      was performed elsewhere.

  Also needed:

    CONFIG_HAVE_CXX=y

  And you may have to tinker with the following to get libxx to compile
  properly:

    CONFIG_CXX_NEWLONG=y or =n

  The argument of the 'new' operators should take a type of size_t.  But size_t
  has an unknown underlying.  In the nuttx sys/types.h header file, size_t
  is typed as uint32_t (which is determined by architecture-specific logic).
  But the C++ compiler may believe that size_t is of a different type resulting
  in compilation errors in the operator.  Using the underlying integer type
  Instead of size_t seems to resolve the compilation issues.

examples/hidkbd
^^^^^^^^^^^^^^^^

  This is a simple test to debug/verify the USB host HID keyboard class
  driver.

    CONFIG_EXAMPLES_HIDKBD_DEFPRIO - Priority of "waiter" thread.  Default:
      50
    CONFIG_EXAMPLES_HIDKBD_STACKSIZE - Stacksize of "waiter" thread. Default
      1024
    CONFIG_EXAMPLES_HIDKBD_DEVNAME - Name of keyboard device to be used.
      Default: "/dev/kbda"
    CONFIG_EXAMPLES_HIDKBD_ENCODED - Decode special key press events in the
      user buffer.  In this case, the example coded will use the interfaces
      defined in include/nuttx/input/kbd_codec.h to decode the returned
      keyboard data.  These special keys include such things as up/down
      arrows, home and end keys, etc.  If this not defined, only 7-bit print-
      able and control ASCII characters will be provided to the user.
      Requires CONFIG_HIDKBD_ENCODED && CONFIG_LIB_KBDCODEC

examples/igmp
^^^^^^^^^^^^^

  This is a trivial test of the NuttX IGMP capability.  It present it
  does not do much of value -- Much more is needed in order to verify
  the IGMP features!

  * CONFIG_EXAMPLES_IGMP_NOMAC
      Set if the hardware has no MAC address; one will be assigned
  * CONFIG_EXAMPLES_IGMP_IPADDR
      Target board IP address
  * CONFIG_EXAMPLES_IGMP_DRIPADDR
      Default router address
  * CONFIG_EXAMPLES_IGMP_NETMASK
      Network mask
  * CONFIG_EXAMPLES_IGMP_GRPADDR
      Multicast group address
  * CONFIG_EXAMPLES_NETLIB
      The networking library is needed

examples/i2cchar
^^^^^^^^^^^^^^^^

  A mindlessly simple test of an I2C driver.  It reads an write garbage data to the
  I2C transmitter and/or received as fast possible.

  This test depends on these specific I2S/AUDIO/NSH configurations settings (your
  specific I2S settings might require additional settings).

    CONFIG_I2S - Enabled I2S support
    CONFIG_AUDIO - Enabled audio support
    CONFIG_AUDIO_DEVICES - Enable audio device support
    CONFIG_AUDIO_I2SCHAR = Enabled support for the I2S character device
    CONFIG_NSH_BUILTIN_APPS - Build the I2S test as an NSH built-in function.
      Default: Built as a standalone problem

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_I2SCHAR - Enables the I2C test
    CONFIG_EXAMPLES_I2SCHAR_DEVPATH - The default path to the ADC device.
      Default: /dev/i2schar0
    CONFIG_EXAMPLES_I2SCHAR_TX - This should be set if the I2S device supports
      a transmitter.
    CONFIG_EXAMPLES_I2SCHAR_TXBUFFERS - This is the default number of audio
      buffers to send before the TX transfers terminate.  When both TX and
      RX transfers terminate, the task exits (and, if an NSH builtin, the
      i2schar command returns).  This number can be changed from the NSH
      command line.
    CONFIG_EXAMPLES_I2SCHAR_TXSTACKSIZE - This is the stack size to use when
      starting the transmitter thread.  Default 1536.
    CONFIG_EXAMPLES_I2SCHAR_RX - This should be set if the I2S device supports
      a transmitter.
    CONFIG_EXAMPLES_I2SCHAR_RXBUFFERS - This is the default number of audio
      buffers to receive before the RX transfers terminate.  When both TX and
      RX transfers terminate, the task exits (and, if an NSH builtin, the
      i2schar command returns).  This number can be changed from the NSH
      command line.
    CONFIG_EXAMPLES_I2SCHAR_RXSTACKSIZE - This is the stack size to use when
      starting the receiver thread.  Default 1536.
    CONFIG_EXAMPLES_I2SCHAR_BUFSIZE - The size of the data payload in one
      audio buffer.  Applies to both TX and RX audio buffers.
    CONFIG_EXAMPLES_I2SCHAR_DEVINIT - Define if architecture-specific I2S
      device initialize is available.  If defined, the the platform specific
      code must provide a function i2schar_devinit() that will be called
      each time that this test executes.  Not available in the kernel build
      mode.

examples/json
^^^^^^^^^^^^^

  This example exercises the cJSON implementation at apps/netutils/json.
  This example contains logic taken from the cJSON project:

    http://sourceforge.net/projects/cjson/

  The example corresponds to SVN revision r42 (with lots of changes for
  NuttX coding standards).  As of r42, the SVN repository was last updated
  on 2011-10-10 so I presume that the code is stable and there is no risk
  of maintaining duplicate logic in the NuttX repository.

examples/keypadtest
^^^^^^^^^^^^^^^^^^^

  This is a generic keypad test example.  It is similar to the USB hidkbd
  example, but makes no assumptions about the underlying keyboard interface.
  It uses the interfaces of include/nuttx/input/keypad.h.

  CONFIG_EXAMPLES_KEYPADTEST - Selects the keypadtest example (only need
    if the mconf/Kconfig tool is used.

  CONFIG_EXAMPLES_KEYPAD_DEVNAME - The name of the keypad device that will
    be opened in order to perform the keypad test.  Default: "/dev/keypad"

examples/lcdrw
^^^^^^^^^^^^^^

  This example may be used to verify if you can or cannot read data
  correctly from an LCD interface.  At present, this supports only LCDs
  with RGB565 color format.

  * CONFIG_EXAMPLES_LDCRW_DEVNO
      LCD device number.  Default: 0
  * CONFIG_EXAMPLES_LDCRW_XRES
      LCD X resolution.  Default: 240
  * CONFIG_EXAMPLES_LDCRW_YRES
      LCD Y resolution.  Default: 320

  NOTE: This test exercises internal lcd driver interfaces.  As such, it
  relies on internal OS interfaces that are not normally available to a
  user-space program.  As a result, this example cannot be used if a
  NuttX is built as a protected, supervisor kernel (CONFIG_BUILD_PROTECTED
  or CONFIG_BUILD_KERNEL).

examples/leds
^^^^^^^^^^^^
  This is a simple test of the board LED driver at nuttx/drivers/leds/userled_*.c.

examples/ltdc
^^^^^^^^^^^^^

  STM32 LTDC framebuffer example.  Depends on:

  * CONFIG_STM32_LTDC

  Enabled with:

  * CONFIG_EXAMPLES_LTDC

examples/media
^^^^^^^^^^^^^^

  The media test simply writes values onto the media hidden behind a
  character driver and verifies that the media can be successfully written
  and read.  This low level test is useful in the early phases of the
  bringup of a new block or mtd driver because it avoids the complexity of
  a file system.

  This test uses a character driver and cannot directly access block or mtd
  drivers.  This test is suitable for use EEPROM character drivers (see
  nuttx/drivers/eeprom), or with block drivers wrapped as character drivers
  (see nuttx/drivers/bch)

    int ret = bchdev_register(<path-to-block-dirver>,
                              <path-to-character-driver>, false);

  MTD drivers need an additional wrapper layer, the FTL wrapper must first
  be used to convert the MTD driver to a block device:

    int ret = ftl_initialize(<N>, mtd);
    ret = bchdev_register(/dev/mtdblock<N>, <path-to-character-driver>,
                          false);

examples/mm
^^^^^^^^^^^

  This is a simple test of the memory manager.

examples/module
^^^^^^^^^^^^^^

  This example builds a small loadable module test case.  This includes on
  character driver under examples/module/drivers.  This driver is  built using
  the relocatable ELF format and installed in a ROMFS file system.  At run time,
  the driver module is loaded and exercises.  Requires CONFIG_MODULE.
  Other configuration options:

    CONFIG_EXAMPLES_ELF_DEVMINOR - The minor device number of the ROMFS block
      driver. For example, the N in /dev/ramN. Used for registering the RAM
      block driver that will hold the ROMFS file system containing the ELF
      executables to be tested.  Default: 0

    CONFIG_EXAMPLES_ELF_DEVPATH - The path to the ROMFS block driver device.  This
      must match EXAMPLES_ELF_DEVMINOR. Used for registering the RAM block driver
      that will hold the ROMFS file system containing the ELF executables to be
      tested.  Default: "/dev/ram0"

  NOTES:

  1. CFLAGS should be provided in CELFFLAGS.  RAM and FLASH memory regions
     may require long allcs.  For ARM, this might be:

       CELFFLAGS = $(CFLAGS) -mlong-calls

     Similarly for C++ flags which must be provided in CXXELFFLAGS.

  2. Your top-level nuttx/Make.defs file must also include an approproate definition,
     LDELFFLAGS, to generate a relocatable ELF object.  With GNU LD, this should
     include '-r' and '-e <entry point>'.

       LDELFFLAGS = -r -e module_initialize

     If you use GCC to link, you make also need to include '-nostdlib' or
     '-nostartfiles' and '-nodefaultlibs'.

  3. This example also requires genromfs.  genromfs can be build as part of the
     nuttx toolchain.  Or can built from the genromfs sources that can be found
     in the NuttX tools repository (genromfs-0.5.2.tar.gz).  In any event, the
     PATH variable must include the path to the genromfs executable.

  4. ELF size:  The ELF files in this example are, be default, quite large
     because they include a lot of "build garbage".  You can greatly reduce the
     size of the ELF binaries are using the 'objcopy --strip-unneeded' command to
     remove un-necessary information from the ELF files.

  5. Simulator.  You cannot use this example with the the NuttX simulator on
     Cygwin.  That is because the Cygwin GCC does not generate ELF file but
     rather some Windows-native binary format.

     If you really want to do this, you can create a NuttX x86 buildroot toolchain
     and use that be build the ELF executables for the ROMFS file system.

  6. Linker scripts.  You might also want to use a linker scripts to combine
     sections better.  An example linker script is at nuttx/sched/module/gnu-elf.ld.
     That example might have to be tuned for your particular linker output to
     position additional sections correctly.  The GNU LD LDELFFLAGS then might
     be:

       LDELFFLAGS = -r -e module_initialize -T$(TOPDIR)/sched/module/gnu-elf.ld

examples/modbus
^^^^^^^^^^^^^^^

  This is a port of the FreeModbus Linux demo.  It derives from the
  demos/LINUX directory of the FreeModBus version 1.5.0 (June 6, 2010)
  that can be downloaded in its entirety from http://developer.berlios.de/project/showfiles.php?group_id=6120.

    CONFIG_EXAMPLES_MODBUS_PORT, Default 0 (for /dev/ttyS0)
    CONFIG_EXAMPLES_MODBUS_BAUD, Default B38400
    CONFIG_EXAMPLES_MODBUS_PARITY, Default MB_PAR_EVEN

    CONFIG_EXAMPLES_MODBUS_REG_INPUT_START, Default 1000
    CONFIG_EXAMPLES_MODBUS_REG_INPUT_NREGS, Default 4
    CONFIG_EXAMPLES_MODBUS_REG_HOLDING_START, Default 2000
    CONFIG_EXAMPLES_MODBUS_REG_HOLDING_NREGS, Default 130

  The FreeModBus library resides at apps/modbus.  See apps/modbus/README.txt
  for additional configuration information.

examples/mount
^^^^^^^^^^^^^^

  This contains a simple test of filesystem mountpoints.

  * CONFIG_EXAMPLES_MOUNT_DEVNAME
      The name of the user-provided block device to mount.
      If CONFIG_EXAMPLES_MOUNT_DEVNAME is not provided, then
      a RAM disk will be configured.

  * CONFIG_EXAMPLES_MOUNT_NSECTORS
      The number of "sectors" in the RAM disk used when
      CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.

  * CONFIG_EXAMPLES_MOUNT_SECTORSIZE
      The size of each sectors in the RAM disk used when
      CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.

  * CONFIG_EXAMPLES_MOUNT_RAMDEVNO
      The RAM device minor number used to mount the RAM disk used
      when CONFIG_EXAMPLES_MOUNT_DEVNAME is not defined.  The
      default is zero (meaning that "/dev/ram0" will be used).

examples/mtdpart
^^^^^^^^^^^^^^^^

  This examples provides a simple test of MTD partition logic.

  * CONFIG_EXAMPLES_MTDPART - Enables the MTD partition test example
  * CONFIG_EXAMPLES_MTDPART_ARCHINIT - The default is to use the RAM MTD
    device at drivers/mtd/rammtd.c. But an architecture-specific MTD driver
    can be used instead by defining CONFIG_EXAMPLES_MTDPART_ARCHINIT.  In
    this case, the initialization logic will call mtdpart_archinitialize()
    to obtain the MTD driver instance.
  * CONFIG_EXAMPLES_MTDPART_NPARTITIONS - This setting provides the number
    of partitions to test.  The test will divide the reported size of the
    MTD device into equal-sized sub-regions for each test partition. Default:
    3

  When CONFIG_EXAMPLES_MTDPART_ARCHINIT is not defined, this test will use
  the RAM MTD device at drivers/mtd/rammtd.c to simulate FLASH. The size of
  the allocated RAM drive will be: CONFIG_EXMPLES_RAMMTD_ERASESIZE *
  CONFIG_EXAMPLES_MTDPART_NEBLOCKS

  * CONFIG_EXAMPLES_MTDPART_ERASESIZE - This value gives the size of one
    erase block in the MTD RAM device. This must exactly match the default
    configuration in drivers/mtd/rammtd.c!
  * CONFIG_EXAMPLES_MTDPART_NEBLOCKS - This value gives the nubmer of erase
    blocks in MTD RAM device.

examples/mtdrwb
^^^^^^^^^^^^^^^^

  This examples provides a simple test of MTD Read-Ahead/Write buffering
  logic.

  * CONFIG_EXAMPLES_MTDRWB - Enables the MTD R/W buffering test example
  * CONFIG_EXAMPLES_MTDRWB_ARCHINIT - The default is to use the RAM MTD
    device at drivers/mtd/rammtd.c. But an architecture-specific MTD driver
    can be used instead by defining CONFIG_EXAMPLES_MTDRWB_ARCHINIT.  In
    this case, the initialization logic will call mtdrwb_archinitialize()
    to obtain the MTD driver instance.

  When CONFIG_EXAMPLES_MTDRWB_ARCHINIT is not defined, this test will use
  the RAM MTD device at drivers/mtd/rammtd.c to simulate FLASH. The size of
  the allocated RAM drive will be: CONFIG_EXMPLES_RAMMTD_ERASESIZE *
  CONFIG_EXAMPLES_MTDRWB_NEBLOCKS

  * CONFIG_EXAMPLES_MTDRWB_ERASESIZE - This value gives the size of one
    erase block in the MTD RAM device. This must exactly match the default
    configuration in drivers/mtd/rammtd.c!
  * CONFIG_EXAMPLES_MTDRWB_NEBLOCKS - This value gives the nubmer of erase
    blocks in MTD RAM device.

examples/netpkt
^^^^^^^^^^^^^^^

  A test of AF_PACKET, "raw" sockets.  Contributed by Lazlo Sitzer.

examples/netloop
^^^^^^^^^^^^^^^^

  This is a simple test of the netwok loopback device.  examples/nettest can
  also be configured to provide (better) test of local loopback transfers.
  This version derives from examples/poll and is focused on testing poll()
  with loopback devices.

    CONFIG_EXAMPLES_NETLOOP=y - Enables the nettest example

  Dependencies:

    CONFIG_NSH_BUILTIN_APPS=n     - Does NOT work as an NSH built-in command
    CONFIG_NET_LOOPBACK           - Requires local loopback supprt
    CONFIG_NET_TCP                - Requires TCP support with the following:
    CONFIG_NET_TCPBACKLOG
    CONFIG_NET_TCP_READAHEAD
    CONFIG_NET_TCP_WRITE_BUFFERS
    CONFIG_NET_IPv4               - Currently supports only IPv4

examples/nettest
^^^^^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality in a TCP/IP connection.

    CONFIG_EXAMPLES_NETTEST=y - Enables the nettest example
    CONFIG_EXAMPLES_NETLIB=y  - The networking library in needed.

  See also examples/tcpecho

examples/nrf24l01_term
^^^^^^^^^^^^^^^^^^^^^^

  These is a simple test of NRF24L01-based wireless connectivity.  Enabled\
  with:

    CONFIG_EXAMPLES_NRF24L01TERM

  Options:

    CONFIG_NSH_BUILTIN_APPS - Built as an NSH built-in applications.

examples/nsh
^^^^^^^^^^^^

  Basic Configuration
  -------------------
  This directory provides an example of how to configure and use
  the NuttShell (NSH) application.  NSH is a simple shell
  application.  NSH is described in its own README located at
  apps/nshlib/README.txt.  This function is enabled with:

    CONFIG_EXAMPLES_NSH=y

  Applications using this example will need to provide an defconfig
  file in the configuration directory with instruction to build
  the NSH library like:

    CONFIG_NSH_LIBRARY=y

  Other Configuration Requirements
  --------------------------------
  NOTE:  If the NSH serial console is used, then following is also
  required to build the readline() library:

    CONFIG_SYSTEM_READLINE=y

  And if networking is included:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_DHCPC=y
    CONFIG_NETDB_DNSCLIENT=y
    CONFIG_NETUTILS_TFTPC=y
    CONFIG_NETUTILS_WEBCLIENT=y

  If the Telnet console is enabled, then the defconfig file should
  also include:

    CONFIG_NETUTILS_TELNETD=y

  Also if the Telnet console is enabled, make sure that you have the
  following set in the NuttX configuration file or else the performance
  will be very bad (because there will be only one character per TCP
  transfer):

    CONFIG_STDIO_BUFFER_SIZE - Some value >= 64
    CONFIG_STDIO_LINEBUFFER=y

  C++ Support
  -----------
  If CONFIG_HAVE_CXX=y and CONFIG_HAVE_CXXINITIALIZE=y, then this NSH
  example can be configured to initialize C++ constructors when it
  is started.  NSH does not use C++ and, by default, assumes that
  constructors are initialized elsewhere.  However, you can force
  NSH to initialize constructors by setting:

    CONFIG_EXAMPLES_NSH_CXXINITIALIZE=y

examples/nx
^^^^^^^^^^^

  This directory contains a simple test of a subset of the NX APIs
  defined in include/nuttx/nx/nx.h.  The following configuration options
  can be selected:

    CONFIG_NSH_BUILTIN_APPS -- Build the NX example as a "built-in"
      that can be executed from the NSH command line
    CONFIG_EXAMPLES_NX_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NX_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
    CONFIG_EXAMPLES_NX_BGCOLOR -- The color of the background.  Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_COLOR1 -- The color of window 1. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_COLOR2 -- The color of window 2. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_TBCOLOR -- The color of the toolbar. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_FONTID - Selects the font (see font ID numbers in
      include/nuttx/nx/nxfonts.h)
    CONFIG_EXAMPLES_NX_FONTCOLOR -- The color of the fonts. Default depends on
      CONFIG_EXAMPLES_NX_BPP.
    CONFIG_EXAMPLES_NX_BPP -- Pixels per pixel to use.  Valid options
      include 2, 4, 8, 16, 24, and 32.  Default is 32.
    CONFIG_EXAMPLES_NX_RAWWINDOWS -- Use raw windows;  Default is to
      use pretty, framed NXTK windows with toolbars.
    CONFIG_EXAMPLES_NX_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
      #else
      FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
      #endif

      and must also define: CONFIG_LIB_BOARDCTL=y and
      CONFIG_BOARDCTL_GRAPHICS=y so that the boardctl() interface
      will be available in order to access this function.

  This test can be performed with either the single-user version of
  NX or with the multiple user version of NX selected with CONFIG_NX_MULTIUSER.
  If CONFIG_NX_MULTIUSER is defined, then the following configuration
  options also apply:

    CONFIG_EXAMPLES_NX_STACKSIZE -- The stacksize to use when creating
      the NX server.  Default 2048
    CONFIG_EXAMPLES_NX_CLIENTPRIO -- The client priority.  Default: 100
    CONFIG_EXAMPLES_NX_SERVERPRIO -- The server priority.  Default: 120
    CONFIG_EXAMPLES_NX_LISTENERPRIO -- The priority of the event listener
      thread. Default 80.
    CONFIG_EXAMPLES_NX_NOTIFYSIGNO -- The signal number to use with
      nx_eventnotify().  Default: 4

  If CONFIG_NX_MULTIUSER is defined, then the example also expects the
  following settings and will generate an error if they are not as expected:

    CONFIG_DISABLE_MQUEUE=n
    CONFIG_DISABLE_SIGNALS=n
    CONFIG_DISABLE_PTHREAD=n
    CONFIG_NX_BLOCKING=y

examples/nxterm
^^^^^^^^^^^^^^^^^^

  This directory contains yet another version of the NuttShell (NSH).  This
  version uses the NX console device defined in include/nuttx/nx/nxterm.h
  for output.  the result is that the NSH input still come from the standard
  console input (probably a serial console).  But the text output will go to
  an NX winbdow.  Prerequisite configuration settings for this test include:

    CONFIG_NX=y              -- NX graphics must be enabled
    CONFIG_NXTERM=y       -- The NX console driver must be built
    CONFIG_NX_MULTIUSER=y    -- NX multi-user support must be enabled.
    CONFIG_DISABLE_MQUEUE=n  -- Message queue support must be available.
    CONFIG_DISABLE_SIGNALS=n -- Signals are needed
    CONFIG_DISABLE_PTHREAD=n -- pthreads are needed
    CONFIG_NX_BLOCKING=y     -- pthread APIs must be blocking
    CONFIG_NSH_CONSOLE=y     -- NSH must be configured to use a console.

  The following configuration options can be selected to customize the
  test:

    CONFIG_EXAMPLES_NXCON_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NXCON_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
    CONFIG_EXAMPLES_NXCON_BGCOLOR -- The color of the background.  Default
      Default is a darker royal blue.
    CONFIG_EXAMPLES_NXCON_WCOLOR -- The color of the window. Default is a light
      slate blue.
    CONFIG_EXAMPLES_NXCON_FONTID -- Selects the font (see font ID numbers in
      include/nuttx/nx/nxfonts.h)
    CONFIG_EXAMPLES_NXCON_FONTCOLOR -- The color of the fonts. Default is
      black.
    CONFIG_EXAMPLES_NXCON_BPP -- Pixels per pixel to use.  Valid options
      include 2, 4, 8, 16, 24, and 32.  Default is 32.
    CONFIG_EXAMPLES_NXCON_TOOLBAR_HEIGHT -- The height of the toolbar.
      Default: 16
    CONFIG_EXAMPLES_NXCON_TBCOLOR -- The color of the toolbar. Default is
      a medium grey.
    CONFIG_EXAMPLES_NXCON_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
      #else
      FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
      #endif

      and must also define: CONFIG_LIB_BOARDCTL=y and
      CONFIG_BOARDCTL_GRAPHICS=y so that the boardctl() interface
      will be available in order to access this function.

    CONFIG_EXAMPLES_NXCON_MINOR -- The NX console device minor number.
      Default is 0 corresponding to /dev/nxterm0
    CONFIG_EXAMPLES_NXCON_DEVNAME -- The quoated, full path to the
      NX console device corresponding to CONFIG_EXAMPLES_NXCON_MINOR.
      Default: "/dev/nxterm0"
    CONFIG_EXAMPLES_NXTERM_PRIO - Priority of the NxTerm task.
      Default: SCHED_PRIORITY_DEFAULT
    CONFIG_EXAMPLES_NXTERM_STACKSIZE - Stack size allocated for the
      NxTerm task. Default: 2048

  The following configuration settings determine how to set up the NX
  server (CONFIG_NX_MULTIUSER):

    CONFIG_EXAMPLES_NXCON_STACKSIZE -- The stacksize to use when creating
      the NX server.  Default 2048
    CONFIG_EXAMPLES_NXCON_CLIENTPRIO -- The client priority.  Default: 100
    CONFIG_EXAMPLES_NXCON_SERVERPRIO -- The server priority.  Default: 120
    CONFIG_EXAMPLES_NXCON_LISTENERPRIO -- The priority of the event listener
      thread. Default 80.
    CONFIG_EXAMPLES_NXCON_NOTIFYSIGNO -- The signal number to use with
      nx_eventnotify().  Default: 4

examples/nxffs
^^^^^^^^^^^^^^

  This is a test of the NuttX NXFFS FLASH file system.  This is an NXFFS
  stress test and beats on the file system very hard.  It should only
  be used in a simulation environment!  Putting this NXFFS test on real
  hardware will most likely destroy your FLASH.  You have been warned.

examples/nxflat
^^^^^^^^^^^^^^^

  This example builds a small NXFLAT test case.  This includes several
  test programs under examples/nxflat tests.  These tests are build using
  the NXFLAT format and installed in a ROMFS file system.  At run time,
  each program in the ROMFS file system is executed.  Requires CONFIG_NXFLAT.

examplex/nxhello
^^^^^^^^^^^^^^^^

  A very simple graphics example that just says "Hello, World!" in the
  center of the display.

  The following configuration options can be selected:

    CONFIG_NSH_BUILTIN_APPS -- Build the NXHELLO example as a "built-in"
      that can be executed from the NSH command line
    CONFIG_EXAMPLES_NXHELLO_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NXHELLO_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
    CONFIG_EXAMPLES_NXHELLO_BGCOLOR -- The color of the background.  Default
      depends on CONFIG_EXAMPLES_NXHELLO_BPP.
    CONFIG_EXAMPLES_NXHELLO_FONTID - Selects the font (see font ID numbers in
      include/nuttx/nx/nxfonts.h)
    CONFIG_EXAMPLES_NXHELLO_FONTCOLOR -- The color of the fonts used in the
      background window. Default depends on CONFIG_EXAMPLES_NXHELLO_BPP.
    CONFIG_EXAMPLES_NXHELLO_BPP -- Pixels per pixel to use.  Valid options
      include 2, 4, 8, 16, 24, and 32.  Default is 32.
    CONFIG_EXAMPLES_NXHELLO_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
      #else
      FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
      #endif

      and must also define: CONFIG_LIB_BOARDCTL=y and
      CONFIG_BOARDCTL_GRAPHICS=y so that the boardctl() interface
      will be available in order to access this function.

examples/nximage
^^^^^^^^^^^^^^^^

  This is a simple example that just puts the NuttX logo image in the center
  of the display.  This only works for RGB23 (888), RGB16 (656), RGB8 (332),
  and 8-bit greyscale for now.

    CONFIG_NSH_BUILTIN_APPS -- Build the NXIMAGE example as a "built-in"
      that can be executed from the NSH command line
    CONFIG_EXAMPLES_NXIMAGE_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NXIMAGE_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
    CONFIG_EXAMPLES_NXIMAGE_BPP -- Pixels per pixel to use.  Valid options
      include 8, 16, and 24.  Default is 16.
    CONFIG_EXAMPLES_NXIMAGE_XSCALEp5, CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5,
    CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0 -- The logo image width is 160 columns.
      One of these may be defined to rescale the image horizontally by .5, 1.5,
      or 2.0.
    CONFIG_EXAMPLES_NXIMAGE_YSCALEp5, CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5,
    CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0 -- The logo image height is 160 rows.
      One of these may be defined to rescale the image vertically by .5, 1.5,
      or 2.0.
    CONFIG_EXAMPLES_NXIMAGE_GREYSCALE -- Grey scale image.  Default: RGB.
    CONFIG_EXAMPLES_NXIMAGE_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
      #else
      FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
      #endif

      and must also define: CONFIG_LIB_BOARDCTL=y and
      CONFIG_BOARDCTL_GRAPHICS=y so that the boardctl() interface
      will be available in order to access this function.

    How was that run-length encoded image produced?

    a. I used GIMP output the image as a .c file.
    b. I added som C logic to palette-ize the RGB image in the GIMP .c file
    c. Then I add some simple run-length encoding to palette-ized image.

    But now there is a tool that can be found in the NxWidgets package at
    NxWidgets/tools/bitmap_converter.py that can be used to convert any
    graphics format to the NuttX RLE format.

    NOTE: As of this writing, most of the pixel depth, scaling options, and
    combinations thereof have not been tested.

examplex/nxlines
^^^^^^^^^^^^^^^^

  A very simple graphics example that just exercised the NX line drawing
  logic.

  The following configuration options can be selected:

    CONFIG_EXAMPLES_NXLINES_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NXLINES_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
    CONFIG_EXAMPLES_NXLINES_BGCOLOR -- The color of the background.  Default
      depends on CONFIG_EXAMPLES_NXLINES_BPP.
    CONFIG_EXAMPLES_NXLINES_LINEWIDTH - Selects the width of the lines in
      pixels (default: 16)
    CONFIG_EXAMPLES_NXLINES_LINECOLOR -- The color of the central lines drawn
      in the background window. Default depends on CONFIG_EXAMPLES_NXLINES_BPP
      (there really is no meaningful default).
    CONFIG_EXAMPLES_NXLINES_BORDERWIDTH -- The width of the circular border
      drawn in the background window. (default: 16).
    CONFIG_EXAMPLES_NXLINES_BORDERCOLOR -- The color of the circular border
      drawn in the background window. Default depends on CONFIG_EXAMPLES_NXLINES_BPP
      (there really is no meaningful default).
    CONFIG_EXAMPLES_NXLINES_CIRCLECOLOR -- The color of the circular region
      filled in the background window. Default depends on CONFIG_EXAMPLES_NXLINES_BPP
      (there really is no meaningful default).
    CONFIG_EXAMPLES_NXLINES_BORDERCOLOR -- The color of the lines drawn in the
      background window. Default depends on CONFIG_EXAMPLES_NXLINES_BPP (there
      really is no meaningful default).

    CONFIG_EXAMPLES_NXLINES_BPP -- Pixels per pixel to use.  Valid options
      include 2, 4, 8, 16, 24, and 32.  Default is 16.
    CONFIG_EXAMPLES_NXLINES_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
      #else
      FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
      #endif

      and must also define: CONFIG_LIB_BOARDCTL=y and
      CONFIG_BOARDCTL_GRAPHICS=y so that the boardctl() interface
      will be available in order to access this function.

    CONFIG_NSH_BUILTIN_APPS - Build the NX lines examples as an NSH built-in
      function.

examples/nxtext
^^^^^^^^^^^^^^^

  This directory contains another simple test of a subset of the NX APIs
  defined in include/nuttx/nx/nx.h.  This text focuses on text displays on
  the dispaly background combined with pop-up displays over the text.
  The text display will continue to update while the pop-up is visible.

  NOTE:  This example will *only* work with FB drivers and with LCD
  drivers that support reading the contents of the internal LCD memory
  *unless* you define CONFIG_EXAMPLES_NXTEXT_NOGETRUN.  If you notice
  garbage on the display or a failure at the point where the display
  should scroll, it is probably because you have an LCD driver that is
  write-only.

  The following configuration options can be selected:

    CONFIG_NSH_BUILTIN_APPS -- Build the NXTEXT example as a "built-in"
      that can be executed from the NSH command line
    CONFIG_EXAMPLES_NXTEXT_VPLANE -- The plane to select from the frame-
      buffer driver for use in the test.  Default: 0
    CONFIG_EXAMPLES_NXTEXT_DEVNO - The LCD device to select from the LCD
      driver for use in the test: Default: 0
    CONFIG_EXAMPLES_NXTEXT_BGCOLOR -- The color of the background.  Default
      depends on CONFIG_EXAMPLES_NXTEXT_BPP.
    CONFIG_EXAMPLES_NXTEXT_BGFONTID - Selects the font to use in the
      background text (see font ID numbers in include/nuttx/nx/nxfonts.h)
    CONFIG_EXAMPLES_NXTEXT_BGFONTCOLOR -- The color of the fonts used in the
      background window. Default depends on CONFIG_EXAMPLES_NXTEXT_BPP.
    CONFIG_EXAMPLES_NXTEXT_PUCOLOR -- The color of the pop-up window.  Default
      depends on CONFIG_EXAMPLES_NXTEXT_BPP.
    CONFIG_EXAMPLES_NXTEXT_PUFONTID - Selects the font to use in the pop-up
      windows (see font ID numbers in include/nuttx/nx/nxfonts.h)
    CONFIG_EXAMPLES_NXTEXT_PUFONTCOLOR -- The color of the fonts used in the
      background window. Default depends on CONFIG_EXAMPLES_NXTEXT_BPP.
    CONFIG_EXAMPLES_NXTEXT_BPP -- Pixels per pixel to use.  Valid options
      include 2, 4, 8, 16, 24, and 32.  Default is 32.
    CONFIG_EXAMPLES_NXTEXT_NOGETRUN -- If your display is read-only OR if
      reading is not reliable, then select this configuration to avoid
      reading from the display.
    CONFIG_EXAMPLES_NXTEXT_EXTERNINIT - The driver for the graphics device on
      this platform requires some unusual initialization.  This is the
      for, for example, SPI LCD/OLED devices.  If this configuration is
      selected, then the platform code must provide an LCD initialization
      function with a prototype like:

      #ifdef CONFIG_NX_LCDDRIVER
      FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno);
      #else
      FAR struct fb_vtable_s *board_graphics_setup(unsigned int devno);
      #endif

      and must also define: CONFIG_LIB_BOARDCTL=y and
      CONFIG_BOARDCTL_GRAPHICS=y so that the boardctl() interface
      will be available in order to access this function.

    CONFIG_EXAMPLES_NXTEXT_BMCACHE - The maximum number of characters that
      can be put in the background window.  Default is 128.
    CONFIG_EXAMPLES_NXTEXT_GLCACHE - The maximum nuber of pre-rendered
      fonts that can be retained for the background window.

  This test can be performed with either the single-user version of
  NX or with the multiple user version of NX selected with CONFIG_NX_MULTIUSER.
  If CONFIG_NX_MULTIUSER is defined, then the following configuration
  options also apply:

    CONFIG_EXAMPLES_NXTEXT_STACKSIZE -- The stacksize to use when creating
      the NX server.  Default 2048
    CONFIG_EXAMPLES_NXTEXT_CLIENTPRIO -- The client priority.  Default: 100
    CONFIG_EXAMPLES_NXTEXT_SERVERPRIO -- The server priority.  Default: 120
    CONFIG_EXAMPLES_NXTEXT_LISTENERPRIO -- The priority of the event listener
      thread. Default 80.
    CONFIG_EXAMPLES_NXTEXT_NOTIFYSIGNO -- The signal number to use with
      nx_eventnotify().  Default: 4

  If CONFIG_NX_MULTIUSER is defined, then the example also expects the
  following settings and will generate an error if they are not as expected:

    CONFIG_DISABLE_MQUEUE=n
    CONFIG_DISABLE_SIGNALS=n
    CONFIG_DISABLE_PTHREAD=n
    CONFIG_NX_BLOCKING=y

examples/null
^^^^^^^^^^^^^

  This is the do nothing application.  It is only used for bringing
  up new NuttX architectures in the most minimal of environments.

examples/oneshot
^^^^^^^^^^^^^^^^

  Simple test of a oneshot driver.

examples/ostest
^^^^^^^^^^^^^^^

  This is the NuttX 'qualification' suite.  It attempts to exercise
  a broad set of OS functionality.  Its coverage is not very extensive
  as of this writing, but it is used to qualify each NuttX release.

  The behavior of the ostest can be modified with the following
  settings in the configs/<board-name>/defconfig file:

  * CONFIG_NSH_BUILTIN_APPS
      Build the OS test example as an NSH built-in application.
  * CONFIG_EXAMPLES_OSTEST_LOOPS
      Used to control the number of executions of the test.  If
      undefined, the test executes one time.  If defined to be
      zero, the test runs forever.
  * CONFIG_EXAMPLES_OSTEST_STACKSIZE
      Used to create the ostest task.  Default is 8192.
  * CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS
      Specifies the number of threads to create in the barrier
      test.  The default is 8 but a smaller number may be needed on
      systems without sufficient memory to start so many threads.
  * CONFIG_EXAMPLES_OSTEST_RR_RANGE
      During round-robin scheduling test two threads are created. Each of the threads
      searches for prime numbers in the configurable range, doing that configurable
      number of times.
      This value specifies the end of search range and together with number of runs
      allows to configure the length of this test - it should last at least a few
      tens of seconds. Allowed values [1; 32767], default 10000
  * CONFIG_EXAMPLES_OSTEST_RR_RUNS
      During round-robin scheduling test two threads are created. Each of the threads
      searches for prime numbers in the configurable range, doing that configurable
      number of times.

examples/pashello
^^^^^^^^^^^^^^^^^

  This is "Hello, World" implemented via the Pascal P-Code interpreter. In
  order to use this example, you must first download and install the
  NuttX pascal module.  After unpacking the pascal module, you can find
  installation instructions in pascal/nuttx/README.txt.

  The correct install location for the NuttX examples and build files is
  apps/interpreters.

examples/pca9635
^^^^^^^^^^^^^^^^

  A simple test of the PCA9635PW LED driver.

examples/pipe
^^^^^^^^^^^^^

  A test of the mkfifo() and pipe() APIs.  Requires CONFIG_PIPES

 * CONFIG_EXAMPLES_PIPE_STACKSIZE
     Sets the size of the stack to use when creating the child tasks.
     The default size is 1024.

examples/poll
^^^^^^^^^^^^^

  A test of the poll() and select() APIs using FIFOs and, if available,
  stdin, and a TCP/IP socket.  In order to build this test, you must the
  following selected in your NuttX configuration file:

  CONFIG_NFILE_DESCRIPTORS          - Defined to be greater than 0
  CONFIG_DISABLE_POLL               - NOT defined

  In order to use the TCP/IP select test, you have also the following
  additional things selected in your NuttX configuration file:

  CONFIG_NET                        - Defined for general network support
  CONFIG_NET_TCP                    - Defined for TCP/IP support
  CONFIG_NSOCKET_DESCRIPTORS        - Defined to be greater than 0
  CONFIG_NET_TCP_READAHEAD          - Defined
  CONFIG_NET_NTCP_READAHEAD_BUFFERS - Defined to be greater than zero

  CONFIG_EXAMPLES_POLL_NOMAC         - (May be defined to use software assigned MAC)
  CONFIG_EXAMPLES_POLL_IPADDR        - Target IP address
  CONFIG_EXAMPLES_POLL_DRIPADDR      - Default router IP addess
  CONFIG_EXAMPLES_POLL_NETMASK       - Network mask

  In order to for select to work with incoming connections, you
  must also select:

  CONFIG_NET_TCPBACKLOG             - Incoming connections pend in a backlog until accept() is called.

  In additional to the target device-side example, there is also
  a host-side application in this directory.  It can be compiled under
  Linux or Cygwin as follows:

    cd examples/usbserial
    make -f Makefile.host TOPDIR=<nuttx-directory> TARGETIP=<target-ip>

  Where <target-ip> is the IP address of your target board.

  This will generate a small program called 'host'.  Usage:

  1. Build the examples/poll target program with TCP/IP poll support
     and start the target.

  3. Then start the host application:

       ./host

  The host and target will exchange are variety of small messages. Each
  message sent from the host should cause the select to return in target.
  The target example should read the small message and send it back to
  the host.  The host should then receive the echo'ed message.

  If networking is enabled, applications using this example will need to
  provide the following definition in the defconfig file to enable the
  networking library:

    CONFIG_NETUTILS_NETLIB=y

examples/posix_spawn
^^^^^^^^^^^^^^^^^^^^

  This is a simple test of the posix_spawn() API. The example derives from
  examples/elf.  As a result, these tests are built using the relocatable
  ELF format installed in a ROMFS file system.  At run time, the test program
  in the ROMFS file system is spawned using posix_spawn().

  Requires:

    CONFIG_BINFMT_DISABLE=n           - Don't disable the binary loader
    CONFIG_ELF=y                      - Enable ELF binary loader
    CONFIG_LIBC_EXECFUNCS=y           - Enable support for posix_spawn
    CONFIG_EXECFUNCS_SYMTAB="exports" - The name of the symbol table
                                        created by the test.
    CONFIG_EXECFUNCS_NSYMBOLS=10      - Value does not matter, it will be
                                        corrected at runtime.
    CONFIG_POSIX_SPAWN_STACKSIZE=768  - This default setting.

  Test-specific configuration options:

    CONFIG_EXAMPLES_POSIXSPAWN_DEVMINOR - The minor device number of the ROMFS
      block. driver.  For example, the N in /dev/ramN. Used for registering the
      RAM block driver that will hold the ROMFS file system containing the ELF
      executables to be tested.  Default: 0

    CONFIG_EXAMPLES_POSIXSPAWN_DEVPATH - The path to the ROMFS block driver
      device.  This must match EXAMPLES_POSIXSPAWN_DEVMINOR. Used for
      registering the RAM block driver that will hold the ROMFS file system
      containing the ELF executables to be tested.  Default: "/dev/ram0"

  NOTES:

  1. CFLAGS should be provided in CELFFLAGS.  RAM and FLASH memory regions
     may require long allcs.  For ARM, this might be:

       CELFFLAGS = $(CFLAGS) -mlong-calls

     Similarly for C++ flags which must be provided in CXXELFFLAGS.

  2. Your top-level nuttx/Make.defs file must also include an approproate
     definition, LDELFFLAGS, to generate a relocatable ELF object.  With GNU
     LD, this should include '-r' and '-e main' (or _main on some platforms).

       LDELFFLAGS = -r -e main

     If you use GCC to link, you make also need to include '-nostdlib' or
     '-nostartfiles' and '-nodefaultlibs'.

  3. This example also requires genromfs.  genromfs can be build as part of the
     nuttx toolchain.  Or can built from the genromfs sources that can be found
     in the NuttX tools repository (genromfs-0.5.2.tar.gz).  In any event, the
     PATH variable must include the path to the genromfs executable.

  4. ELF size:  The ELF files in this example are, be default, quite large
     because they include a lot of "build garbage".  You can greatly reduce the
     size of the ELF binaries are using the 'objcopy --strip-unneeded' command to
     remove un-necessary information from the ELF files.

  5. Simulator.  You cannot use this example with the the NuttX simulator on
     Cygwin.  That is because the Cygwin GCC does not generate ELF file but
     rather some Windows-native binary format.

     If you really want to do this, you can create a NuttX x86 buildroot toolchain
     and use that be build the ELF executables for the ROMFS file system.

  6. Linker scripts.  You might also want to use a linker scripts to combine
     sections better.  An example linker script is at nuttx/binfmt/libelf/gnu-elf.ld.
     That example might have to be tuned for your particular linker output to
     position additional sections correctly.  The GNU LD LDELFFLAGS then might
     be:

       LDELFFLAGS = -r -e main -T$(TOPDIR)/binfmt/libelf/gnu-elf.ld

examples/pty_test
^^^^^^^^^^^^^^^^^

  A test of NuttX pseudo-terminals.  Provided by Alan Carvalho de Assis.

examples/pwm
^^^^^^^^^^^^

  A test of a PWM device driver. It simply enables a pulsed output for
  a specified frequency and duty for a specified period of time.  This
  example can ONLY be built as an NSH built-in function.

  This test depends on these specific PWM/NSH configurations settings (your
  specific PWM settings might require additional settings).

    CONFIG_PWM - Enables PWM support.
    CONFIG_PWM_PULSECOUNT - Enables PWM pulse count support (if the hardware
      supports it).
    CONFIG_NSH_BUILTIN_APPS - Build the PWM test as an NSH built-in function.
      Default: Not built!  The example can only be used as an NSH built-in
      application

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_PWM_DEVPATH - The path to the default PWM device. Default: /dev/pwm0
    CONFIG_EXAMPLES_PWM_FREQUENCY - The initial PWM frequency.  Default: 100 Hz
    CONFIG_EXAMPLES_PWM_DUTYPCT - The initial PWM duty as a percentage.  Default: 50%
    CONFIG_EXAMPLES_PWM_DURATION - The initial PWM pulse train duration in seconds.
       Used only if the current pulse count is zero (pulse count is only supported
       if CONFIG_PWM_PULSECOUNT is defined). Default: 5 seconds
    CONFIG_EXAMPLES_PWM_PULSECOUNT - The initial PWM pulse count.  This option is
       only available if CONFIG_PWM_PULSECOUNT is non-zero. Default: 0 (i.e., use
       the duration, not the count).

examples/qencoder
^^^^^^^^^^^^^^^^^

  This example is a simple test of a Quadrature Encoder driver.  It simply reads
  positional data from the encoder and prints it.,

  This test depends on these specific QE/NSH configurations settings (your
  specific PWM settings might require additional settings).

    CONFIG_QENCODER - Enables quadrature encoder support (upper-half driver).
    CONFIG_NSH_BUILTIN_APPS - Build the QE test as an NSH built-in function.
      Default: Built as a standalone progrem.

  Additional configuration options will mostly likely be required for the board-
  specific lower-half driver.  See the README.txt file in your board configuration
  directory.

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_QENCODER_DEVPATH - The path to the QE device. Default:
      /dev/qe0
    CONFIG_EXAMPLES_QENCODER_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
      is defined, then the number of samples is provided on the command line
      and this value is ignored.  Otherwise, this number of samples is
      collected and the program terminates.  Default:  Samples are collected
      indefinitely.
    CONFIG_EXAMPLES_QENCODER_DELAY - This value provides the delay (in
      milliseonds) between each sample.  If CONFIG_NSH_BUILTIN_APPS
      is defined, then this value is the default delay if no other delay is
      provided on the command line.  Default:  100 milliseconds

examples/random
^^^^^^^^^^^^^^^

  This is a very simply test of /dev/random.  It simple collects random
  numbers and displays them on the console.

  Prerequistes:

    CONFIG_DEV_RANDOM - Support for /dev/random must be enabled in order
      to select this example.

  Configuration:

    CONFIG_EXAMPLES_RANDOM - Enables the /dev/random test
    CONFIG_EXAMPLES_MAXSAMPLES - This is the size of the /dev/random I/O
      buffer in units of 32-bit samples.  Careful!  This buffer is allocated
      on the stack as needed! Default 64.
    CONFIG_EXAMPLES_NSAMPLES; - When you execute the rand command, a number
      of samples ranging from 1 to EXAMPLES_MAXSAMPLES may be specified.  If
      no argument is specified, this is the default number of samples that\
      will be collected and displayed.  Default 8

examples/relays
^^^^^^^^^^^^^^^

  Requires CONFIG_ARCH_RELAYS.
  Contributed by Darcy Gong.

  NOTE: This test exercises internal relay driver interfaces.  As such, it
  relies on internal OS interfaces that are not normally available to a
  user-space program.  As a result, this example cannot be used if a
  NuttX is built as a protected, supervisor kernel (CONFIG_BUILD_PROTECTED
  or CONFIG_BUILD_KERNEL).

examples/rfid_readuid
^^^^^^^^^^^^^^^^^^^^^

  RFID READUID example

examples/rgbled
^^^^^^^^^^^^^^^

  This example demonstrates the use of the RGB led driver to drive an RGB LED
  with PWM outputs so that all color characteristcs of RGB LED can be controlled.

examples/romfs
^^^^^^^^^^^^^^

  This example exercises the romfs filesystem.  Configuration options
  include:

  * CONFIG_EXAMPLES_ROMFS_RAMDEVNO
      The minor device number to use for the ROM disk.  The default is
      1 (meaning /dev/ram1)

  * CONFIG_EXAMPLES_ROMFS_SECTORSIZE
      The ROM disk sector size to use.  Default is 64.

  * CONFIG_EXAMPLES_ROMFS_MOUNTPOINT
      The location to mount the ROM disk.  Deafault: "/usr/local/share"

examples/sendmail
^^^^^^^^^^^^^^^^^

  This examples exercises the uIP SMTP logic by sending a test message
  to a selected recipient.  This test can also be built to execute on
  the Cygwin/Linux host environment:

    cd examples/sendmail
    make -f Makefile.host TOPDIR=<nuttx-directory>

 Settings unique to this example include:

    CONFIG_EXAMPLES_SENDMAIL_NOMAC     - May be defined to use software assigned MAC (optional)
    CONFIG_EXAMPLES_SENDMAIL_IPADDR    - Target IP address (required)
    CONFIG_EXAMPLES_SENDMAIL_DRIPADDR  - Default router IP addess (required)
    CONFIG_EXAMPLES_SENDMAILT_NETMASK  - Network mask (required)
    CONFIG_EXAMPLES_SENDMAIL_RECIPIENT - The recipient of the email (required)
    CONFIG_EXAMPLES_SENDMAIL_SENDER    - Optional. Default: "nuttx-testing@example.com"
    CONFIG_EXAMPLES_SENDMAIL_SUBJECT   - Optional. Default: "Testing SMTP from NuttX"
    CONFIG_EXAMPLES_SENDMAIL_BODY   -    Optional. Default: "Test message sent by NuttX"

  NOTE: This test has not been verified on the NuttX target environment.
  As of this writing, unit-tested in the Cygwin/Linux host environment.

  NOTE 2: This sendmail example only works for the simplest of
  environments.  Virus protection software on your host may have
  to be disabled to allow you to send messages.  Only very open,
  unprotected recipients can be used.  Most will protect themselves
  from this test email because it looks like SPAM.

  Applications using this example will need to enble the following
  netutils libraries in their defconfig file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_SMTP=y

examples/serialblaster
^^^^^^^^^^^^^^^^^^^^^^

  Sends a repeating pattern (the alphabet) out a serial port continuously.
  This may be useful if you are trying run down other problems that you
  think might only occur when the serial port usage is high.

examples/serialrx
^^^^^^^^^^^^^^^^^

  Constant receives serial data.  This is the complement to serialblaster.
  This may be useful if you are trying run down other problems that you
  think might only occur when the serial port usage is high.

examples/serloop
^^^^^^^^^^^^^^^^

  This is a mindlessly simple loopback test on the console.  Useful
  for testing new serial drivers.  Configuration options include:

  * CONFIG_EXAMPLES_SERLOOP_BUFIO
      Use C buffered I/O (getchar/putchar) vs. raw console I/O
      (read/read).

examples/slcd
^^^^^^^^^^^^^
  A simple test of alphanumeric, segment LCDs (SLCDs).

  * CONFIG_EXAMPLES_SLCD - Enable the SLCD test

examples/smart
^^^^^^^^^^^^^^

  This is a test of the SMART file system that derives from
  examples/nxffs.

  * CONFIG_EXAMPLES_SMART: - Enable the SMART file system example
  * CONFIG_EXAMPLES_SMART_ARCHINIT: The default is to use the RAM MTD
    device at drivers/mtd/rammtd.c.  But an architecture-specific MTD
    driver can be used instead by defining CONFIG_EXAMPLES_SMART_ARCHINIT.  In
    this case, the initialization logic will call smart_archinitialize()
    to obtain the MTD driver instance.
  * CONFIG_EXAMPLES_SMART_NEBLOCKS: When CONFIG_EXAMPLES_SMART_ARCHINIT is not
    defined, this test will use the RAM MTD device at drivers/mtd/rammtd.c
    to simulate FLASH.  In this case, this value must be provided to give
    the nubmer of erase blocks in MTD RAM device.  The size of the allocated
    RAM drive will be: CONFIG_RAMMTD_ERASESIZE * CONFIG_EXAMPLES_SMART_NEBLOCKS
  * CONFIG_EXAMPLES_SMART_MAXNAME: Determines the maximum size of names used
    in the filesystem
  * CONFIG_EXAMPLES_SMART_MAXFILE: Determines the maximum size of a file
  * CONFIG_EXAMPLES_SMART_MAXIO: Max I/O, default 347.
  * CONFIG_EXAMPLES_SMART_MAXOPEN: Max open files.
  * CONFIG_EXAMPLES_SMART_MOUNTPT: SMART mountpoint
  * CONFIG_EXAMPLES_SMART_NLOOPS: Number of test loops. default 100
  * CONFIG_EXAMPLES_SMART_VERBOSE: Verbose output

examples/smart_test
^^^^^^^^^^^^^^^^^^^

  Performs a file-based test on a SMART (or any) filesystem. Validates
  seek, append and seek-with-write operations.

    * CONFIG_EXAMPLES_SMART_TEST=y

  Dependencies:

    * CONFIG_NSH_BUILTIN_APPS=y: This test can be built only as an NSH
      command

examples/smp
^^^^^^^^^^^^

  This is a simple test for SMP functionality.  It is basically just the
  pthread barrier test with some custom instrumentation.

examples/system
^^^^^^^^^^^^^^^

  This is a simple test of the system() command.  The test simply executes this
  system command:

    ret = system("ls -Rl /");

examples/tcpecho
^^^^^^^^^^^^^^^^

  Simple single threaded, poll based TCP echo server. This example implements
  the TCP Echo Server from W. Richard Stevens UNIX Network Programming Book.
  Contributed by Max Holtberg.

  See also examples/nettest

    * CONFIG_EXAMPLES_TCPECHO =y: Enables the TCP echo server.
    * CONFIG_XAMPLES_TCPECHO_PORT: Server Port, default 80
    * CONFIG_EXAMPLES_TCPECHO_BACKLOG: Listen Backlog, default 8
    * CONFIG_EXAMPLES_TCPECHO_NCONN: Number of Connections, default 8
    * CONFIG_EXAMPLES_TCPECHO_DHCPC: DHCP Client, default n
    * CONFIG_EXAMPLES_TCPECHO_NOMAC: Use Canned MAC Address, default n
    * CONFIG_EXAMPLES_TCPECHO_IPADDR: Target IP address, default 0x0a000002
    * CONFIG_EXAMPLES_TCPECHO_DRIPADDR: Default Router IP address (Gateway), default 0x0a000001
    * CONFIG_EXAMPLES_TCPECHO_NETMASK: Network Mask, default 0xffffff00

examples/telnetd
^^^^^^^^^^^^^^^^

  This directory contains a functional port of the tiny uIP shell.  In
  the NuttX environment, the NuttShell (at apps/nshlib) supercedes this
  tiny shell and also supports telnetd.

    CONFIG_EXAMPLES_TELNETD - Enable the Telnetd example
    CONFIG_NETUTILS_NETLIB, CONFIG_NETUTILS_TELNED - Enable netutils
      libraries needed by the Telnetd example.
    CONFIG_EXAMPLES_TELNETD_DAEMONPRIO - Priority of the Telnet daemon.
      Default: SCHED_PRIORITY_DEFAULT
    CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE - Stack size allocated for the
      Telnet daemon. Default: 2048
    CONFIG_EXAMPLES_TELNETD_CLIENTPRIO- Priority of the Telnet client.
      Default: SCHED_PRIORITY_DEFAULT
    CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE - Stack size allocated for the
      Telnet client. Default: 2048
    CONFIG_EXAMPLES_TELNETD_NOMAC - If the hardware has no MAC address of its
      own, define this =y to provide a bogus address for testing.
    CONFIG_EXAMPLES_TELNETD_IPADDR - The target IP address.  Default 10.0.0.2
    CONFIG_EXAMPLES_TELNETD_DRIPADDR - The default router address. Default
      10.0.0.1
    CONFIG_EXAMPLES_TELNETD_NETMASK - The network mask.  Default: 255.255.255.0

  Also, make sure that you have the following set in the NuttX configuration
  file or else the performance will be very bad (because there will be only
  one character per TCP transfer):

    CONFIG_STDIO_BUFFER_SIZE - Some value >= 64
    CONFIG_STDIO_LINEBUFFER=y

examples/thttpd
^^^^^^^^^^^^^^^

  An example that builds netutils/thttpd with some simple NXFLAT
  CGI programs.  see configs/README.txt for most THTTPD settings.
  In addition to those, this example accepts:

    CONFIG_EXAMPLES_THTTPD_NOMAC    - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLES_THTTPD_DRIPADDR - Default router IP addess
    CONFIG_EXAMPLES_THTTPD_NETMASK  - Network mask

  Applications using this example will need to enable the following
  netutils libraries in the defconfig file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_THTTPD=y

examples/tiff
^^^^^^^^^^^^^

  This is a simple unit test for the TIFF creation library at apps/graphic/tiff.
  It is configured to work in the Linux user-mode simulation and has not been
  tested in any other environment.  Since the example also depends on some
  other logic to mount a file system, currently it will only work as an NSH
  built-on, i.e., if the following is defined:

    CONFIG_NSH_BUILTIN_APPS=y

  At a miniumum, to run in an embedded environment, you will probably have to
  change the configured paths to the TIFF files defined in the example.

    CONFIG_EXAMPLES_TIFF_OUTFILE - Name of the resulting TIFF file.  Default is
       "/tmp/result.tif"
    CONFIG_EXAMPLES_TIFF_TMPFILE1/2 - Names of two temporaries files that
      will be used in the file creation.  Defaults are "/tmp/tmpfile1.dat" and
      "/tmp/tmpfile2.dat"

  The following must also be defined in your apps/ configuration file:

    CONFIG_EXAMPLES_TIFF=y
    CONFIG_GRAPHICS_TIFF=y

examples/timer
^^^^^^^^^^^^^^

  This is a simple test of the timer driver (see include/nuttx/timers/timer.h).

  Dependencies:
    CONFIG_TIMER - The timer driver must be selected

  Example configuration:

    CONFIG_EXAMPLE_TIMER_DEVNAME - This is the name of the timer device that
      will be tested.  Default: "/dev/timer0"
    CONFIG_EXAMPLE_TIMER_INTERVAL - This is the timer interval in
      microseconds.  Default: 1000000
    CONFIG_EXAMPLE_TIMER_DELAY - This is the delay between timer samples in
      microseconds.  Default: 10000
    CONFIG_EXAMPLES_TIMER_APPNAME - This is the name of the built-in
      application:  Default:  "timer"
    CONFIG_EXAMPLES_TIMER_STACKSIZE - This is the stack size allocated when
      the timer task runs.  Default: 2048
    CONFIG_EXAMPLES_TIMER_PRIORITY - This is the priority of the timer task:
      Default: 100
    CONFIG_EXAMPLES_TIMER_PROGNAME - This is the name of the program that
      will be use when the NSH ELF program is installed.  Default: "timer"

examples/touchscreen
^^^^^^^^^^^^^^^^^^^^

  This configuration implements a simple touchscreen test at
  apps/examples/touchscreen.  This test will create an empty X11 window
  and will print the touchscreen output as it is received from the
  simulated touchscreen driver.

    CONFIG_NSH_BUILTIN_APPS - Build the touchscreen test as
      an NSH built-in function.  Default: Built as a standalone problem
    CONFIG_EXAMPLES_TOUCHSCREEN_MINOR - The minor device number.  Minor=N
      corresponds to touchscreen device /dev/inputN.  Note this value must
      with CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH.  Default 0.
    CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH - The path to the touchscreen
      device.  This must be consistent with CONFIG_EXAMPLES_TOUCHSCREEN_MINOR.
      Default: "/dev/input0"
    CONFIG_EXAMPLES_TOUCHSCREEN_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
      is defined, then the number of samples is provided on the command line
      and this value is ignored.  Otherwise, this number of samples is
      collected and the program terminates.  Default:  Samples are collected
      indefinitely.
    CONFIG_EXAMPLES_TOUCHSCREEN_MOUSE - The touchscreen test can also be
      configured to work with a mouse driver by setting this option.

  The following additional configurations must be set in the NuttX
  configuration file:

    CONFIG_INPUT=y
    (Plus any touchscreen-specific settings).

  The following must also be defined in your apps configuration file:

    CONFIG_EXAMPLES_TOUCHSREEN=y

  This example code will call boardctl() to setup the touchscreen driver
  for texting.  The implementation of boardctl() will require that board-
  specific logic  provide the following interfaces that will be called by
  the boardctl() in order to initialize and uninitialize the touchscreen hardware:

    int board_tsc_setup(int minor);
    void board_tsc_teardown(void);

examples/uavcan
^^^^^^^^^^^^^^^

  Illustrates use of canutils/uavcan.  Contributed by Paul Alexander
  Patience.

examples/udp
^^^^^^^^^^^^

  This is a simple network test for verifying client- and server-
  functionality over UDP.

  Applications using this example will need to enabled the following
  netutils libraries in the defconfig file:

    CONFIG_NETUTILS_NETLIB=y

examples/udpblaster
^^^^^^^^^^^^^^^^^^^

  This is a simple network test for stressing UDP transfers.  It simply
  sends UDP packets from both the host and the target and the highest ratei
  possible.


examples/unionfs
^^^^^^^^^^^^^^^^

  This is at trivial test of the Union File System.  See
  nuttx/fs/unionfs/README.txt.  Dependencies:

    CONFIG_DISABLE_MOUNTPOINT          - Mountpoint support must not be disabled
    CONFIG_NFILE_DESCRIPTORS < 4       - Some file descriptors must be allocated
    CONFIG_FS_ROMFS                    - ROMFS support is required
    CONFIG_FS_UNIONFS                  - Union File System support is required

  Configuration options.  Use the defaults if you are unsure of what you are doing:

    CONFIG_EXAMPLES_UNIONFS            - Enables the example
    CONFIG_EXAMPLES_UNIONFS_MOUNTPT    - Mountpoint path for the Union File System
    CONFIG_EXAMPLES_UNIONFS_TMPA       - Temporary mount point for file system 1
    CONFIG_EXAMPLES_UNIONFS_TMPB       - Temporary mount point for file system 2
    CONFIG_EXAMPLES_UNIONFS_RAMDEVNO_A - ROMFS file system 1 RAM disk device number
    CONFIG_EXAMPLES_UNIONFS_RAMDEVNO_B - ROMFS file system 2 RAM disk device number
    CONFIG_EXAMPLES_UNIONFS_SECTORSIZE - ROM disk sector size.

  See the README.txt file at nuttx/configs/sim/README.txt for a walk-through of
  the output of this text.

examples/usbserial
^^^^^^^^^^^^^^^^^^

  TARGET CONFIGURATION:

    This is another implementation of "Hello, World" but this one uses
    a USB serial driver.  Configuration options can be used to simply
    the test. These options include:

    CONFIG_EXAMPLES_USBSERIAL_INONLY
       Only verify IN (device-to-host) data transfers.  Default: both
    CONFIG_EXAMPLES_USBSERIAL_OUTONLY
       Only verify OUT (host-to-device) data transfers.  Default: both
    CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL
       Send only small, single packet messages.  Default: Send large and small.
    CONFIG_EXAMPLES_USBSERIAL_ONLYBIG
       Send only large, multi-packet messages.  Default: Send large and small.

    If CONFIG_USBDEV_TRACE is enabled (or CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_USB), then
    the example code will also manage the USB trace output.  The amount of trace output
    can be controlled using:

    CONFIG_EXAMPLES_USBSERIAL_TRACEINIT
      Show initialization events
    CONFIG_EXAMPLES_USBSERIAL_TRACECLASS
      Show class driver events
    CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS
      Show data transfer events
    CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER
      Show controller events
    CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS
      Show interrupt-related events.

    Error results are always shown in the trace output

  HOST-SIDE TEST PROGRAM

    In additional to the target device-side example, there is also a
    host-side application in this directory.  This host side application
    must be executed on a Linux host in order to perform the USBSERIAL
    test.  The host application can be compiled under Linux (or Cygwin?)
    as follows:

      cd examples/usbserial
      make -f Makefile.host TOPDIR=<nuttx-directory>

  RUNNING THE TEST

    This will generate a small program called 'host'.  Usage:

    1. Build the examples/usbserial target program and start the target.

    2. Wait a bit, then do enter:

       dmesg

       At the end of the dmesg output, you should see the serial
       device was successfully idenfied and assigned to a tty device,
       probably /dev/ttyUSB0 or /dev/ttyACM0 (depending on the configured
       USB serial driver).

    3. Then start the host application:

         ./host [<tty-dev>]

       Where:

         <tty-dev> is the USB TTY device to use.   The default is
         "/dev/ttyUSB0" (for the PL2303 emulation) or "/dev/ttyACM0" (for
         the CDC/ACM serial device).

    The host and target will exchange are variety of very small and very large
    serial messages.

examples/usbterm
^^^^^^^^^^^^^^^^

  This example implements a little USB terminal.. more of a USB "chat"
  edited lines are received from the remote host connected via USB
  serial and echoed out the target serial console.  Edited lines from
  the local target serial console are received and forwarded to the
  remote host via USB serial.

  Usage:
    - Build the example and load into the target FLASH
    - Connect on  terminal to the target RS-232 connect and configure
      for 115200 8N1.  For example, suppose this Tera Term on a Windows
      box.
    - Power up the target board
    - Connect the USB to a Linux box.  Use the Linux dmesg command to
      assure that the connect was successful.  The USB CDC ACM device
      should appear as /dev/ttyACM0
    - On the Linux box, open minicom with tty=/dev/ttyACM0.
      Configure minicom so that (1) local characters are echoed and (2)
      so that no CR is required.
    - Now what you type on the target Tera Term window should echo on
      the Linux  minicom window and, conversely, what you type on the
      minicom winow should be echo in the target Tera Term window.

  Configuration options:

    CONFIG_NSH_BUILTIN_APPS - Build the usbterm example as an NSH
      built-in command.  NOTE:  This is not fully functional as of this
      writing.. It should work, but there is no mechanism in place yet
      to exit the USB terminal program and return to NSH.
    CONFIG_EXAMPLES_USBTERM_DEVINIT - If defined, then the example will
      call a user provided function as part of its initialization:
         int usbterm_devinit(void);
      And another user provided function at termination:
         void usbterm_devuninit(void);
    CONFIG_EXAMPLES_USBTERM_BUFLEN - The size of the input and output
      buffers used for receiving data. Default 256 bytes.

  If CONFIG_USBDEV_TRACE is enabled (or CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_USB, or
  CONFIG_USBDEV_TRACE), then the example code will also manage the USB trace
  output.  The amount of trace output can be controlled using:

    CONFIG_EXAMPLES_USBTERM_TRACEINIT
      Show initialization events
    CONFIG_EXAMPLES_USBTERM_TRACECLASS
      Show class driver events
    CONFIG_EXAMPLES_USBTERM_TRACETRANSFERS
      Show data transfer events
    CONFIG_EXAMPLES_USBTERM_TRACECONTROLLER
      Show controller events
    CONFIG_EXAMPLES_USBTERM_TRACEINTERRUPTS
      Show interrupt-related events.

  NOTE:  By default, USBterm uses readline to get data from stdin.  So your
  defconfig file must have the following build path:

    CONFIG_SYSTEM_READLINE=y

  NOTE: If you use the USBterm task over a telnet NSH connection, then you
  should set the following configuration item:

    CONFIG_EXAMPLES_USBTERM_FGETS=y

  By default, the USBterm client will use readline() to get characters from
  the console.  Readline includes and command-line editor and echos
  characters received in stdin back through stdout.  Neither of these
  behaviors are desire-able if Telnet is used.

  Error results are always shown in the trace output

  Other relevant configuration options:  CONFIG_CDCACM selected by the
  Prolifics emulation (not defined) and the CDC serial implementation
  (when defined). CONFIG_USBDEV_TRACE_INITIALIDSET.

examples/ustream
^^^^^^^^^^^^^^^^

  This is the same test as examples/udp and similar to examples/ustream,
  but using Unix domain datagram sockets.

  Dependencies:
    CONFIG_NET_LOCAL - Depends on support for Unix domain sockets

  Configuration:
    CONFIG_EXAMPLES_UDGRAM - Enables the Unix domain socket example.
    CONFIG_EXAMPLES_UDGRAM_ADDR - Specifics the Unix domain address.
      Default "/dev/fifo".

examples/ustream
^^^^^^^^^^^^^^^^

  This is the same test as examples/udp and similar to examples/udgram,
  but using Unix domain stream sockets.

  Dependencies:
    CONFIG_NET_LOCAL - Depends on support for Unix domain sockets

  Configuration:
    CONFIG_EXAMPLES_USTREAM - Enables the Unix domain socket example.
    CONFIG_EXAMPLES_USTREAM_ADDR - Specifics the Unix domain address.
      Default "/dev/fifo".

examples/watchdog
^^^^^^^^^^^^^^^^^

  A simple test of a watchdog timer driver.  Initializes starts the watchdog
  timer.  It pings the watchdog timer for a period of time then lets the
  watchdog timer expire... resetting the CPU is successful.  This
  example can ONLY be built as an NSH built-in function.

  This test depends on these specific Watchdog/NSH configurations settings (your
  specific watchdog hardware settings might require additional settings).

    CONFIG_WATCHDOG- Enables watchdog timer support support.
    CONFIG_NSH_BUILTIN_APPS - Build the watchdog time test as an NSH
      built-in function. Default: Not built!  The example can only be used
      as an NSH built-in application

  Specific configuration options for this example include:

    CONFIG_EXAMPLES_WATCHDOG_DEVPATH - The path to the Watchdog device.
      Default: /dev/watchdog0
    CONFIG_EXAMPLES_WATCHDOG_PINGTIME - Time in milliseconds that the example
      will ping the watchdog before letting the watchdog expire. Default: 5000
      milliseconds
    CONFIG_EXAMPLES_WATCHDOG_PINGDELAY - Time delay between pings in
      milliseconds. Default: 500 milliseconds.
    CONFIG_EXAMPLES_WATCHDOG_TIMEOUT - The watchdog timeout value in
      milliseconds before the watchdog timer expires.  Default:  2000
      milliseconds.

examples/webserver
^^^^^^^^^^^^^^^^^^

  This is a port of uIP tiny webserver example application.  Settings
  specific to this example include:

    CONFIG_EXAMPLES_WEBSERVER_NOMAC     - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLES_WEBSERVER_IPADDR    - Target IP address
    CONFIG_EXAMPLES_WEBSERVER_DRIPADDR  - Default router IP addess
    CONFIG_EXAMPLES_WEBSERVER_NETMASK   - Network mask
    CONFIG_EXAMPLES_WEBSERVER_DHCPC     - Select to get IP address via DHCP

  If you use DHCPC, then some special configuration network options are
  required.  These include:

    CONFIG_NET=y                 - Of course
    CONFIG_NSOCKET_DESCRIPTORS   - And, of course, you must allocate some
                                   socket descriptors.
    CONFIG_NET_UDP=y             - UDP support is required for DHCP
                                   (as well as various other UDP-related
                                   configuration settings).
    CONFIG_NET_BROADCAST=y       - UDP broadcast support is needed.
    CONFIG_NET_ETH_MTU=650       - Per RFC2131 (p. 9), the DHCP client must be
    (or larger)                    prepared to receive DHCP messages of up to
                                   576 bytes (excluding Ethernet, IP, or UDP
                                   headers and FCS).
                                   NOTE: Note that the actual MTU setting will
                                   depend upon the specific link protocol.
                                   Here Ethernet is indicated.

  Other configuration items apply also to the selected webserver net utility.
  Additional relevant settings for the uIP webserver net utility are:

    CONFIG_NETUTILS_HTTPDSTACKSIZE
    CONFIG_NETUTILS_HTTPDFILESTATS
    CONFIG_NETUTILS_HTTPDNETSTATS

  Applications using this example will need to enable the following
  netutils libraries in their defconfig file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETUTILS_DHCPC=y
    CONFIG_NETDB_DNSCLIENT=y
    CONFIG_NETUTILS_WEBSERVER=y

  NOTE:  This example does depend on the perl script at
  nuttx/tools/mkfsdata.pl.  You must have perl installed on your
  development system at /usr/bin/perl.

examples/wget
^^^^^^^^^^^^^

  A simple web client example.  It will obtain a file from a server using the HTTP
  protocol.  Settings unique to this example include:

    CONFIG_EXAMPLES_WGET_URL       - The URL of the file to get
    CONFIG_EXAMPLES_WGET_NOMAC     - (May be defined to use software assigned MAC)
    CONFIG_EXAMPLES_WGET_IPADDR    - Target IP address
    CONFIG_EXAMPLES_WGET_DRIPADDR  - Default router IP addess
    CONFIG_EXAMPLES_WGET_NETMASK   - Network mask

  This example uses netutils/webclient.  Additional configuration settings apply
  to that code as follows (but built-in defaults are probably OK):

    CONFIG_WEBCLIENT_GETMIMETYPE, CONFIG_WEBCLIENT_MAXHTTPLINE,
    CONFIG_WEBCLIENT_MAXMIMESIZE, CONFIG_WEBCLIENT_MAXHOSTNAME,
    CONFIG_WEBCLIENT_MAXFILENAME

  Of course, the example also requires other settings including CONFIG_NET and
  CONFIG_NET_TCP.  The example also uses the uIP resolver which requires CONFIG_UDP.

  WARNNG: As of this writing, wget is untested on the target platform.  At present
  it has been tested only in the host-based configuration described in the following
  note.  The primary difference is that the target version will rely on the also
  untested uIP name resolver.

  NOTE: For test purposes, this example can be built as a host-based wget function.
  This can be built as follows:

    cd examples/wget
    make -f Makefile.host

  Applications using this example will need to enable the following netutils
  libraries in the defconfig file:

    CONFIG_NETUTILS_NETLIB=y
    CONFIG_NETDB_DNSCLIENT=y
    CONFIG_NETUTILS_WEBCLIENT=y

examples/wget
^^^^^^^^^^^^^

  Uses wget to get a JSON encoded file, then decodes the file.

    CONFIG_EXAMPLES_WDGETJSON_MAXSIZE - Max. JSON Buffer Size
    CONFIG_EXAMPLES_EXAMPLES_WGETJSON_URL - wget URL

examples/xmlrpc
^^^^^^^^^^^^^^^

  This example exercises the "Embeddable Lightweight XML-RPC Server" which
  is discussed at:

    http://www.drdobbs.com/web-development/an-embeddable-lightweight-xml-rpc-server/184405364

  Configuration options:

    CONFIG_EXAMPLES_XMLRPC_BUFFERSIZE - HTTP buffer size. Default 1024
    CONFIG_EXAMPLES_XMLRPC_DHCPC - Use DHCP Client.  Default n. Ignored
      if CONFIG_NSH_BUILTIN_APPS is selected.
    CONFIG_EXAMPLES_XMLRPC_NOMAC - Use Canned MAC Address. Defaul n. Ignored
      if CONFIG_NSH_BUILTIN_APPS is selected.
    CONFIG_EXAMPLES_XMLRPC_IPADDR - Target IP address. Default 0x0a000002.
      Ignored if CONFIG_NSH_BUILTIN_APPS is selected.
    CONFIG_EXAMPLES_XMLRPC_DRIPADDR - Default Router IP address (Gateway).
      Default 0x0a000001. Ignored if CONFIG_NSH_BUILTIN_APPS is selected.
    CONFIG_EXAMPLES_XMLRPC_NETMASK - Network Mask.  Default 0xffffff00
      Ignored if CONFIG_NSH_BUILTIN_APPS is selected.

examples/zerocross
^^^^^^^^^^^^^^^^^^

  A simple test of the Zero Crossing device driver.
