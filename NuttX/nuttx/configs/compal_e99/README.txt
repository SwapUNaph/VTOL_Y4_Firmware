compal_e99
==========

This directory contains the board support for compal e99 phones.

This port is based on patches contributed by Denis Carikli for both the
compal e99 and e88. At the time of initial check-in, the following phones
were tested:

* Motorola c155 (compal e99) with the compalram and highram configuration
* Motorola W220 (compal e88)
* The openmoko freerunner baseband(compal e88)

The patches were made by Alan Carvalho de Assis and Denis Carikli using
the Stefan Richter's patches that can be found here:

http://cgit.osmocom.org/cgit/nuttx-bb/log/?h=lputt%2Ftesting

Osmocom-BB Dependencies and Sercomm
===================================

The build environment assumes that you have the osmocom-bb project
directory at same level as the nuttx project:

  |- nuttx
  |- apps
  `- osmocom-bb

If you attempt to build this configuration without osmocom-bb, and that
you added support for sercomm in your configuration(CONFIG_SERCOMM_CONSOLE=y)
you will get compilation errors in drivers/sercomm due to header files that
are needed from the osmocom-bb directory.

By default, NuttX will not use sercomm (HDLC protocol) to communicate with
the host system. Sercomm is the transport used by osmocom-bb that runs on top
of serial.  See http://bb.osmocom.org/trac/wiki/nuttx-bb/run for detailed
the usage of nuttx with sercomm.

Loading NuttX
=============

The osmocom-bb wiki describes how to load NuttX.  See
http://bb.osmocom.org/trac/wiki/nuttx-bb for detailed information.
The way that nuttx is loaded depends on the configuration (highram/compalram)
and phone:

o compalram is for the ramloader(for phone having a bootloader on flash)
o highram is for phones having the romloader(if the phone has a bootrom)
  or for loading in the ram trough a special loader(loaded first on ram
  by talking to the ramloader) when having a ramloader(which can only
  load 64k).

Configurations
==============

  1. Each Compal E99 configuration is maintained in a sub-directory and
     can be selected as follow:

       cd tools
       ./configure.sh compal_e99/<subdir>
       cd -
       . ./setenv.sh

     Where <subdir> is one of the configuration sub-directories under
     nuttx/configs/compal_e99.

  2. These configurations use the mconf-based configuration tool.  To
     change a configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. By default, all configurations assume the NuttX Buildroot toolchain
     under Linux (should work under Windows with Cygwin as well).  This
     is easily reconfigured:

        CONFIG_HOST_LINUX=y
        CONFIG_ARM_TOOLCHAIN_BUILDROOT=y
