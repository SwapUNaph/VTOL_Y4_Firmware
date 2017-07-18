tools/README.txt
================

This README file addresses the contents of the NuttX tools/ directory.

The tools/ directory contains miscellaneous scripts and host C programs
that are necessary parts of the NuttX build system.  These files
include:

README.txt
----------

  This file!

Config.mk
---------

  This file contains common definitions used by many configuration files.
  This file (along with <nuttx>/.config) must be included at the top of
  each configuration-specific Make.defs file like:

    -include $(TOPDIR)/.config
    include $(TOPDIR)/tools/Config.mk

  Subsequent logic within the configuration-specific Make.defs file may then
  override these default definitions as necessary.

configure.sh
configure.bat
configure.c, cfgparser.c, and cfgparser.h
------------

  configure.sh is a bash script that is used to configure NuttX for a given
  target board in a environment that supports POSIX paths (Linux, Cygwin,
  OSX, or similar).  See configs/README.txt or Documentation/NuttxPortingGuide.html
  for a description of how to configure NuttX with this script.

  configure.c, cfgparser.c, and cfgparser.h can be used to build a work-alike
  program as a replacement for configure.sh.  This work-alike program would be
  used in environments that do not support Bash scripting (such as the Windows
  native environment).

  configure.bat is a small Windows batch file that can be used as a replacement
  for configure.sh in a Windows native environment.  configure.bat is actually
  just a thin layer that executes configure.exe if it is available. If
  configure.exe is not available, then configure.bat will attempt to build it
  first.

  In order two build configure.exe from configure.c in the Windows native
  environment, two assumptions are made:

  1) You have installed the MinGW GCC toolchain.  This toolchain can be
     downloaded from http://www.mingw.org/.  Tt is recommended the you not
     install the optional MSYS components as there may be conflicts.
  2) That path to bin bin/ directory containing mingw-gcc.exe must be
     included in the PATH variable.

discover.py
-----------

  Example script for discovering devices in the local network.
  It is the counter part to apps/netutils/discover

mkconfig.c, cfgdefine.c, and cfgdefine.h
----------------------------------------

  These are Cs file that are used to build mkconfig program.  The mkconfig
  program is used during the initial NuttX build.

  When you configure NuttX, you will copy a configuration file called .config
  in the top level NuttX directory (See configs/README.txt or
  Documentation/NuttxPortingGuide.html).  The first time you make NuttX,
  the top-level makefile will build the mkconfig executable from mkconfig.c
  (using Makefile.host).  The top-level Makefile will then execute the
  mkconfig program to convert the .config file in the top level directory
  into include/nuttx/config.h.  config.h is a another version of the
  NuttX configuration that can be included by C files.

cmdconfig.c
-----------

  This C file can be used to build a utility for comparing two NuttX
  configuration files.

kconfig2html.c
--------------

  This is a C file that can be used build a utility for converting the
  NuttX configuration in the Kconfig files to an HTML document.  This
  auto-generated documentation will, eventually, replace the manually
  updated configuration documentation that is fallling woefully behind.

  $ tools/kconfig2html.exe -h
  USAGE: tools/kconfig2html [-d] [-a <apps directory>] {-o <out file>] [<Kconfig root>]
         tools/kconfig2html [-h]

  Where:

    -a : Select relative path to the apps/ directory. Theis path is relative
         to the <Kconfig directory>.  Default: ../apps
    -o : Send output to <out file>.  Default: Output goes to stdout
    -d : Enable debug output
    -h : Prints this message and exits
    <Kconfig root> is the directory containing the root Kconfig file.
         Default <Kconfig directory>: .

  NOTE: In order to use this tool, some configuration must be in-place will
  all necessary symbolic links.  You can establish the configured symbolic
  links with:

    make context

  or more quickly with:

    make dirlinks

mkconfigvars.sh
---------------

  The HTML documentation expects to have a copy of the auto-generated
  configuration variable documentation Documentation/NuttXConfigVariables.html.
  The script mkconfigvars.sh is a simple script that can be used to
  re-generated that file as needed.

  $ tools/mkconfigvars.sh -h
  tools/mkconfigvars.sh is a tool for generation of configuration variable documentation

  USAGE: tools/mkconfigvars.sh [-d|h] [-v <major.minor>]

  Where:
    -v <major.minor>
       The NuttX version number expressed as a major and minor number separated
       by a period
    -d
       Enable script debug
    -h
       show this help message and exit

mkexport.sh and Makefile.export
-------------------------------

  These implement part of the top-level Makefile's 'export' target.  That
  target will bundle up all of the NuttX libraries, header files, and the
  startup object into an export-able, binary NuttX distribution.  The
  Makefile.export is used only by the mkexport.sh script to parse out
  options from the top-level Make.defs file.

  USAGE: tools/mkexport.sh [-d] [-z] [-u] [-w|wy|wn] -t <top-dir> [-x <lib-ext>] -l "lib1 [lib2 [lib3 ...]]"

  Thais script also depends on the environment variable MAKE which is set
  in the top-level Makefile before starting mkexport.sh.  If MAKE is not
  defined, the script will set it to `which make`.

mkfsdata.pl
-----------

  This perl script is used to build the "fake" file system and CGI support
  as needed for the apps/netutils/webserver.  It is currently used only
  by the Makefile at apps/examples/uip.  That example serves as an example
  of how to configure the uIP webserver "fake" file system.

  NOTE:  This perl script comes from uIP and was (probably) written
  by Adam Dunkels.  uIP has a license that is compatible with NuttX.

mkversion.c, cfgdefine.c, and cfgdefine.h
-----------------------------------------

  This is C file that is used to build mkversion program.  The mkversion
  program is used during the initial NuttX build.

  When you build NuttX there should be a version file called .version in
  the top level NuttX directory (See Documentation/NuttxPortingGuide.html).
  The first time you make NuttX, the top-level makefile will build th
  mkversion executable from mkversion.c (using Makefile.host).  The top-
  level Makefile will then execute the mkversion program to convert the
  .version file in the top level directory into include/nuttx/version.h.
  version.h provides version information that can be included by C files.

mksyscall.c, cvsparser.c, and cvsparser.h
-----------------------------------------

  This is a C file that is used to build mksyscall program.  The mksyscall
  program is used during the initial NuttX build by the logic in the top-
  level syscall/ directory.

  If you build NuttX as a separately compiled, monolithic kernel and separate
  applications, then there is a syscall layer that is used to get from the
  user application space to the NuttX kernel space.  In the user application
  "proxies" for each of the kernel functions are provided.  The proxies have
  the same function signature as the kernel function, but only execute a
  system call.

  Within the kernel, there are "stubs" for each of the system calls.  The
  stubs receive the marshalled system call data, and perform the actually
  kernel function call (in kernel-mode) on behalf of the proxy function.

  Information about the stubs and proxies is maintained in a comma separated
  value (CSV) file in the syscall/ directory.  The mksyscall program will
  accept this CVS file as input and generate all of the required proxy or
  stub files as output.  See syscall/README.txt for additonal information.

mksymtab.c, cvsparser.c, and cvsparser.h
----------------------------------------

  This is a C file that is used to build symbol tables from common-separated
  value (CSV) files.  This tool is not used during the NuttX build, but
  can be used as needed to generate files.

  USAGE: ./mksymtab <cvs-file> <symtab-file>

  Where:

    <cvs-file>   : The path to the input CSV file
    <symtab-file>: The path to the output symbol table file
    -d           : Enable debug output

  Example:

    cd nuttx/tools
    cat ../syscall/syscall.csv ../lib/libc.csv | sort >tmp.csv
    ./mksymtab.exe tmp.csv tmp.c

mkctags.sh
----------

  A script for creating ctags from Ken Pettit.  See http://en.wikipedia.org/wiki/Ctags
  and http://ctags.sourceforge.net/

nxstyle.c
---------

  I am embarassed that this is here.  This program is a complete hack
  but, unfortunately, it has become so useful to me that I need to keep
  it here.

  A little background:  I have tinkered with pretty printers for some
  time and have not been happy with the results.  An alternative that
  occurred to me would be just a standard checker that examines a C
  file that gives warnings for violations of the coding standard.

  This turns out to be more difficult that you might think. A pretty
  printer understands C syntax:  They break the file up into its C
  components then reassembles the output in the format. But parsing the
  C loses the original file layout and so it not useful in this case.

  This program instead, uses a collection of heuristics (i.e., hacks and
  bandaids) to examine the C file for obvious violations of the coding
  standard.  This program is completely ignorant of C syntax; it simply
  performs crude pattern matching to check the file.

  Usage: nxstyle <path-to-file-to-check>

pic32mx
-------

  This directory contains build tools used only for PIC32MX platforms

bdf-convert.c
-------------

  This C file is used to build the bdf-converter program.  The bdf-converter
  program be used to convert fonts in Bitmap Distribution Format (BDF)
  into fonts that can be used in the NX graphics system.

  Below are general instructions for creating and installing a new font
  in the NX graphic system:

    1. Locate a font in BDF format,
    2. Use the bdf-converter program to convert the BDF font to the NuttX
       font format.  This will result in a C header file containing
       definitions.  That header file should be installed at, for example,
       graphics/nxfonts/nxfonts_myfont.h.

  Create a new NuttX configuration variable.  For example, suppose
  you define the following variable:  CONFIG_NXFONT_MYFONT.  Then
  you would need to:

    3. Define CONFIG_NXFONT_MYFONT=y in your NuttX configuration file.

  A font ID number has to be assigned for each new font.  The font ID
  is defined in the file include/nuttx/nx/nxfonts.h.  Those definitions
  have to be extended to support your new font.  Look at how the font ID
  enabled by CONFIG_NXFONT_SANS23X27 is defined and add an ID for your
  new font in a similar fashion:

    4. include/nuttx/nx/nxfonts.h. Add you new font as a possible system
       default font:

       #if defined(CONFIG_NXFONT_SANS23X27)
       # define NXFONT_DEFAULT FONTID_SANS23X27
       #elif defined(CONFIG_NXFONT_MYFONT)
       # define NXFONT_DEFAULT FONTID_MYFONT
       #endif

       Then define the actual font ID.  Make sure that the font ID value
       is unique:

       enum nx_fontid_e
       {
         FONTID_DEFAULT     = 0      /* The default font */
       #ifdef CONFIG_NXFONT_SANS23X27
         , FONTID_SANS23X27 = 1      /* The 23x27 sans serif font */
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
         , FONTID_MYFONT    = 2      /* My shiny, new font */
       #endif
       ...

  New Add the font to the NX build system.  There are several files that
  you have to modify to to this.  Look how the build system uses the
  font CONFIG_NXFONT_SANS23X27 for examples:

    5. nuttx/graphics/Makefile.  This file needs logic to auto-generate
       a C source file from the header file that you generated with the
       the bdf-converter program.  Notice NXFONTS_FONTID=2; this must be
       set to the same font ID value that you defined in the
       include/nuttx/nx/nxfonts.h file.

       genfontsources:
         ifeq ($(CONFIG_NXFONT_SANS23X27),y)
          @$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=1 EXTRADEFINES=$(EXTRADEFINES)
        endif
         ifeq ($(CONFIG_NXFONT_MYFONT),y)
          @$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=2 EXTRADEFINES=$(EXTRADEFINES)
        endif

    6. nuttx/graphics/nxfonts/Make.defs.  Set the make variable NXFSET_CSRCS.
       NXFSET_CSRCS determines the name of the font C file to build when
       NXFONTS_FONTID=2:

       ifeq ($(CONFIG_NXFONT_SANS23X27),y)
       NXFSET_CSRCS    += nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(CONFIG_NXFONT_MYFONT),y)
       NXFSET_CSRCS    += nxfonts_bitmaps_myfont.c
       endif

    7. nuttx/graphics/nxfonts/Makefile.sources.  This is the Makefile used
       in step 5 that will actually generate the font C file.  So, given
       your NXFONTS_FONTID=2, it needs to determine a prefix to use for
       auto-generated variable and function names and (again) the name of
       the auto-generated file to create (this must be the same name that
       was used in nuttx/graphics/nxfonts/Make.defs):

       ifeq ($(NXFONTS_FONTID),1)
       NXFONTS_PREFIX    := g_sans23x27_
       GEN_CSRC    = nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(NXFONTS_FONTID),2)
       NXFONTS_PREFIX    := g_myfont_
       GEN_CSRC    = nxfonts_bitmaps_myfont.c
       endif

    8. graphics/nxfonts/nxfonts_bitmaps.c.  This is the file that contains
       the generic font structures.  It is used as a "template" file by
       nuttx/graphics/nxfonts/Makefile.sources to create your customized
       font data set.

       #if NXFONTS_FONTID == 1
       #  include "nxfonts_sans23x27.h"
       #elif NXFONTS_FONTID == 2
       #  include "nxfonts_myfont.h"
       #else
       #  error "No font ID specified"
       #endif

       Where nxfonts_myfont.h is the NuttX font file that we generated in
       step 2 using the bdf-converter tool.

    9. graphics/nxfonts/nxfonts_getfont.c.  Finally, we need to extend the
       logic that does the run-time font lookups so that can find our new
       font.  The lookup function is NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid).
       The new font information needs to be added to data structures used by
       that function:

       #ifdef CONFIG_NXFONT_SANS23X27
       extern const struct nx_fontpackage_s g_sans23x27_package;
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       extern const struct nx_fontpackage_s g_myfont_package;
       #endif

       static FAR const struct nx_fontpackage_s *g_fontpackages[] =
       {
       #ifdef CONFIG_NXFONT_SANS23X27
       &g_sans23x27_package,
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       &g_myfont_package,
       #endif
       NULL
       };

Makefile.host
-------------

  This is the makefile that is used to make the mkconfig program from
  the mkconfig.c C file, the cmpconfig program from cmpconfig.c C file
  the mkversion program from the mkconfig.c C file, or the mksyscall
  program from the mksyscall.c file.  Usage:

  cd tools/
  make -f Makefile.host <program>

mkromfsimg.sh
-------------

  This script may be used to automate the generate of a ROMFS file system
  image.  It accepts an rcS script "template" and generates and image that
  may be mounted under /etc in the NuttX pseudo file system.

  TIP: Edit the resulting header file and mark the generated data values
  as 'const' so that they will be stored in FLASH.

mkdeps.c
cnvwindeps.c
mkwindeps.sh
mknulldeps.sh
-------------

  NuttX uses the GCC compilers capabilities to create Makefile dependencies.
  The program  mkdeps is used to run GCC in order to create the dependencies.
  If a NuttX configuration uses the GCC toolchain, its Make.defs file (see
  configs/README.txt) will include a line like:

    MKDEP = $(TOPDIR)/tools/mkdeps[.exe] (See NOTE below)

  If the NuttX configuration does not use a GCC compatible toolchain, then
  it cannot use the dependencies and instead it uses mknulldeps.sh:

    MKDEP = $(TOPDIR)/tools/mknulldeps.sh

  The mknulldeps.sh is a stub script that does essentially nothing.

  mkwindeps.sh is a version that creates dependencies using the Windows
  native toolchain.  That generates Windows native paths in the dependency
  file.  But the mkwindeps.sh uses cnvwindeps.c to convert the Windows
  paths to POSIX paths.  This adds some time to the Windows dependency
  generation but is generally th best option available for that mixed
  environment of Cygwin with a native Windows GCC toolchain.

  mkdeps.c generates mkdeps (on Linux) or mkdeps.exe (on Windows).
  However, this verison is still under-development.  It works well in
  the all POSIX environment or in the all Windows environment but also
  does not work well in mixed POSIX environment with a Windows toolchain.
  In that case, there are still issues with the conversion of things like
  'c:\Program Files' to 'c:program files' by bash.  Those issues may,
  eventually be solvable but for now continue to use mkwindeps.sh in
  that mixed environment.

define.sh
define.bat
---------

  Different compilers have different conventions for specifying pre-
  processor definitions on the compiler command line.  This bash
  script allows the build system to create create command line definitions
  without concern for the particular compiler in use.

  The define.bat script is a counterpart for use in the native Windows
  build.

ide_exporter.py
---------------

  This Python script will help to create nuttx project in the IAR and
  uVision IDEs.  These are few simple the steps to export the IDE
  workspaces.  See for example configs/stm3220g-eval/README.txt.

incdir.sh
incdir.bat
---------

  Different compilers have different conventions for specifying lists
  of include file paths on the compiler command line.  This incdir.sh
  bash script allows the build system to create include file paths without
  concern for the particular compiler in use.

  The incdir.bat script is a counterpart for use in the native Windows
  build.  However, there is currently only one compiler supported in
  that context:  MinGW-GCC.

link.sh
link.bat
copydir.sh
copydir.bat
unlink.sh
unlink.bat
----------

  Different file system have different capabilities for symbolic links.
  Some windows file systems have no native support for symbolic links.
  Cygwin running under windows has special links built in that work with
  all cygwin tools.  However, they do not work when Windows native tools
  are used with cygwin.  In that case something different must be done.

  If you are building under Linux or under cygwin with a cygwin tool
  chain, then your Make.defs file may have definitions like the
  following:

    DIRLINK = $(TOPDIR)/tools/link.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  The first definition is not always present because link.sh is the
  default.  link.sh is a bash script that performs a normal, Linux-style
  symbolic link;  unlink.sh is a do-it-all unlinking script.

  But if you are building under cygwin using a Windows native toolchain
  within a POSIX framework (such as Cygwin), then you will need something
  like the following in you Make.defs file:

    DIRLINK = $(TOPDIR)/tools/copydir.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  copydir.sh will copy the whole directory instead of linking it.

  Finally, if you are running in a pure native Windows environment with
  a CMD.exe shell, then you will need something like this:

    DIRLINK = $(TOPDIR)/tools/copydir.bat
    DIRUNLINK = (TOPDIR)/tools/unlink.bat

  Note that this will copy directories.  ;ink.bat might also be used in
  this case.  link.bat will attempt to create a symbolic link using the
  NTFS mklink.exe command instead of copying files.  That logic, however,
  has not been verified as of this writing.

kconfig.bat
-----------

  Recent versions of NuttX support building NuttX from a native Windows
  CMD.exe shell.  But kconfig-frontends is a Linux tool and is not yet
  available in the pure CMD.exe environment.  At this point, there are
  only a few options for the Windows user (see the top-level README.txt
  file).

  You can, with some effort, run the Cygwin kconfig-mconf tool directly
  in the CMD.exe shell.  In this case, you do not have to modify the
  .config file, but there are other complexities:  You need to
  temporarily set the Cgywin directories in the PATH variable and
  then run kconfig-mconf outside of the Make system.

  kconfig.bat is a Windows batch file at tools/kconfig.bat that automates
  these steps.  It is used from the top-level NuttX directory like:

    tools/kconfig menuconfig

  NOTE: There is an currently an issue with accessing DOS environment
  variables from the Cygwin kconfig-mconf running in the CMD.exe shell.
  The following change to the top-level Kconfig file seems to work around
  these problems:

     config APPSDIR
          string
     -   option env="APPSDIR"
     +   default "../apps"

mkimage.sh
----------

  The creates a downloadable image as needed with the rrload bootloader.

indent.sh
---------

  This script can be used to indent .c and .h files in a manner similar
  to my coding NuttX coding style.  It doesn't do a really good job,
  however (see the comments at the top of the indent.sh file).

  USAGE:
    ./indent.sh [-d] -o <out-file> <in-file>
    ./indent.sh [-d] <in-file-list>
    ./indent.sh [-d] -h

  Where:
    -<in-file>
      A single, unformatted input file
    -<in-file-list>
      A list of unformatted input files that will be reformatted in place.
    -o <out-file>
      Write the single, reformatted <in-file> to <out-file>.  <in-file>
      will not be modified.
    -d
      Enable script debug
    -h
      Show this help message and exit

sethost.sh
----------

  Saved configurations may run on Linux, Cygwin (32- or 64-bit), or other
  platforms.  The platform characteristics can be changed use 'make
  menuconfig'.  Sometimes this can be confusing due to the differences
  between the platforms.  Enter sethost.sh

  sethost.sh is a simple script that changes a configuration to your
  host platform.  This can greatly simplify life if you use many different
  configurations.  For example, if you are running on Linux and you
  configure like this:

    $ cd tools
    $ ./configure.sh board/configuration
    $ cd ..

  The you can use the following command to both (1) make sure that the
  configuration is up to date, AND (2) the configuration is set up
  correctly for Linux:

    $ tools/sethost.sh -l

  Or, if you are on a Windows/Cygwin 64-bit platform:

    $ tools/sethost.sh -w

  Other options are available:

    $ tools/sethost.sh -h

    USAGE: tools/sethost.sh [-w|l] [-c|n] [-32|64] [<config>]
           tools/sethost.sh -h

    Where:
      -w|l selects Windows (w) or Linux (l).  Default: Linux
      -c|n selects Windows native (n) or Cygwin (c).  Default Cygwin
      -32|64 selects 32- or 64-bit host (Only for Cygwin).  Default 64
      -h will show this help test and terminate

refresh.sh
----------

  This is a bash script that automatics refreshing of board default
  configuration (defconfig) files.  It does not do anything special
  that you cannot do manually, but is useful for me when I have to
  update dozens of confuration files.

  Configuration files have to be updated because over time, the
  configuration settings change:  New configurations are added and
  new dependencies are added.  So an old configuration file may
  not be usable anymore until it is refreshed.

  Help is also available:

    $ tools/refresh.sh --help
    tools/refresh.sh is a tool for refreshing board configurations

    USAGE: tools/refresh.sh [--debug|--help] <board>/<config>

    Where:
      --debug
         Enable script debug
      --silent
         Update board configuration without interaction
      --help
         Show this help message and exit
      <board>
         The board directory under nuttx/configs
      <config>
         The board configuration directory under nuttx/configs/<board>

  The steps to refresh the file taken by refresh.sh are:

  1. Make tools/cmpconfig if it is not already built.
  2. Copy the the defconfig file to the top-level NuttX
     directory as .config (being careful to save any previous
     .config file that you might want to keep!).
  3. Execute 'make oldconfig' to update the configuration.
     'make oldconfig' will prompt you for each change in the
     configuration that requires that you make some decision.
     With the --silent option, the script will use 'make
     oldefconfig' instead and you won't have to answer any
     questions;  the refresh will simply accept the default
     value for any new configuration settings.
  4. Then it runs tools/cmpconfig to show the real differences
     between the configuration files.  Configuration files are
     complex and things can move around so a simple 'diff' between
     two configuration files is often not useful.  But tools/cmpconfig
     will show only the meaningful differences between the two
     configuration files.
  4. It will edit the .config file to comment out the setting
     of the CONFIG_APPS_DIR= setting.  This setting should not
     be in checked-in defconfig files because the actually must
     be determined at the next time that the configuration is
     installed.
  5. Finally, the refreshed defconfig file is copied back in
     place where it can be committed with the next set of
     difference to the command line.  If you select the --silent
     option, this file copy will occur autiomatically.  Otherwise,
     refresh.sh will prompt you first to avoid overwriting the
     defconfig file with changes that you may not want.

showsize.sh
-----------

  Show the top 10 biggest memory hogs in code and data spaces.  This
  must be executed from the top-level NuttX directory like:

    $ tools/showsize.sh
    TOP 10 BIG DATA
    ...
    TOP 10 BIG CODE
    ...

testbuild.sh
------------

  This script automates building of a set of configurations.  The intent is
  simply to assure that the set of configurations build correctly.  The -h
  option shows the usage:

    $ ./testbuild.sh -h

    USAGE: ./testbuild.sh [-w|l] [-c|n] [-s] <testlist-file>
    USAGE: ./testbuild.sh -h

    where
      -w|l selects Windows (w) or Linux (l).  Default: Linux
      -c|n selects Windows native (n) or Cygwin (c).  Default Cygwin
      -s Use C++ long size_t in new operator. Default unsigned long
      -h will show this help test and terminate
      <testlist-file> selects the list of configurations to test.  No default

    Your PATH variable must include the path to both the build tools and the
    kconfig-frontends tools

  These script needs two pieces of information.

    a. A description of the platform that you are testing on.  This
       description is provided by the optional -w, -l, -c, and -n options.
    b. A list of configurations to build.  That list is provided by a test
       list file.  The final, non-optional parameter, <testlist-file>,
       provides the path to that file.

  The test list file is a sequence of build descriptons, one per line.  One
  build descriptions consists of two comma separated values.  For example:

    stm32f429i-disco/nsh,CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL

  The first value is the usual configuration description of the form
  form <board-name>/<configuration-name> and must correspond to a
  configuration in the nuttx/configs directory.

  The second value is valid name for a toolchain configuration to use
  when building the configuration.  The set of valid toolchain
  configuration names depends on the underlying architecture of the
  configured board.

  NOTE: The environment variable APPSDIR should be set to the relative
  path to the application directory when running this script like:

    $ export APPSDIR=../apps

zipme.sh
--------

  I use this script to create the nuttx-xx.yy.tar.gz tarballs for
  release on Bitbucket.org.  It is handy because it also does the
  kind of clean that you need to do to make a clean code release.
