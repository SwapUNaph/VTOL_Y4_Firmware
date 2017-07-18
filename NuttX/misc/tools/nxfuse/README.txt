nxfuse
======

A Linux FUSE implmentation that enables native mounting of NuttX filesystems
in Linux.  Currently the implementation supports SmartFS and NXFFS.  The
NXFFS filesystem has some architectural limitations such as no support for
directories, no O_APPEND support of an existing file, etc.  Some of the
Linux utilities depend on this capability (overwriting an existing file
with new content for instance) and may fail.  

To build nxfuse, you must first install the libfuse-dev library as
root:

  sudo apt-get install libfuse-dev
  sudo yum install libfuse-dev
  etc.

Then just type 'make' within the smartfuse directory.  This project does
not contain an autoconf configuration script.  It was developed under 
Ubuntu and so it may need slight modifications for other distributions.

To compile correctly, all of the the NuttX FS and support code that has 
been copied from the NuttX source directory depends on a valid config.h file 
to define the system configuration.  The make system automatically copies
this config file from the NuttX directory $(NUTTXDIR) of a pre-configured
NuttX build into the nxfuse/include/nuttx directory.  If the NUTTXDIR variable
is not define, the Makefile will use the default value of "../../nuttx", 
which makes the assumption that the NuttX tools repo is located at the same
directory level as the main "nuttx" source repo.

To enable / disable SmartFS/NXFFS features (wear leveling, CRC, etc.) in the 
nxfuse implementation, run the 'make menuconfig' routine from within the nuttx 
directory and select the features required.  Then run 'make context' within 
the nuttx directory.  The newly generated nuttx/include/nuttx/config.h file 
will be copied to the tools/nxfuse/include/nuttx directory when the nxfuse is 
next built.


Using nxfuse
============

To use nxfuse, invoke it directly from within the nxfuse directory or run the
'make install' command as root (or sudo) to install it in /usr/bin.  The 
invocation format for nxfuse is:

   nxfuse [options] [-t fstype] [fuse_options] mount_dir data_source

The mount_dir must already exist in the Linux filesystem as a normal directory.  
For details about the options, see the man page (automatically installed by the
'make install' target).  To view the man page without installing it, simply
use the command 'man -l man/nxfuse.1'.  The default options for nxfuse are:

   fstype:       smartfs
   erasesize:    Set by CONFIG_FILEMTD_ERASESIZE in the config file
   pagesize:     Set by CONFIG_FILEMTD_BLOCKSIZE in the config file
   logsectsize:  Set by CONFIG_MTD_SMART_SECTOR_SIZE in config file

An example invocation might be:

   ./nxfuse -t smartfs /tmp/fuse /tmp/smartfs_data_file.bin
   ./nxfuse -t nxffs -e 32768 /tmp/nxffs /tmp/nxffs.bin
   etc.

For a list of fuse_options that are supported, add the -h option to the command
line as follows:

   ./nxfuse -h /tmp/fuse /tmp/smartfs_data_file.bin

When using nxfuse with a block device, such as /dev/sdb, etc. the nxfuse
command will likely need to be executed with 'sudo' access.  Doing this will 
cause the resuling mount (e.g. /tmp/fuse) to be inaccesable to anyone but
root.  To get around this, the fuse_option "-o allow_other" can be used on the
nxfuse command line.  This will cause the mounted directory to be setup 
with "rwxrwxrwx" permissions, allowing any user to access the mount.  As an
example:

   mkdir /tmp/fuse
   sudo losetup /dev/loop0 /tmp/smartfs_data_file.bin
   sudo ./nxfuse -o allow_other -t smartfs /tmp/fuse /dev/loop0
   ls -l /tmp/fuse

To unmount a FUSE device, use the fusermount command as follows:

   fusermount -u /tmp/fuse


Formatting / reformatting
=========================

The nxfuse command also has the ability to create (or re-create) a new 
fiilesystem within the data file (a built-in 'mkfs').  This is done by
specifying the -m option on the command line (-m for mkfs).  When invoked
with the -m option, the mount_point argument should be ommited, such as:

   ./nxfuse -t smartfs -m /tmp/emptyfile

If the target file already contains a valid filesystem, then nxfuse will
report an error and request the addition of the -c option for confirmation
of the re-format.  After the format is complete, the newly created filesystem
is not mounted ... mounting must be performed as detailed above as a 
secondary step.

The NXFFS implementation has the ability to auto-detect and auto-format
an empty volume, therefore the 'mkfs' feature is not implemented for this FS.


