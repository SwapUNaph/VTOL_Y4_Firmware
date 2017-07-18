/****************************************************************************
 * apps/nshlib/nsh_mntcmds.c
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <sys/statfs.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <nuttx/fs/nfs.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_fstype
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_MOUNT)
#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static const char* get_fstype(FAR struct statfs *statbuf)
{
  FAR const char *fstype;

  /* Get the file system type */

  switch (statbuf->f_type)
    {
#ifdef CONFIG_FS_FAT
      case MSDOS_SUPER_MAGIC:
        fstype = "vfat";
        break;
#endif

#ifdef CONFIG_FS_ROMFS
      case ROMFS_MAGIC:
        fstype = "romfs";
        break;
#endif

#ifdef CONFIG_FS_TMPFS
      case TMPFS_MAGIC:
        fstype = "tmpfs";
        break;
#endif

#ifdef CONFIG_FS_BINFS
      case BINFS_MAGIC:
        fstype = "binfs";
        break;
#endif

#ifdef CONFIG_FS_NXFFS
      case NXFFS_MAGIC:
        fstype = "nxffs";
        break;
#endif

#ifdef CONFIG_NFS
      case NFS_SUPER_MAGIC:
        fstype = "nfs";
        break;
#endif

#ifdef CONFIG_FS_SMARTFS
      case SMARTFS_MAGIC:
        fstype = "smartfs";
        break;
#endif

#ifdef CONFIG_FS_PROCFS
      case PROCFS_MAGIC:
        fstype = "procfs";
        break;
#endif

#ifdef CONFIG_FS_UNIONFS
      case UNIONFS_MAGIC:
        fstype = "unionfs";
        break;
#endif

#ifdef CONFIG_FS_HOSTFS
      case HOSTFS_MAGIC:
        fstype = "hostfs";
        break;
#endif

      default:
        fstype = "Unrecognized";
        break;
    }

  return fstype;
}
#endif
#endif

/****************************************************************************
 * Name: df_handler
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_DF)
static int df_handler(FAR const char *mountpoint,
                      FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct nsh_vtbl_s *vtbl = (FAR struct nsh_vtbl_s *)arg;

  DEBUGASSERT(mountpoint && statbuf && vtbl);

  nsh_output(vtbl, "%6ld %8ld %8ld  %8ld %s\n",
             statbuf->f_bsize, statbuf->f_blocks,
             statbuf->f_blocks - statbuf->f_bavail, statbuf->f_bavail,
             mountpoint);

  return OK;
}

/****************************************************************************
 * Name: df_man_readable_handler
 ****************************************************************************/

#ifdef CONFIG_NSH_CMDOPT_DF_H
static int df_man_readable_handler(FAR const char *mountpoint,
                      FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct nsh_vtbl_s *vtbl = (FAR struct nsh_vtbl_s *)arg;
  uint32_t      size;
  uint32_t      used;
  uint32_t      free;
  int           which;
  char          sizelabel;
  char          freelabel;
  char          usedlabel;
  const char    labels[5] = { 'B', 'K', 'M', 'G', 'T' };

  DEBUGASSERT(mountpoint && statbuf && vtbl);

  size = statbuf->f_bsize * statbuf->f_blocks;
  free = statbuf->f_bsize * statbuf->f_bavail;
  used = size - free;

  /* Find the label for size */

  which = 0;
  while (size >= 9999 || ((size & 0x3ff) == 0 && size != 0))
    {
      which++;
      size >>= 10;
    }

  sizelabel = labels[which];

  /* Find the label for free */

  which = 0;
  while (free >= 9999 || ((free & 0x3ff) == 0 && free != 0))
    {
      which++;
      free >>= 10;
    }

  freelabel = labels[which];

  /* Find the label for used */

  which = 0;
  while (used >= 9999 || ((used & 0x3ff) == 0 && used != 0))
    {
      which++;
      used >>= 10;
    }

  usedlabel = labels[which];

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
  nsh_output(vtbl, "%-10s %6ld%c %8ld%c  %8ld%c %s\n", get_fstype(statbuf),
             size, sizelabel, used, usedlabel, free, freelabel,
             mountpoint);
#else
  nsh_output(vtbl, "%6ld%c %8ld%c  %8ld%c %s\n", size, sizelabel, used,
             usedlabel, free, freelabel, mountpoint);
#endif

  return OK;
}
#endif /* CONFIG_NSH_CMDOPT_DF_H */

#endif /* CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) &&
            defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_DF) */

/****************************************************************************
 * Name: mount_handler
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_MOUNT)
#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static int mount_handler(FAR const char *mountpoint,
                         FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct nsh_vtbl_s *vtbl = (FAR struct nsh_vtbl_s *)arg;
  FAR const char *fstype;

  DEBUGASSERT(mountpoint && statbuf && vtbl);

  /* Get the file system type */

  fstype = get_fstype(statbuf);

  nsh_output(vtbl, "  %s type %s\n", mountpoint, fstype);
  return OK;
}
#endif
#endif

/****************************************************************************
 * Name: mount_show
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_MOUNT)
#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
static inline int mount_show(FAR struct nsh_vtbl_s *vtbl, FAR const char *progname)
{
  return foreach_mountpoint(mount_handler, (FAR void *)vtbl);
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_df
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_DF)
int cmd_df(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
#ifdef CONFIG_NSH_CMDOPT_DF_H
  if (argc > 1 && strcmp(argv[1], "-h") == 0)
    {
#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
      nsh_output(vtbl, "Filesystem    Size      Used  Available Mounted on\n");
#else
      nsh_output(vtbl, "Size      Used  Available Mounted on\n");
#endif
      return foreach_mountpoint(df_man_readable_handler, (FAR void *)vtbl);
    }
  else
#endif
    {
      nsh_output(vtbl, "  Block  Number\n");
      nsh_output(vtbl, "  Size   Blocks     Used Available Mounted on\n");
      return foreach_mountpoint(df_handler, (FAR void *)vtbl);
    }
}
#endif

/****************************************************************************
 * Name: cmd_mount
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_MOUNT)
int cmd_mount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR const char *source;
  FAR char *fullsource;
  FAR const char *target;
  FAR char *fulltarget;
  FAR const char *filesystem = NULL;
  FAR const char *options = NULL;
  bool badarg = false;
  int option;
  int ret;

  /* The mount command behaves differently if no parameters are provided */

#if !defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)
  if (argc < 2)
    {
      return mount_show(vtbl, argv[0]);
    }
#endif

  /* Get the mount options.  NOTE: getopt() is not thread safe nor re-entrant.
   * To keep its state proper for the next usage, it is necessary to parse to
   * the end of the line even if an error occurs.  If an error occurs, this
   * logic just sets 'badarg' and continues.
   */

  while ((option = getopt(argc, argv, ":o:t:")) != ERROR)
    {
      switch (option)
        {
          case 't':
            filesystem = optarg;
            break;

          case 'o':
            options = optarg;
            break;

          case ':':
            nsh_output(vtbl, g_fmtargrequired, argv[0]);
            badarg = true;
            break;

          case '?':
          default:
            nsh_output(vtbl, g_fmtarginvalid, argv[0]);
            badarg = true;
            break;
        }
    }

  /* If a bad argument was encountered, then return without processing the
   * command.
   */

  if (badarg)
    {
      return ERROR;
    }

  /* There may be one or two required arguments after the options: the source
   * and target paths.  Some file systems do not require the source parameter
   * so if there is only one parameter left, it must be the target.
   */

  if (optind >= argc)
    {
      nsh_output(vtbl, g_fmtargrequired, argv[0]);
      return ERROR;
    }

  source = NULL;
  target = argv[optind];
  optind++;

  if (optind < argc)
    {
      source = target;
      target = argv[optind];
      optind++;

      if (optind < argc)
        {
          nsh_output(vtbl, g_fmttoomanyargs, argv[0]);
          return ERROR;
        }
    }

  /* While the above parsing for the -t argument looks nice, the -t argument
   * not really optional.
   */

  if (!filesystem)
    {
      nsh_output(vtbl, g_fmtargrequired, argv[0]);
      return ERROR;
    }

  /* The source and target paths might be relative to the current
   * working directory.
   */

  fullsource = NULL;
  fulltarget = NULL;

  if (source)
    {
      fullsource = nsh_getfullpath(vtbl, source);
      if (!fullsource)
        {
          return ERROR;
        }
    }

  fulltarget = nsh_getfullpath(vtbl, target);
  if (!fulltarget)
    {
      ret = ERROR;
      goto errout;
    }

  /* Perform the mount */

  ret = mount(fullsource, fulltarget, filesystem, 0, options);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mount", NSH_ERRNO);
    }

errout:
  if (fullsource)
    {
      nsh_freefullpath(fullsource);
    }

  if (fulltarget)
    {
      nsh_freefullpath(fulltarget);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_nfsmount
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_NET) && defined(CONFIG_NFS) && !defined(CONFIG_NSH_DISABLE_NFSMOUNT)
int cmd_nfsmount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct nfs_args data;
  FAR char *address;
  FAR char *lpath;
  FAR char *rpath;
  bool badarg = false;
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *sin;
  struct in6_addr inaddr;
#else
  FAR struct sockaddr_in *sin;
  struct in_addr inaddr;
#endif
  int ret;

  /* If a bad argument was encountered, then return without processing the
   * command.
   */

  if (badarg)
    {
      return ERROR;
    }

  /* The fist argument on the command line should be the NFS server IP address
   * in standard IPv4 (or IPv6) dot format.
   */

  address = argv[1];
  if (!address)
    {
      return ERROR;
    }

  /* The local mount point path (lpath) might be relative to the current working
   * directory.
   */

  lpath = nsh_getfullpath(vtbl, argv[2]);
  if (!lpath)
    {
      return ERROR;
    }

  /* Get the remote mount point path */

  rpath = argv[3];

   /* Convert the IP address string into its binary form */

#ifdef CONFIG_NET_IPv6
  ret = inet_pton(AF_INET6, address, &inaddr);
#else
  ret = inet_pton(AF_INET, address, &inaddr);
#endif
  if (ret != 1)
    {
      nsh_freefullpath(lpath);
      return ERROR;
    }

  /* Place all of the NFS arguements into the nfs_args structure */

  memset(&data, 0, sizeof(data));

#ifdef CONFIG_NET_IPv6
  sin                  = (FAR struct sockaddr_in6 *)&data.addr;
  sin->sin6_family     = AF_INET6;
  sin->sin6_port       = htons(NFS_PMAPPORT);
  memcpy(&sin->sin6_addr, &inaddr, sizeof(struct in6_addr));
  data.addrlen         = sizeof(struct sockaddr_in6);
#else
  sin                  = (FAR struct sockaddr_in *)&data.addr;
  sin->sin_family      = AF_INET;
  sin->sin_port        = htons(NFS_PMAPPORT);
  sin->sin_addr        = inaddr;
  data.addrlen         = sizeof(struct sockaddr_in);
#endif

  data.sotype          = SOCK_DGRAM;
  data.path            = rpath;
  data.flags           = 0;       /* 0=Use all defaults */

  /* Perform the mount */

  ret = mount(NULL, lpath, "nfs", 0, (FAR void *)&data);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mount", NSH_ERRNO);
    }

  /* We no longer need the allocated mount point path */

  nsh_freefullpath(lpath);
  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_umount
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_UMOUNT)
int cmd_umount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {
      /* Perform the umount */

      ret = umount(fullpath);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "umount", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }

  return ret;
}
#endif
