/****************************************************************************
 * fs/smartfs/smartfs_ioctl.c
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <semaphore.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/smart.h>

#include "smartfs.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartfs_deletedirentry
 *
 * Description: Deletes a direntry from the filesystem (file or dir) but
 *              does not free the file's data.  This is used when changing
 *              the file's time, mode, etc. within the directory.
 *
 ****************************************************************************/

static int smartfs_deletedirentry(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry)
{
  int                             ret;
  uint16_t                        nextsector;
  uint16_t                        sector;
  uint16_t                        count;
  uint16_t                        entrysize;
  uint16_t                        offset;
  struct smartfs_entry_header_s  *direntry;
  struct smartfs_chain_header_s  *header;
  struct smart_read_write_s       readwrite;

  /* Remove the entry from the directory tree */

  readwrite.logsector = entry->dsector;
  readwrite.offset = 0;
  readwrite.count = fs->fs_llformat.availbytes;
  readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
  ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
  if (ret < 0)
    {
      fdbg("Error reading directory info at sector %d\n", entry->dsector);
      goto errout;
    }

  /* Mark this entry as inactive */

  direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[entry->doffset];
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&direntry->flags, smartfs_rdle16(&direntry->flags) & ~SMARTFS_DIRENT_ACTIVE);
#else
  direntry->flags &= ~SMARTFS_DIRENT_ACTIVE;
#endif
#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&direntry->flags, smartfs_rdle16(&direntry->flags) | SMARTFS_DIRENT_ACTIVE);
#else
  direntry->flags |= SMARTFS_DIRENT_ACTIVE;
#endif
#endif  /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */

  /* Write the updated flags back to the sector */

  readwrite.offset = entry->doffset;
  readwrite.count = sizeof(uint16_t);
  readwrite.buffer = (uint8_t *) &direntry->flags;
  ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
  if (ret < 0)
    {
      fdbg("Error marking entry inactive at sector %d\n", entry->dsector);
      goto errout;
    }

  /* Test if any entries in this sector are being used */

  if ((entry->dsector != fs->fs_rootsector) &&
      (entry->dsector != entry->dfirst))
    {
      /* Scan the sector and count used entries */

      count = 0;
      offset = sizeof(struct smartfs_chain_header_s);
      entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;
      while (offset + entrysize < fs->fs_llformat.availbytes)
        {
          /* Test the next entry */

          direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
          if (((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#else
          if (((direntry->flags & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((direntry->flags & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#endif
            {
              /* Count this entry */
              count++;
            }

          /* Advance to next entry */

          offset += entrysize;
        }

      /* Test if the count it zero.  If it is, then we will release the sector */

      if (count == 0)
        {
          /* Okay, to release the sector, we must find the sector that we
           * are chained to and remove ourselves from the chain.  First
           * save our nextsector value so we can "unchain" ourselves.
           */

          nextsector = SMARTFS_NEXTSECTOR(header);

          /* Now loop through the dir sectors to find ourselves in the chain */

          sector = entry->dfirst;
          readwrite.offset = 0;
          readwrite.count = sizeof(struct smartfs_chain_header_s);
          readwrite.buffer = (uint8_t *) fs->fs_rwbuffer;
          while (sector != SMARTFS_ERASEDSTATE_16BIT)
            {
              /* Read the header for the next sector */

              readwrite.logsector = sector;
              ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
              if (ret < 0)
                {
                  fdbg("Error reading sector %d\n", nextsector);
                  break;
                }

              /* Test if this sector "points" to us */

              if (SMARTFS_NEXTSECTOR(header) == entry->dsector)
                {
                  /* We found ourselves in the chain.  Update the chain. */

                  SMARTFS_NEXTSECTOR(header) = nextsector;
                  readwrite.offset = offsetof(struct smartfs_chain_header_s, nextsector);
                  readwrite.count = sizeof(uint16_t);
                  readwrite.buffer = header->nextsector;
                  ret = FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
                  if (ret < 0)
                    {
                      fdbg("Error unchaining sector (%d)\n", nextsector);
                      goto errout;
                    }

                  /* Now release our sector */

                  ret = FS_IOCTL(fs, BIOC_FREESECT, (unsigned long) entry->dsector);
                  if (ret < 0)
                    {
                      fdbg("Error freeing sector %d\n", entry->dsector);
                      goto errout;
                    }

                  /* Break out of the loop, we are done! */

                  break;
                }

              /* Chain to the next sector */

              sector = SMARTFS_NEXTSECTOR(header);
            }
        }
    }

  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartfs_ioctl
 ****************************************************************************/

int smartfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int                      ret = -ENOSYS;
  struct inode             *inode;
  struct smartfs_mountpt_s *fs;
  struct smartfs_ofile_s   *sf;
  struct smartfs_entry_s    entry;

  /* We onlty handle the UTIME and CHMOD ioctl commands */

  if (cmd != FIOUTIME && cmd != FIOCHMOD)
    return ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  inode = filep->f_inode;
  fs    = inode->i_private;
  sf    = filep->f_priv;

  /* Take the semaphore */

  smartfs_semtake(fs);

  /* Handle the UTIME and CHMOD ioctl commands */

  switch (cmd)
    {
      case FIOUTIME:
        sf->entry.utc = arg;
        break;

      case FIOCHMOD:
        sf->entry.flags &= ~SMARTFS_DIRENT_MODE;
        sf->entry.flags |= arg & SMARTFS_DIRENT_MODE;
        break;

      default:
        break;
    }

  /* Delete the exsiting direntry in the directory */

  entry = sf->entry;
  ret = smartfs_deletedirentry(fs, &sf->entry);
  if (ret != OK)
    {
      smartfs_semgive(fs);
      return ret;
    }

  /* Create a new direntry for this file */

  smartfs_createentry(fs, entry.dfirst, entry.name, 
          entry.flags & SMARTFS_DIRENT_TYPE, 
          entry.flags & SMARTFS_DIRENT_MODE,
          entry.utc, &sf->entry, entry.firstsector, sf);

  smartfs_semgive(fs);
  return OK;
}

