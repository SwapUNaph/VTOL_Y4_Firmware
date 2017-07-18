/****************************************************************************
 * drivers/mtd/filemtd.c
 *
 *   Copyright (C) 2015 Ken Pettit. All rights reserved.
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
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/fs.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_FILEMTD_BLOCKSIZE
#  define CONFIG_FILEMTD_BLOCKSIZE 512
#endif

#ifndef CONFIG_FILEMTD_ERASESIZE
#  define CONFIG_FILEMTD_ERASESIZE 4096
#endif

#ifndef CONFIG_FILEMTD_ERASESTATE
#  define CONFIG_FILEMTD_ERASESTATE 0xff
#endif

#if CONFIG_FILEMTD_ERASESTATE != 0xff && CONFIG_FILEMTD_ERASESTATE != 0x00
#  error "Unsupported value for CONFIG_FILEMTD_ERASESTATE"
#endif

#if CONFIG_FILEMTD_BLOCKSIZE > CONFIG_FILEMTD_ERASESIZE
#  error "Must have CONFIG_FILEMTD_BLOCKSIZE <= CONFIG_FILEMTD_ERASESIZE"
#endif

#undef  FILEMTD_BLKPER
#define FILEMTD_BLKPER (CONFIG_FILEMTD_ERASESIZE/CONFIG_FILEMTD_BLOCKSIZE)

#if FILEMTD_BLKPER*CONFIG_FILEMTD_BLOCKSIZE != CONFIG_FILEMTD_ERASESIZE
#  error "CONFIG_FILEMTD_ERASESIZE must be an even multiple of CONFIG_FILEMTD_BLOCKSIZE"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct file_dev_s.
 */

struct file_dev_s
{
  struct mtd_dev_s mtd;        /* MTD device */
  int              fd;         /* File descriptor of underlying file */
  size_t           nblocks;    /* Number of erase blocks */
  size_t           offset;     /* Offset from start of file */
  size_t           erasesize;  /* Offset from start of file */
  size_t           blocksize;  /* Offset from start of file */
  size_t           blkper;     /* Blocks per erase block */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t filemtd_read(FAR struct file_dev_s *priv,
                 FAR unsigned char *buffer, size_t offsetbytes,
                 unsigned int nbytes);
static ssize_t filemtd_write(FAR struct file_dev_s *priv, size_t offset,
                 FAR const void *src, size_t len);

/* MTD driver methods */

static int file_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t file_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR uint8_t *buf);
static ssize_t file_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR const uint8_t *buf);
static ssize_t file_byteread(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t file_bytewrite(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR const uint8_t *buf);
#endif
static int file_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                 unsigned long arg);

extern uint64_t linux_get_block_dev_size(int fd);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: filemtd_write
 ****************************************************************************/

static ssize_t filemtd_write(FAR struct file_dev_s *priv, size_t offset, 
                             FAR const void *src, size_t len)
{
  FAR const uint8_t *pin  = (FAR const uint8_t *)src;
  FAR uint8_t       *pout;
  char               buf[128];
  int                buflen = 0;
  uint8_t            oldvalue;
  uint8_t            srcvalue;
  uint8_t            newvalue;
  size_t             seekpos;

  /* Set the starting location in the file */

  seekpos = priv->offset + offset;

  while (len-- > 0)
    {
      if (buflen == 0)
        {
          /* Read more data from the file */

          lseek(priv->fd, seekpos, SEEK_SET);
          buflen = read(priv->fd, buf, sizeof(buf));
          pout = (FAR uint8_t *) buf;
        }

      /* Get the source and destination values */

      oldvalue = *pout;
      srcvalue = *pin++;

      /* Get the new destination value, accounting for bits that cannot be
       * changes because they are not in the erased state.
       */

#if CONFIG_FILEMTD_ERASESTATE == 0xff
      newvalue = oldvalue & srcvalue; /* We can only clear bits */
#else /* CONFIG_FILEMTD_ERASESTATE == 0x00 */
      newvalue = oldvalue | srcvalue; /* We can only set bits */
#endif

      /* Report any attempt to change the value of bits that are not in the
       * erased state.
       */

#ifdef CONFIG_DEBUG
      if (newvalue != srcvalue)
        {
          dbg("ERROR: Bad write: source=%02x dest=%02x result=%02x\n",
              srcvalue, oldvalue, newvalue);
        }
#endif

      /* Write the modified value to simulated FLASH */

      *pout++ = newvalue;
      buflen--;

      /* If our buffer is full, then seek back to beginning of 
       * the file and write the buffer contents 
       */

      if (buflen == 0)
        { 
          lseek(priv->fd, seekpos, SEEK_SET);
          write(priv->fd, buf, sizeof(buf));
          seekpos += sizeof(buf);
        } 
    }

  /* Write remaining bytes */

  if (buflen != 0)
    {
      lseek(priv->fd, seekpos, SEEK_SET);
      write(priv->fd, buf, sizeof(buf));
    }

  return len;
}

/****************************************************************************
 * Name: filemtd_read
 ****************************************************************************/

static ssize_t filemtd_read(FAR struct file_dev_s *priv,
                            FAR unsigned char *buffer, size_t offsetbytes,
                             unsigned int nbytes)
{
  /* Set the starting location in the file */

  lseek(priv->fd, priv->offset + offsetbytes, SEEK_SET);

  return read(priv->fd, buffer, nbytes);
}

/****************************************************************************
 * Name: file_erase
 ****************************************************************************/

static int file_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  size_t    nbytes;
  size_t    offset;
  char      buffer[128];

  DEBUGASSERT(dev);

  /* Don't let the erase exceed the original size of the file */

  if (startblock >= priv->nblocks)
    {
      return 0;
    }

  if (startblock + nblocks > priv->nblocks)
    {
      nblocks = priv->nblocks - startblock;
    }

  /* Convert the erase block to a logical block and the number of blocks
   * in logical block numbers
   */

  startblock *= priv->blkper;
  nblocks    *= priv->blkper;

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  /* Then erase the data in the file */

  lseek(priv->fd, priv->offset + offset, SEEK_SET);
  memset(buffer, CONFIG_FILEMTD_ERASESTATE, sizeof(buffer));
  while (nbytes)
    {
      write(priv->fd, buffer, sizeof(buffer));
      nbytes -= sizeof(buffer); 
    }

  return OK;
}

/****************************************************************************
 * Name: file_bread
 ****************************************************************************/

static ssize_t file_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t offset;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the read exceed the original size of the file */

  maxblock = priv->nblocks * priv->blkper;
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  /* Then read the data from the file */

  filemtd_read(priv, buf, offset, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: file_bwrite
 ****************************************************************************/

static ssize_t file_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t offset;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the original size of the file */

  maxblock = priv->nblocks * priv->blkper;
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  /* Then write the data to the file */

  filemtd_write(priv, offset, buf, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: file_byteread
 ****************************************************************************/

static ssize_t file_byteread(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;

  DEBUGASSERT(dev && buf);

  /* Don't let read read past end of buffer */

  if (offset + nbytes > priv->nblocks * priv->erasesize)
   {
     return 0;
   }

  filemtd_read(priv, buf, offset, nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: file_bytewrite
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t file_bytewrite(FAR struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, FAR const uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t maxoffset;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the original size of the file */

  maxoffset = priv->nblocks * priv->erasesize;
  if (offset + nbytes > maxoffset)
    {
      return 0;
    }

  /* Then write the data to the file */

  filemtd_write(priv, offset, buf, nbytes);
  return nbytes;
}
#endif

/****************************************************************************
 * Name: file_ioctl
 ****************************************************************************/

static int file_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               */

              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->erasesize;
              geo->neraseblocks = priv->nblocks;
              ret               = OK;
            }
        }
        break;

      case MTDIOC_XIPBASE:
        ret = -ENOTTY; /* Bad command */
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          file_erase(dev, 0, priv->nblocks);
          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: filemtd_initialize
 *
 * Description:
 *   Create and initialize a FILE MTD device instance.
 *
 * Input Parameters:
 *   path - Path name of the file backing the MTD device
 *
 ****************************************************************************/

FAR struct mtd_dev_s *filemtd_initialize(FAR const char *path, size_t offset,
                            int16_t sectsize, int32_t erasesize)
{
  FAR struct file_dev_s *priv;
  struct stat sb;
  size_t nblocks;
  size_t filelen;
  int    mode, ret;

  /* Create an instance of the FILE MTD device state structure */

  priv = (FAR struct file_dev_s *)kmm_zalloc(sizeof(struct file_dev_s));
  if (!priv)
    {
      fdbg("Failed to allocate the FILE MTD state structure\n");
      return NULL;
    }

  /* Determine the file open mode */

  mode = O_RDONLY;
#ifdef CONFIG_FS_WRITABLE
  mode |= O_RDWR;
#endif

  /* Stat the file */

  ret = stat(path, &sb);
  if (ret < 0)
    {
      dbg("Failed to stat %s: %d\n", path, get_errno());
      return NULL;
    }

  filelen = sb.st_size;

  /* Try to open the file */

  priv->fd = open(path, mode);
  if (priv->fd == -1)
    {
      fdbg("Failed to open the FILE MTD file %s\n", path);
      kmm_free(priv);
      return NULL;
    }

  if (filelen == 0)
    filelen = linux_get_block_dev_size(priv->fd);

  /* Set the block size based on the provided sectsize parameter */

  if (sectsize <= 0)
    {
      priv->blocksize = CONFIG_FILEMTD_BLOCKSIZE;
    }
  else
    {
      priv->blocksize = sectsize;
    }

  /* Set the erase size based on the provided erasesize parameter */

  if (erasesize <= 0)
    {
      priv->erasesize = CONFIG_FILEMTD_ERASESIZE;
    }
  else
    {
      priv->erasesize = erasesize;
    }

  priv->blkper = priv->erasesize / priv->blocksize;

  /* Force the size to be an even number of the erase block size */

  nblocks = (filelen - offset) / priv->erasesize;
  if (nblocks < 3)
    {
      fdbg("Need to provide at least three full erase block\n");
      kmm_free(priv);
      return NULL;
    }

  /* Perform initialization as necessary. (unsupported methods were
   * nullified by kmm_zalloc).
   */

  priv->mtd.erase  = file_erase;
  priv->mtd.bread  = file_bread;
  priv->mtd.bwrite = file_bwrite;
  priv->mtd.read   = file_byteread;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = file_bytewrite;
#endif
  priv->mtd.ioctl  = file_ioctl;
  priv->offset     = offset;
  priv->nblocks    = nblocks;

  /* Register the MTD with the procfs system if enabled */

#ifdef CONFIG_MTD_REGISTRATION
  mtd_register(&priv->mtd, "filemtd");
#endif

  return &priv->mtd;
}

/****************************************************************************
 * Name: filemtd_teardown
 *
 * Description:
 *   Teardown a previously created filemtd device.
 *
 * Input Parameters:
 *   path - Path name of the file backing the MTD device
 *
 ****************************************************************************/

void filemtd_teardown(FAR struct mtd_dev_s *dev)
{
  FAR struct file_dev_s *priv;

  /* Close the enclosed file */

  priv = (FAR struct file_dev_s *) dev;
  close(priv->fd);

  /* Register the MTD with the procfs system if enabled */

#ifdef CONFIG_MTD_REGISTRATION
  mtd_unregister(&priv->mtd);
#endif

  /* Free the memory */

  kmm_free(priv);
}

/****************************************************************************
 * Name: filemtd_isfilemtd
 *
 * Description:
 *   Tests if the provided mtd is a filemtd device.
 *
 * Input Parameters:
 *   mtd - Pointer to the mtd.
 *
 ****************************************************************************/

bool filemtd_isfilemtd(FAR struct mtd_dev_s *dev)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *) dev;

  if (priv->mtd.erase == file_erase)
    return 1;

  return 0;
}
