/****************************************************************************
 * sched/module/mod_read.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <elf32.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/module.h>

#include "module.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef ELF_DUMP_READDATA       /* Define to dump all file data read */

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_dumpreaddata
 ****************************************************************************/

#if defined(ELF_DUMP_READDATA)
static inline void mod_dumpreaddata(FAR char *buffer, int buflen)
{
  FAR uint32_t *buf32 = (FAR uint32_t *)buffer;
  int i;
  int j;

  for (i = 0; i < buflen; i += 32)
    {
      syslog(LOG_DEBUG, "%04x:", i);
      for (j = 0; j < 32; j += sizeof(uint32_t))
        {
          syslog(LOG_DEBUG, "  %08x", *buf32++);
        }

      syslog(LOG_DEBUG, "\n");
    }
}
#else
#  define mod_dumpreaddata(b,n)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mod_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'.  The data is
 *   read into 'buffer.'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int mod_read(FAR struct mod_loadinfo_s *loadinfo, FAR uint8_t *buffer,
             size_t readsize, off_t offset)
{
  ssize_t nbytes;      /* Number of bytes read */
  off_t   rpos;        /* Position returned by lseek */

  sinfo("Read %ld bytes from offset %ld\n", (long)readsize, (long)offset);

  /* Loop until all of the requested data has been read. */

  while (readsize > 0)
    {
      /* Seek to the next read position */

      rpos = lseek(loadinfo->filfd, offset, SEEK_SET);
      if (rpos != offset)
        {
          int errval = errno;
          serr("ERROR: Failed to seek to position %lu: %d\n",
               (unsigned long)offset, errval);
          return -errval;
        }

      /* Read the file data at offset into the user buffer */

       nbytes = read(loadinfo->filfd, buffer, readsize);
       if (nbytes < 0)
         {
           int errval = errno;

           /* EINTR just means that we received a signal */

           if (errval != EINTR)
             {
               serr("ERROR: Read from offset %lu failed: %d\n",
                    (unsigned long)offset, errval);
               return -errval;
             }
         }
       else if (nbytes == 0)
         {
           serr("ERROR: Unexpected end of file\n");
           return -ENODATA;
         }
       else
         {
           readsize -= nbytes;
           buffer   += nbytes;
           offset   += nbytes;
         }
    }

  mod_dumpreaddata(buffer, readsize);
  return OK;
}
