#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/fs.h>
#include <stdint.h>

uint64_t linux_get_block_dev_size(int fd)
{
  uint64_t  len;

  ioctl(fd, BLKGETSIZE64, &len);

  return len;
}

