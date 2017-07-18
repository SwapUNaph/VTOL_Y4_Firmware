/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file px4_posix.h
 *
 * Includes POSIX-like functions for virtual character devices
 */

#pragma once

#include <px4_defines.h>
#include <px4_tasks.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>

#if defined(__PX4_QURT)
#include <dspal_types.h>
#else
#include <sys/types.h>
#endif

#include "px4_sem.h"


#ifdef __PX4_NUTTX

#define  PX4_F_RDONLY 1
#define  PX4_F_WRONLY 2

typedef struct pollfd px4_pollfd_struct_t;

#if defined(__cplusplus)
#define _GLOBAL ::
#else
#define _GLOBAL
#endif
#define px4_open 	_GLOBAL open
#define px4_close 	_GLOBAL close
#define px4_ioctl 	_GLOBAL ioctl
#define px4_write 	_GLOBAL write
#define px4_read 	_GLOBAL read
#define px4_poll 	_GLOBAL poll
#define px4_fsync 	_GLOBAL fsync
#define px4_access 	_GLOBAL access
#define px4_getpid 	_GLOBAL getpid

#define  PX4_STACK_OVERHEAD	0

#elif defined(__PX4_POSIX)

#define  PX4_F_RDONLY O_RDONLY
#define  PX4_F_WRONLY O_WRONLY
#define  PX4_F_CREAT  O_CREAT
#define	 PX4_STACK_OVERHEAD	8192

typedef short pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	pollevent_t 	events;   /* The input event flags */
	pollevent_t 	revents;  /* The output event flags */

	/* Required for PX4 compatibility */
	px4_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} px4_pollfd_struct_t;

__BEGIN_DECLS

__EXPORT int 		px4_open(const char *path, int flags, ...);
__EXPORT int 		px4_close(int fd);
__EXPORT ssize_t	px4_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	px4_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		px4_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		px4_poll(px4_pollfd_struct_t *fds, nfds_t nfds, int timeout);
__EXPORT int		px4_fsync(int fd);
__EXPORT int		px4_access(const char *pathname, int mode);
__EXPORT px4_task_t	px4_getpid(void);

__EXPORT void		px4_enable_sim_lockstep(void);
__EXPORT void		px4_sim_start_delay(void);
__EXPORT void		px4_sim_stop_delay(void);
__EXPORT bool		px4_sim_delay_enabled(void);

__END_DECLS
#else
#error "No TARGET OS Provided"
#endif


// The stack size is intended for 32-bit architectures; therefore
// we often run out of stack space when pointers are larger than 4 bytes.
// Double the stack size on posix when we're on a 64-bit architecture.
// Most full-scale OS use 1-4K of memory from the stack themselves
#define PX4_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + PX4_STACK_OVERHEAD)

__BEGIN_DECLS
extern int px4_errno;

__EXPORT void		px4_show_devices(void);
__EXPORT void		px4_show_files(void);
__EXPORT const char 	*px4_get_device_names(unsigned int *handle);

__EXPORT void		px4_show_topics(void);
__EXPORT const char 	*px4_get_topic_names(unsigned int *handle);

#ifndef __PX4_QURT
/*
 * The UNIX epoch system time following the system clock
 */
__EXPORT uint64_t	hrt_system_time(void);

__EXPORT bool		px4_exit_requested(void);


#endif

__END_DECLS
