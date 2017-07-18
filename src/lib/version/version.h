/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file version.h
 *
 * Tools for system version detection.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <px4_config.h>
#include <systemlib/px4_macros.h>
#include <stdint.h>

/* The preferred method for publishing a board name is to
 * define it in board_config.h as BOARD_NAME
 */

#ifndef BOARD_NAME
#  error "board_config.h must define BOARD_NAME"
#endif


__BEGIN_DECLS

/**
 * get the board name as string (including the version if there are multiple)
 */
static inline const char *px4_board_name(void)
{
	return BOARD_NAME;
}

/**
 * get the build URI (used for crash logging)
 */
static inline const char *px4_build_uri(void)
{
	return STRINGIFY(BUILD_URI);
}

/**
 * get the PX4 Firmware version
 * @return version in the form 0xAABBCCTT (AA: Major, BB: Minor, CC: Patch, TT Type @see FIRMWARE_TYPE)
 */
__EXPORT uint32_t px4_firmware_version(void);

/**
 * get the board version (last 8 bytes should be silicon ID, if any)
 */
__EXPORT uint32_t px4_board_version(void);

/**
 * operating system version
 * @return version in the form 0xAABBCCTT (AA: Major, BB: Minor, CC: Patch, TT Type @see FIRMWARE_TYPE)
 */
__EXPORT uint32_t px4_os_version(void);

/**
 * Operating system version as human readable string (git tag)
 * @return string or NULL if not defined
 */
__EXPORT const char *px4_os_version_string(void);

/**
 * name of the operating system
 * @return human readable string
 */
__EXPORT const char *px4_os_name(void);

/**
 * Toolchain name used to compile PX4
 */
__EXPORT const char *px4_toolchain_name(void);

/**
 * Toolchain version used to compile PX4 (no particular format)
 */
__EXPORT const char *px4_toolchain_version(void);

/**
 * Firmware version as human readable string (git tag)
 */
__EXPORT const char *px4_firmware_version_string(void);

/**
 * Firmware version in binary form (first part of the git tag)
 */
__EXPORT uint64_t px4_firmware_version_binary(void);

/**
 * Operating system version in binary form (first part of the git tag)
 * @return this is not available on all OSes and can return 0
 */
__EXPORT uint64_t px4_os_version_binary(void);

__END_DECLS

