/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file dataman.h
 *
 * DATAMANAGER driver.
 */
#ifndef _DATAMANAGER_H
#define _DATAMANAGER_H

#include <string.h>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/fence_vertex.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Types of items that the data manager can store */
typedef enum {
	DM_KEY_SAFE_POINTS = 0,		/* Safe points coordinates, safe point 0 is home point */
	DM_KEY_FENCE_POINTS,		/* Fence vertex coordinates */
	DM_KEY_WAYPOINTS_OFFBOARD_0,	/* Mission way point coordinates sent over mavlink */
	DM_KEY_WAYPOINTS_OFFBOARD_1,	/* (alernate between 0 and 1) */
	DM_KEY_WAYPOINTS_ONBOARD,	/* Mission way point coordinates generated onboard */
	DM_KEY_MISSION_STATE,		/* Persistent mission state */
	DM_KEY_COMPAT,
	DM_KEY_NUM_KEYS			/* Total number of item types defined */
} dm_item_t;

#define DM_KEY_WAYPOINTS_OFFBOARD(_id) (_id == 0 ? DM_KEY_WAYPOINTS_OFFBOARD_0 : DM_KEY_WAYPOINTS_OFFBOARD_1)

#if defined(MEMORY_CONSTRAINED_SYSTEM)
enum {
	DM_KEY_SAFE_POINTS_MAX = 8,
#ifdef __cplusplus
	DM_KEY_FENCE_POINTS_MAX = fence_s::GEOFENCE_MAX_VERTICES,
#else
	DM_KEY_FENCE_POINTS_MAX = GEOFENCE_MAX_VERTICES,
#endif
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_WAYPOINTS_ONBOARD_MAX = (NUM_MISSIONS_SUPPORTED / 10),
	DM_KEY_MISSION_STATE_MAX = 1,
	DM_KEY_COMPAT_MAX = 1
};
#else
/** The maximum number of instances for each item type */
enum {
	DM_KEY_SAFE_POINTS_MAX = 8,
#ifdef __cplusplus
	DM_KEY_FENCE_POINTS_MAX = fence_s::GEOFENCE_MAX_VERTICES,
#else
	DM_KEY_FENCE_POINTS_MAX = GEOFENCE_MAX_VERTICES,
#endif
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_WAYPOINTS_ONBOARD_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_MISSION_STATE_MAX = 1,
	DM_KEY_COMPAT_MAX = 1
};
#endif
/** Data persistence levels */
typedef enum {
	DM_PERSIST_POWER_ON_RESET = 0,	/* Data survives all resets */
	DM_PERSIST_IN_FLIGHT_RESET,     /* Data survives in-flight resets only */
	DM_PERSIST_VOLATILE             /* Data does not survive resets */
} dm_persitence_t;

/** The reason for the last reset */
typedef enum {
	DM_INIT_REASON_POWER_ON = 0,	/* Data survives resets */
	DM_INIT_REASON_IN_FLIGHT,		/* Data survives in-flight resets only */
	DM_INIT_REASON_VOLATILE			/* Data does not survive reset */
} dm_reset_reason;

struct dataman_compat_s {
	uint64_t key;
};

/* increment this define whenever a binary incompatible change is performed */
#define DM_COMPAT_VERSION	1ULL

#define DM_COMPAT_KEY ((DM_COMPAT_VERSION << 32) + (sizeof(struct mission_item_s) << 24) + (sizeof(struct mission_s) << 16) + (sizeof(struct fence_vertex_s) << 8) + sizeof(struct dataman_compat_s))

/** Maximum size in bytes of a single item instance is
 * defined by adding the structure type to the union below
 */

typedef union dataman_max_size_t {
	struct mission_item_s		mission_item;
	struct mission_s			mission;
	struct fence_vertex_s		vertex;
	struct dataman_compat_s		compat;
} dataman_max_size_t;


#define DM_MAX_DATA_SIZE sizeof(dataman_max_size_t)

/** Retrieve from the data manager store */
__EXPORT ssize_t
dm_read(
	dm_item_t item,			/* The item type to retrieve */
	unsigned index,			/* The index of the item */
	void *buffer,			/* Pointer to caller data buffer */
	size_t buflen			/* Length in bytes of data to retrieve */
);

/** write to the data manager store */
__EXPORT ssize_t
dm_write(
	dm_item_t  item,		/* The item type to store */
	unsigned index,			/* The index of the item */
	dm_persitence_t persistence,	/* The persistence level of this item */
	const void *buffer,		/* Pointer to caller data buffer */
	size_t buflen			/* Length in bytes of data to retrieve */
);

/** Lock all items of this type */
__EXPORT void
dm_lock(
	dm_item_t item			/* The item type to clear */
);

/** Unlock all items of this type */
__EXPORT void
dm_unlock(
	dm_item_t item			/* The item type to clear */
);

/** Erase all items of this type */
__EXPORT int
dm_clear(
	dm_item_t item			/* The item type to clear */
);

/** Tell the data manager about the type of the last reset */
__EXPORT int
dm_restart(
	dm_reset_reason restart_type	/* The last reset type */
);

#if defined(FLASH_BASED_DATAMAN)
typedef struct dm_sector_descriptor_t {
	uint8_t       page;
	uint32_t      size;
	uint32_t      address;
} dm_sector_descriptor_t;

/**
 * Set the flash sector description were data should persist data
 *
 * Important: do not use a Flash sector from the same bank that STM32 read
 * instructions or the CPU will held for sometime during Flash erase and write
 * and this could cause your drone to fall.
 */
__EXPORT int
dm_flash_sector_description_set(
	const dm_sector_descriptor_t *description
);
#endif

#ifdef __cplusplus
}
#endif

#endif
