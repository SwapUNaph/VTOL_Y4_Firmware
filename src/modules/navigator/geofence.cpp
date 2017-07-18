/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file geofence.cpp
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#include "geofence.h"
#include "navigator.h"

#include <ctype.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <systemlib/mavlink_log.h>

#define GEOFENCE_RANGE_WARNING_LIMIT 5000000

Geofence::Geofence(Navigator *navigator) :
	SuperBlock(navigator, "GF"),
	_navigator(navigator),
	_param_action(this, "GF_ACTION", false),
	_param_altitude_mode(this, "GF_ALTMODE", false),
	_param_source(this, "GF_SOURCE", false),
	_param_counter_threshold(this, "GF_COUNT", false),
	_param_max_hor_distance(this, "GF_MAX_HOR_DIST", false),
	_param_max_ver_distance(this, "GF_MAX_VER_DIST", false)
{
	updateParams();
}

bool Geofence::inside(const struct vehicle_global_position_s &global_position)
{
	return inside(global_position.lat, global_position.lon, global_position.alt);
}

bool Geofence::inside(const struct vehicle_global_position_s &global_position, float baro_altitude_amsl)
{
	return inside(global_position.lat, global_position.lon, baro_altitude_amsl);
}

bool Geofence::inside(const struct vehicle_global_position_s &global_position,
		      const struct vehicle_gps_position_s &gps_position, float baro_altitude_amsl)
{
	if (getAltitudeMode() == Geofence::GF_ALT_MODE_WGS84) {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return inside(global_position);

		} else {
			return inside((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
				      (double)gps_position.alt * 1.0e-3);
		}

	} else {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return inside(global_position, baro_altitude_amsl);

		} else {
			return inside((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
				      baro_altitude_amsl);
		}
	}
}

bool Geofence::inside(const struct mission_item_s &mission_item)
{
	return inside(mission_item.lat, mission_item.lon, mission_item.altitude);
}

bool Geofence::inside(double lat, double lon, float altitude)
{
	bool inside_fence = true;

	if (isHomeRequired() && _navigator->home_position_valid()) {

		const float max_horizontal_distance = _param_max_hor_distance.get();
		const float max_vertical_distance = _param_max_ver_distance.get();

		const double home_lat = _navigator->get_home_position()->lat;
		const double home_lon = _navigator->get_home_position()->lon;
		const double home_alt = _navigator->get_home_position()->alt;

		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		get_distance_to_point_global_wgs84(lat, lon, altitude, home_lat, home_lon, home_alt, &dist_xy, &dist_z);

		if (max_vertical_distance > FLT_EPSILON && (dist_z > max_vertical_distance)) {
			if (hrt_elapsed_time(&_last_vertical_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Maximum altitude above home exceeded by %.1f m",
						     (double)(dist_z - max_vertical_distance));
				_last_vertical_range_warning = hrt_absolute_time();
			}

			inside_fence = false;
		}

		if (max_horizontal_distance > FLT_EPSILON && (dist_xy > max_horizontal_distance)) {
			if (hrt_elapsed_time(&_last_horizontal_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Maximum distance from home exceeded by %.1f m",
						     (double)(dist_xy - max_horizontal_distance));
				_last_horizontal_range_warning = hrt_absolute_time();
			}

			inside_fence = false;
		}
	}

	// to be inside the geofence both fences have to report being inside
	// as they both report being inside when not enabled
	inside_fence = inside_fence && inside_polygon(lat, lon, altitude);

	if (inside_fence) {
		_outside_counter = 0;
		return inside_fence;

	} else {
		_outside_counter++;

		if (_outside_counter > _param_counter_threshold.get()) {
			return inside_fence;

		} else {
			return true;
		}
	}
}

bool Geofence::inside_polygon(double lat, double lon, float altitude)
{
	if (valid()) {
		if (!isEmpty()) {
			/* Vertical check */
			if (altitude > _altitude_max || altitude < _altitude_min) {
				return false;
			}

			/*Horizontal check */
			/* Adaptation of algorithm originally presented as
			 * PNPOLY - Point Inclusion in Polygon Test
			 * W. Randolph Franklin (WRF) */

			bool c = false;

			struct fence_vertex_s temp_vertex_i;
			struct fence_vertex_s temp_vertex_j;

			/* Red until fence is finished */
			for (unsigned i = 0, j = _vertices_count - 1; i < _vertices_count; j = i++) {
				if (dm_read(DM_KEY_FENCE_POINTS, i, &temp_vertex_i, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
					break;
				}

				if (dm_read(DM_KEY_FENCE_POINTS, j, &temp_vertex_j, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
					break;
				}

				// skip vertex 0 (return point)
				if (((double)temp_vertex_i.lon >= lon) != ((double)temp_vertex_j.lon >= lon) &&
				    (lat <= (double)(temp_vertex_j.lat - temp_vertex_i.lat) * (lon - (double)temp_vertex_i.lon) /
				     (double)(temp_vertex_j.lon - temp_vertex_i.lon) + (double)temp_vertex_i.lat)) {
					c = !c;
				}

			}

			return c;

		} else {
			/* Empty fence --> accept all points */
			return true;
		}

	} else {
		/* Invalid fence --> accept all points */
		return true;
	}
}

bool
Geofence::valid()
{
	// NULL fence is valid
	if (isEmpty()) {
		return true;
	}

	// Otherwise
	if ((_vertices_count < 4) || (_vertices_count > fence_s::GEOFENCE_MAX_VERTICES)) {
		warnx("Fence must have at least 3 sides and not more than %d", fence_s::GEOFENCE_MAX_VERTICES - 1);
		return false;
	}

	return true;
}

void
Geofence::addPoint(int argc, char *argv[])
{
	int ix, last;
	double lon, lat;
	struct fence_vertex_s vertex;
	char *end;

	if ((argc == 1) && (strcmp("-clear", argv[0]) == 0)) {
		dm_clear(DM_KEY_FENCE_POINTS);
		publishFence(0);
		return;
	}

	if (argc < 3) {
		PX4_WARN("Specify: -clear | sequence latitude longitude [-publish]");
	}

	ix = atoi(argv[0]);

	if (ix >= DM_KEY_FENCE_POINTS_MAX) {
		PX4_WARN("Sequence must be less than %d", DM_KEY_FENCE_POINTS_MAX);
	}

	lat = strtod(argv[1], &end);
	lon = strtod(argv[2], &end);

	last = 0;

	if ((argc > 3) && (strcmp(argv[3], "-publish") == 0)) {
		last = 1;
	}

	vertex.lat = (float)lat;
	vertex.lon = (float)lon;

	if (dm_write(DM_KEY_FENCE_POINTS, ix, DM_PERSIST_POWER_ON_RESET, &vertex, sizeof(vertex)) == sizeof(vertex)) {
		if (last) {
			publishFence((unsigned)ix + 1);
		}

		return;
	}

	PX4_WARN("can't store fence point");
}

void
Geofence::publishFence(unsigned vertices)
{
	if (_fence_pub == nullptr) {
		_fence_pub = orb_advertise(ORB_ID(fence), &vertices);

	} else {
		orb_publish(ORB_ID(fence), _fence_pub, &vertices);
	}
}

int
Geofence::loadFromFile(const char *filename)
{
	FILE		*fp;
	char		line[120];
	int			pointCounter = 0;
	bool		gotVertical = false;
	const char commentChar = '#';
	int rc = PX4_ERROR;

	/* Make sure no data is left in the datamanager */
	clearDm();

	/* open the mixer definition file */
	fp = fopen(GEOFENCE_FILENAME, "r");

	if (fp == nullptr) {
		return PX4_ERROR;
	}

	/* create geofence points from valid lines and store in DM */
	for (;;) {
		/* get a line, bail on error/EOF */
		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* Trim leading whitespace */
		size_t textStart = 0;

		while ((textStart < sizeof(line) / sizeof(char)) && isspace(line[textStart])) { textStart++; }

		/* if the line starts with #, skip */
		if (line[textStart] == commentChar) {
			continue;
		}

		/* if there is only a linefeed, skip it */
		if (line[0] == '\n') {
			continue;
		}

		if (gotVertical) {
			/* Parse the line as a geofence point */
			struct fence_vertex_s vertex;

			/* if the line starts with DMS, this means that the coordinate is given as degree minute second instead of decimal degrees */
			if (line[textStart] == 'D' && line[textStart + 1] == 'M' && line[textStart + 2] == 'S') {
				/* Handle degree minute second format */
				float lat_d, lat_m, lat_s, lon_d, lon_m, lon_s;

				if (sscanf(line, "DMS %f %f %f %f %f %f", &lat_d, &lat_m, &lat_s, &lon_d, &lon_m, &lon_s) != 6) {
					warnx("Scanf to parse DMS geofence vertex failed.");
					goto error;
				}

//				warnx("Geofence DMS: %.5f %.5f %.5f ; %.5f %.5f %.5f", (double)lat_d, (double)lat_m, (double)lat_s, (double)lon_d, (double)lon_m, (double)lon_s);

				vertex.lat = lat_d + lat_m / 60.0f + lat_s / 3600.0f;
				vertex.lon = lon_d + lon_m / 60.0f + lon_s / 3600.0f;

			} else {
				/* Handle decimal degree format */
				if (sscanf(line, "%f %f", &(vertex.lat), &(vertex.lon)) != 2) {
					warnx("Scanf to parse geofence vertex failed.");
					goto error;
				}
			}

			if (dm_write(DM_KEY_FENCE_POINTS, pointCounter, DM_PERSIST_POWER_ON_RESET, &vertex, sizeof(vertex)) != sizeof(vertex)) {
				goto error;
			}

			warnx("Geofence: point: %d, lat %.5f: lon: %.5f", pointCounter, (double)vertex.lat, (double)vertex.lon);

			pointCounter++;

		} else {
			/* Parse the line as the vertical limits */
			if (sscanf(line, "%f %f", &_altitude_min, &_altitude_max) != 2) {
				goto error;
			}

			warnx("Geofence: alt min: %.4f, alt_max: %.4f", (double)_altitude_min, (double)_altitude_max);
			gotVertical = true;
		}
	}

	/* Check if import was successful */
	if (gotVertical && pointCounter > 0) {
		_vertices_count = pointCounter;
		warnx("Geofence: imported successfully");
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Geofence imported");
		rc = PX4_OK;

	} else {
		warnx("Geofence: import error");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence import error");
	}

error:
	fclose(fp);
	return rc;
}

int Geofence::clearDm()
{
	dm_clear(DM_KEY_FENCE_POINTS);
	return PX4_OK;
}

bool Geofence::isHomeRequired()
{
	bool max_horizontal_enabled = (_param_max_hor_distance.get() > FLT_EPSILON);
	bool max_vertical_enabled = (_param_max_ver_distance.get() > FLT_EPSILON);
	bool geofence_action_rtl = (getGeofenceAction() == geofence_result_s::GF_ACTION_RTL);

	return max_horizontal_enabled || max_vertical_enabled || geofence_action_rtl;
}
