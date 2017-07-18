/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file sdlog2.c
 *
 * Simple SD logger for flight data. Buffers new sensor values and
 * does the heavy SD I/O in a low-priority worker thread.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#include <ctype.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/task_stack_info.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/printload.h>
#include <systemlib/mavlink_log.h>
#include <version/version.h>

#include "logbuffer.h"
#include "sdlog2_format.h"
#include "sdlog2_messages.h"

#define PX4_EPOCH_SECS 1234567890L

#define LOGBUFFER_WRITE_AND_COUNT(_msg) pthread_mutex_lock(&logbuffer_mutex); \
	if (logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(_msg))) { \
		log_msgs_written++; \
	} else { \
		log_msgs_skipped++; \
	} \
	pthread_mutex_unlock(&logbuffer_mutex);

#define SDLOG_MIN(X,Y) ((X) < (Y) ? (X) : (Y))

static bool main_thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;			/**< Deamon status flag */
static int deamon_task;						/**< Handle of deamon task / thread */
static bool logwriter_should_exit = false;	/**< Logwriter thread exit flag */
static const unsigned MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
static const unsigned MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */
static const int LOG_BUFFER_SIZE_DEFAULT = 8192;

#if defined __PX4_POSIX
static const int MAX_WRITE_CHUNK = 2048;
static const int MIN_BYTES_TO_WRITE = 512;
#else
static const int MAX_WRITE_CHUNK = 512;
static const int MIN_BYTES_TO_WRITE = 512;
#endif

static bool _extended_logging = false;
static bool _gpstime_only = false;
static int32_t _utc_offset = 0;

#if !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)
#define MOUNTPOINT PX4_ROOTFSDIR"/fs/microsd"
#else
#define MOUNTPOINT "/root"
#endif
static const char *mountpoint = MOUNTPOINT;
static const char *log_root = MOUNTPOINT "/log";
static orb_advert_t mavlink_log_pub = NULL;
struct logbuffer_s lb;

/* mutex / condition to synchronize threads */
static pthread_mutex_t logbuffer_mutex;
static pthread_cond_t logbuffer_cond;

#ifdef __PX4_NUTTX
#define LOG_BASE_PATH_LEN	64
#else
#define LOG_BASE_PATH_LEN	256
#endif

static char log_dir[LOG_BASE_PATH_LEN];

/* statistics counters */
static uint64_t start_time = 0;
static unsigned long log_bytes_written = 0;
static unsigned long last_checked_bytes_written = 0;
static unsigned long log_msgs_written = 0;
static unsigned long log_msgs_skipped = 0;

/* GPS time, used for log files naming */
static uint64_t gps_time_sec = 0;
static bool has_gps_3d_fix = false;

/* current state of logging */
static bool logging_enabled = false;
/* use date/time for naming directories and files (-t option) */
static bool log_name_timestamp = false;

/* helper flag to track system state changes */
static bool flag_system_armed = false;

/* flag if warning about MicroSD card being almost full has already been sent */
static bool space_warning_sent = false;

static pthread_t logwriter_pthread = 0;
static pthread_attr_t logwriter_attr;

static perf_counter_t perf_write;

/* Keep track if we've already created a folder named sessXXX because
 * we don't want to create yet another one. */
static bool sess_folder_created = false;

/**
 * Log buffer writing thread. Open and close file here.
 */
static void *logwriter_thread(void *arg);

/**
 * SD log management function.
 */
__EXPORT int sdlog2_main(int argc, char *argv[]);

static bool copy_if_updated(orb_id_t topic, int *handle, void *buffer);
static bool copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer);

/**
 * Mainloop of sd log deamon.
 */
int sdlog2_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void sdlog2_usage(const char *reason);

/**
 * Print the current status.
 */
static void sdlog2_status(void);

/**
 * Start logging: create new file and start log writer thread.
 */
static void sdlog2_start_log(void);

/**
 * Stop logging: stop log writer thread and close log file.
 */
static void sdlog2_stop_log(void);

/**
 * Write a header to log file: list of message formats.
 */
static int write_formats(int fd);

/**
 * Write version message to log file.
 */
static int write_version(int fd);

/**
 * Write parameters to log file.
 */
static int write_parameters(int fd);

static bool file_exist(const char *filename);

/**
 * Check if there is still free space available
 */
static int check_free_space(void);

static void handle_command(struct vehicle_command_s *cmd);

static void handle_status(struct vehicle_status_s *cmd);

/**
 * Create dir for current logging session. Store dir name in 'log_dir'.
 */
static int create_log_dir(void);

/**
 * Get the time struct from the currently preferred time source
 */
static bool get_log_time_tt(struct tm *tt, bool boot_time);

/**
 * Select first free log file name and open it.
 */
static int open_log_file(void);

static int open_perf_file(const char* str);

static void
sdlog2_usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	PX4_WARN("usage: sdlog2 {start|stop|status|on|off} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
		 "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
		 "\t-b\tLog buffer size in KiB, default is 8\n"
		 "\t-e\tEnable logging by default (if not, can be started by command)\n"
		 "\t-a\tLog only when armed (can be still overriden by command)\n"
		 "\t-t\tUse date/time for naming log directories and files\n"
		 "\t-x\tExtended logging");
}

/**
 * The logger deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int sdlog2_main(int argc, char *argv[])
{
	if (argc < 2) {
		sdlog2_usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}

		// get sdlog priority boost parameter. This can be used to avoid message drops
		// in the log file. However, it considered to be used only for developers.
		param_t prio_boost_handle = param_find("SDLOG_PRIO_BOOST");
		int prio_boost = 0;
		param_get(prio_boost_handle, &prio_boost);
		int task_priority = SCHED_PRIORITY_DEFAULT - 30;

		switch(prio_boost) {
			case 1:
				task_priority = SCHED_PRIORITY_DEFAULT;
				break;
			case 2:
				task_priority = SCHED_PRIORITY_DEFAULT + (SCHED_PRIORITY_MAX - SCHED_PRIORITY_DEFAULT) / 2;
				break;
			case 3:
				task_priority = SCHED_PRIORITY_MAX;
				break;
			default:
				// use default priority already set above
				break;
		}

		main_thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("sdlog2",
						 SCHED_DEFAULT,
						 task_priority,
						 3400,
						 sdlog2_thread_main,
						 (char * const *)argv);

		/* wait for the task to launch */
		unsigned const max_wait_us = 1000000;
		unsigned const max_wait_steps = 2000;

		unsigned i;
		for (i = 0; i < max_wait_steps; i++) {
			usleep(max_wait_us / max_wait_steps);
			if (thread_running) {
				break;
			}
		}

		return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			PX4_WARN("not started");
		}

		main_thread_should_exit = true;
		return 0;
	}

	if (!thread_running) {
		PX4_WARN("not started\n");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		sdlog2_status();
		return 0;
	}

	if (!strncmp(argv[1], "on", 2)) {
		struct vehicle_command_s cmd;
		cmd.command = VEHICLE_CMD_PREFLIGHT_STORAGE;
		cmd.param1 = -1;
		cmd.param2 = -1;
		cmd.param3 = 1;
		orb_advertise(ORB_ID(vehicle_command), &cmd);
		return 0;
	}

	if (!strcmp(argv[1], "off")) {
		struct vehicle_command_s cmd;
		cmd.command = VEHICLE_CMD_PREFLIGHT_STORAGE;
		cmd.param1 = -1;
		cmd.param2 = -1;
		cmd.param3 = 2;
		orb_advertise(ORB_ID(vehicle_command), &cmd);
		return 0;
	}

	sdlog2_usage("unrecognized command");
	return 1;
}

bool get_log_time_tt(struct tm *tt, bool boot_time) {
	struct timespec ts;
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	/* use RTC time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.px4log */
	time_t utc_time_sec = 0;

	if (_gpstime_only && has_gps_3d_fix) {
		utc_time_sec = gps_time_sec;
	} else {
		utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);
	}

	if (utc_time_sec > PX4_EPOCH_SECS) {
		/* strip the time elapsed since boot */
		if (boot_time) {
			utc_time_sec -= hrt_absolute_time() / 1e6;
		}

		/* apply utc offset (min, not hour) */
		utc_time_sec += _utc_offset*60;

		struct tm *ttp = gmtime_r(&utc_time_sec, tt);
		return (ttp != NULL);
	} else {
		return false;
	}
}

int create_log_dir()
{
	/* create dir on sdcard if needed */
	uint16_t dir_number = 1; // start with dir sess001
	int mkdir_ret;

	struct tm tt;
	bool time_ok = get_log_time_tt(&tt, true);

	if (log_name_timestamp && time_ok) {
		int n = snprintf(log_dir, sizeof(log_dir), "%s/", log_root);
		if (n >= sizeof(log_dir)) {
			PX4_ERR("log path too long");
			return -1;
		}
		strftime(log_dir + n, sizeof(log_dir) - n, "%Y-%m-%d", &tt);
		mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if ((mkdir_ret != OK) && (errno != EEXIST)) {
			warn("failed creating new dir: %s", log_dir);
			return -1;
		}

	} else {
		/* Look for the next dir that does not exist.
		 * However, if we've already crated a sessXXX folder in this session
		 * let's re-use it. */
		while (dir_number <= MAX_NO_LOGFOLDER && !sess_folder_created) {
			/* format log dir: e.g. /fs/microsd/sess001 */
			int n = snprintf(log_dir, sizeof(log_dir), "%s/sess%03u", log_root, dir_number);
			if (n >= sizeof(log_dir)) {
				PX4_ERR("log path too long");
				return -1;
			}

			mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

			if (mkdir_ret == 0) {
				sess_folder_created = true;
				break;

			} else if (errno != EEXIST) {
				warn("failed creating new dir: %s", log_dir);
				return -1;
			}

			/* dir exists already */
			dir_number++;
		}

		if (dir_number >= MAX_NO_LOGFOLDER) {
			/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
			PX4_WARN("all %d possible dirs exist already", MAX_NO_LOGFOLDER);
			return -1;
		}
	}

	/* print logging path, important to find log file later */
	mavlink_and_console_log_info(&mavlink_log_pub, "[blackbox] %s", log_dir);

	return 0;
}

int open_log_file()
{
	/* string to hold the path to the log */
	char log_file_name[64] = "";
	char log_file_path[sizeof(log_file_name) + LOG_BASE_PATH_LEN] = "";

	struct tm tt;
	bool time_ok = get_log_time_tt(&tt, false);

	/* start logging if we have a valid time and the time is not in the past */
	if (log_name_timestamp && time_ok) {
		strftime(log_file_name, sizeof(log_file_name), "%H_%M_%S.px4log", &tt);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

	} else {
		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.px4log */
			snprintf(log_file_name, sizeof(log_file_name), "log%03u.px4log", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			mavlink_log_critical(&mavlink_log_pub, "[blackbox] ERR: max files %d", MAX_NO_LOGFILE);
			return -1;
		}
	}

#ifdef __PX4_NUTTX
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);
#else
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC, PX4_O_MODE_666);
#endif

	if (fd < 0) {
		mavlink_log_critical(&mavlink_log_pub, "[blackbox] failed: %s", log_file_name);

	} else {
		mavlink_and_console_log_info(&mavlink_log_pub, "[blackbox] recording: %s", log_file_name);
	}

	return fd;
}

int open_perf_file(const char* str)
{
	/* string to hold the path to the log */
	char log_file_name[64] = "";
	char log_file_path[sizeof(log_file_name) + LOG_BASE_PATH_LEN] = "";

	struct tm tt;
	bool time_ok = get_log_time_tt(&tt, false);

	if (log_name_timestamp && time_ok) {
		strftime(log_file_name, sizeof(log_file_name), "perf%H_%M_%S.txt", &tt);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s_%s", log_dir, str, log_file_name);

	} else {
		unsigned file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.txt */
			snprintf(log_file_name, sizeof(log_file_name), "perf%03u.txt", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s_%s", log_dir, str, log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			mavlink_log_critical(&mavlink_log_pub, "[blackbox] ERR: max files %d", MAX_NO_LOGFILE);
			return -1;
		}
	}

#ifdef __PX4_NUTTX
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);
#else
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC, 0666);
#endif

	if (fd < 0) {
		mavlink_log_critical(&mavlink_log_pub, "[blackbox] failed: %s", log_file_name);

	}

	return fd;
}

static void *logwriter_thread(void *arg)
{
	/* set name */
	px4_prctl(PR_SET_NAME, "sdlog2_writer", 0);

	int log_fd = open_log_file();

	if (log_fd < 0) {
		return NULL;
	}

	struct logbuffer_s *logbuf = (struct logbuffer_s *)arg;

	/* write log messages formats, version and parameters */
	log_bytes_written += write_formats(log_fd);

	log_bytes_written += write_version(log_fd);

	log_bytes_written += write_parameters(log_fd);

	fsync(log_fd);

	int poll_count = 0;

	void *read_ptr;

	int n = 0;

	bool should_wait = false;

	bool is_part = false;

	while (true) {
		/* make sure threads are synchronized */
		pthread_mutex_lock(&logbuffer_mutex);

		/* update read pointer if needed */
		if (n > 0) {
			logbuffer_mark_read(&lb, n);
		}

		/* only wait if no data is available to process */
		if (should_wait && !logwriter_should_exit) {
			/* blocking wait for new data at this line */
			pthread_cond_wait(&logbuffer_cond, &logbuffer_mutex);
		}

		/* only get pointer to thread-safe data, do heavy I/O a few lines down */
		int available = logbuffer_get_ptr(logbuf, &read_ptr, &is_part);

		/* continue */
		pthread_mutex_unlock(&logbuffer_mutex);

		if (available > 0) {

			/* do heavy IO here */
			if (available > MAX_WRITE_CHUNK) {
				n = MAX_WRITE_CHUNK;

			} else {
				n = available;
			}

			perf_begin(perf_write);
			n = write(log_fd, read_ptr, n);
			perf_end(perf_write);

			should_wait = (n == available) && !is_part;

			if (n < 0) {
				main_thread_should_exit = true;
				warn("error writing log file");
				break;
			}

			if (n > 0) {
				log_bytes_written += n;
			}

		} else {
			n = 0;

			/* exit only with empty buffer */
			if (main_thread_should_exit || logwriter_should_exit) {
				break;
			}

			should_wait = true;
		}

		if (++poll_count == 10) {
			fsync(log_fd);
			poll_count = 0;

		}

		if (log_bytes_written - last_checked_bytes_written > 20*1024*1024) {
			/* check if space is available, if not stop everything */
			if (check_free_space() != OK) {
				logwriter_should_exit = true;
				main_thread_should_exit = true;
			}
			last_checked_bytes_written = log_bytes_written;
		}
	}

	fsync(log_fd);
	close(log_fd);

	return NULL;
}

void sdlog2_start_log()
{
	if (logging_enabled) {
		return;
	}

	/* create log dir if needed */
	if (create_log_dir() != 0) {
		mavlink_log_critical(&mavlink_log_pub, "[blackbox] error creating log dir");
		return;
	}

	/* initialize statistics counter */
	log_bytes_written = 0;
	start_time = hrt_absolute_time();
	log_msgs_written = 0;
	log_msgs_skipped = 0;

	/* initialize log buffer emptying thread */
	pthread_attr_init(&logwriter_attr);

#if !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)
	struct sched_param param;
	(void)pthread_attr_getschedparam(&logwriter_attr, &param);
	/* low priority, as this is expensive disk I/O. */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 5;
	if (pthread_attr_setschedparam(&logwriter_attr, &param)) {
		PX4_WARN("sdlog2: failed setting sched params");
	}
#endif

	pthread_attr_setstacksize(&logwriter_attr, PX4_STACK_ADJUSTED(2048));

	logwriter_should_exit = false;

	/* allocate write performance counter */
	perf_write = perf_alloc(PC_ELAPSED, "sd write");

	/* start log buffer emptying thread */
	if (0 != pthread_create(&logwriter_pthread, &logwriter_attr, logwriter_thread, &lb)) {
		PX4_WARN("error creating logwriter thread");
	}

	/* write all performance counters */
	hrt_abstime curr_time = hrt_absolute_time();
	struct print_load_s load;
	int perf_fd = open_perf_file("preflight");
	init_print_load_s(curr_time, &load);
	print_load(curr_time, perf_fd, &load);
	dprintf(perf_fd, "PERFORMANCE COUNTERS PRE-FLIGHT\n\n");
	perf_print_all(perf_fd);
	dprintf(perf_fd, "\nLOAD PRE-FLIGHT\n\n");
	usleep(500 * 1000);
	print_load(hrt_absolute_time(), perf_fd, &load);
	close(perf_fd);

	/* reset performance counters to get in-flight min and max values in post flight log */
	perf_reset_all();

	logging_enabled = true;
}

void sdlog2_stop_log()
{
	if (!logging_enabled) {
		return;
	}

	/* disabling the logging will trigger the skipped count to increase,
	 * so we take a local copy before interrupting the disk I/O.
	 */
	unsigned long skipped_count = log_msgs_skipped;

	logging_enabled = false;

	/* wake up write thread one last time */
	pthread_mutex_lock(&logbuffer_mutex);
	logwriter_should_exit = true;
	pthread_cond_signal(&logbuffer_cond);
	/* unlock, now the writer thread may return */
	pthread_mutex_unlock(&logbuffer_mutex);

	/* wait for write thread to return */
	int ret;

	if ((ret = pthread_join(logwriter_pthread, NULL)) != 0) {
		PX4_WARN("error joining logwriter thread: %i", ret);
	}

	logwriter_pthread = 0;
	pthread_attr_destroy(&logwriter_attr);

	/* write all performance counters */
	int perf_fd = open_perf_file("postflight");
	hrt_abstime curr_time = hrt_absolute_time();
	dprintf(perf_fd, "PERFORMANCE COUNTERS POST-FLIGHT\n\n");
	perf_print_all(perf_fd);
	struct print_load_s load;
	dprintf(perf_fd, "\nLOAD POST-FLIGHT\n\n");
	init_print_load_s(curr_time, &load);
	print_load(curr_time, perf_fd, &load);
	sleep(1);
	print_load(hrt_absolute_time(), perf_fd, &load);
	close(perf_fd);

	/* free log writer performance counter */
	perf_free(perf_write);

	/* reset the logbuffer */
	logbuffer_reset(&lb);

	mavlink_and_console_log_info(&mavlink_log_pub, "[blackbox] stopped (%lu drops)", skipped_count);

	sdlog2_status();
}

int write_formats(int fd)
{
	/* construct message format packet */
	struct {
		LOG_PACKET_HEADER;
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	int written = 0;

	/* fill message format packet for each format and write it */
	for (unsigned i = 0; i < log_formats_num; i++) {
		log_msg_format.body = log_formats[i];
		written += write(fd, &log_msg_format, sizeof(log_msg_format));
	}

	return written;
}

int write_version(int fd)
{
	/* construct version message */
	struct {
		LOG_PACKET_HEADER;
		struct log_VER_s body;
	} log_msg_VER = {
		LOG_PACKET_HEADER_INIT(LOG_VER_MSG),
	};

	/* fill version message and write it */
	strncpy(log_msg_VER.body.fw_git, px4_firmware_version_string(), sizeof(log_msg_VER.body.fw_git));
	strncpy(log_msg_VER.body.arch, px4_board_name(), sizeof(log_msg_VER.body.arch));
	log_msg_VER.body.arch[sizeof(log_msg_VER.body.arch) - 1] = '\0';
	return write(fd, &log_msg_VER, sizeof(log_msg_VER));
}

int write_parameters(int fd)
{
	/* construct parameter message */
	struct {
		LOG_PACKET_HEADER;
		struct log_PARM_s body;
	} log_msg_PARM = {
		LOG_PACKET_HEADER_INIT(LOG_PARM_MSG),
	};

	int written = 0;
	param_t params_cnt = param_count();

	for (param_t param = 0; param < params_cnt; param++) {
		/* fill parameter message and write it */
		strncpy(log_msg_PARM.body.name, param_name(param), sizeof(log_msg_PARM.body.name));
		log_msg_PARM.body.name[sizeof(log_msg_PARM.body.name) - 1] = '\0';
		float value = NAN;

		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				int32_t i;
				param_get(param, &i);
				value = i;	// cast integer to float
				break;
			}

		case PARAM_TYPE_FLOAT:
			param_get(param, &value);
			break;

		default:
			break;
		}

		log_msg_PARM.body.value = value;
		written += write(fd, &log_msg_PARM, sizeof(log_msg_PARM));
	}

	return written;
}

bool copy_if_updated(orb_id_t topic, int *handle, void *buffer)
{
	return copy_if_updated_multi(topic, 0, handle, buffer);
}

bool copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer)
{
	bool updated = false;

	if (*handle < 0) {
		if (OK == orb_exists(topic, multi_instance))
		{
			*handle = orb_subscribe_multi(topic, multi_instance);
			/* copy first data */
			if (*handle >= 0) {

				/* but only if it has really been updated */
				orb_check(*handle, &updated);

				if (updated) {
					orb_copy(topic, *handle, buffer);
				}
			}
		}
	} else {
		orb_check(*handle, &updated);

		if (updated) {
			orb_copy(topic, *handle, buffer);
		}
	}

	return updated;
}

int sdlog2_thread_main(int argc, char *argv[])
{
	/* default log rate: 50 Hz */
	int32_t log_rate = 50;
	int log_buffer_size = LOG_BUFFER_SIZE_DEFAULT;
	logging_enabled = false;
	/* enable logging on start (-e option) */
	bool log_on_start = false;
	/* enable logging when armed (-a option) */
	bool log_when_armed = false;
	log_name_timestamp = false;

	flag_system_armed = false;

#ifdef __PX4_NUTTX
	/* the NuttX optarg handler does not
	 * ignore argv[0] like the POSIX handler
	 * does, nor does it deal with non-flag
	 * verbs well. So we Remove the application
	 * name and the verb.
	 */
	argc -= 2;
	argv += 2;
#endif

	int ch;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int myoptind = 1;
	const char *myoptarg = NULL;
	while ((ch = px4_getopt(argc, argv, "r:b:eatx", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, NULL, 10);

				if (r <= 0) {
					r = 1;
				}

				log_rate = r;
			}
			break;

		case 'b': {
				unsigned long s = strtoul(myoptarg, NULL, 10);

				if (s < 1) {
					s = 1;
				}

				log_buffer_size = 1024 * s;
			}
			break;

		case 'e':
			log_on_start = true;
			log_when_armed = true;
			break;

		case 'a':
			log_when_armed = true;
			break;

		case 't':
			log_name_timestamp = true;
			break;

		case 'x':
			_extended_logging = true;
			break;

		case '?':
			if (optopt == 'c') {
				PX4_WARN("option -%c requires an argument", optopt);

			} else if (isprint(optopt)) {
				PX4_WARN("unknown option `-%c'", optopt);

			} else {
				PX4_WARN("unknown option character `\\x%x'", optopt);
			}
			err_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		sdlog2_usage(NULL);
	}

	gps_time_sec = 0;

	/* interpret logging params */
	int32_t param_log_rate = -1;
	param_t log_rate_ph = param_find("SDLOG_RATE");

	if (log_rate_ph != PARAM_INVALID) {
		param_get(log_rate_ph, &param_log_rate);

		if (param_log_rate > 0) {

			/* we can't do more than ~ 500 Hz, even with a massive buffer */
			if (param_log_rate > 250) {
				param_log_rate = 250;
			}

		} else if (param_log_rate == 0) {
			/* we need at minimum 10 Hz to be able to see anything */
			param_log_rate = 10;
		}
	}

	// if parameter was provided use it, if not use command line argument
	log_rate = param_log_rate > -1 ? param_log_rate : log_rate;

	param_t log_ext_ph = param_find("SDLOG_EXT");

	if (log_ext_ph != PARAM_INVALID) {

		int32_t param_log_extended;
		param_get(log_ext_ph, &param_log_extended);

		if (param_log_extended > 0) {
			_extended_logging = true;
		} else if (param_log_extended == 0) {
			_extended_logging = false;
		}
		/* any other value means to ignore the parameter, so no else case */

	}

	param_t log_gpstime_ph = param_find("SDLOG_GPSTIME");

	if (log_gpstime_ph != PARAM_INVALID) {

		int32_t param_log_gpstime;
		param_get(log_gpstime_ph, &param_log_gpstime);

		if (param_log_gpstime > 0) {
			_gpstime_only = true;
		} else if (param_log_gpstime == 0) {
			_gpstime_only = false;
		}
		/* any other value means to ignore the parameter, so no else case */

	}

	param_t log_utc_offset = param_find("SDLOG_UTC_OFFSET");

	if ( log_utc_offset != PARAM_INVALID ) {
	    int32_t param_utc_offset;
	    param_get(log_utc_offset, &param_utc_offset);
	    _utc_offset = param_utc_offset;
	}

	if (check_free_space() != OK) {
		return 1;
	}


	/* create log root dir */
	int mkdir_ret = mkdir(log_root, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret != 0 && errno != EEXIST) {
		warn("ERR: failed creating log dir: %s", log_root);
		return 1;
	}

	/* initialize log buffer with specified size */
	PX4_DEBUG("log buffer size: %i bytes", log_buffer_size);

	if (OK != logbuffer_init(&lb, log_buffer_size)) {
		PX4_WARN("can't allocate log buffer, exiting");
		return 1;
	}

	struct vehicle_status_s buf_status;
	memset(&buf_status, 0, sizeof(buf_status));

	struct vehicle_gps_position_s buf_gps_pos;
	memset(&buf_gps_pos, 0, sizeof(buf_gps_pos));

	struct vehicle_command_s buf_cmd;
	memset(&buf_cmd, 0, sizeof(buf_cmd));

	struct commander_state_s buf_commander_state;
	memset(&buf_commander_state, 0, sizeof(buf_commander_state));

	/* There are different log types possible on different platforms. */
	enum {
		LOG_TYPE_NORMAL,
		LOG_TYPE_REPLAY_ONLY,
		LOG_TYPE_ALL
	} log_type;

	/* Check if we are gathering data for a replay log for ekf2. */
	param_t replay_handle = param_find("EKF2_REC_RPL");
	int32_t tmp = 0;
	param_get(replay_handle, &tmp);
	bool record_replay_log = (bool)tmp;

	if (record_replay_log) {
#if defined(__PX4_QURT) || defined(__PX4_POSIX)
		log_type = LOG_TYPE_ALL;
#else
		log_type = LOG_TYPE_REPLAY_ONLY;
#endif
	} else {
		log_type = LOG_TYPE_NORMAL;
	}

	/* warning! using union here to save memory, elements should be used separately! */
	union {
		struct vehicle_command_s cmd;
		struct sensor_combined_s sensor;
		struct vehicle_attitude_s att;
		struct vehicle_attitude_setpoint_s att_sp;
		struct vehicle_rates_setpoint_s rates_sp;
		struct actuator_outputs_s act_outputs;
		struct actuator_controls_s act_controls;
		struct actuator_controls_s act_controls1;
		struct vehicle_local_position_s local_pos;
		struct vehicle_local_position_setpoint_s local_pos_sp;
		struct vehicle_global_position_s global_pos;
		struct position_setpoint_triplet_s triplet;
		struct att_pos_mocap_s att_pos_mocap;
		struct vehicle_local_position_s vision_pos;
		struct vehicle_attitude_s vision_att;
		struct optical_flow_s flow;
		struct rc_channels_s rc;
		struct differential_pressure_s diff_pres;
		struct airspeed_s airspeed;
		struct esc_status_s esc;
		struct vehicle_global_velocity_setpoint_s global_vel_sp;
		struct battery_status_s battery;
		struct telemetry_status_s telemetry;
		struct distance_sensor_s distance_sensor;
		struct estimator_status_s estimator_status;
		struct tecs_status_s tecs_status;
		struct system_power_s system_power;
		struct servorail_status_s servorail_status;
		struct satellite_info_s sat_info;
		struct wind_estimate_s wind_estimate;
		struct vtol_vehicle_status_s vtol_status;
		struct time_offset_s time_offset;
		struct mc_att_ctrl_status_s mc_att_ctrl_status;
		struct control_state_s ctrl_state;
		struct ekf2_innovations_s innovations;
		struct camera_trigger_s camera_trigger;
		struct ekf2_replay_s replay;
		struct vehicle_land_detected_s land_detected;
		struct cpuload_s cpuload;
		struct vehicle_gps_position_s dual_gps_pos;
		struct task_stack_info_s task_stack_info;
	} buf;

	memset(&buf, 0, sizeof(buf));

	/* log message buffer: header + body */
#pragma pack(push, 1)
	struct {
		LOG_PACKET_HEADER;
		union {
			struct log_TIME_s log_TIME;
			struct log_ATT_s log_ATT;
			struct log_ATSP_s log_ATSP;
			struct log_IMU_s log_IMU;
			struct log_SENS_s log_SENS;
			struct log_LPOS_s log_LPOS;
			struct log_LPSP_s log_LPSP;
			struct log_GPS_s log_GPS;
			struct log_ATTC_s log_ATTC;
			struct log_STAT_s log_STAT;
			struct log_VTOL_s log_VTOL;
			struct log_RC_s log_RC;
			struct log_OUT_s log_OUT;
			struct log_AIRS_s log_AIRS;
			struct log_ARSP_s log_ARSP;
			struct log_FLOW_s log_FLOW;
			struct log_GPOS_s log_GPOS;
			struct log_GPSP_s log_GPSP;
			struct log_ESC_s log_ESC;
			struct log_GVSP_s log_GVSP;
			struct log_BATT_s log_BATT;
			struct log_DIST_s log_DIST;
			struct log_TEL_s log_TEL;
			struct log_EST0_s log_EST0;
			struct log_EST1_s log_EST1;
			struct log_EST2_s log_EST2;
			struct log_EST3_s log_EST3;
			struct log_PWR_s log_PWR;
			struct log_MOCP_s log_MOCP;
			struct log_VISN_s log_VISN;
			struct log_GS0A_s log_GS0A;
			struct log_GS0B_s log_GS0B;
			struct log_GS1A_s log_GS1A;
			struct log_GS1B_s log_GS1B;
			struct log_TECS_s log_TECS;
			struct log_WIND_s log_WIND;
			struct log_ENCD_s log_ENCD;
			struct log_TSYN_s log_TSYN;
			struct log_MACS_s log_MACS;
			struct log_CTS_s log_CTS;
			struct log_EST4_s log_INO1;
			struct log_EST5_s log_INO2;
			struct log_CAMT_s log_CAMT;
			struct log_RPL1_s log_RPL1;
			struct log_RPL2_s log_RPL2;
			struct log_EST6_s log_INO3;
			struct log_RPL3_s log_RPL3;
			struct log_RPL4_s log_RPL4;
			struct log_RPL5_s log_RPL5;
			struct log_LAND_s log_LAND;
			struct log_RPL6_s log_RPL6;
			struct log_LOAD_s log_LOAD;
			struct log_DPRS_s log_DPRS;
			struct log_STCK_s log_STCK;
		} body;
	} log_msg = {
		LOG_PACKET_HEADER_INIT(0)
	};
#pragma pack(pop)
	memset(&log_msg.body, 0, sizeof(log_msg.body));

	struct {
		int cmd_sub;
		int status_sub;
		int vtol_status_sub;
		int sensor_sub;
		int att_sub;
		int att_sp_sub;
		int rates_sp_sub;
		int act_outputs_sub;
		int act_outputs_1_sub;
		int act_controls_sub;
		int act_controls_1_sub;
		int local_pos_sub;
		int local_pos_sp_sub;
		int global_pos_sub;
		int triplet_sub;
		int gps_pos_sub[2];
		int sat_info_sub;
		int att_pos_mocap_sub;
		int vision_pos_sub;
		int vision_att_sub;
		int flow_sub;
		int rc_sub;
		int airspeed_sub;
		int esc_sub;
		int global_vel_sp_sub;
		int battery_sub;
		int telemetry_subs[ORB_MULTI_MAX_INSTANCES];
		int distance_sensor_sub;
		int estimator_status_sub;
		int tecs_status_sub;
		int system_power_sub;
		int servorail_status_sub;
		int wind_sub;
		int tsync_sub;
		int mc_att_ctrl_status_sub;
		int ctrl_state_sub;
		int innov_sub;
		int cam_trig_sub;
		int replay_sub;
		int land_detected_sub;
		int commander_state_sub;
		int cpuload_sub;
		int diff_pres_sub;
		int task_stack_info_sub;
	} subs;

	subs.cmd_sub = -1;
	subs.status_sub = -1;
	subs.vtol_status_sub = -1;
	subs.gps_pos_sub[0] = -1;
	subs.gps_pos_sub[1] = -1;
	subs.sensor_sub = -1;
	subs.att_sub = -1;
	subs.att_sp_sub = -1;
	subs.rates_sp_sub = -1;
	subs.act_outputs_sub = -1;
	subs.act_outputs_1_sub = -1;
	subs.act_controls_sub = -1;
	subs.act_controls_1_sub = -1;
	subs.local_pos_sub = -1;
	subs.local_pos_sp_sub = -1;
	subs.global_pos_sub = -1;
	subs.triplet_sub = -1;
	subs.att_pos_mocap_sub = -1;
	subs.vision_pos_sub = -1;
	subs.vision_att_sub = -1;
	subs.flow_sub = -1;
	subs.rc_sub = -1;
	subs.airspeed_sub = -1;
	subs.esc_sub = -1;
	subs.global_vel_sp_sub = -1;
	subs.battery_sub = -1;
	subs.distance_sensor_sub = -1;
	subs.estimator_status_sub = -1;
	subs.tecs_status_sub = -1;
	subs.system_power_sub = -1;
	subs.servorail_status_sub = -1;
	subs.wind_sub = -1;
	subs.tsync_sub = -1;
	subs.mc_att_ctrl_status_sub = -1;
	subs.ctrl_state_sub = -1;
	subs.innov_sub = -1;
	subs.cam_trig_sub = -1;
	subs.replay_sub = -1;
	subs.land_detected_sub = -1;
	subs.commander_state_sub = -1;
	subs.cpuload_sub = -1;
	subs.diff_pres_sub = -1;
	subs.task_stack_info_sub = -1;

	/* add new topics HERE */


	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		subs.telemetry_subs[i] = -1;
	}

	subs.sat_info_sub = -1;

	/* initialize thread synchronization */
	pthread_mutex_init(&logbuffer_mutex, NULL);
	pthread_cond_init(&logbuffer_cond, NULL);

	/* track changes in sensor_combined topic */
	hrt_abstime gyro_timestamp = 0;
	hrt_abstime accelerometer_timestamp = 0;
	hrt_abstime magnetometer_timestamp = 0;
	hrt_abstime barometer_timestamp = 0;

	/* initialize calculated mean SNR */
	float snr_mean = 0.0f;

	/* enable logging on start if needed */
	if (log_on_start) {
		/* check GPS topic to get GPS time */
		if (log_name_timestamp) {
			if (!copy_if_updated_multi(ORB_ID(vehicle_gps_position), 0, &subs.gps_pos_sub[0], &buf_gps_pos)) {
				gps_time_sec = buf_gps_pos.time_utc_usec / 1e6;
			}
		}

		sdlog2_start_log();
	}

	/* running, report */
	thread_running = true;

	// wakeup source
	px4_pollfd_struct_t fds[2];
	unsigned px4_pollfd_len = 0;

	int poll_counter = 0;

	int poll_to_logging_factor = 1;

	switch (log_type) {
		case LOG_TYPE_ALL:
			subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
			fds[0].fd = subs.sensor_sub;
			fds[0].events = POLLIN;

			subs.replay_sub = orb_subscribe(ORB_ID(ekf2_replay));
			fds[1].fd = subs.replay_sub;
			fds[1].events = POLLIN;

			px4_pollfd_len = 2;

			poll_to_logging_factor = 1;

			break;

		case LOG_TYPE_NORMAL:

			subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
			fds[0].fd = subs.sensor_sub;
			fds[0].events = POLLIN;

			px4_pollfd_len = 1;

			// TODO Remove hardcoded rate!
			poll_to_logging_factor = 250 / (log_rate < 1 ? 1 : log_rate);

			break;

		case LOG_TYPE_REPLAY_ONLY:

			subs.replay_sub = orb_subscribe(ORB_ID(ekf2_replay));
			fds[0].fd = subs.replay_sub;
			fds[0].events = POLLIN;

			px4_pollfd_len = 1;

			poll_to_logging_factor = 1;

			break;
	}

	if (poll_to_logging_factor < 1) {
		poll_to_logging_factor = 1;
	}


	while (!main_thread_should_exit) {

		/* Check below's topics first even if logging is not enabled.
		 * We need to do this because should only poll further below if we're
		 * actually going to orb_copy the data after the poll. */

		/* --- VEHICLE COMMAND - LOG MANAGEMENT --- */
		if (copy_if_updated(ORB_ID(vehicle_command), &subs.cmd_sub, &buf_cmd)) {
			handle_command(&buf_cmd);
		}

		/* --- VEHICLE STATUS - LOG MANAGEMENT --- */
		bool status_updated = copy_if_updated(ORB_ID(vehicle_status), &subs.status_sub, &buf_status);

		if (status_updated) {
			if (log_when_armed) {
				handle_status(&buf_status);
			}
		}

		/* --- GPS POSITION - LOG MANAGEMENT --- */
		bool gps_pos_updated = copy_if_updated_multi(ORB_ID(vehicle_gps_position), 0, &subs.gps_pos_sub[0], &buf_gps_pos);

		if (gps_pos_updated && log_name_timestamp) {
			gps_time_sec = buf_gps_pos.time_utc_usec / 1e6;
			has_gps_3d_fix = buf_gps_pos.fix_type == 3;
		}

		if (!logging_enabled) {
			usleep(50000);
			continue;
		}

		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], px4_pollfd_len, 100);

		// timed out - periodic check for _task_should_exit
		if (pret == 0) {
			continue;
		}

		// this is undesirable but not much we can do - might want to flag unhappy status
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if ((poll_counter+1) >= poll_to_logging_factor) {
			poll_counter = 0;
		} else {

			/* In this case, we still need to do orb_copy, otherwise we'll stall. */
			switch (log_type) {
				case LOG_TYPE_ALL:
					if (fds[0].revents & POLLIN) {
						orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.sensor);
					}

					if (fds[1].revents & POLLIN) {
						orb_copy(ORB_ID(ekf2_replay), subs.replay_sub, &buf.replay);
					}
					break;

				case LOG_TYPE_NORMAL:
					if (fds[0].revents & POLLIN) {
						orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.sensor);
					}
					break;

				case LOG_TYPE_REPLAY_ONLY:
					if (fds[0].revents & POLLIN) {
						orb_copy(ORB_ID(ekf2_replay), subs.replay_sub, &buf.replay);
					}
					break;
			}
			poll_counter++;
			continue;
		}

		/* write time stamp message */
		log_msg.msg_type = LOG_TIME_MSG;
		log_msg.body.log_TIME.t = hrt_absolute_time();
		LOGBUFFER_WRITE_AND_COUNT(TIME);

		/* --- COMMANDER INTERNAL STATE --- */
		copy_if_updated(ORB_ID(commander_state), &subs.commander_state_sub,
				&buf_commander_state);

		/* --- VEHICLE STATUS --- */
		if (status_updated) {
			log_msg.msg_type = LOG_STAT_MSG;
			log_msg.body.log_STAT.main_state = buf_commander_state.main_state;
			log_msg.body.log_STAT.nav_state = buf_status.nav_state;
			log_msg.body.log_STAT.arming_state = buf_status.arming_state;
			log_msg.body.log_STAT.failsafe = (uint8_t) buf_status.failsafe;
			log_msg.body.log_STAT.is_rot_wing = (uint8_t)buf_status.is_rotary_wing;
			LOGBUFFER_WRITE_AND_COUNT(STAT);
		}

		/* --- EKF2 REPLAY --- */
		if (log_type == LOG_TYPE_ALL || log_type == LOG_TYPE_REPLAY_ONLY) {

			bool replay_updated = false;

			if (log_type == LOG_TYPE_ALL) {

				if (fds[1].revents & POLLIN) {
					orb_copy(ORB_ID(ekf2_replay), subs.replay_sub, &buf.replay);
					replay_updated = true;
				}

			} else if (log_type == LOG_TYPE_REPLAY_ONLY) {
				if (fds[0].revents & POLLIN) {
					orb_copy(ORB_ID(ekf2_replay), subs.replay_sub, &buf.replay);
					replay_updated = true;
				}
			}

			if (replay_updated) {
				log_msg.msg_type = LOG_RPL1_MSG;
				log_msg.body.log_RPL1.time_ref = buf.replay.timestamp;
				log_msg.body.log_RPL1.gyro_integral_dt = buf.replay.gyro_integral_dt;
				log_msg.body.log_RPL1.accelerometer_integral_dt = buf.replay.accelerometer_integral_dt;
				log_msg.body.log_RPL1.magnetometer_timestamp = buf.replay.magnetometer_timestamp;
				log_msg.body.log_RPL1.baro_timestamp = buf.replay.baro_timestamp;
				log_msg.body.log_RPL1.gyro_x_rad = buf.replay.gyro_rad[0];
				log_msg.body.log_RPL1.gyro_y_rad = buf.replay.gyro_rad[1];
				log_msg.body.log_RPL1.gyro_z_rad = buf.replay.gyro_rad[2];
				log_msg.body.log_RPL1.accelerometer_x_m_s2 = buf.replay.accelerometer_m_s2[0];
				log_msg.body.log_RPL1.accelerometer_y_m_s2 = buf.replay.accelerometer_m_s2[1];
				log_msg.body.log_RPL1.accelerometer_z_m_s2 = buf.replay.accelerometer_m_s2[2];
				log_msg.body.log_RPL1.magnetometer_x_ga = buf.replay.magnetometer_ga[0];
				log_msg.body.log_RPL1.magnetometer_y_ga = buf.replay.magnetometer_ga[1];
				log_msg.body.log_RPL1.magnetometer_z_ga = buf.replay.magnetometer_ga[2];
				log_msg.body.log_RPL1.baro_alt_meter = buf.replay.baro_alt_meter;
				LOGBUFFER_WRITE_AND_COUNT(RPL1);

				// only log the gps replay data if it actually updated
				if (buf.replay.time_usec > 0) {
					log_msg.msg_type = LOG_RPL2_MSG;
					log_msg.body.log_RPL2.time_pos_usec = buf.replay.time_usec;
					log_msg.body.log_RPL2.time_vel_usec = buf.replay.time_usec;
					log_msg.body.log_RPL2.lat = buf.replay.lat;
					log_msg.body.log_RPL2.lon = buf.replay.lon;
					log_msg.body.log_RPL2.alt = buf.replay.alt;
					log_msg.body.log_RPL2.fix_type = buf.replay.fix_type;
					log_msg.body.log_RPL2.nsats = buf.replay.nsats;
					log_msg.body.log_RPL2.eph = buf.replay.eph;
					log_msg.body.log_RPL2.epv = buf.replay.epv;
					log_msg.body.log_RPL2.sacc = buf.replay.sacc;
					log_msg.body.log_RPL2.vel_m_s = buf.replay.vel_m_s;
					log_msg.body.log_RPL2.vel_n_m_s = buf.replay.vel_n_m_s;
					log_msg.body.log_RPL2.vel_e_m_s = buf.replay.vel_e_m_s;
					log_msg.body.log_RPL2.vel_d_m_s = buf.replay.vel_d_m_s;
					log_msg.body.log_RPL2.vel_ned_valid = buf.replay.vel_ned_valid;
					LOGBUFFER_WRITE_AND_COUNT(RPL2);
				}

				if (buf.replay.flow_timestamp > 0) {
					log_msg.msg_type = LOG_RPL3_MSG;
					log_msg.body.log_RPL3.time_flow_usec = buf.replay.flow_timestamp;
					log_msg.body.log_RPL3.flow_integral_x = buf.replay.flow_pixel_integral[0];
					log_msg.body.log_RPL3.flow_integral_y = buf.replay.flow_pixel_integral[1];
					log_msg.body.log_RPL3.gyro_integral_x = buf.replay.flow_gyro_integral[0];
					log_msg.body.log_RPL3.gyro_integral_y = buf.replay.flow_gyro_integral[1];
					log_msg.body.log_RPL3.flow_time_integral = buf.replay.flow_time_integral;
					log_msg.body.log_RPL3.flow_quality = buf.replay.flow_quality;
					LOGBUFFER_WRITE_AND_COUNT(RPL3);
				}

				if (buf.replay.rng_timestamp > 0) {
					log_msg.msg_type = LOG_RPL4_MSG;
					log_msg.body.log_RPL4.time_rng_usec = buf.replay.rng_timestamp;
					log_msg.body.log_RPL4.range_to_ground = buf.replay.range_to_ground;
					LOGBUFFER_WRITE_AND_COUNT(RPL4);
				}

				if (buf.replay.asp_timestamp > 0) {
					log_msg.msg_type = LOG_RPL6_MSG;
					log_msg.body.log_RPL6.time_airs_usec = buf.replay.asp_timestamp;
					log_msg.body.log_RPL6.indicated_airspeed_m_s = buf.replay.indicated_airspeed_m_s;
					log_msg.body.log_RPL6.true_airspeed_m_s = buf.replay.true_airspeed_m_s;;
					LOGBUFFER_WRITE_AND_COUNT(RPL6);
				}

				if (buf.replay.ev_timestamp > 0) {
					log_msg.msg_type = LOG_RPL5_MSG;
					log_msg.body.log_RPL5.time_ev_usec = buf.replay.ev_timestamp;
					log_msg.body.log_RPL5.x = buf.replay.pos_ev[0];
					log_msg.body.log_RPL5.y = buf.replay.pos_ev[1];
					log_msg.body.log_RPL5.z = buf.replay.pos_ev[2];
					log_msg.body.log_RPL5.q0 = buf.replay.quat_ev[0];
					log_msg.body.log_RPL5.q1 = buf.replay.quat_ev[1];
					log_msg.body.log_RPL5.q2 = buf.replay.quat_ev[2];
					log_msg.body.log_RPL5.q3 = buf.replay.quat_ev[3];
					log_msg.body.log_RPL5.pos_err = buf.replay.pos_err;
					log_msg.body.log_RPL5.ang_err = buf.replay.ang_err;
					LOGBUFFER_WRITE_AND_COUNT(RPL5);
				}
			}
		}

		if (log_type == LOG_TYPE_ALL || log_type == LOG_TYPE_NORMAL) {

			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.sensor);

				bool write_IMU = false;
				bool write_SENS = false;

				if (buf.sensor.timestamp != gyro_timestamp) {
					gyro_timestamp = buf.sensor.timestamp;
					write_IMU = true;
				}

				if (buf.sensor.timestamp + buf.sensor.accelerometer_timestamp_relative != accelerometer_timestamp) {
					accelerometer_timestamp = buf.sensor.timestamp + buf.sensor.accelerometer_timestamp_relative;
					write_IMU = true;
				}

				if (buf.sensor.timestamp + buf.sensor.magnetometer_timestamp_relative != magnetometer_timestamp) {
					magnetometer_timestamp = buf.sensor.timestamp + buf.sensor.magnetometer_timestamp_relative;
					write_IMU = true;
				}

				if (buf.sensor.timestamp + buf.sensor.baro_timestamp_relative != barometer_timestamp) {
					barometer_timestamp = buf.sensor.timestamp + buf.sensor.baro_timestamp_relative;
					write_SENS = true;
				}

				if (write_IMU) {
					log_msg.msg_type = LOG_IMU_MSG;

					log_msg.body.log_IMU.gyro_x = buf.sensor.gyro_rad[0];
					log_msg.body.log_IMU.gyro_y = buf.sensor.gyro_rad[1];
					log_msg.body.log_IMU.gyro_z = buf.sensor.gyro_rad[2];
					log_msg.body.log_IMU.acc_x = buf.sensor.accelerometer_m_s2[0];
					log_msg.body.log_IMU.acc_y = buf.sensor.accelerometer_m_s2[1];
					log_msg.body.log_IMU.acc_z = buf.sensor.accelerometer_m_s2[2];
					log_msg.body.log_IMU.mag_x = buf.sensor.magnetometer_ga[0];
					log_msg.body.log_IMU.mag_y = buf.sensor.magnetometer_ga[1];
					log_msg.body.log_IMU.mag_z = buf.sensor.magnetometer_ga[2];
					log_msg.body.log_IMU.temp_gyro = 0;
					log_msg.body.log_IMU.temp_acc = 0;
					log_msg.body.log_IMU.temp_mag = 0;
					LOGBUFFER_WRITE_AND_COUNT(IMU);
				}

				if (write_SENS) {
					log_msg.msg_type = LOG_SENS_MSG;

					log_msg.body.log_SENS.baro_pres = 0;
					log_msg.body.log_SENS.baro_alt = buf.sensor.baro_alt_meter;
					log_msg.body.log_SENS.baro_temp = buf.sensor.baro_temp_celcius;
					LOGBUFFER_WRITE_AND_COUNT(SENS);
				}
			}

			/* --- VTOL VEHICLE STATUS --- */
			if(copy_if_updated(ORB_ID(vtol_vehicle_status), &subs.vtol_status_sub, &buf.vtol_status)) {
				log_msg.msg_type = LOG_VTOL_MSG;
				log_msg.body.log_VTOL.airspeed_tot = buf.vtol_status.airspeed_tot;
				log_msg.body.log_VTOL.rw_mode = buf.vtol_status.vtol_in_rw_mode;
				log_msg.body.log_VTOL.trans_mode = buf.vtol_status.vtol_in_trans_mode;
				log_msg.body.log_VTOL.failsafe_mode = buf.vtol_status.vtol_transition_failsafe;
				LOGBUFFER_WRITE_AND_COUNT(VTOL);
			}

			/* --- GPS POSITION - UNIT #1 --- */
			if (gps_pos_updated) {

				log_msg.msg_type = LOG_GPS_MSG;
				log_msg.body.log_GPS.gps_time = buf_gps_pos.time_utc_usec;
				log_msg.body.log_GPS.fix_type = buf_gps_pos.fix_type;
				log_msg.body.log_GPS.eph = buf_gps_pos.eph;
				log_msg.body.log_GPS.epv = buf_gps_pos.epv;
				log_msg.body.log_GPS.lat = buf_gps_pos.lat;
				log_msg.body.log_GPS.lon = buf_gps_pos.lon;
				log_msg.body.log_GPS.alt = buf_gps_pos.alt * 0.001f;
				log_msg.body.log_GPS.vel_n = buf_gps_pos.vel_n_m_s;
				log_msg.body.log_GPS.vel_e = buf_gps_pos.vel_e_m_s;
				log_msg.body.log_GPS.vel_d = buf_gps_pos.vel_d_m_s;
				log_msg.body.log_GPS.cog = buf_gps_pos.cog_rad;
				log_msg.body.log_GPS.sats = buf_gps_pos.satellites_used;
				log_msg.body.log_GPS.snr_mean = snr_mean;
				log_msg.body.log_GPS.noise_per_ms = buf_gps_pos.noise_per_ms;
				log_msg.body.log_GPS.jamming_indicator = buf_gps_pos.jamming_indicator;
				LOGBUFFER_WRITE_AND_COUNT(GPS);
			}

			/* --- GPS POSITION - UNIT #2 --- */
			if (copy_if_updated_multi(ORB_ID(vehicle_gps_position), 1, &subs.gps_pos_sub[1], &buf.dual_gps_pos)) {
				log_msg.msg_type = LOG_DGPS_MSG;
				log_msg.body.log_GPS.gps_time = buf.dual_gps_pos.time_utc_usec;
				log_msg.body.log_GPS.fix_type = buf.dual_gps_pos.fix_type;
				log_msg.body.log_GPS.eph = buf.dual_gps_pos.eph;
				log_msg.body.log_GPS.epv = buf.dual_gps_pos.epv;
				log_msg.body.log_GPS.lat = buf.dual_gps_pos.lat;
				log_msg.body.log_GPS.lon = buf.dual_gps_pos.lon;
				log_msg.body.log_GPS.alt = buf.dual_gps_pos.alt * 0.001f;
				log_msg.body.log_GPS.vel_n = buf.dual_gps_pos.vel_n_m_s;
				log_msg.body.log_GPS.vel_e = buf.dual_gps_pos.vel_e_m_s;
				log_msg.body.log_GPS.vel_d = buf.dual_gps_pos.vel_d_m_s;
				log_msg.body.log_GPS.cog = buf.dual_gps_pos.cog_rad;
				log_msg.body.log_GPS.sats = buf.dual_gps_pos.satellites_used;
				log_msg.body.log_GPS.snr_mean = snr_mean;
				log_msg.body.log_GPS.noise_per_ms = buf.dual_gps_pos.noise_per_ms;
				log_msg.body.log_GPS.jamming_indicator = buf.dual_gps_pos.jamming_indicator;
				LOGBUFFER_WRITE_AND_COUNT(GPS);
			}

			/* --- SATELLITE INFO - UNIT #1 --- */
			if (_extended_logging) {

				if (copy_if_updated(ORB_ID(satellite_info), &subs.sat_info_sub, &buf.sat_info)) {

					/* log the SNR of each satellite for a detailed view of signal quality */
					unsigned sat_info_count = SDLOG_MIN(buf.sat_info.count, sizeof(buf.sat_info.snr) / sizeof(buf.sat_info.snr[0]));
					unsigned log_max_snr = sizeof(log_msg.body.log_GS0A.satellite_snr) / sizeof(log_msg.body.log_GS0A.satellite_snr[0]);

					log_msg.msg_type = LOG_GS0A_MSG;
					memset(&log_msg.body.log_GS0A, 0, sizeof(log_msg.body.log_GS0A));
					snr_mean = 0.0f;

					/* fill set A and calculate mean SNR */
					for (unsigned i = 0; i < sat_info_count; i++) {

						snr_mean += buf.sat_info.snr[i];

						int satindex = buf.sat_info.svid[i] - 1;

						/* handles index exceeding and wraps to to arithmetic errors */
						if ((satindex >= 0) && (satindex < (int)log_max_snr)) {
							/* map satellites by their ID so that logs from two receivers can be compared */
							log_msg.body.log_GS0A.satellite_snr[satindex] = buf.sat_info.snr[i];
						}
					}
					LOGBUFFER_WRITE_AND_COUNT(GS0A);
					snr_mean /= sat_info_count;

					log_msg.msg_type = LOG_GS0B_MSG;
					memset(&log_msg.body.log_GS0B, 0, sizeof(log_msg.body.log_GS0B));

					/* fill set B */
					for (unsigned i = 0; i < sat_info_count; i++) {

						/* get second bank of satellites, thus deduct bank size from index */
						int satindex = buf.sat_info.svid[i] - 1 - log_max_snr;

						/* handles index exceeding and wraps to to arithmetic errors */
						if ((satindex >= 0) && (satindex < (int)log_max_snr)) {
							/* map satellites by their ID so that logs from two receivers can be compared */
							log_msg.body.log_GS0B.satellite_snr[satindex] = buf.sat_info.snr[i];
						}
					}
					LOGBUFFER_WRITE_AND_COUNT(GS0B);
				}
			}

			/* --- ATTITUDE SETPOINT --- */
			if (copy_if_updated(ORB_ID(vehicle_attitude_setpoint), &subs.att_sp_sub, &buf.att_sp)) {
				log_msg.msg_type = LOG_ATSP_MSG;
				log_msg.body.log_ATSP.roll_sp = buf.att_sp.roll_body;
				log_msg.body.log_ATSP.pitch_sp = buf.att_sp.pitch_body;
				log_msg.body.log_ATSP.yaw_sp = buf.att_sp.yaw_body;
				log_msg.body.log_ATSP.thrust_sp = buf.att_sp.thrust;
				log_msg.body.log_ATSP.q_w = buf.att_sp.q_d[0];
				log_msg.body.log_ATSP.q_x = buf.att_sp.q_d[1];
				log_msg.body.log_ATSP.q_y = buf.att_sp.q_d[2];
				log_msg.body.log_ATSP.q_z = buf.att_sp.q_d[3];
				LOGBUFFER_WRITE_AND_COUNT(ATSP);
			}

			/* --- RATES SETPOINT --- */
			if (copy_if_updated(ORB_ID(vehicle_rates_setpoint), &subs.rates_sp_sub, &buf.rates_sp)) {
				log_msg.msg_type = LOG_ARSP_MSG;
				log_msg.body.log_ARSP.roll_rate_sp = buf.rates_sp.roll;
				log_msg.body.log_ARSP.pitch_rate_sp = buf.rates_sp.pitch;
				log_msg.body.log_ARSP.yaw_rate_sp = buf.rates_sp.yaw;
				LOGBUFFER_WRITE_AND_COUNT(ARSP);
			}

			/* --- ACTUATOR OUTPUTS --- */
			if (copy_if_updated_multi(ORB_ID(actuator_outputs), 0, &subs.act_outputs_sub, &buf.act_outputs)) {
				log_msg.msg_type = LOG_OUT0_MSG;
				memcpy(log_msg.body.log_OUT.output, buf.act_outputs.output, sizeof(log_msg.body.log_OUT.output));
				LOGBUFFER_WRITE_AND_COUNT(OUT);
			}

			if (copy_if_updated_multi(ORB_ID(actuator_outputs), 1, &subs.act_outputs_1_sub, &buf.act_outputs)) {
				log_msg.msg_type = LOG_OUT1_MSG;
				memcpy(log_msg.body.log_OUT.output, buf.act_outputs.output, sizeof(log_msg.body.log_OUT.output));
				LOGBUFFER_WRITE_AND_COUNT(OUT);
			}

			/* --- ACTUATOR CONTROL --- */
			if (copy_if_updated(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &subs.act_controls_sub, &buf.act_controls)) {
				log_msg.msg_type = LOG_ATTC_MSG;
				log_msg.body.log_ATTC.roll = buf.act_controls.control[0];
				log_msg.body.log_ATTC.pitch = buf.act_controls.control[1];
				log_msg.body.log_ATTC.yaw = buf.act_controls.control[2];
				log_msg.body.log_ATTC.thrust = buf.act_controls.control[3];
				LOGBUFFER_WRITE_AND_COUNT(ATTC);
			}

			/* --- ACTUATOR CONTROL FW VTOL --- */
			if(copy_if_updated(ORB_ID(actuator_controls_1), &subs.act_controls_1_sub,&buf.act_controls)) {
				log_msg.msg_type = LOG_ATC1_MSG;
				log_msg.body.log_ATTC.roll = buf.act_controls.control[0];
				log_msg.body.log_ATTC.pitch = buf.act_controls.control[1];
				log_msg.body.log_ATTC.yaw = buf.act_controls.control[2];
				log_msg.body.log_ATTC.thrust = buf.act_controls.control[3];
				LOGBUFFER_WRITE_AND_COUNT(ATTC);
			}

			/* --- LOCAL POSITION --- */
			if (copy_if_updated(ORB_ID(vehicle_local_position), &subs.local_pos_sub, &buf.local_pos)) {
				log_msg.msg_type = LOG_LPOS_MSG;
				log_msg.body.log_LPOS.x = buf.local_pos.x;
				log_msg.body.log_LPOS.y = buf.local_pos.y;
				log_msg.body.log_LPOS.z = buf.local_pos.z;
				log_msg.body.log_LPOS.ground_dist = buf.local_pos.dist_bottom;
				log_msg.body.log_LPOS.ground_dist_rate = buf.local_pos.dist_bottom_rate;
				log_msg.body.log_LPOS.vx = buf.local_pos.vx;
				log_msg.body.log_LPOS.vy = buf.local_pos.vy;
				log_msg.body.log_LPOS.vz = buf.local_pos.vz;
				log_msg.body.log_LPOS.ref_lat = buf.local_pos.ref_lat * 1e7;
				log_msg.body.log_LPOS.ref_lon = buf.local_pos.ref_lon * 1e7;
				log_msg.body.log_LPOS.ref_alt = buf.local_pos.ref_alt;
				log_msg.body.log_LPOS.pos_flags = (buf.local_pos.xy_valid ? 1 : 0) |
												  (buf.local_pos.z_valid ? 2 : 0) |
												  (buf.local_pos.v_xy_valid ? 4 : 0) |
												  (buf.local_pos.v_z_valid ? 8 : 0) |
												  (buf.local_pos.xy_global ? 16 : 0) |
												  (buf.local_pos.z_global ? 32 : 0);
				log_msg.body.log_LPOS.ground_dist_flags = (buf.local_pos.dist_bottom_valid ? 1 : 0);
				log_msg.body.log_LPOS.eph = buf.local_pos.eph;
				log_msg.body.log_LPOS.epv = buf.local_pos.epv;
				LOGBUFFER_WRITE_AND_COUNT(LPOS);
			}

			/* --- LOCAL POSITION SETPOINT --- */
			if (copy_if_updated(ORB_ID(vehicle_local_position_setpoint), &subs.local_pos_sp_sub, &buf.local_pos_sp)) {
				log_msg.msg_type = LOG_LPSP_MSG;
				log_msg.body.log_LPSP.x = buf.local_pos_sp.x;
				log_msg.body.log_LPSP.y = buf.local_pos_sp.y;
				log_msg.body.log_LPSP.z = buf.local_pos_sp.z;
				log_msg.body.log_LPSP.yaw = buf.local_pos_sp.yaw;
				log_msg.body.log_LPSP.vx = buf.local_pos_sp.vx;
				log_msg.body.log_LPSP.vy = buf.local_pos_sp.vy;
				log_msg.body.log_LPSP.vz = buf.local_pos_sp.vz;
				log_msg.body.log_LPSP.acc_x = buf.local_pos_sp.acc_x;
				log_msg.body.log_LPSP.acc_y = buf.local_pos_sp.acc_y;
				log_msg.body.log_LPSP.acc_z = buf.local_pos_sp.acc_z;
				LOGBUFFER_WRITE_AND_COUNT(LPSP);
			}

			/* --- GLOBAL POSITION --- */
			if (copy_if_updated(ORB_ID(vehicle_global_position), &subs.global_pos_sub, &buf.global_pos)) {
				log_msg.msg_type = LOG_GPOS_MSG;
				log_msg.body.log_GPOS.lat = buf.global_pos.lat * 1e7;
				log_msg.body.log_GPOS.lon = buf.global_pos.lon * 1e7;
				log_msg.body.log_GPOS.alt = buf.global_pos.alt;
				log_msg.body.log_GPOS.vel_n = buf.global_pos.vel_n;
				log_msg.body.log_GPOS.vel_e = buf.global_pos.vel_e;
				log_msg.body.log_GPOS.vel_d = buf.global_pos.vel_d;
				log_msg.body.log_GPOS.eph = buf.global_pos.eph;
				log_msg.body.log_GPOS.epv = buf.global_pos.epv;
				if (buf.global_pos.terrain_alt_valid) {
					log_msg.body.log_GPOS.terrain_alt = buf.global_pos.terrain_alt;
				} else {
					log_msg.body.log_GPOS.terrain_alt = -1.0f;
				}
				LOGBUFFER_WRITE_AND_COUNT(GPOS);
			}

			/* --- BATTERY --- */
			if (copy_if_updated(ORB_ID(battery_status), &subs.battery_sub, &buf.battery)) {
				log_msg.msg_type = LOG_BATT_MSG;
				log_msg.body.log_BATT.voltage = buf.battery.voltage_v;
				log_msg.body.log_BATT.voltage_filtered = buf.battery.voltage_filtered_v;
				log_msg.body.log_BATT.current = buf.battery.current_a;
				log_msg.body.log_BATT.current_filtered = buf.battery.current_filtered_a;
				log_msg.body.log_BATT.discharged = buf.battery.discharged_mah;
				log_msg.body.log_BATT.remaining = buf.battery.remaining;
				log_msg.body.log_BATT.scale = buf.battery.scale;
				log_msg.body.log_BATT.warning = buf.battery.warning;
				LOGBUFFER_WRITE_AND_COUNT(BATT);
			}

			/* --- GLOBAL POSITION SETPOINT --- */
			if (copy_if_updated(ORB_ID(position_setpoint_triplet), &subs.triplet_sub, &buf.triplet)) {

				if (buf.triplet.current.valid) {
					log_msg.msg_type = LOG_GPSP_MSG;
					log_msg.body.log_GPSP.lat = (int32_t)(buf.triplet.current.lat * (double)1e7);
					log_msg.body.log_GPSP.lon = (int32_t)(buf.triplet.current.lon * (double)1e7);
					log_msg.body.log_GPSP.alt = buf.triplet.current.alt;
					log_msg.body.log_GPSP.yaw = buf.triplet.current.yaw;
					log_msg.body.log_GPSP.type = buf.triplet.current.type;
					log_msg.body.log_GPSP.loiter_radius = buf.triplet.current.loiter_radius;
					log_msg.body.log_GPSP.loiter_direction = buf.triplet.current.loiter_direction;
					log_msg.body.log_GPSP.pitch_min = buf.triplet.current.pitch_min;
					LOGBUFFER_WRITE_AND_COUNT(GPSP);
				}
			}

			/* --- MOCAP ATTITUDE AND POSITION --- */
			if (copy_if_updated(ORB_ID(att_pos_mocap), &subs.att_pos_mocap_sub, &buf.att_pos_mocap)) {
				log_msg.msg_type = LOG_MOCP_MSG;
				log_msg.body.log_MOCP.qw = buf.att_pos_mocap.q[0];
				log_msg.body.log_MOCP.qx = buf.att_pos_mocap.q[1];
				log_msg.body.log_MOCP.qy = buf.att_pos_mocap.q[2];
				log_msg.body.log_MOCP.qz = buf.att_pos_mocap.q[3];
				log_msg.body.log_MOCP.x = buf.att_pos_mocap.x;
				log_msg.body.log_MOCP.y = buf.att_pos_mocap.y;
				log_msg.body.log_MOCP.z = buf.att_pos_mocap.z;
				LOGBUFFER_WRITE_AND_COUNT(MOCP);
			}

			/* --- VISION POSITION --- */
			if (copy_if_updated(ORB_ID(vehicle_vision_position), &subs.vision_pos_sub, &buf.vision_pos) ||
			    copy_if_updated(ORB_ID(vehicle_vision_attitude), &subs.vision_att_sub, &buf.vision_att)) {
				log_msg.msg_type = LOG_VISN_MSG;
				log_msg.body.log_VISN.x = buf.vision_pos.x;
				log_msg.body.log_VISN.y = buf.vision_pos.y;
				log_msg.body.log_VISN.z = buf.vision_pos.z;
				log_msg.body.log_VISN.vx = buf.vision_pos.vx;
				log_msg.body.log_VISN.vy = buf.vision_pos.vy;
				log_msg.body.log_VISN.vz = buf.vision_pos.vz;
				log_msg.body.log_VISN.qw = buf.vision_att.q[0]; // vision_position_estimate uses [w,x,y,z] convention
				log_msg.body.log_VISN.qx = buf.vision_att.q[1];
				log_msg.body.log_VISN.qy = buf.vision_att.q[2];
				log_msg.body.log_VISN.qz = buf.vision_att.q[3];
				LOGBUFFER_WRITE_AND_COUNT(VISN);
			}

			/* --- FLOW --- */
			if (copy_if_updated(ORB_ID(optical_flow), &subs.flow_sub, &buf.flow)) {
				log_msg.msg_type = LOG_FLOW_MSG;
				log_msg.body.log_FLOW.ground_distance_m = buf.flow.ground_distance_m;
				log_msg.body.log_FLOW.gyro_temperature = buf.flow.gyro_temperature;
				log_msg.body.log_FLOW.gyro_x_rate_integral = buf.flow.gyro_x_rate_integral;
				log_msg.body.log_FLOW.gyro_y_rate_integral = buf.flow.gyro_y_rate_integral;
				log_msg.body.log_FLOW.gyro_z_rate_integral = buf.flow.gyro_z_rate_integral;
				log_msg.body.log_FLOW.integration_timespan = buf.flow.integration_timespan;
				log_msg.body.log_FLOW.pixel_flow_x_integral = buf.flow.pixel_flow_x_integral;
				log_msg.body.log_FLOW.pixel_flow_y_integral = buf.flow.pixel_flow_y_integral;
				log_msg.body.log_FLOW.quality = buf.flow.quality;
				log_msg.body.log_FLOW.sensor_id = buf.flow.sensor_id;
				LOGBUFFER_WRITE_AND_COUNT(FLOW);
			}

			/* --- RC CHANNELS --- */
			if (copy_if_updated(ORB_ID(rc_channels), &subs.rc_sub, &buf.rc)) {
				log_msg.msg_type = LOG_RC_MSG;
				/* Copy only the first 12 channels of 18 */
				memcpy(log_msg.body.log_RC.channel, buf.rc.channels, sizeof(log_msg.body.log_RC.channel));
				log_msg.body.log_RC.rssi = buf.rc.rssi;
				log_msg.body.log_RC.channel_count = buf.rc.channel_count;
				log_msg.body.log_RC.signal_lost = buf.rc.signal_lost;
				log_msg.body.log_RC.frame_drop = buf.rc.frame_drop_count;
				LOGBUFFER_WRITE_AND_COUNT(RC);
			}

			/* --- AIRSPEED --- */
			if (copy_if_updated(ORB_ID(airspeed), &subs.airspeed_sub, &buf.airspeed)) {
				log_msg.msg_type = LOG_AIRS_MSG;
				log_msg.body.log_AIRS.indicated_airspeed_m_s = buf.airspeed.indicated_airspeed_m_s;
				log_msg.body.log_AIRS.true_airspeed_m_s = buf.airspeed.true_airspeed_m_s;
				log_msg.body.log_AIRS.true_airspeed_unfiltered_m_s = buf.airspeed.true_airspeed_unfiltered_m_s;
				log_msg.body.log_AIRS.air_temperature_celsius = buf.airspeed.air_temperature_celsius;
				log_msg.body.log_AIRS.confidence = buf.airspeed.confidence;
				LOGBUFFER_WRITE_AND_COUNT(AIRS);
			}

			/* --- DIFFERENTIAL PRESSURE --- */
			if (copy_if_updated(ORB_ID(differential_pressure), &subs.diff_pres_sub, &buf.diff_pres)) {
				log_msg.msg_type = LOG_DPRS_MSG;
				log_msg.body.log_DPRS.error_count = buf.diff_pres.error_count;
				log_msg.body.log_DPRS.differential_pressure_raw_pa = buf.diff_pres.differential_pressure_raw_pa;
				log_msg.body.log_DPRS.differential_pressure_filtered_pa = buf.diff_pres.differential_pressure_filtered_pa;
				log_msg.body.log_DPRS.max_differential_pressure_pa = buf.diff_pres.max_differential_pressure_pa;
				log_msg.body.log_DPRS.temperature = buf.diff_pres.temperature;
				LOGBUFFER_WRITE_AND_COUNT(DPRS);
			}

			/* --- ESCs --- */
			if (copy_if_updated(ORB_ID(esc_status), &subs.esc_sub, &buf.esc)) {
				for (uint8_t i = 0; i < buf.esc.esc_count; i++) {
					log_msg.msg_type = LOG_ESC_MSG;
					log_msg.body.log_ESC.counter = buf.esc.counter;
					log_msg.body.log_ESC.esc_count = buf.esc.esc_count;
					log_msg.body.log_ESC.esc_connectiontype = buf.esc.esc_connectiontype;
					log_msg.body.log_ESC.esc_num = i;
					log_msg.body.log_ESC.esc_address = buf.esc.esc[i].esc_address;
					log_msg.body.log_ESC.esc_version = buf.esc.esc[i].esc_version;
					log_msg.body.log_ESC.esc_voltage = buf.esc.esc[i].esc_voltage;
					log_msg.body.log_ESC.esc_current = buf.esc.esc[i].esc_current;
					log_msg.body.log_ESC.esc_rpm = buf.esc.esc[i].esc_rpm;
					log_msg.body.log_ESC.esc_temperature = buf.esc.esc[i].esc_temperature;
					log_msg.body.log_ESC.esc_setpoint = buf.esc.esc[i].esc_setpoint;
					log_msg.body.log_ESC.esc_setpoint_raw = buf.esc.esc[i].esc_setpoint_raw;
					LOGBUFFER_WRITE_AND_COUNT(ESC);
				}
			}

			/* --- GLOBAL VELOCITY SETPOINT --- */
			if (copy_if_updated(ORB_ID(vehicle_global_velocity_setpoint), &subs.global_vel_sp_sub, &buf.global_vel_sp)) {
				log_msg.msg_type = LOG_GVSP_MSG;
				log_msg.body.log_GVSP.vx = buf.global_vel_sp.vx;
				log_msg.body.log_GVSP.vy = buf.global_vel_sp.vy;
				log_msg.body.log_GVSP.vz = buf.global_vel_sp.vz;
				LOGBUFFER_WRITE_AND_COUNT(GVSP);
			}

			/* --- BATTERY --- */
			if (copy_if_updated(ORB_ID(battery_status), &subs.battery_sub, &buf.battery)) {
				log_msg.msg_type = LOG_BATT_MSG;
				log_msg.body.log_BATT.voltage = buf.battery.voltage_v;
				log_msg.body.log_BATT.voltage_filtered = buf.battery.voltage_filtered_v;
				log_msg.body.log_BATT.current = buf.battery.current_a;
				log_msg.body.log_BATT.current_filtered = buf.battery.current_filtered_a;
				log_msg.body.log_BATT.discharged = buf.battery.discharged_mah;
				LOGBUFFER_WRITE_AND_COUNT(BATT);
			}

			/* --- SYSTEM POWER RAILS --- */
			if (copy_if_updated(ORB_ID(system_power), &subs.system_power_sub, &buf.system_power)) {
				log_msg.msg_type = LOG_PWR_MSG;
				log_msg.body.log_PWR.peripherals_5v = buf.system_power.voltage5V_v;
				log_msg.body.log_PWR.usb_ok = buf.system_power.usb_connected;
				log_msg.body.log_PWR.brick_ok = buf.system_power.brick_valid;
				log_msg.body.log_PWR.servo_ok = buf.system_power.servo_valid;
				log_msg.body.log_PWR.low_power_rail_overcurrent = buf.system_power.periph_5V_OC;
				log_msg.body.log_PWR.high_power_rail_overcurrent = buf.system_power.hipower_5V_OC;

				/* copy servo rail status topic here too */
				orb_copy(ORB_ID(servorail_status), subs.servorail_status_sub, &buf.servorail_status);
				log_msg.body.log_PWR.servo_rail_5v = buf.servorail_status.voltage_v;
				log_msg.body.log_PWR.servo_rssi = buf.servorail_status.rssi_v;

				LOGBUFFER_WRITE_AND_COUNT(PWR);
			}

			/* --- TELEMETRY --- */
			for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if (copy_if_updated_multi(ORB_ID(telemetry_status), i, &subs.telemetry_subs[i], &buf.telemetry)) {
					log_msg.msg_type = LOG_TEL0_MSG + i;
					log_msg.body.log_TEL.rssi = buf.telemetry.rssi;
					log_msg.body.log_TEL.remote_rssi = buf.telemetry.remote_rssi;
					log_msg.body.log_TEL.noise = buf.telemetry.noise;
					log_msg.body.log_TEL.remote_noise = buf.telemetry.remote_noise;
					log_msg.body.log_TEL.rxerrors = buf.telemetry.rxerrors;
					log_msg.body.log_TEL.fixed = buf.telemetry.fixed;
					log_msg.body.log_TEL.txbuf = buf.telemetry.txbuf;
					log_msg.body.log_TEL.heartbeat_time = buf.telemetry.heartbeat_time;
					LOGBUFFER_WRITE_AND_COUNT(TEL);
				}
			}

			/* --- DISTANCE SENSOR --- */
			if (copy_if_updated(ORB_ID(distance_sensor), &subs.distance_sensor_sub, &buf.distance_sensor)) {
				log_msg.msg_type = LOG_DIST_MSG;
				log_msg.body.log_DIST.id = buf.distance_sensor.id;
				log_msg.body.log_DIST.type = buf.distance_sensor.type;
				log_msg.body.log_DIST.orientation = buf.distance_sensor.orientation;
				log_msg.body.log_DIST.current_distance = buf.distance_sensor.current_distance;
				log_msg.body.log_DIST.covariance = buf.distance_sensor.covariance;
				LOGBUFFER_WRITE_AND_COUNT(DIST);
			}

			/* --- ESTIMATOR STATUS --- */
			if (copy_if_updated(ORB_ID(estimator_status), &subs.estimator_status_sub, &buf.estimator_status)) {
				log_msg.msg_type = LOG_EST0_MSG;
				unsigned maxcopy0 = (sizeof(buf.estimator_status.states) < sizeof(log_msg.body.log_EST0.s)) ? sizeof(buf.estimator_status.states) : sizeof(log_msg.body.log_EST0.s);
				memset(&(log_msg.body.log_EST0.s), 0, sizeof(log_msg.body.log_EST0.s));
				memcpy(&(log_msg.body.log_EST0.s), buf.estimator_status.states, maxcopy0);
				log_msg.body.log_EST0.n_states = buf.estimator_status.n_states;
				log_msg.body.log_EST0.nan_flags = buf.estimator_status.nan_flags;
				log_msg.body.log_EST0.fault_flags = buf.estimator_status.filter_fault_flags;
				log_msg.body.log_EST0.timeout_flags = buf.estimator_status.timeout_flags;
				LOGBUFFER_WRITE_AND_COUNT(EST0);

				log_msg.msg_type = LOG_EST1_MSG;
				unsigned maxcopy1 = ((sizeof(buf.estimator_status.states) - maxcopy0) < sizeof(log_msg.body.log_EST1.s)) ? (sizeof(buf.estimator_status.states) - maxcopy0) : sizeof(log_msg.body.log_EST1.s);
				memset(&(log_msg.body.log_EST1.s), 0, sizeof(log_msg.body.log_EST1.s));
				memcpy(&(log_msg.body.log_EST1.s), ((char*)buf.estimator_status.states) + maxcopy0, maxcopy1);
				LOGBUFFER_WRITE_AND_COUNT(EST1);

				log_msg.msg_type = LOG_EST2_MSG;
				unsigned maxcopy2 = (sizeof(buf.estimator_status.covariances) < sizeof(log_msg.body.log_EST2.cov)) ? sizeof(buf.estimator_status.covariances) : sizeof(log_msg.body.log_EST2.cov);
				memset(&(log_msg.body.log_EST2.cov), 0, sizeof(log_msg.body.log_EST2.cov));
				memcpy(&(log_msg.body.log_EST2.cov), buf.estimator_status.covariances, maxcopy2);
				log_msg.body.log_EST2.gps_check_fail_flags = buf.estimator_status.gps_check_fail_flags;
				log_msg.body.log_EST2.control_mode_flags = buf.estimator_status.control_mode_flags;
				log_msg.body.log_EST2.health_flags = buf.estimator_status.health_flags;
				log_msg.body.log_EST2.innov_test_flags = buf.estimator_status.innovation_check_flags;
				LOGBUFFER_WRITE_AND_COUNT(EST2);

				log_msg.msg_type = LOG_EST3_MSG;
				unsigned maxcopy3 = ((sizeof(buf.estimator_status.covariances) - maxcopy2) < sizeof(log_msg.body.log_EST3.cov)) ? (sizeof(buf.estimator_status.covariances) - maxcopy2) : sizeof(log_msg.body.log_EST3.cov);
				memset(&(log_msg.body.log_EST3.cov), 0, sizeof(log_msg.body.log_EST3.cov));
				memcpy(&(log_msg.body.log_EST3.cov), ((char*)buf.estimator_status.covariances) + maxcopy2, maxcopy3);
				LOGBUFFER_WRITE_AND_COUNT(EST3);
			}

			/* --- EKF2 INNOVATIONS --- */
			if (copy_if_updated(ORB_ID(ekf2_innovations), &subs.innov_sub, &buf.innovations)) {
				log_msg.msg_type = LOG_EST4_MSG;
				memset(&(log_msg.body.log_INO1.s), 0, sizeof(log_msg.body.log_INO1.s));
				for (unsigned i = 0; i < 6; i++) {
					log_msg.body.log_INO1.s[i] = buf.innovations.vel_pos_innov[i];
					log_msg.body.log_INO1.s[i + 6] = buf.innovations.vel_pos_innov_var[i];
				}
				for (unsigned i = 0; i < 3; i++) {
					log_msg.body.log_INO1.s[i + 12] = buf.innovations.output_tracking_error[i];
				}
				LOGBUFFER_WRITE_AND_COUNT(EST4);

				log_msg.msg_type = LOG_EST5_MSG;
				memset(&(log_msg.body.log_INO2.s), 0, sizeof(log_msg.body.log_INO2.s));
				for (unsigned i = 0; i < 3; i++) {
					log_msg.body.log_INO2.s[i] = buf.innovations.mag_innov[i];
					log_msg.body.log_INO2.s[i + 3] = buf.innovations.mag_innov_var[i];
				}

				log_msg.body.log_INO2.s[6] = buf.innovations.heading_innov;
				log_msg.body.log_INO2.s[7] = buf.innovations.heading_innov_var;
				log_msg.body.log_INO2.s[8] = buf.innovations.airspeed_innov;
				log_msg.body.log_INO2.s[9] = buf.innovations.airspeed_innov_var;
				log_msg.body.log_INO2.s[10] = buf.innovations.beta_innov;
				log_msg.body.log_INO2.s[11] = buf.innovations.beta_innov_var;
				LOGBUFFER_WRITE_AND_COUNT(EST5);

				log_msg.msg_type = LOG_EST6_MSG;
				memset(&(log_msg.body.log_INO3.s), 0, sizeof(log_msg.body.log_INO3.s));
				for(unsigned i = 0; i < 2; i++) {
					log_msg.body.log_INO3.s[i] = buf.innovations.flow_innov[i];
					log_msg.body.log_INO3.s[i + 2] = buf.innovations.flow_innov_var[i];
				}
				log_msg.body.log_INO3.s[4] = buf.innovations.hagl_innov;
				log_msg.body.log_INO3.s[5] = buf.innovations.hagl_innov_var;
				LOGBUFFER_WRITE_AND_COUNT(EST6);
			}

			/* --- TECS STATUS --- */
			if (copy_if_updated(ORB_ID(tecs_status), &subs.tecs_status_sub, &buf.tecs_status)) {
				log_msg.msg_type = LOG_TECS_MSG;
				log_msg.body.log_TECS.altitudeSp = buf.tecs_status.altitudeSp;
				log_msg.body.log_TECS.altitudeFiltered = buf.tecs_status.altitude_filtered;
				log_msg.body.log_TECS.flightPathAngleSp = buf.tecs_status.flightPathAngleSp;
				log_msg.body.log_TECS.flightPathAngle = buf.tecs_status.flightPathAngle;
				log_msg.body.log_TECS.airspeedSp = buf.tecs_status.airspeedSp;
				log_msg.body.log_TECS.airspeedFiltered = buf.tecs_status.airspeed_filtered;
				log_msg.body.log_TECS.airspeedDerivativeSp = buf.tecs_status.airspeedDerivativeSp;
				log_msg.body.log_TECS.airspeedDerivative = buf.tecs_status.airspeedDerivative;
				log_msg.body.log_TECS.totalEnergyError = buf.tecs_status.totalEnergyError;
				log_msg.body.log_TECS.totalEnergyRateError = buf.tecs_status.totalEnergyRateError;
				log_msg.body.log_TECS.energyDistributionError = buf.tecs_status.energyDistributionError;
				log_msg.body.log_TECS.energyDistributionRateError = buf.tecs_status.energyDistributionRateError;
				log_msg.body.log_TECS.pitch_integ = buf.tecs_status.pitch_integ;
				log_msg.body.log_TECS.throttle_integ = buf.tecs_status.throttle_integ;
				log_msg.body.log_TECS.mode = (uint8_t)buf.tecs_status.mode;
				LOGBUFFER_WRITE_AND_COUNT(TECS);
			}

			/* --- WIND ESTIMATE --- */
			if (copy_if_updated(ORB_ID(wind_estimate), &subs.wind_sub, &buf.wind_estimate)) {
				log_msg.msg_type = LOG_WIND_MSG;
				log_msg.body.log_WIND.x = buf.wind_estimate.windspeed_north;
				log_msg.body.log_WIND.y = buf.wind_estimate.windspeed_east;
				log_msg.body.log_WIND.cov_x = buf.wind_estimate.covariance_north;
				log_msg.body.log_WIND.cov_y = buf.wind_estimate.covariance_east;
				LOGBUFFER_WRITE_AND_COUNT(WIND);
			}

			/* --- TIMESYNC OFFSET --- */
			if (copy_if_updated(ORB_ID(time_offset), &subs.tsync_sub, &buf.time_offset)) {
				log_msg.msg_type = LOG_TSYN_MSG;
				log_msg.body.log_TSYN.time_offset = buf.time_offset.offset_ns;
				LOGBUFFER_WRITE_AND_COUNT(TSYN);
			}

			/* --- MULTIROTOR ATTITUDE CONTROLLER STATUS --- */
			if (copy_if_updated(ORB_ID(mc_att_ctrl_status), &subs.mc_att_ctrl_status_sub, &buf.mc_att_ctrl_status)) {
				log_msg.msg_type = LOG_MACS_MSG;
				log_msg.body.log_MACS.roll_rate_integ = buf.mc_att_ctrl_status.roll_rate_integ;
				log_msg.body.log_MACS.pitch_rate_integ = buf.mc_att_ctrl_status.pitch_rate_integ;
				log_msg.body.log_MACS.yaw_rate_integ = buf.mc_att_ctrl_status.yaw_rate_integ;
				LOGBUFFER_WRITE_AND_COUNT(MACS);
			}

			/* --- CONTROL STATE --- */
			if (copy_if_updated(ORB_ID(control_state), &subs.ctrl_state_sub, &buf.ctrl_state)) {
				log_msg.msg_type = LOG_CTS_MSG;
				log_msg.body.log_CTS.vx_body = buf.ctrl_state.x_vel;
				log_msg.body.log_CTS.vy_body = buf.ctrl_state.y_vel;
				log_msg.body.log_CTS.vz_body = buf.ctrl_state.z_vel;
				log_msg.body.log_CTS.airspeed = buf.ctrl_state.airspeed;
				log_msg.body.log_CTS.roll_rate = buf.ctrl_state.roll_rate;
				log_msg.body.log_CTS.pitch_rate = buf.ctrl_state.pitch_rate;
				log_msg.body.log_CTS.yaw_rate = buf.ctrl_state.yaw_rate;
				LOGBUFFER_WRITE_AND_COUNT(CTS);
			}
		}

		/* --- ATTITUDE --- */
		if (copy_if_updated(ORB_ID(vehicle_attitude), &subs.att_sub, &buf.att)) {
			log_msg.msg_type = LOG_ATT_MSG;
			float q0 = buf.att.q[0];
			float q1 = buf.att.q[1];
			float q2 = buf.att.q[2];
			float q3 = buf.att.q[3];
			log_msg.body.log_ATT.q_w = q0;
			log_msg.body.log_ATT.q_x = q1;
			log_msg.body.log_ATT.q_y = q2;
			log_msg.body.log_ATT.q_z = q3;
			log_msg.body.log_ATT.roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
			log_msg.body.log_ATT.pitch = asinf(2*(q0*q2 - q3*q1));
			log_msg.body.log_ATT.yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
			log_msg.body.log_ATT.roll_rate = buf.att.rollspeed;
			log_msg.body.log_ATT.pitch_rate = buf.att.pitchspeed;
			log_msg.body.log_ATT.yaw_rate = buf.att.yawspeed;
			LOGBUFFER_WRITE_AND_COUNT(ATT);
		}

		/* --- CAMERA TRIGGER --- */
		if (copy_if_updated(ORB_ID(camera_trigger), &subs.cam_trig_sub, &buf.camera_trigger)) {
			log_msg.msg_type = LOG_CAMT_MSG;
			log_msg.body.log_CAMT.timestamp = buf.camera_trigger.timestamp;
			log_msg.body.log_CAMT.seq = buf.camera_trigger.seq;
			LOGBUFFER_WRITE_AND_COUNT(CAMT);
		}

		/* --- LAND DETECTED --- */
		if (copy_if_updated(ORB_ID(vehicle_land_detected), &subs.land_detected_sub, &buf.land_detected)) {
			log_msg.msg_type = LOG_LAND_MSG;
			log_msg.body.log_LAND.landed = buf.land_detected.landed;
			LOGBUFFER_WRITE_AND_COUNT(LAND);
		}

		/* --- LOAD --- */
		if (copy_if_updated(ORB_ID(cpuload), &subs.cpuload_sub, &buf.cpuload)) {
			log_msg.msg_type = LOG_LOAD_MSG;
			log_msg.body.log_LOAD.cpu_load = buf.cpuload.load;
			LOGBUFFER_WRITE_AND_COUNT(LOAD);
		}

		/* --- STACK --- */
		if (copy_if_updated(ORB_ID(task_stack_info), &subs.task_stack_info_sub, &buf.task_stack_info)) {
			log_msg.msg_type = LOG_STCK_MSG;
			log_msg.body.log_STCK.stack_free = buf.task_stack_info.stack_free;
			strncpy(log_msg.body.log_STCK.task_name, (char*)buf.task_stack_info.task_name,
					sizeof(log_msg.body.log_STCK.task_name));
			LOGBUFFER_WRITE_AND_COUNT(STCK);
		}

		pthread_mutex_lock(&logbuffer_mutex);

		/* signal the other thread new data, but not yet unlock */
		if (logbuffer_count(&lb) > MIN_BYTES_TO_WRITE) {
			/* only request write if several packets can be written at once */
			pthread_cond_signal(&logbuffer_cond);
		}

		/* unlock, now the writer thread may run */
		pthread_mutex_unlock(&logbuffer_mutex);
	}

	if (logging_enabled) {
		sdlog2_stop_log();
	}

	pthread_mutex_destroy(&logbuffer_mutex);
	pthread_cond_destroy(&logbuffer_cond);

	/* free log buffer */
	logbuffer_free(&lb);

	thread_running = false;

	return 0;
}

void sdlog2_status()
{
	PX4_WARN("extended logging: %s", (_extended_logging) ? "ON" : "OFF");
	PX4_WARN("time: gps: %u seconds", (unsigned)gps_time_sec);
	if (!logging_enabled) {
		PX4_WARN("not logging");
	} else {

		float kibibytes = log_bytes_written / 1024.0f;
		float mebibytes = kibibytes / 1024.0f;
		float seconds = ((float)(hrt_absolute_time() - start_time)) / 1000000.0f;

		PX4_WARN("wrote %lu msgs, %4.2f MiB (average %5.3f KiB/s), skipped %lu msgs", log_msgs_written, (double)mebibytes, (double)(kibibytes / seconds), log_msgs_skipped);
		mavlink_log_info(&mavlink_log_pub, "[blackbox] wrote %lu msgs, skipped %lu msgs", log_msgs_written, log_msgs_skipped);
	}
}

/**
 * @return true if file exists
 */
bool file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int check_free_space()
{
	/* use statfs to determine the number of blocks left */
	FAR struct statfs statfs_buf;
	if (statfs(mountpoint, &statfs_buf) != OK) {
		mavlink_log_critical(&mavlink_log_pub, "[blackbox] no microSD card, disabling logging");
		return PX4_ERROR;
	}

	/* use a threshold of 50 MiB */
	if (statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(50 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_log_critical(&mavlink_log_pub, "[blackbox] no space on MicroSD: %u MiB",
			(unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize) / (1024U * 1024U));
		/* we do not need a flag to remember that we sent this warning because we will exit anyway */
		return PX4_ERROR;

	/* use a threshold of 100 MiB to send a warning */
	} else if (!space_warning_sent && statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(100 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_log_critical(&mavlink_log_pub, "[blackbox] space on MicroSD low: %u MiB",
			(unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize) / (1024U * 1024U));
		/* we don't want to flood the user with warnings */
		space_warning_sent = true;
	}

	return PX4_OK;
}

void handle_command(struct vehicle_command_s *cmd)
{
	int param;

	/* request to set different system mode */
	switch (cmd->command) {

	case VEHICLE_CMD_PREFLIGHT_STORAGE:
		param = (int)(cmd->param3 + 0.5f);

		if (param == 1)	{
			sdlog2_start_log();

		} else if (param == 2)	{
			sdlog2_stop_log();
		} else {
			// Silently ignore non-matching command values, as they could be for params.
		}

		break;

	default:
		/* silently ignore */
		break;
	}
}

void handle_status(struct vehicle_status_s *status)
{
	// TODO use flag from actuator_armed here?
	bool armed = status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR;

	if (armed != flag_system_armed) {
		flag_system_armed = armed;

		if (flag_system_armed) {
			sdlog2_start_log();

		} else {
			sdlog2_stop_log();
		}
	}
}
