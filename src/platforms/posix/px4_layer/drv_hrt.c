/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file drv_hrt.c
 *
 * High-resolution timer with callouts and timekeeping.
 */

#include <px4_time.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <px4_workqueue.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>
#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include "hrt_work.h"

static struct sq_queue_s	callout_queue;

/* latency histogram */
#define LATENCY_BUCKET_COUNT 8
__EXPORT const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
__EXPORT const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t	latency_counters[LATENCY_BUCKET_COUNT + 1];

static void		hrt_call_reschedule(void);

// Intervals in usec
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	50000000

static px4_sem_t 	_hrt_lock;
static struct work_s	_hrt_work;
#ifndef __PX4_QURT
static hrt_abstime px4_timestart = 0;
#else
static int32_t dsp_offset = 0;
#endif
static hrt_abstime _start_delay_time = 0;
static hrt_abstime _delay_interval = 0;
static hrt_abstime max_time = 0;
pthread_mutex_t _hrt_mutex = PTHREAD_MUTEX_INITIALIZER;

static void
hrt_call_invoke(void);

static hrt_abstime
_hrt_absolute_time_internal(void);

__EXPORT hrt_abstime hrt_reset(void);

static void hrt_lock(void)
{
	px4_sem_wait(&_hrt_lock);
}

static void hrt_unlock(void)
{
	px4_sem_post(&_hrt_lock);
}

#if defined(__PX4_APPLE_LEGACY)
#include <sys/time.h>

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	struct timeval now;
	int rv = gettimeofday(&now, NULL);

	if (rv) {
		return rv;
	}

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	/* do nothing right now */
	return 0;
}

#elif defined(__PX4_QURT)

#include "dspal_time.h"

int px4_clock_settime(clockid_t clk_id, struct timespec *tp)
{
	/* do nothing right now */
	return 0;
}

int px4_clock_gettime(clockid_t clk_id, struct timespec *tp)
{
	return clock_gettime(clk_id, tp);
}

#endif

#ifndef __PX4_QURT
/*
 * Get system time in us
 */
uint64_t hrt_system_time(void)
{
	struct timespec ts;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);
}
#endif

/*
 * Get absolute time.
 */
hrt_abstime _hrt_absolute_time_internal(void)
{
	struct timespec ts;

#if defined(__PX4_QURT)
	// Don't use the timestart on the DSP on Snapdragon because we manually
	// set the px4_timestart using the hrt_set_absolute_time_offset().
	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts) + dsp_offset;

#elif (defined(__PX4_POSIX_EAGLE) || defined(__PX4_POSIX_EXCELSIOR))
	// Don't do any offseting on the Linux side on the Snapdragon.
	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);

#else

	if (!px4_timestart) {
		px4_clock_gettime(CLOCK_MONOTONIC, &ts);
		px4_timestart = ts_to_abstime(&ts);
	}

	px4_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts) - px4_timestart;
#endif
}

#ifdef __PX4_QURT
int hrt_set_absolute_time_offset(int32_t time_diff_us)
{
	dsp_offset = time_diff_us;
	return 0;
}
#endif

/*
 * Get absolute time.
 */
hrt_abstime hrt_absolute_time(void)
{
	pthread_mutex_lock(&_hrt_mutex);

	hrt_abstime ret;

	if (_start_delay_time > 0) {
		ret = _start_delay_time;

	} else {
		ret = _hrt_absolute_time_internal();
	}

	ret -= _delay_interval;

	if (ret < max_time) {
		PX4_ERR("WARNING! TIME IS NEGATIVE! %d vs %d", (int)ret, (int)max_time);
		ret = max_time;
	}

	max_time = ret;
	pthread_mutex_unlock(&_hrt_mutex);

	return ret;
}

__EXPORT hrt_abstime hrt_reset(void)
{
#ifndef __PX4_QURT
	px4_timestart = 0;
#endif
	max_time = 0;
	return _hrt_absolute_time_internal();
}

/*
 * Convert a timespec to absolute time.
 */
hrt_abstime ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/*
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is safe to use even if the timestamp is updated
 * by an interrupt during execution.
 */
hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then)
{
	hrt_abstime delta = hrt_absolute_time() - *then;
	return delta;
}

/*
 * Store the absolute time in an interrupt-safe fashion.
 *
 * This function ensures that the timestamp cannot be seen half-written by an interrupt handler.
 */
hrt_abstime hrt_store_absolute_time(volatile hrt_abstime *now)
{
	hrt_abstime ts = hrt_absolute_time();
	return ts;
}


/*
 * If this returns true, the entry has been invoked and removed from the callout list,
 * or it has never been entered.
 *
 * Always returns false for repeating callouts.
 */
bool	hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/*
 * Remove the entry from the callout list.
 */
void	hrt_cancel(struct hrt_call *entry)
{
	hrt_lock();
	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;
	hrt_unlock();
	// endif
}

/*
 * initialise a hrt_call structure
 */
void	hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

/*
 * delay a hrt_call_every() periodic call by the given number of
 * microseconds. This should be called from within the callout to
 * cause the callout to be re-scheduled for a later time. The periodic
 * callouts will then continue from that new base time at the
 * previously specified period.
 */
void	hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

/*
 * Initialise the HRT.
 */
void	hrt_init(void)
{
	sq_init(&callout_queue);

	int sem_ret = px4_sem_init(&_hrt_lock, 0, 1);

	if (sem_ret) {
		PX4_ERR("SEM INIT FAIL: %s", strerror(errno));
	}

	memset(&_hrt_work, 0, sizeof(_hrt_work));
}

void	hrt_start_delay()
{
	pthread_mutex_lock(&_hrt_mutex);
	_start_delay_time = _hrt_absolute_time_internal();
	pthread_mutex_unlock(&_hrt_mutex);
}

void	hrt_stop_delay()
{
	pthread_mutex_lock(&_hrt_mutex);
	uint64_t delta = _hrt_absolute_time_internal() - _start_delay_time;
	_delay_interval += delta;
	_start_delay_time = 0;

	if (delta > 100000) {
		PX4_INFO("Computer load temporarily too high for real-time simulation. (slowdown delay: %" PRIu64 " us)", delta);
	}

	pthread_mutex_unlock(&_hrt_mutex);

}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	//PX4_INFO("hrt_call_enter");
	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		//if (call != NULL) PX4_INFO("call enter at head, reschedule (%lu %lu)", entry->deadline, call->deadline);
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				//lldbg("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	//PX4_INFO("scheduled");
}

/**
 * Timer interrupt handler
 *
 * This routine simulates a timer interrupt handler
 */
static void
hrt_tim_isr(void *p)
{

	//PX4_INFO("hrt_tim_isr");
	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	hrt_lock();

	/* and schedule the next interrupt */
	hrt_call_reschedule();

	hrt_unlock();
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	hrt_abstime	delay = HRT_INTERVAL_MAX;
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	//PX4_INFO("hrt_call_reschedule");

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 16 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		//lldbg("entry in queue\n");
		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			//lldbg("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			delay = HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			//lldbg("due soon\n");
			delay = next->deadline - now;
		}
	}

	// There is no timer ISR, so simulate one by putting an event on the
	// high priority work queue

	// Remove the existing expiry and update with the new expiry
	hrt_work_cancel(&_hrt_work);

	hrt_work_queue(&_hrt_work, (worker_t)&hrt_tim_isr, NULL, delay);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	PX4_DEBUG("hrt_call_internal deadline=%lu interval = %lu", deadline, interval);
	hrt_lock();

	//PX4_INFO("hrt_call_internal after lock");
	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

#if 1

	// Use this to debug busy CPU that keeps rescheduling with 0 period time
	/*if (interval < HRT_INTERVAL_MIN) {*/
	/*PX4_ERR("hrt_call_internal interval too short: %" PRIu64, interval);*/
	/*PX4_BACKTRACE();*/
	/*}*/

#endif
	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);
	hrt_unlock();
}

/*
 * Call callout(arg) after delay has elapsed.
 *
 * If callout is NULL, this can be used to implement a timeout by testing the call
 * with hrt_called().
 */
void	hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	//printf("hrt_call_after\n");
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/*
 * Call callout(arg) after delay, and then after every interval.
 *
 * Note thet the interval is timed between scheduled, not actual, call times, so the call rate may
 * jitter but should not drift.
 */
void	hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

/*
 * Call callout(arg) at absolute time calltime.
 */
void	hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

#if 0
/*
 * Convert absolute time to a timespec.
 */
void	abstime_to_ts(struct timespec *ts, hrt_abstime abstime);
#endif

static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	hrt_lock();

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		//PX4_INFO("call pop");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			// Unlock so we don't deadlock in callback
			hrt_unlock();

			//PX4_INFO("call %p: %p(%p)", call, call->callout, call->arg);
			call->callout(call->arg);

			hrt_lock();
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
				//PX4_INFO("call deadline set to %lu now=%lu", call->deadline,  now);
			}

			hrt_call_enter(call);
		}
	}

	hrt_unlock();
}

