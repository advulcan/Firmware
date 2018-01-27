/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file hc_sr04.cpp
 *
 * Driver for the hc_sr04 sonar range finders .
 */
#ifndef HC_SR04_H
#define HC_SR04_H
//#include <drivers/device/device.h>
#include <px4_defines.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <pthread.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_defines.h>

#include <board_config.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define SR04_MAX_RANGEFINDERS 6
#define SR04_ID_BASE	 0x10

/* Configuration Constants */
#define SR04_DEVICE_PATH	"/dev/hc_sr04"

#define SUBSYSTEM_TYPE_RANGEFINDER 131072
/* Device limits */
#define SR04_MIN_DISTANCE 	(0.10f)
#define SR04_MAX_DISTANCE 	(4.00f)

#define SR04_CONVERSION_INTERVAL	100000 /* 100ms for one sonar */
#define PIN_TRIG 4
#define PIN_ECHO 5

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class HC_SR04 
{
public:
	HC_SR04(unsigned sonars = 1);
	virtual ~HC_SR04();
	virtual int 		init();
	void				print_info();


private:
	int					_waiting_fall;
	uint64_t			_rise_time;
	uint64_t			_fall_time;
	bool				_debug_enabled;
	float				_min_distance;
	float				_max_distance;
	float				_current_distance;
	struct				work_s _work;
	ringbuffer::RingBuffer	*_reports;
	int					_class_instance;
	int					_orb_class_instance;
	int					_cycling_rate;
	orb_advert_t		_distance_sensor_topic;

	void				start();
	void				stop();

	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	void				cycle();
	static void			cycle_trampoline(void *arg);
};
#endif