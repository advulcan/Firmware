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

#include "HC_SR04.h"

namespace hc_sr04_c {
	double _distance;
	int waiting_fall = 0;
	struct timeval tv_start, tv_end;
	struct timespec ts_start, ts_end;
	double sum;
	double varianceSum;
	int count;
	long calc_interval_us(struct timeval tv_start, struct timeval tv_end);
	long calc_interval_ns(struct timespec start, struct timespec end);
	void reset();
	void onChange(void);
	void * run(void *args);
	void start();

	long calc_interval_us(struct timeval start, struct timeval end) {
		return ((end.tv_sec - start.tv_sec) * 1000 * 1000 + (end.tv_usec - start.tv_usec));
	}
	long calc_interval_ns(struct timespec start, struct timespec end) {
		struct timespec diff;
		diff.tv_sec = (end.tv_sec - start.tv_sec);
		diff.tv_nsec = (end.tv_nsec - start.tv_nsec);
		if (diff.tv_nsec < 0) {
			diff.tv_sec--;
			diff.tv_nsec += 1000000000;
		}
		long nsec = diff.tv_nsec + diff.tv_sec * 100000000;
		//printf("TimeMeasure: used time %9.1lf[ns]\n", nsec);
		return nsec;
	}
	void reset() {
		//printf("reset");
		tv_start.tv_sec = 0;
		tv_start.tv_usec = 0;
		tv_end.tv_sec = 0;
		tv_end.tv_usec = 0;
		ts_start.tv_sec = 0;
		ts_start.tv_nsec = 0;
		ts_end.tv_sec = 0;
		ts_end.tv_nsec = 0;
	}
	void onChange(void) {
		int current = digitalRead(PIN_ECHO);
		if (current == HIGH) {
			//printf("echo HIGH\n");
			//gettimeofday(&tv_start, NULL);
			clock_gettime(CLOCK_REALTIME, &ts_start);
			waiting_fall = 1;
		}
		else if (current == LOW) {
			//printf("echo LOW\n");
			if (waiting_fall == 1) {
				//gettimeofday(&tv_end, NULL);
				//long usecond = calc_interval_us(tv_start, tv_end);
				//double distance = 340.0 * usecond / 1000.0; //mm
				clock_gettime(CLOCK_REALTIME, &ts_end);
				long usecond = calc_interval_ns(ts_start, ts_end);
				double distance = 340.0 * usecond / 2 / 1000000000.0; //m
				/*sum += distance;
				count++;
				double avg = sum / count;
				double variance = (distance - avg) * (distance - avg);
				varianceSum += variance;
				double varAvg = varianceSum / count;
				//printf("elps :%ld\n", usecond);
				printf("dis: %f, elps:%ld, avg: %f, var: %f\n", distance, usecond, avg, varAvg);*/
				waiting_fall = 0;
				_distance = distance;
				reset();
			}
			else {
				printf("invalid LOW\n");
			}
		}
	}
	void * run(void *args) {
		while (1) {
			//((HC_SR04*)args)->collect();
			//usleep(1 * 1000);//1ms - 1000hz 
			pinMode(PIN_TRIG, OUTPUT);
			//printf("trigger.\n");
			digitalWrite(PIN_TRIG, HIGH);
			usleep(15);
			digitalWrite(PIN_TRIG, LOW);
			usleep(SR04_CONVERSION_INTERVAL);
		}
		return (void *)0;
	}
	void start()
	{
		wiringPiSetup();
		pinMode(PIN_ECHO, INPUT);
		wiringPiISR(PIN_ECHO, INT_EDGE_BOTH, onChange);
		pthread_t tidp;
		int error;
		PX4_INFO("thread rangfinder start");
		error = ::pthread_create(&tidp, NULL, run, NULL);
		if (error)
		{
			printf("phread is not created...\n");
			return;
		}
		printf("end");
	}
}

HC_SR04::HC_SR04(unsigned sonars) :
	_min_distance(SR04_MIN_DISTANCE),
	_max_distance(SR04_MAX_DISTANCE),
	_reports(nullptr),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr)
{
	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

HC_SR04::~HC_SR04()
{
	stop();
	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}
}

int
HC_SR04::init()
{
	int ret = PX4_ERROR;
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}
	struct distance_sensor_s ds_report = {};
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object. Did you start uOrb?");
	}
	usleep(200000); /* wait for 200ms; */
	_cycling_rate = SR04_CONVERSION_INTERVAL;
	//PX4_INFO("Number of sonars set: %d", _sonars);
	ret = OK;
	
	hc_sr04_c::start();
	start();
	return ret;
}


void
HC_SR04::start()
{
	PX4_INFO("start");
	_reports->flush();
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = SUBSYSTEM_TYPE_RANGEFINDER;
	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
	work_queue(HPWORK, &_work,(worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(_cycling_rate));
}

void
HC_SR04::stop()
{
	PX4_INFO("stop");
	work_cancel(HPWORK, &_work);
}

void
HC_SR04::cycle_trampoline(void *arg)
{
	//PX4_INFO("cycle_trampoline");
	HC_SR04 *dev = (HC_SR04 *)arg;

	dev->cycle();
}

void
HC_SR04::cycle()
{
	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.current_distance = hc_sr04_c::_distance;
	report.id = SR04_ID_BASE;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.orientation = 0;
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}
	_reports->force(&report);
	work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(_cycling_rate));

}

void
HC_SR04::print_info()
{
	_reports->print_info("report queue");
}


void
HC_SR04::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
HC_SR04::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
HC_SR04::get_minimum_distance()
{
	return _min_distance;
}

float
HC_SR04::get_maximum_distance()
{
	return _max_distance;
}
