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

HC_SR04::HC_SR04(unsigned sonars) :
	//CDev("HC_SR04", SR04_DEVICE_PATH, 0),
	_min_distance(SR04_MIN_DISTANCE),
	_max_distance(SR04_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "hc_sr04_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hc_sr04_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0), 	/* initialising temp sonar i2c address to zero */
	_sonars(sonars),
	_raising_time(0),
	_falling_time(0),
	_status(0)

{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

HC_SR04::~HC_SR04()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	/*if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}*/

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
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
	PX4_INFO("Number of sonars set: %d", _sonars);
	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
	
	/*init GPIO*/
	wiringPiSetup();
	pinMode(PIN_ECHO, INPUT);
	int i = wiringPiISR(PIN_ECHO, INT_EDGE_BOTH, onChange);
	
	/*start new thread to run sr04*/
	pthread_t tidp;
	int error;
	PX4_INFO("thread rangfinder start");
	error = ::pthread_create(&tidp, NULL, sub_run, this);
	if (error)
	{
		printf("phread is not created...\n");
		return -1;
	}
	start();
	return ret;
}

void onChange(void) {
	int current = digitalRead(PIN_ECHO);
	if (current == HIGH) {
		_rise_time = hrt_absolute_time();
		_waiting_fall = 1;
	}
	else if (current == LOW) {
		if (waiting_fall == 1) {
			_fall_time = hrt_absolute_time();
			uint64_t usecond = _fall_time - _rise_time;
			double distance = 340.0 * usecond / 2 / 1000000.0; //mm
			//sum += distance;
			//count++;
			//double avg = sum / count;
			//double variance = (distance - avg) * (distance - avg);
			//varianceSum += variance;
			//double varAvg = varianceSum / count;
			////printf("elps :%ld\n", usecond);
			printf("dis: %f, elps:%ld, avg: %f", distance, usecond);
			waiting_fall = 0;
		}
		else {
			PX4_INFO("invalid LOW");
		}
	}
}

void
HC_SR04::collect() {
	//PX4_INFO("collect");
	/*_current_distance = _current_distance + 0.001f;
	if (_current_distance > 3) {
		_current_distance = 0;
	}*/
	while (1) {
		pinMode(PIN_TRIG, OUTPUT);
		//printf("trigger.\n");
		digitalWrite(PIN_TRIG, HIGH);
		usleep(15);
		digitalWrite(PIN_TRIG, LOW);
		usleep(1000000);
	}
}
void *
HC_SR04::sub_run(void *args) {
	while (1) {
		((HC_SR04*)args)->collect();
		usleep(1 * 1000);//1000hz 

	}
	return (void *)0;
}
int
HC_SR04::probe()
{
	return (OK);
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
void
HC_SR04::interrupt(unsigned time)
{
	if (_status == 0) {
		_raising_time = time;
		_status++;
		return;

	} else if (_status == 1) {
		_falling_time = time;
		_status++;
		return;
	}

	return;
}

int
HC_SR04::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return 0;
}

ssize_t
HC_SR04::read(struct file *filp, char *buffer, size_t buflen)
{
	return 0;
}

int
HC_SR04::measure()
{
	return 0;
}




void
HC_SR04::start()
{
	PX4_INFO("start");
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();
	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = SUBSYSTEM_TYPE_RANGEFINDER;
	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
		PX4_INFO("31");
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
		PX4_INFO("32");
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
	report.current_distance = _current_distance;
	report.id = SR04_ID_BASE;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.orientation = 0;
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}
	_reports->force(&report);
	/*if (OK != collect()) {
		PX4_ERR("collection error");
	}*/
	work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(_cycling_rate));

}

void
HC_SR04::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

const HC_SR04::GPIOConfig HC_SR04::_gpio_tab[] = {
	//{GPIO_GPIO6_OUTPUT,      GPIO_GPIO7_INPUT,       0},
	//{GPIO_GPIO6_OUTPUT,      GPIO_GPIO8_INPUT,       0},
	//{GPIO_GPIO6_OUTPUT,      GPIO_GPIO9_INPUT,       0},
	//{GPIO_GPIO6_OUTPUT,      GPIO_GPIO10_INPUT,       0},
	//{GPIO_GPIO6_OUTPUT,      GPIO_GPIO11_INPUT,       0},
	{ PIN_TRIG,      PIN_ECHO,       0 }
};