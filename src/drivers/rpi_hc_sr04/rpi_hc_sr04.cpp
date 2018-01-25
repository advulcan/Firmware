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
/*
 * Driver 'main' command.
 */
extern "C"  __EXPORT int rpi_hc_sr04_main(int argc, char *argv[]);
//static int sonar_isr(int irq, void *context);
/**
 * Local functions in support of the shell command.
 */
namespace  hc_sr04
{

HC_SR04	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
	}
	g_dev = new HC_SR04();
	if (g_dev == nullptr) {
		PX4_ERR("instance nptr");
	}
	PX4_INFO("instance");
	if (OK != g_dev->init()) {
		PX4_ERR("init failed");
	}
	PX4_INFO("inited");
}

void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");
	} else {
		PX4_ERR("driver not running");
	}
}

void
info()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
	}
	else {
		printf("state @ %p\n", g_dev);
		g_dev->print_info();
	}
}

} /* namespace */
//
//static int sonar_isr(int irq, void *context)
//{
//	unsigned time = hrt_absolute_time();
//	/* ack the interrupts we just read */
//
//	if (hc_sr04::g_dev != nullptr) {
//		hc_sr04::g_dev->interrupt(time);
//	}
//
//	return OK;
//}

int
rpi_hc_sr04_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("missing command");
		return 1;
	}else if (!strcmp(argv[1], "start")) {
		hc_sr04::start();
	}
	else if (!strcmp(argv[1], "stop")) {
		hc_sr04::stop();
	}
	else if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		hc_sr04::info();
	}
	else {
		PX4_ERR("unrecognized command, try 'start', 'stop', 'status' or 'info'");
	}
	return 0;
}
