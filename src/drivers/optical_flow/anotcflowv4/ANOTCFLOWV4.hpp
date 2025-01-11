/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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
 * @file ANOTCFLOWV4.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Christoph Tobler <christoph@px4.io>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 *
 * Driver for the niming ANOTCFLOWV4 laser rangefinder series
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>

#include <uORB/topics/sensor_optical_flow.h>
#include <conversion/rotation.h>

#include "anotcflowv4_parser.h"

#define ANOTCFLOWV4_DEFAULT_PORT "/dev/ttyS2"

using namespace time_literals;

class ANOTCFLOWV4 : public px4::ScheduledWorkItem {
    public:
	ANOTCFLOWV4(const char* port, enum Rotation rotation = Rotation::ROTATION_NONE);
	virtual ~ANOTCFLOWV4();
	int init();
	void print_info();

    private:
	int collect();
	void Run() override;
	void start();
	void stop();
	void fill_report(const ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN_STRUCT parse_result, uint64_t timestamp_sample);

	uORB::PublicationMulti<sensor_optical_flow_s> _sensor_optical_flow_pub { ORB_ID(sensor_optical_flow) };

	int _fd { -1 };
	char _port[20] {};
	static constexpr int kCONVERSIONINTERVAL { 9_ms };

	char _linebuf[static_cast<int>(ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE11_AC_VALUE_OF_ORIGIN)] {};
	ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN _parse_state { ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN::STATE1_HEAD };
	unsigned int _linebuf_index { 0 };

	hrt_abstime _last_read { 0 };

	perf_counter_t _comms_errors { perf_alloc(PC_COUNT, MODULE_NAME ": com_err") };
	perf_counter_t _sample_perf { perf_alloc(PC_ELAPSED, MODULE_NAME ": read") };

	enum Rotation _yaw_rotation {};
	uint64_t _previous_collect_timestamp { 0 };
	int _flow_sum_x { 0 };
	int _flow_sum_y { 0 };
	uint64_t _flow_dt_sum_usec { 0 };
	uint16_t _flow_quality_sum { 0 };
	uint8_t _flow_sample_counter { 0 };
};
