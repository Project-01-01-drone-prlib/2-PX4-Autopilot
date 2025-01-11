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

#include "ANOTCFLOWV4.hpp"

#include <fcntl.h>
#include <lib/drivers/device/Device.hpp>

ANOTCFLOWV4::ANOTCFLOWV4(const char* port, enum Rotation rotation)
    : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
    _yaw_rotation(rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);
	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
}

ANOTCFLOWV4::~ANOTCFLOWV4()
{
	// make sure we are truly inactive
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int ANOTCFLOWV4::init()
{
	int32_t hw_model = 1; // only one model so far...

	switch (hw_model) {
	case 1: // ANOTCFLOWV4 (12m, 100 Hz)
		// Note:
		// Sensor specification shows 0.3m as minimum, but in practice
		// 0.3 is too close to minimum so chattering of invalid sensor decision
		// is happening sometimes. this cause EKF to believe inconsistent range readings.
		// So we set 0.4 as valid minimum.
		//_px4_rangefinder.set_min_distance(0.05f);
		//_px4_rangefinder.set_max_distance(12.0f);
		//_px4_rangefinder.set_fov(math::radians(1.15f));

		break;

	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
		return -1;
	}

	// status
	int ret = 0;

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		// baudrate 500000, 8 bits, no parity, 1 stop bit
		unsigned speed = B500000;
		termios uart_config {};
		int termios_state {};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD); // ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8; // 8-bit characters
		uart_config.c_cflag &= ~PARENB; // no parity bit
		uart_config.c_cflag &= ~CSTOPB; // only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

void ANOTCFLOWV4::fill_report(const ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN_STRUCT parse_result, uint64_t timestamp_sample)
{
	sensor_optical_flow_s report {};
	report.timestamp_sample = timestamp_sample;

	report.pixel_flow[0] = static_cast<float>(parse_result.dx_0);
	report.pixel_flow[1] = static_cast<float>(parse_result.dy_0);

	// rotate measurements in yaw from sensor frame to body frame
	float zeroval = 0.0f;
	rotate_3f(_yaw_rotation, report.pixel_flow[0], report.pixel_flow[1], zeroval);

	report.integration_timespan_us = 20000; // microseconds
	report.quality = parse_result.quality;

	/* No gyro on this board */
	report.delta_angle[0] = NAN;
	report.delta_angle[1] = NAN;
	report.delta_angle[2] = NAN;

	// set (conservative) specs according to datasheet
	report.max_flow_rate = 5.0f; // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.1f; // Datasheet: 80mm
	report.max_ground_distance = 30.0f; // Datasheet: infinity
	report.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(report);

	_flow_dt_sum_usec = 0;
	_flow_sum_x = 0;
	_flow_sum_y = 0;
	_flow_sample_counter = 0;
	_flow_quality_sum = 0;
}

int ANOTCFLOWV4::collect()
{

	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	int ret = 0;
	int ret_parse = -1;
	ANOTCFLOWV4_PARSE_STATE_OF_ORIGIN_STRUCT optflow_origin_date = { 0 };

	// Check the number of bytes available in the buffer
	char readbuf[sizeof(_linebuf)] {};
	unsigned readlen = sizeof(readbuf);
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return 0;
	}

	// parse entire buffer
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		// read from the sensor (uart buffer)
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		// parse buffer
		for (int i = 0; i < ret; i++) {
			ret_parse = anotcflowv4_parse_of_origin(
			    readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &optflow_origin_date);
			if (ret_parse == 0) {
				tcflush(_fd, TCIFLUSH);
				break;
			}
		}
		if (ret_parse == 0) {
			break;
		}
		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);

	if (ret_parse == -1) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}
	fill_report(optflow_origin_date, timestamp_sample);
	perf_end(_sample_perf);
	return PX4_OK;
}

void ANOTCFLOWV4::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(7_ms);
}

void ANOTCFLOWV4::stop() { ScheduleClear(); }

void ANOTCFLOWV4::Run()
{
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(7_ms, 87 * 9);
		return;
	}
}

void ANOTCFLOWV4::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
