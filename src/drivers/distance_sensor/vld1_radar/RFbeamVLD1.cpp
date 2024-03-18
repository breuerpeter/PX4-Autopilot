#include "RFbeamVLD1.hpp"

// TODO: figure out why this is not in header file
#include <lib/drivers/device/Device.hpp>

RFbeamVLD1::RFbeamVLD1(const char *port, uint8_t rotation)
	: ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)), _px4_rangefinder(0, rotation)
{
	// Store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null terminal
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);  // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_RFBEAM);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
}

RFbeamVLD1::~RFbeamVLD1()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int RFbeamVLD1::init()
{
	// Update parameter values (only this one time)
	ModuleParams::updateParams();

	// Open serial port before writing to it
	open_serial_port();

	/* ------------------------- Initialization command ------------------------- */

	int bytes_written = ::write(_fd, _cmd_INIT_default, INIT_PACKET_BYTES);

	if (bytes_written != INIT_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: init cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	/* --------------------------- Target filter mode --------------------------- */

	uint8_t *TGFI_msg = _cmd_TGFI_default;
	int32_t tgfi_setting = _param_sensor_tgfi.get();

	switch (tgfi_setting) {

	case RFBEAM_PARAM_TGFI_DEFAULT:
		PX4_INFO("DEBUG: target filter setting: nearest (default)");
		break;

	case 0:
		TGFI_msg = _cmd_TGFI_strong;
		PX4_INFO("DEBUG: target filter setting: strongest");
		break;

	case 2:
		TGFI_msg = _cmdTGFI_FAR;
		PX4_INFO("DEBUG: target filter setting: farthest");
		break;

	default:
		PX4_ERR("invalid target filter setting: %" PRId32, tgfi_setting);
		return PX4_ERROR;
	}

	bytes_written = ::write(_fd, TGFI_msg, TGFI_PACKET_BYTES);

	if (bytes_written != TGFI_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: TGFI cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	/* ------------------------------- Range mode ------------------------------- */

	uint8_t *RRAI_msg = _cmd_RRAI_default;
	int32_t range_setting = _param_sensor_range.get();

	switch (range_setting) {
	case RFBEAM_PARAM_RNG_DEFAULT:
		PX4_INFO("DEBUG: max range setting: 20 m (default)");
		_range_resolution_cm = RFBEAM_RANGE_RESOLUTION_20M_CM;
		break;

	case 1:
		RRAI_msg = _cmd_RRAI_50;
		PX4_INFO("DEBUG: max range setting: 50 m");
		_range_resolution_cm = RFBEAM_RANGE_RESOLUTION_50M_CM;
		break;

	default:
		PX4_ERR("invalid range setting: %" PRId32, range_setting);
		return PX4_ERROR;
	}

	bytes_written = ::write(_fd, RRAI_msg, RRAI_PACKET_BYTES);

	if (bytes_written != RRAI_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: RRAI cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	/* --------------------------- Short range filter --------------------------- */

	uint8_t *SRDF_msg = _cmd_SRDF_default;
	int32_t short_range_filter_enabled = _param_sensor_srng.get();

	switch (short_range_filter_enabled) {
	case RFBEAM_PARAM_SRNG_DEFAULT:
		PX4_INFO("DEBUG: short range filter: off (default)");
		break;

	case 1:
		SRDF_msg = _cmd_SRDF_on;
		PX4_INFO("DEBUG: short range filter: on");
		break;

	default:
		PX4_ERR("invalid short range filter setting: %" PRId32, short_range_filter_enabled);
		return PX4_ERROR;
	}

	bytes_written = ::write(_fd, SRDF_msg, SRDF_PACKET_BYTES);

	if (bytes_written != SRDF_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: SRDF cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	/* -------------------------- Minimum range filter -------------------------- */

	uint8_t *MIRA_msg = _cmd_MIRA_default;
	int32_t min_range_setting = _param_sensor_minf.get();

	// Calculate and set minimum detection distance (according to datasheet)
	float min_distance_m = (min_range_setting * _range_resolution_cm + RFBEAM_INTERNAL_OFFSET_CM) / 100;
	_px4_rangefinder.set_min_distance(min_distance_m);

	switch (min_range_setting) {
	case RFBEAM_PARAM_MINF_DEFAULT:
		PX4_INFO("DEBUG: min range filter: bin %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_MINF_DEFAULT);
		break;

	default:
		if (min_range_setting >= RFBEAM_PARAM_MINF_MIN && min_range_setting <= RFBEAM_PARAM_MINF_MAX) {
			uint16_t min_range = (uint16_t)min_range_setting;
			// Fill payload section (LSB first) of MIRA packet
			MIRA_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)(min_range & 0xFF); // mask away the higher byte
			MIRA_msg[PACKET_PAYLOAD_START_IDX + 1] = (uint8_t)(min_range >> 8); //
			PX4_INFO("DEBUG: min range filter: bin %" PRId32, min_range_setting);
			break;

		} else {
			PX4_ERR("invalid min range filter setting: %" PRId32, min_range_setting);
			return PX4_ERROR;
		}
	}

	bytes_written = ::write(_fd, MIRA_msg, MIRA_PACKET_BYTES);

	if (bytes_written != MIRA_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: MIRA cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	// /* -------------------------- Maximum range filter -------------------------- */

	uint8_t *MARA_msg = _cmd_MARA_default;
	int32_t max_range_setting = _param_sensor_maxf.get();

	// Calculate and set maximum detection distance (according to datasheet)
	float max_distance_m = (max_range_setting * _range_resolution_cm + RFBEAM_INTERNAL_OFFSET_CM) / 100;
	_px4_rangefinder.set_max_distance(max_distance_m);

	switch (max_range_setting) {
	case RFBEAM_PARAM_MAXF_DEFAULT:
		PX4_INFO("DEBUG: max range filter: bin %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_MAXF_DEFAULT);
		break;

	default:
		if (max_range_setting >= RFBEAM_PARAM_MAXF_MIN && max_range_setting <= RFBEAM_PARAM_MAXF_MAX) {
			uint16_t max_range = (uint16_t)max_range_setting;
			// Fill payload section (LSB first) of MARA packet
			MARA_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)(max_range & 0xFF); // mask away the higher byte
			MARA_msg[PACKET_PAYLOAD_START_IDX + 1] = (uint8_t)(max_range >> 8); //
			PX4_INFO("DEBUG: max range filter: bin %" PRId32, max_range_setting);
			break;

		} else {
			PX4_ERR("invalid max range filter setting: %" PRId32 ".", max_range_setting);
			return PX4_ERROR;
		}
	}

	bytes_written = ::write(_fd, MARA_msg, MARA_PACKET_BYTES);

	if (bytes_written != MARA_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: MARA cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	// /* ---------------------------- Threshold offset ---------------------------- */

	uint8_t *THOF_msg = _cmd_THOF_default;
	int32_t threshold_offset = _param_sensor_thrs.get();

	switch (threshold_offset) {
	case RFBEAM_PARAM_THRS_DEFAULT:
		PX4_INFO("DEBUG: threshold offset: %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_THRS_DEFAULT);
		break;

	default:
		if (threshold_offset >= RFBEAM_PARAM_THRS_MIN && threshold_offset <= RFBEAM_PARAM_THRS_MAX) {
			// Fill payload section (LSB first) of THOF packet
			THOF_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)threshold_offset;
			PX4_INFO("DEBUG: threshold offset: %" PRId32, threshold_offset);
			break;

		} else {
			PX4_ERR("invalid threshold offset: %" PRId32, threshold_offset);
			return PX4_ERROR;
		}
	}

	bytes_written = ::write(_fd, THOF_msg, THOF_PACKET_BYTES);

	if (bytes_written != THOF_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: THOF cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);

	/* ---------------------------- Chirp integration --------------------------- */

	uint8_t *INTN_msg = _cmd_INTN_default;
	int32_t chirp_count = _param_sensor_chrp.get();

	switch (chirp_count) {
	case RFBEAM_PARAM_CHRP_DEFAULT:
		PX4_INFO("DEBUG: chirp integration count: %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_CHRP_DEFAULT);
		break;

	default:
		if (chirp_count >= RFBEAM_PARAM_CHRP_MIN && chirp_count <= RFBEAM_PARAM_CHRP_MAX) {
			// Fill payload section (LSB first) of INTN packet
			INTN_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)chirp_count;
			PX4_INFO("chirp integration count: %" PRId32, chirp_count);
			break;

		} else {
			PX4_ERR("invalid chirp integration count: %" PRId32, chirp_count);
			return PX4_ERROR;
		}
	}

	bytes_written = ::write(_fd, INTN_msg, INTN_PACKET_BYTES);

	if (bytes_written != INTN_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: INTN cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);


	/* --------------------------- Distance averaging --------------------------- */

	uint8_t *RAVG_msg = _cmd_RAVG_default;
	int32_t average = _param_sensor_avg.get();

	switch (average) {
	case RFBEAM_PARAM_AVG_DEFAULT:
		PX4_INFO("DEBUG: measurements to average: %" PRId32 " (default)", (uint32_t)RFBEAM_PARAM_AVG_DEFAULT);
		break;

	default:
		if (average >= RFBEAM_PARAM_AVG_MIN && average <= RFBEAM_PARAM_AVG_MAX) {
			// Fill payload section (LSB first) of RAVG packet
			RAVG_msg[PACKET_PAYLOAD_START_IDX] = (uint8_t)average;
			PX4_INFO("DEBUG: measurements to average: %" PRId32, average);
			break;

		} else {
			PX4_ERR("invalid average setting: %" PRId32, average);
			return PX4_ERROR;
		}
	}

	bytes_written = ::write(_fd, RAVG_msg, RAVG_PACKET_BYTES);

	if (bytes_written != RAVG_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: RAVG cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait to give the sensor some time to process (50 ms)
	px4_usleep(50000);


	/* ----------------------------------- End ---------------------------------- */

	// Close the fd
	::close(_fd);
	_fd = -1;

	start();

	return PX4_OK;
}

int RFbeamVLD1::measure()
{
	// Flush the receive buffer
	tcflush(_fd, TCIFLUSH);

	int bytes_written = ::write(_fd, _cmd_GNFD_PDAT, GNFD_PACKET_BYTES);

	if (bytes_written != GNFD_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_DEBUG("measure cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// _read_buffer_len = 0;
	return PX4_OK;
}

int RFbeamVLD1::collect()
{
	perf_begin(_sample_perf);

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		PX4_INFO("DEBUG: nothing in RX buffer");
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	PX4_INFO("DEBUG: bytes_available: %d:", bytes_available);

	// int64_t read_elapsed = hrt_elapsed_time(&_last_read_time);

	_read_time = hrt_absolute_time(); // TODO: e.g. LeddarOne driver sets timestamp in measure()

	const int buffer_size = sizeof(_read_buffer);
	PX4_INFO("DEBUG: buffer_size: %d:", buffer_size);

	/* const int message_size = sizeof(PDAT_msg);
	PX4_INFO("DEBUG: message_size: %d:", message_size); */

	// int bytes_read = ::read(_fd, _read_buffer + _read_buffer_len, buffer_size - _read_buffer_len);
	int bytes_read = ::read(_fd, _read_buffer, buffer_size);

	PX4_INFO("DEBUG: bytes_read: %d:", bytes_read);

	if (bytes_read < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		return PX4_ERROR;

		/* // Only throw error on timeout
		if (read_elapsed > (_interval_us * 2)) {
			PX4_INFO("DEBUG: timeout on read");
			return bytes_read;

		} else {
			PX4_INFO("DEBUG: read error: %d", bytes_read);
			return -EAGAIN;
		}

		} else if (bytes_read == 0) {
		PX4_INFO("DEBUG: 0 bytes read");
		return -EAGAIN; */
	}

	/* _read_buffer_len += bytes_read;

	if (_read_buffer_len < message_size) {
		PX4_INFO("DEBUG: incomplete read");
		// Return on next scheduled cycle to collect remaining data
		return -EAGAIN;
	}

	_last_read_time = hrt_absolute_time();

	float distance_m = 5.0f;

	PDAT_msg *msg = nullptr;
	msg = (PDAT_msg *)_read_buffer;

	distance_m = msg->distance;

	PX4_INFO("DEBUG: distance_m: %f:", (double)distance_m); */

	else if (bytes_read != 23) {
		PX4_INFO("DEBUG: no target detected, bytes read: %d", bytes_read);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	uint8_t _read_buffer_distance[sizeof(float)] = {}; // should be 4 bytes according to sensor datasheet

	PX4_INFO("DEBUG: distance buffer size: %d:", sizeof(float));

	_read_buffer_distance[0] = _read_buffer[17 + 0];
	PX4_INFO("DEBUG: distance buffer element 1/4: %d:", _read_buffer_distance[0]);
	_read_buffer_distance[1] = _read_buffer[17 + 1];
	PX4_INFO("DEBUG: distance buffer element 2/4: %d:", _read_buffer_distance[1]);
	_read_buffer_distance[2] = _read_buffer[17 + 2];
	PX4_INFO("DEBUG: distance buffer element 3/4: %d:", _read_buffer_distance[2]);
	_read_buffer_distance[3] = _read_buffer[17 + 3];
	PX4_INFO("DEBUG: distance buffer element 4/4: %d:", _read_buffer_distance[3]);

	// float distance_m = *((float *)_read_buffer_distance);

	float distance_m;
	memcpy(&distance_m, _read_buffer_distance, sizeof(distance_m));

	PX4_INFO("DEBUG: distance: %f:", (double)distance_m);

	// Send via uORB
	_px4_rangefinder.update(_read_time, distance_m); // TODO: distance_m, may need LSB conversion?

	perf_end(_sample_perf);

	return PX4_OK;
}

int RFbeamVLD1::open_serial_port(const speed_t speed)
{
	// Skip the rest if  descriptor already initialized
	if (_fd > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	// Configure port flags (read/write, non-controlling, non-blocking)
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port
	_fd = ::open(_port, flags);

	if (_fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_fd)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	struct termios uart_config;

	int termios_state;

	// Store the current port configuration attributes
	tcgetattr(_fd, &uart_config);

	/* Input flags (turn off input processing):
	Convert break to null byte, no CR to NL translation,
	No NL to CR translation, don't mark parity errors or breaks
	No input parity check, don't strip high bit off,
	No XON/XOFF software flow control */
	// uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);

	// Output flags (turn off output processing):
	// uart_config.c_oflag &= ~OPOST;
	// uart_config.c_oflag = 0;
	// Clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// 8 data bits
	// uart_config.c_cflag &= ~CSIZE;
	// uart_config.c_cflag |= CS8;

	// Enable receiver
	// uart_config.c_cflag |= CREAD;

	// Ignore modem status lines
	// uart_config.c_cflag |= CLOCAL;

	// One stop bit (clear 2 stop bits flag)
	uart_config.c_cflag &= ~CSTOPB;

	// Even parity
	uart_config.c_cflag &= ~PARODD;

	uart_config.c_cflag |= PARENB;

	// No flow control (clear flow control flag)
	uart_config.c_cflag &= ~CRTSCTS;

	/* Turn off line processing:
	Echo off, echo newline off, canonical mode off, extended input processing off, signal chars off
	*/
	// uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Set the input baud rate in the uart_config struct
	termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	// Apply the modified port attributes
	termios_state = tcsetattr(_fd, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	// Flush the hardware buffers // TODO: not sure if this is necessary
	// tcflush(_fd, TCIOFLUSH);

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void RFbeamVLD1::Run()
{
	// Ensure the serial port is open
	open_serial_port();

	// Collection phase
	if (_collect_phase) {
		int ret = collect();

		if (ret != PX4_OK) {
			// Case 1/2: try again upon incomplete/failed read
			if (ret == -EAGAIN) {
				PX4_INFO("DEBUG: trying read again");
				// Reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps // TODO: choose right interval
				ScheduleClear();
				ScheduleOnInterval(7_ms, 87 * 9);
			}

			// Case 2/2: data collection error (timeout)
			else {
				PX4_INFO("DEBUG: collection error");
				// Restart the measurement state machine i.e. skip collection on next iteration
				start();
			}

			return;
		}

		// Next phase is measurement
		_collect_phase = false;
	}

	// Measurement phase
	if (measure() != PX4_OK) {
		PX4_INFO("DEBUG: measure error");
	}

	// Next phase is collection
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(_interval_us);
}

int RFbeamVLD1::request_sensor_settings()
{
	int bytes_written = ::write(_fd, _cmd_GRPS, GRPS_PACKET_BYTES);

	if (bytes_written != GRPS_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: GRPS cmd write fail %d", bytes_written);
		return bytes_written;
	}

	return PX4_OK;
}

int RFbeamVLD1::restore_factory_settings()
{
	int bytes_written = ::write(_fd, _cmd_RFSE, RFSE_PACKET_BYTES);

	if (bytes_written != RFSE_PACKET_BYTES) {
		perf_count(_comms_errors);
		PX4_INFO("DEBUG: RFSE cmd write fail %d", bytes_written);
		return bytes_written;
	}

	return PX4_OK;
}

void RFbeamVLD1::start()
{
	// Reset the report ring and state machine
	_collect_phase = false;

	// Schedule a cycle to start things
	ScheduleDelayed(5); // 5 us
}

void RFbeamVLD1::stop()
{
	// Ensure the serial port is closed
	::close(_fd);

	// Clear the work queue schedule
	ScheduleClear();
}

void RFbeamVLD1::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
