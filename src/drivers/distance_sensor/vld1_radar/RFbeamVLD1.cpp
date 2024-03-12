#define DEBUG_BUILD

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

	_px4_rangefinder.set_min_distance(RFBEAM_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(RFBEAM_MAX_DISTANCE);
}

RFbeamVLD1::~RFbeamVLD1()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int RFbeamVLD1::init()
{
	// Open serial port before writing to it
	open_serial_port();

	int bytes_written = ::write(_file_descriptor, _cmdINIT, sizeof(_cmdINIT));

	if (bytes_written != sizeof(_cmdINIT)) {
		perf_count(_comms_errors);
		PX4_DEBUG("init cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait for a while (50 ms)
	px4_usleep(50000);

	// int32_t target_filter_mode = 0;
	// param_get(param_find("SENS_VLD1_MODE"), &target_filter_mode);

	const uint8_t *TGFI_msg = _cmdTGFI_STRONG;

	// Update parameter values (only this one time)
	ModuleParams::updateParams();

	switch (_param_sensor_mode.get()) {
	case 0: // provide strongest reading
		TGFI_msg = _cmdTGFI_STRONG;
		PX4_DEBUG("target filter mode: strongest");
		break;

	case 1: // provide nearest reading
		TGFI_msg = _cmdTGFI_NEAR;
		PX4_DEBUG("target filter mode: nearest");
		break;

	case 2: // provide farthest reading
		TGFI_msg = _cmdTGFI_FAR;
		PX4_DEBUG("target filter mode: farthest");
		break;

	default:
		TGFI_msg = _cmdTGFI_STRONG;
		PX4_DEBUG("target filter mode: default fallback (strongest)");
	}

	bytes_written = ::write(_file_descriptor, TGFI_msg, sizeof(TGFI_msg));

	if (bytes_written != sizeof(TGFI_msg)) {
		perf_count(_comms_errors);
		PX4_DEBUG("TGFI cmd write fail %d", bytes_written);
		return bytes_written;
	}

	// TODO: check for response from sensor

	// Wait for a while (50 ms)
	px4_usleep(50000);

	// int32_t max_range_mode = 0;
	// param_get(param_find("SENS_VLD1_RNG"), &max_range_mode);

	const uint8_t *RRAI_msg = _cmdRRAI20;

	switch (_param_sensor_range.get()) {
	case 0: // 20 m setting
		RRAI_msg = _cmdRRAI20;
		PX4_DEBUG("max range mode: 20 m");
		break;

	case 1: // 50 m setting
		RRAI_msg = _cmdRRAI50;
		PX4_DEBUG("max range mode: 50 m");
		break;

	default:
		RRAI_msg = _cmdRRAI20;
		PX4_DEBUG("max range mode: default fallback (20 m)");
	}

	bytes_written = ::write(_file_descriptor, RRAI_msg, sizeof(RRAI_msg));

	if (bytes_written != sizeof(RRAI_msg)) {
		perf_count(_comms_errors);
		PX4_DEBUG("RRAI cmd write fail %d", bytes_written);
		return bytes_written;
	}

	start();

	// TODO: check for response from sensor

	return PX4_OK;

	// TODO: e.g. LeddarOne driver has more thorough init routine
}

int RFbeamVLD1::measure()
{
	// Flush the receive buffer // TODO: not sure if this is necessary
	tcflush(_file_descriptor, TCIFLUSH);

	int bytes_written = ::write(_file_descriptor, _cmdGNFD, sizeof(_cmdGNFD));

	if (bytes_written != sizeof(_cmdGNFD)) {
		perf_count(_comms_errors);
		PX4_DEBUG("measure cmd write fail %d", bytes_written);
		return bytes_written;
	}

	_read_buffer_len = 0;
	return PX4_OK;
}

int RFbeamVLD1::collect()
{
	perf_begin(_sample_perf);

	int64_t read_elapsed = hrt_elapsed_time(&_last_read_time);

	_read_time = hrt_absolute_time(); // TODO: e.g. LeddarOne driver sets timestamp in measure()

	const int buffer_size = sizeof(_read_buffer);
	const int message_size = sizeof(reading_msg);

	int bytes_read = ::read(_file_descriptor, _read_buffer + _read_buffer_len, buffer_size - _read_buffer_len);

	/* // Buffer for reading chars is buffer length minus null termination
	char readbuf[sizeof(_read_buffer)]; // TODO: why is this step even needed?
	unsigned readlen = sizeof(_read_buffer) - 1;

	// Read from sensor (UART buffer)
	int bytes_read = ::read(_file_descriptor, &_read_buffer[0], readlen); */

	if (bytes_read < 0) {
		PX4_DEBUG("read err: %d", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		// Throw an error on timeout
		if (read_elapsed > (_interval_us * 2)) {
			return bytes_read;

		} else {
			return -EAGAIN;
		}

	} else if (bytes_read == 0) {
		return -EAGAIN;
	}

	_read_buffer_len += bytes_read;

	if (_read_buffer_len < message_size) {
		// Return on next scheduled cycle to collect remaining data
		return PX4_OK;
	}

	_last_read_time = hrt_absolute_time();

	// reading_msg *msg {nullptr};
	// msg = (reading_msg *)_read_buffer;

	// float distance_m = msg->distance;

	// Send via uORB
	_px4_rangefinder.update(_read_time, 2.0f); // TODO: distance_m, may need LSB conversion?

	perf_end(_sample_perf);

	return PX4_OK;
}

int RFbeamVLD1::open_serial_port(const speed_t speed)
{
	// File descriptor already initialized?
	if (_file_descriptor > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	// Configure port flags (read/write, non-controlling, non-blocking)
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	termios uart_config{};

	// Store the current port configuration attributes
	tcgetattr(_file_descriptor, &uart_config);

	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// No line processing - echo off, echo newline off, canonical mode off, extended input processing off, signal chars off
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Set the input baud rate in the uart_config struct
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Flush the hardware buffers // TODO: not sure if this is necessary
	tcflush(_file_descriptor, TCIOFLUSH);

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void RFbeamVLD1::Run()
{
	// Ensure the serial port is open
	open_serial_port();

	// Collection phase
	if (_collect_phase) {
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			// Restart the measurement state machine
			start();
			return;
		}

		// Next phase is measurement
		_collect_phase = false;
	}

	// Measurement phase
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	// Next phase is collection
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(_interval_us);
}

void RFbeamVLD1::start()
{
	// Reset the report ring and state machine
	_collect_phase = false;

	// Schedule a cycle to start things
	ScheduleNow();
}

void RFbeamVLD1::stop()
{
	// Ensure the serial port is closed
	::close(_file_descriptor);

	// Clear the work queue schedule
	ScheduleClear();
}

void RFbeamVLD1::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
