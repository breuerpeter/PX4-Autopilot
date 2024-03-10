#include "RFbeamVLD1.hpp"

// TODO: figure out why this is not in header file
#include <lib/drivers/device/Device.hpp>

RFbeamVLD1::RFbeamVLD1(const char *port, uint8_t rotation)
    : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)), _px4_rangefinder(0, rotation) {
        // Store port name
        strncpy(_port, port, sizeof(_port) - 1);

        // Enforce null terminal
        _port[sizeof(_port) - 1] = '\0';

        device::Device::DeviceId device_id;
        device_id.device_s.bus_type = device::Device::Device::DeviceBusType_SERIAL;

        uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);  // Assuming '/dev/ttySx'

        if (bus_num < 10) {
                device_id.device_s.bus = bus_num;
        }
        _px4_rangefinder.set_device_id(device_id.devid);
        _px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_RFBEAM);
        _px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);

        _px4_rangefinder.set_min_distance(RFBEAM_MIN_DISTANCE);
        _px4_rangefinder.set_max_distance(RFBEAM_MAX_DISTANCE);
}

RFbeamVLD1::~RFbeamVLD1() {
        stop();

        perf_free(_sample_perf);
        perf_free(_comms_errors);
}

int RFbeamVLD1::init() {
        start();

        return PX4_OK;
}

int RFbeamVLD1::collect() {
        perf_begin(_sample_perf);
\
        float distance_m = -1.0f;

        // TODO: Send command to radar
        // ::write()

        // TODO: Read from sensor UART buffer
        const hrt_abstime timestamp_sample = hrt_absolute_time();

        // Send via uORB
        _px4_rangefinder.update(timestamp_sample, 2.0f); // TODO: distance_m

        perf_end(_sample_perf);

        return PX4_OK;
}

int RFbeamVLD1::open_serial_port(const speed_t speed) {
        // File descriptor already initialized?
        if (_file_descriptor > 0) {
                PX4_DEBUG("serial port already open");
                return PX4_OK;
        }

        // Configure port flags for read/write, non-controlling, non-blocking
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

        // Store the current port configuration. attributes
        tcgetattr(_file_descriptor, &uart_config);

        uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

        // Clear ONLCR flag (which appends a CR for every LF)
        uart_config.c_oflag &= ~ONLCR;

        // No parity, one stop bit.
        uart_config.c_cflag &= ~(CSTOPB | PARENB);

        // No line processing - echo off, echo newline off, canonical mode off, extended input processing off, signal
        // chars off
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

        PX4_INFO("successfully opened UART port %s", _port);
        return PX4_OK;
}

void RFbeamVLD1::Run() {
        // Ensure the serial port is open
        open_serial_port();

        collect();
}

void RFbeamVLD1::start() {
        // Schedule the driver at regular intervals
        ScheduleOnInterval(RFBEAM_MEASURE_INTERVAL, 0);
}

void RFbeamVLD1::stop() {
        // Ensure the serial port is closed
        ::close(_file_descriptor);

        // Clear the work queue schedule
        ScheduleClear();
}

void RFbeamVLD1::print_info() {
        perf_print_counter(_sample_perf);
        perf_print_counter(_comms_errors);
}
