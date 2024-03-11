#pragma once

#include <termios.h>
#include <unistd.h>

#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>

// TODO: figure out what the following includes are for/if needed at all
#include <fcntl.h>
#include <poll.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

using namespace time_literals;

// Datasheet values independent of settings
#define RFBEAM_STARTUP_TIME         15_ms

// Settings
#define RFBEAM_RANGE_SETTING_M      20  // 20 m or 50 m
#define RFBEAM_HIGH_PRECISION_MODE  1   // on (1) or off (0); on by default for higher precision
#define RFBEAM_CHIRP_INTEGRATION    1   // 1-100; higher value means slower rate but higher SNR
#define RFBEAM_SHORT_RANGE_FILTER   1   // on (1) or off (0); choose on to enable short range measurements of strong reflectors

// Values depending on settings
#if RFBEAM_RANGE_SETTING_M == 20
#define RFBEAM_MIN_DISTANCE         0.039f
#define RFBEAM_MAX_DISTANCE         20.14f
#else
#define RFBEAM_MIN_DISTANCE         0.099f
#define RFBEAM_MAX_DISTANCE         50.91f
#endif

#if RFBEAM_HIGH_PRECISION_MODE == 1
#define RFBEAM_FRAME_PROC_TIME      21_ms
#define RF_BEAM_RESOLUTION          0.001f // in meters
#else
#define RFBEAM_FRAME_PROC_TIME      15_ms
#if RF_BEAM_RANGE_SETTING_M == 20
#define RF_BEAM_RESOLUTION          0.03934f // in meters
#else
#define RF_BEAM_RESOLUTION          0.09943f // in meters
#endif
#endif

#if RFBEAM_SHORT_RANGE_FILTER == 1
#define RFBEAM_MEASURE_INTERVAL     RFBEAM_FRAME_PROC_TIME + (RFBEAM_CHIRP_INTEGRATION - 1) * (3_ms + 5_ms)
#else
#define RFBEAM_MEASURE_INTERVAL     RFBEAM_FRAME_PROC_TIME + (RFBEAM_CHIRP_INTEGRATION - 1) * 3_ms
#endif

/**
 * Assume standard deviation to be equal to sensor resolution.
 * Static bench tests have shown that the sensor output does
 * not vary if the unit is not moved.
 */
#define SENS_VARIANCE               RF_BEAM_RESOLUTION * RF_BEAM_RESOLUTION

// Packet format
#define PACKET_HEADER_BYTES                     4 // Number of bytes to specify packet header
#define PACKET_PAYLOAD_LENGTH_BYTES             4 // Number of bytes to specify payload length

// Commands
#define INIT_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for INIT command
#define GNFD_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for GNFD command
#define GRPS_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 0                        // Payload length for GRPS command
#define RFSE_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 0                        // Payload length for RFSE command
#define GBYE_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 0                        // Payload length for GBYE command
#define RRAI_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for RRAI command
#define THOF_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for THOF command
#define MIRA_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 2 * sizeof(uint16_t)     // Payload length for MIRA command
#define MARA_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 2 * sizeof(uint16_t)     // Payload length for MARA command
#define RAVG_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for RAVG command
#define TGFI_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for TGFI command
#define PREC_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for PREC command
#define TXPW_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for TXPW command
#define INTN_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for INTN command
#define SRDF_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 1 * sizeof(uint8_t)      // Payload length for SRDF command
#define JBTL_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 0                        // Payload length for JBTL command

// Messages
#define RESP_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 2048 * sizeof(uint8_t)   // Payload length for RESP command
#define RADC_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 2048 * sizeof(int16_t)   // Payload length for RADC command
#define DONE_PACKET_BYTES       PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES + 4 * sizeof(uint32_t)     // Payload length for DONE command

struct __attribute__((__packed__)) reading_msg {
        float distance;
        uint16_t mag;
};

class RFbeamVLD1 : public px4::ScheduledWorkItem {
       public:
        /**
         * @brief Default constructor
         *
         * @param port The serial port to open for communicating with the sensor.
         * @param rotation The sensor rotation relative to the vehicle body.
         */
        RFbeamVLD1(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
        ~RFbeamVLD1() override;

        /**
         * @brief Send INIT command to sensor
         *
         * @return int
         */
        int init();

        /**
         * @brief Print some basic information about the driver.
         *
         */
        void print_info();

       private:
        /**
         * @brief Reads data from serial UART and places it into a buffer.
         *
         * @return int
         */
        int collect();

        /**
         * @brief Send command to obtain reading from sensor.
         *
         * @return int
         */
        int measure();

        /**
         * @brief Opens and configures the serial communications port.
         *
         * @param speed The baudrate (speed) to configure the serial UART port.
         * @return int
         */
        int open_serial_port(const speed_t speed = B115200);

        /**
         * @brief Request or read measurement at intervals
         *
         */
        void Run() override;

        /**
         * @brief Initialise the automatic measurement state machine and start it.
         *
         */
        void start();

        /**
         * @brief Stop the automatic measurement state machine.
         *
         */
        void stop();

        PX4Rangefinder _px4_rangefinder;

        char _port[20]{};

        int _file_descriptor{-1};

        uint8_t _read_buffer[sizeof(reading_msg)];
        uint8_t _read_buffer_len{0};

        hrt_abstime _last_read_time{0};
        hrt_abstime _read_time{0};

        int _interval{RFBEAM_MEASURE_INTERVAL};

        bool _collect_phase{false};

        perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": comm_err")};
        perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};

        // std::array<uint8_t, NO_BYTES> _cmdINIT = {...}
        // TODO: length of arrays not necessary

        // {INIT, 1, 0} = 115200 bit/s
        const uint8_t _cmdINIT[INIT_PACKET_BYTES] = {0x49, 0x4E, 0x49, 0x54, 0x01, 0x00, 0x00, 0x00, 0x00};


        // {GNFD, 1, 4}
        const uint8_t _cmdGNFD[GNFD_PACKET_BYTES] = {0x47, 0x4E, 0x46, 0x44, 0x01, 0x00, 0x00, 0x00, 0x04};


        // {PREC, 1, 1} = high precision mode (default)
        const uint8_t _cmdHIGHPREC[PREC_PACKET_BYTES] = {0x50, 0x52, 0x45, 0x43, 0x01, 0x00, 0x00, 0x00, 0x01};

        // {PREC, 1, 0} = low precision mode
        const uint8_t _cmdLOWPREC[PREC_PACKET_BYTES] = {0x50, 0x52, 0x45, 0x43, 0x01, 0x00, 0x00, 0x00, 0x00};


        // {TGFI, 1, 0} = target filter: strongest first
        const uint8_t _cmdTGFI_STRONG[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00};

        // {TGFI, 1, 0} = target filter: nearest first
        const uint8_t _cmdTGFI_NEAR[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x01};

        // {TGFI, 1, 2} = target filter: farthest first
        const uint8_t _cmdTGFI_FAR[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x02};


        // {RFSE, 0} = restore factory settings
        const uint8_t _cmdRFSE[RFSE_PACKET_BYTES] = {0x52, 0x46, 0x53, 0x45, 0x00, 0x00, 0x00, 0x00};
};
