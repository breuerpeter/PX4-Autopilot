#pragma once

#include <termios.h>
#include <unistd.h>

#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/module.h>

// TODO: figure out what the following includes are for/if needed at all
#include <fcntl.h>
#include <poll.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

using namespace time_literals;

/* -------------------------------------------------------------------------- */
/*                                   Defines                                  */
/* -------------------------------------------------------------------------- */

/* ---------------------------------- Basic --------------------------------- */
#define RFBEAM_STARTUP_TIME         15_ms

#define RFBEAM_CHIRP_INTEGRATION    1   // 1-100; higher value means slower rate but higher SNR
#define RFBEAM_SHORT_RANGE_FILTER   1   // on (1) or off (0); choose on to enable short range measurements of strong reflectors


/* ----------------------------- Precision mode ----------------------------- */

// TODO: add PX4 parameter to switch between high and low precision mode
#define RFBEAM_HIGH_PRECISION_MODE  1

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

/* ---------------------------- Default settings ---------------------------- */

#define RFBEAM_PARAM_TGFI_DEFAULT 	1
#define RFBEAM_PARAM_RNG_DEFAULT 	0
#define RFBEAM_PARAM_SRNG_DEFAULT 	0
#define RFBEAM_PARAM_MINF_DEFAULT 	5
#define RFBEAM_PARAM_MAXF_DEFAULT 	460
#define RFBEAM_PARAM_THRS_DEFAULT 	60
#define RFBEAM_PARAM_CHRP_DEFAULT 	1
#define RFBEAM_PARAM_AVG_DEFAULT 	5

#if RFBEAM_SHORT_RANGE_FILTER == 1
#define RFBEAM_MEASURE_INTERVAL_MS     RFBEAM_FRAME_PROC_TIME + (RFBEAM_CHIRP_INTEGRATION - 1) * (3_ms + 5_ms)
#else
#define RFBEAM_MEASURE_INTERVAL_MS     RFBEAM_FRAME_PROC_TIME + (RFBEAM_CHIRP_INTEGRATION - 1) * 3_ms
#endif

/**
 * Assume standard deviation to be equal to sensor resolution.
 * Static bench tests have shown that the sensor output does
 * not vary if the unit is not moved.
 */
#define SENS_VARIANCE               RF_BEAM_RESOLUTION * RF_BEAM_RESOLUTION

/* --------------------------------- Packets -------------------------------- */

#define PACKET_HEADER_BYTES                     4 // Number of bytes for  packet header
#define PACKET_PAYLOAD_LENGTH_BYTES             4 // Number of bytes to specify payload length

#define PACKET_PAYLOAD_START_IDX 		PACKET_HEADER_BYTES + PACKET_PAYLOAD_LENGTH_BYTES

// Driver -> radar (requests)
#define INIT_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of INIT command
#define GNFD_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of GNFD command
#define GRPS_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of GRPS command
#define RFSE_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of RFSE command
#define GBYE_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of GBYE command
#define RRAI_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of RRAI command
#define THOF_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of THOF command
#define MIRA_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 2	// Payload bytes of MIRA command
#define MARA_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 2	// Payload bytes of MARA command
#define RAVG_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of RAVG command
#define TGFI_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of TGFI command
#define PREC_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of PREC command
#define TXPW_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of TXPW command
#define INTN_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of INTN command
#define SRDF_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of SRDF command
#define JBTL_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 0	// Payload bytes of JBTL command

// Radar -> driver (responses)
#define RESP_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 1	// Payload bytes of RESP response
#define RADC_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 2048	// Payload bytes of RADC response
#define DONE_PACKET_BYTES       PACKET_PAYLOAD_START_IDX + 4	// Payload bytes of DONE response

struct __attribute__((__packed__)) PDAT_msg {
	float distance;
	uint16_t mag;
};

class RFbeamVLD1 : public ModuleParams, public px4::ScheduledWorkItem
{
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
	 * @brief Initialise the automatic measurement state machine and start it
	 *
	 */
	void start();

	/**
	 * @brief Stop the automatic measurement state machine
	 *
	 */
	void stop();

	PX4Rangefinder _px4_rangefinder;

	char _port[20] = {};

	int _fd = -1;

	uint8_t _read_buffer[50] = {}; // TODO: don't hardcode size
	// uint8_t _read_buffer_len = 0;

	// hrt_abstime _last_read_time = 0;
	hrt_abstime _read_time = 0;

	int _interval_us = 1.5 * RFBEAM_MEASURE_INTERVAL_MS *
			   1000; // TODO: factor 1.5 to give sensor extra time, might not be necessary

	bool _collect_phase = false;

	perf_counter_t _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME ": comm_err");
	perf_counter_t _sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": read");

	/* -------------------------------------------------------------------------- */
	/*                    Message definitions (from datasheet)                    */
	/* -------------------------------------------------------------------------- */

	/* ---------------------------------- INIT ---------------------------------- */

	// Initialize with a baudrate of 115200 bit/s (default): {INIT, 1, 0}
	uint8_t _cmd_INIT_default[INIT_PACKET_BYTES] = {0x49, 0x4E, 0x49, 0x54, 0x01, 0x00, 0x00, 0x00, 0x00};

	/* ---------------------------------- GNFD ---------------------------------- */

	// Request distance to target: {GNFD, 1, 4}
	uint8_t _cmd_GNFD_PDAT[GNFD_PACKET_BYTES] = {0x47, 0x4E, 0x46, 0x44, 0x01, 0x00, 0x00, 0x00, 0x04};

	/* ---------------------------------- TGFI ---------------------------------- */

	// Target filter: strongest first: {TGFI, 1, 0}
	uint8_t _cmd_TGFI_strong[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00};

	// Target filter: nearest first (default): {TGFI, 1, 1}
	uint8_t _cmd_TGFI_near[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x01}; // (uint8_t)RFBEAM_PARAM_TGFI_DEFAULT

	// Target filter: farthest first: {TGFI, 1, 2}
	uint8_t _cmdTGFI_FAR[TGFI_PACKET_BYTES] = {0x54, 0x47, 0x46, 0x49, 0x01, 0x00, 0x00, 0x00, 0x02};

	uint8_t *_cmd_TGFI_default = _cmd_TGFI_near;

	/* ---------------------------------- RRAI ---------------------------------- */

	// Max range setting: 20 m (default): {RRAI, 1, 0}
	uint8_t _cmd_RRAI_20[RRAI_PACKET_BYTES] = {0x52, 0x52, 0x41, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00}; // (uint8_t)RFBEAM_PARAM_RNG_DEFAULT

	// Max range setting: 50 m: {RRAI, 1, 1}
	uint8_t _cmd_RRAI_50[RRAI_PACKET_BYTES] = {0x52, 0x52, 0x41, 0x49, 0x01, 0x00, 0x00, 0x00, 0x01};

	uint8_t *_cmd_RRAI_default = _cmd_RRAI_20;

	/* ---------------------------------- SRDF ---------------------------------- */

	// Short range filter off (default): {SRDF, 1, 0}
	uint8_t _cmd_SRDF_off[SRDF_PACKET_BYTES] = {0x53, 0x52, 0x44, 0x46, 0x01, 0x00, 0x00, 0x00, 0x00}; // (uint8_t)RFBEAM_PARAM_SRNG_DEFAULT

	// Short range filter on: {SRDF, 1, 1}
	uint8_t _cmd_SRDF_on[SRDF_PACKET_BYTES] = {0x53, 0x52, 0x44, 0x46, 0x01, 0x00, 0x00, 0x00, 0x01};

	uint8_t *_cmd_SRDF_default = _cmd_SRDF_off;

	/* ---------------------------------- MIRA ---------------------------------- */

	// Minimum range filter (default): {MIRA, 2, 5}
	uint8_t _cmd_MIRA_default[MIRA_PACKET_BYTES] = {0x4D, 0x49, 0x52, 0x41, 0x02, 0x00, 0x00, 0x00, 0x05, 0x00}; // (uint16_t)RFBEAM_PARAM_MINF_DEFAULT

	/* ---------------------------------- MARA ---------------------------------- */

	// Maximum range filter (default): {MARA, 2, 460}
	uint8_t _cmd_MARA_default[MARA_PACKET_BYTES] = {0x4D, 0x41, 0x52, 0x41, 0x02, 0x00, 0x00, 0x00, 0xCC, 0x01}; // (uint16_t)RFBEAM_PARAM_MAXF_DEFAULT // TODO: check

	/* ---------------------------------- THOF ---------------------------------- */

	// Threshold offset (default): {THOF, 1, 60}
	uint8_t _cmd_THOF_default[THOF_PACKET_BYTES] = {0x54, 0x48, 0x4F, 0x46, 0x01, 0x00, 0x00, 0x00, 0x3C}; // (uint16_t)RFBEAM_PARAM_THRS_DEFAULT

	/* ---------------------------------- INTN ---------------------------------- */

	// Chirp integration count (default): {INTN, 1, 1}
	uint8_t _cmd_INTN_default[INTN_PACKET_BYTES] = {0x49, 0x4E, 0x54, 0x4E, 0x01, 0x00, 0x00, 0x00, 0x01}; // (uint8_t)RFBEAM_PARAM_CHRP_DEFAULT

	/* ---------------------------------- RAVG ---------------------------------- */

	// Distance average count (default): {RAVG, 1, 5}
	uint8_t _cmd_RAVG_default[RAVG_PACKET_BYTES] = {0x52, 0x41, 0x56, 0x47, 0x01, 0x00, 0x00, 0x00, 0x05}; // (uint8_t)RFBEAM_PARAM_AVG_DEFAULT

	/* -------------------------------------------------------------------------- */
	/*                         Unused message definitions                         */
	/* -------------------------------------------------------------------------- */

	/* ---------------------------------- PREC ---------------------------------- */

	// High precision mode (default): {PREC, 1, 1}
	uint8_t _cmd_PREC_high[PREC_PACKET_BYTES] = {0x50, 0x52, 0x45, 0x43, 0x01, 0x00, 0x00, 0x00, 0x01};

	// Low precision mode: {PREC, 1, 0}
	uint8_t _cmd_PREC_low[PREC_PACKET_BYTES] = {0x50, 0x52, 0x45, 0x43, 0x01, 0x00, 0x00, 0x00, 0x00};

	uint8_t *_cmd_PREC_default = _cmd_PREC_high;

	/* ---------------------------------- RFSE ---------------------------------- */

	// Restore factory settings: {RFSE, 0}
	uint8_t _cmd_RFSE[RFSE_PACKET_BYTES] = {0x52, 0x46, 0x53, 0x45, 0x00, 0x00, 0x00, 0x00};

	/* -------------------------------------------------------------------------- */
	/*                                 Parameters                                 */
	/* -------------------------------------------------------------------------- */

	uORB::Subscription _parameter_update_sub = ORB_ID(parameter_update);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_VLD1>)    _param_sensor_enabled,
		(ParamInt<px4::params::SENS_VLD1_TGFI>)  _param_sensor_tgfi,
		(ParamInt<px4::params::SENS_VLD1_RNG>)  _param_sensor_range,
		(ParamInt<px4::params::SENS_VLD1_SRNG>)  _param_sensor_srng,
		(ParamInt<px4::params::SENS_VLD1_MINF>)  _param_sensor_minf,
		(ParamInt<px4::params::SENS_VLD1_MAXF>)  _param_sensor_maxf,
		(ParamInt<px4::params::SENS_VLD1_THRS>)  _param_sensor_thrs,
		(ParamInt<px4::params::SENS_VLD1_CHRP>)  _param_sensor_chrp,
		(ParamInt<px4::params::SENS_VLD1_AVG>)  _param_sensor_avg
	);
};
