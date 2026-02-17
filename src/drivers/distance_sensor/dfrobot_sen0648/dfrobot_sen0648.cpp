/*
 * @file dfrobot_sen0648.cpp
 * @author DAAWG Development Team
 *
 * Driver for the DFRobot SEN0648 (TOFSense-F / C50M) I2C TOF laser rangefinder.
 *
 * Sensor register map (I2C mode):
 * 0x00-0x1F: Configuration registers (ID, interface mode, baud rate, etc.)
 * 0x20-0x23: System time (uint32_t, little-endian, ms)
 * 0x24-0x27: Distance (uint32_t, little-endian, millimeters)
 * 0x28-0x29: Distance status (uint16_t, little-endian)
 * 0x2A-0x2B: Signal strength (uint16_t, little-endian)
 * 0x2C:      Range precision (uint8_t)
 *
 * Default I2C address: 0x08
 * Max I2C read per transaction: 32 bytes
 * Sensor range: 0.01m to 50m
 * Update rate: ~10 Hz (sensor internal), polled at 10 Hz
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>              // Added for PRINT_MODULE_*
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_sensor.h>
#include <drivers/device/i2c.h>

using namespace time_literals;

/* Default I2C address for SEN0648 */
static constexpr uint8_t DFROBOT_SEN0648_BASE_ADDR = 0x08;

/* I2C bus frequency */
static constexpr uint32_t DFROBOT_SEN0648_BUS_SPEED = 100000; // 100 kHz standard mode

/* Measurement interval - 10 Hz polling to match sensor update rate */
static constexpr uint32_t DFROBOT_SEN0648_MEASURE_INTERVAL_US = 100000; // 100ms

/* Register addresses for distance data block */
static constexpr uint8_t REG_DISTANCE_DATA_START = 0x20;
static constexpr uint8_t REG_DISTANCE_DATA_LEN   = 13;  // 0x20-0x2C inclusive

/* Offsets within the 13-byte read buffer starting at 0x20 */
static constexpr uint8_t OFF_SYSTEM_TIME   = 0;  // 0x20-0x23, 4 bytes
static constexpr uint8_t OFF_DISTANCE      = 4;  // 0x24-0x27, 4 bytes
static constexpr uint8_t OFF_DIS_STATUS    = 8;  // 0x28-0x29, 2 bytes
static constexpr uint8_t OFF_SIGNAL_STR    = 10; // 0x2A-0x2B, 2 bytes
static constexpr uint8_t OFF_RANGE_PREC    = 12; // 0x2C, 1 byte

/* Distance status values - 0 indicates valid measurement */
static constexpr uint16_t DIS_STATUS_VALID = 0;

/* Sensor physical limits */
static constexpr float DFROBOT_SEN0648_MIN_DISTANCE = 0.01f; // 1 cm
static constexpr float DFROBOT_SEN0648_MAX_DISTANCE = 50.0f; // 50 m
static constexpr float DFROBOT_SEN0648_FOV_RAD      = 0.04f; // ~2 degrees, narrow laser beam

/* Maximum consecutive errors before reporting failure */
static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 5;

class DFRobotSEN0648 : public device::I2C, public I2CSPIDriver<DFRobotSEN0648>
{
public:
	DFRobotSEN0648(const I2CSPIDriverConfig &config);
	~DFRobotSEN0648() override;

	static void print_usage();

	void RunImpl();

	int init() override;

	void print_status() override;

private:
	int collect();

	int read_register_block(uint8_t reg_addr, uint8_t *buf, uint8_t len);

	PX4Rangefinder _px4_rangefinder;

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};

	uint8_t _consecutive_errors{0};
};

DFRobotSEN0648::DFRobotSEN0648(const I2CSPIDriverConfig &config) :
    I2C(0, MODULE_NAME, config.bus, config.i2c_address, config.bus_frequency),
    I2CSPIDriver(config),
    _px4_rangefinder(get_device_id(), config.rotation)
{
    _px4_rangefinder.set_min_distance(DFROBOT_SEN0648_MIN_DISTANCE);
    _px4_rangefinder.set_max_distance(DFROBOT_SEN0648_MAX_DISTANCE);
    _px4_rangefinder.set_fov(DFROBOT_SEN0648_FOV_RAD);

    // Use the uORB constant for the device type
    _px4_rangefinder.set_device_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

DFRobotSEN0648::~DFRobotSEN0648()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
DFRobotSEN0648::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	/* Probe: try reading the distance data block to verify the sensor is present */
	uint8_t buf[REG_DISTANCE_DATA_LEN];
	ret = read_register_block(REG_DISTANCE_DATA_START, buf, REG_DISTANCE_DATA_LEN);

	if (ret != PX4_OK) {
		DEVICE_DEBUG("probe failed: no response from sensor at addr 0x%02x", get_device_address());
		return ret;
	}

	/* Start periodic measurements */
	ScheduleOnInterval(DFROBOT_SEN0648_MEASURE_INTERVAL_US);

	PX4_INFO("sensor found at addr 0x%02x on bus %d", get_device_address(), get_device_bus());

	return PX4_OK;
}

int
DFRobotSEN0648::read_register_block(uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
	/*
	 * I2C transaction: write register address, then read data.
	 * The PX4 I2C::transfer() handles the repeated start automatically.
	 * write: [reg_addr]  read: [len bytes]
	 */
	return transfer(&reg_addr, 1, buf, len);
}

int
DFRobotSEN0648::collect()
{
	perf_begin(_sample_perf);

	uint8_t buf[REG_DISTANCE_DATA_LEN];
	int ret = read_register_block(REG_DISTANCE_DATA_START, buf, REG_DISTANCE_DATA_LEN);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		_consecutive_errors++;
		return ret;
	}

	_consecutive_errors = 0;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/* Parse distance: 4 bytes little-endian at offset 4, value in millimeters */
	const uint32_t distance_mm =
		(uint32_t)buf[OFF_DISTANCE]         |
		((uint32_t)buf[OFF_DISTANCE + 1] << 8)  |
		((uint32_t)buf[OFF_DISTANCE + 2] << 16) |
		((uint32_t)buf[OFF_DISTANCE + 3] << 24);

	const float distance_m = (float)distance_mm / 1000.0f;

	/* Parse distance status: 2 bytes little-endian at offset 8 */
	const uint16_t dis_status =
		(uint16_t)buf[OFF_DIS_STATUS] |
		((uint16_t)buf[OFF_DIS_STATUS + 1] << 8);

	/* Parse signal strength: 2 bytes little-endian at offset 10 */
	const uint16_t signal_strength =
		(uint16_t)buf[OFF_SIGNAL_STR] |
		((uint16_t)buf[OFF_SIGNAL_STR + 1] << 8);

	/*
	 * Determine signal quality (0-100 scale for uORB).
	 * Signal strength from sensor is raw; we clamp to 0-100.
	 * A value of 0 with invalid status means no return.
	 */
	int8_t signal_quality = -1; // unknown by default

	if (dis_status == DIS_STATUS_VALID && distance_m >= DFROBOT_SEN0648_MIN_DISTANCE
	    && distance_m <= DFROBOT_SEN0648_MAX_DISTANCE) {
		/* Valid measurement - map signal strength to 0-100 */
		if (signal_strength > 0) {
			signal_quality = (int8_t)math::min((unsigned)signal_strength, (unsigned)100);

		} else {
			signal_quality = 0;
		}

	} else {
		/* Invalid measurement */
		signal_quality = 0;
	}

	_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);

	perf_end(_sample_perf);
	return PX4_OK;
}

void
DFRobotSEN0648::RunImpl()
{
	/* Collect distance measurement */
	if (collect() != PX4_OK) {
		if (_consecutive_errors > MAX_CONSECUTIVE_ERRORS) {
			PX4_ERR("sensor not responding at addr 0x%02x", get_device_address());
			_consecutive_errors = 0; // reset to avoid log spam
		}
	}
}

void
DFRobotSEN0648::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
DFRobotSEN0648::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for the DFRobot SEN0648 (TOFSense-F / C50M) TOF laser rangefinder.

The sensor must be configured for I2C (IIC) output mode. Default I2C address is 0x08.
Multiple sensors can be used on the same bus with different addresses.

### Examples

Start a sensor on external I2C bus with default address:
$ dfrobot_sen0648 start -I

Start a sensor with a specific address and forward-facing rotation:
$ dfrobot_sen0648 start -I -a 0x08 -R 0

Start three sensors for obstacle avoidance:
$ dfrobot_sen0648 start -I -a 0x08 -R 0
$ dfrobot_sen0648 start -I -a 0x09 -R 2
$ dfrobot_sen0648 start -I -a 0x0a -R 6
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dfrobot_sen0648", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(DFROBOT_SEN0648_BASE_ADDR);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 35, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int dfrobot_sen0648_main(int argc, char *argv[])
{
	using ThisDriver = DFRobotSEN0648;
	BusCLIArguments cli{true, false}; // I2C only, no SPI
	cli.default_i2c_frequency = DFROBOT_SEN0648_BUS_SPEED;
	cli.i2c_address = DFROBOT_SEN0648_BASE_ADDR;
	// Fixed cast for v1.15 compatibility
	cli.rotation = static_cast<Rotation>(distance_sensor_s::ROTATION_DOWNWARD_FACING);

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, 0);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
