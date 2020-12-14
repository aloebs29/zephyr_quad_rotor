/**
 * @file		fxos8700.cpp
 * @author		Andrew Loebs
 * @brief		Source file of the fxos8700 module
 *
 */

#include "fxos8700.hpp"

#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(fxos8700, LOG_LEVEL_DBG);

// constants
static const struct sensor_value data_rate = {.val1 = 200, .val2 = 0}; // Hz

// private variables
static MargSensor *s_output_sink;

// private function definitions
static void trig_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    ARG_UNUSED(trigger);

    // fetch data
    int err = sensor_sample_fetch(dev);
    // store
    WriteLock<MargData> write_lock = s_output_sink->get_write_lock();
    if (!err) {
        err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, write_lock.get_ref().accel);
    }
    if (!err) {
        err = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, write_lock.get_ref().magn);
    }

    // log errors
    if (err) {
        LOG_ERR("FXOS8700 trigger handler err: %d.", err);
    }
}

// public function definitions
int fxos8700::setup(const char *dev_name, MargSensor *output_sink)
{
    int err = 0;
    // input validation
    if (dev_name == nullptr || output_sink == nullptr) {
        LOG_ERR("FXOS8700 nullptr error at line: %d.", __LINE__);
        err = EINVAL;
    }
    // get device from name
    const struct device *dev;
    if (!err) {
        dev = device_get_binding(dev_name);
        if (!dev) {
            LOG_ERR("FXOS8700 binding failed.");
            err = ENXIO;
        }
    }
    // setup device
    if (!err) {
        err = sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &data_rate);
        if (err) {
            LOG_ERR("Unable to set FXOS8700 sample rate; err: %d.", err);
        }
    }
    if (!err) {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ACCEL_XYZ,
        };
        err = sensor_trigger_set(dev, &trig, trig_handler);
        if (err) {
            LOG_ERR("Unable to set FXOS8700 trigger; err: %d.", err);
        }
    }

    s_output_sink = output_sink;
    return err;
}
