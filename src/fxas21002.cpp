/**
 * @file		fxas21002.cpp
 * @author		Andrew Loebs
 * @brief		Source file of the fxas21002 module
 *
 */

#include "fxas21002.hpp"

#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(fxas21002, LOG_LEVEL_DBG);

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
        err = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, write_lock.get_ref().gyro);
    }

    // log errors
    if (err) {
        LOG_ERR("FXAS21002 trigger handler err: %d.", err);
    }
}

// public function definitions
int fxas21002::setup(const char *dev_name, MargSensor *output_sink)
{
    int err = 0;
    // input validation
    if (dev_name == nullptr || output_sink == nullptr) {
        err = EINVAL;
    }
    // get device from name
    const struct device *dev;
    if (!err) {
        dev = device_get_binding(dev_name);
        if (!dev) {
            LOG_ERR("FXAS21002 binding failed.");
            err = ENXIO;
        }
    }
    if (!err) {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_GYRO_XYZ,
        };
        err = sensor_trigger_set(dev, &trig, trig_handler);
        if (err) {
            LOG_ERR("Unable to set FXAS21002 trigger; err: %d.", err);
        }
    }

    s_output_sink = output_sink;
    return err;
}
