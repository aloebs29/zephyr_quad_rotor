/**
 * @file		dps310.cpp
 * @author		Andrew Loebs
 * @brief		Source file of the dps310 module
 *
 */

#include "dps310.hpp"

#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(dps310, LOG_LEVEL_DBG);

// private variables
static const struct device *s_dev;

// public function definitions
int dps310::setup(const char *dev_name)
{
    int err = 0;
    // input validation
    if (dev_name == nullptr) {
        LOG_ERR("DPS310 nullptr error at line: %d.", __LINE__);
        err = EINVAL;
    }
    // get device from name
    if (!err) {
        s_dev = device_get_binding(dev_name);
        if (!s_dev) {
            LOG_ERR("DPS310 binding failed.");
            err = ENXIO;
        }
    }

    return err;
}

int dps310::read_pressure(PressureSensor *output)
{
    int err = 0;
    // input validation
    if (output == nullptr) {
        LOG_ERR("DPS310 nullptr error at line: %d.", __LINE__);
        err = EINVAL;
    }
    // fetch & store data
    if (!err) {
        err = sensor_sample_fetch(s_dev);
        if (!err) {
            WriteLock<struct sensor_value> write_access = output->get_write_lock();
            err = sensor_channel_get(s_dev, SENSOR_CHAN_PRESS, &write_access.get_ref());
        }
    }

    if (err) {
        LOG_ERR("Error reading pressure sensor: %d", err);
    }
    return err;
}