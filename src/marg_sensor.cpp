/**
 * @file		marg_sensor.cpp
 * @author		Andrew Loebs
 * @brief		Source file of the MARG sensor module
 *
 */

#include "marg_sensor.hpp"

#include <logging/log.h>

namespace z_quad_rotor {

LOG_MODULE_REGISTER(marg_sensor, LOG_LEVEL_DBG);

// constants
static const struct sensor_value fxos8700_dr = {.val1 = 200, .val2 = 0};
// note -- fxas21002 data rate is set in kconfig

// private functions
int MargSensor::setup_fxos8700(const char *dev_name, sensor_trigger_handler_t trig_handler)
{
    int err = 0;
    m_fxos8700 = device_get_binding(dev_name);
    if (!m_fxos8700) {
        LOG_ERR("FXOS8700 binding failed.");
        err = ENXIO;
    }
    if (!err) {
        err = sensor_attr_set(m_fxos8700, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY,
                              &fxos8700_dr);
        if (err) {
            LOG_ERR("Unable to set FXOS8700 sample rate; err: %d.", err);
        }
    }
    if (!err) {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ACCEL_XYZ,
        };
        err = sensor_trigger_set(m_fxos8700, &trig, trig_handler);
        if (err) {
            LOG_ERR("Unable to set FXOS8700 trigger; err: %d.", err);
        }
    }

    return err;
}

int MargSensor::setup_fxas21002(const char *dev_name, sensor_trigger_handler_t trig_handler)
{
    int err = 0;
    m_fxas21002 = device_get_binding(dev_name);
    if (!m_fxas21002) {
        LOG_ERR("FXAS21002 binding failed.");
        err = ENXIO;
    }
    if (!err) {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_GYRO_XYZ,
        };
        err = sensor_trigger_set(m_fxas21002, &trig, trig_handler);
        if (err) {
            LOG_ERR("Unable to set FXAS21002 trigger; err: %d.", err);
        }
    }

    return err;
}

// public functions
int MargSensor::init(const char *fxos8700_dev_name, const char *fxas21002_dev_name,
                     sensor_trigger_handler_t fxos8700_trig_handler,
                     sensor_trigger_handler_t fxas21002_trig_handler)
{
    int err = setup_fxos8700(fxos8700_dev_name, fxos8700_trig_handler);
    if (!err) {
        err = setup_fxas21002(fxas21002_dev_name, fxas21002_trig_handler);
    }

    return err;
}

void MargSensor::fxos8700_trig_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    ARG_UNUSED(trigger);

    // fetch data
    int err = sensor_sample_fetch(dev);
    // store
    SyncedWriteAccess<MargData> write_access = m_marg_data.write_access();
    if (!err) {
        err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, write_access.get_ref().accel);
    }
    if (!err) {
        err = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, write_access.get_ref().magn);
    }

    // log errors
    if (err) {
        LOG_ERR("FXOS8700 trigger handler err: %d.", err);
    }
}

void MargSensor::fxas21002_trig_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    ARG_UNUSED(trigger);

    // fetch data
    int err = sensor_sample_fetch(dev);
    // store
    SyncedWriteAccess<MargData> write_access = m_marg_data.write_access();
    if (!err) {
        err = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, write_access.get_ref().gyro);
    }

    // log errors
    if (err) {
        LOG_ERR("FXAS21002 trigger handler err: %d.", err);
    }
}

MargData MargSensor::get_marg() { return m_marg_data.read_access().get_var(); }

} // namespace z_quad_rotor