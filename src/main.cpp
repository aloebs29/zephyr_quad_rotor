/**
 * @file		main.cpp
 * @author		Andrew Loebs
 * @brief		Main application
 *
 */

#include <cmath>
#include <cstdlib>

#include <device.h>
#include <logging/log.h>
#include <usb/usb_device.h>
#include <zephyr.h>

#include "marg_sensor.hpp"
#include "pressure_sensor.hpp"

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// MARG sensor
static MargSensor marg_sensor;
MARG_DEFINE_FXOS8700_TRIG_HANDLER(marg_sensor, fxos8700_trig_handler);
MARG_DEFINE_FXAS21002_TRIG_HANDLER(marg_sensor, fxas21002_trig_handler);
// Altitude sensor
static PressureSensor pressure_sensor;
static k_thread pressure_sensor_thread;
K_THREAD_STACK_DEFINE(pressure_sensor_stack, PressureSensor::THREAD_STACK_SIZE);
PRESSURE_DEFINE_THREAD_ENTRY_FUNC(pressure_sensor, pressure_entry_func);

void main(void)
{
    // enable USB for shell backend
    usb_enable(NULL);

    // setup position & orientation subsystem
    int err =
        marg_sensor.init(DT_LABEL(DT_INST(0, nxp_fxos8700)), DT_LABEL(DT_INST(0, nxp_fxas21002)),
                         fxos8700_trig_handler, fxas21002_trig_handler);
    if (err) {
        LOG_ERR("MARG sensor initialization error: %d", err);
    }
    else {
        pressure_sensor.init(DT_LABEL(DT_INST(0, infineon_dps310)), pressure_entry_func,
                             &pressure_sensor_thread, pressure_sensor_stack,
                             K_THREAD_STACK_SIZEOF(pressure_sensor_stack), 10);
    }
    if (err) {
        LOG_ERR("Altitude sensor initialization error: %d", err);
    }

    // log samples as they are ready
    for (;;) {
        k_sleep(K_MSEC(1000));

        MargData marg_data = marg_sensor.get_data();
        struct sensor_value pressure = pressure_sensor.get_pressure();

        LOG_INF("AX:%3d.%06d AY:%3d.%06d AZ:%3d.%06d", marg_data.accel[0].val1,
                abs(marg_data.accel[0].val2), marg_data.accel[1].val1, abs(marg_data.accel[1].val2),
                marg_data.accel[2].val1, abs(marg_data.accel[2].val2));

        LOG_INF("GX:%3d.%06d GY:%3d.%06d GZ:%3d.%06d", marg_data.gyro[0].val1,
                abs(marg_data.gyro[0].val2), marg_data.gyro[1].val1, abs(marg_data.gyro[1].val2),
                marg_data.gyro[2].val1, abs(marg_data.gyro[2].val2));

        LOG_INF("MX:%3d.%06d MY:%3d.%06d MZ:%3d.%06d", marg_data.magn[0].val1,
                abs(marg_data.magn[0].val2), marg_data.magn[1].val1, abs(marg_data.magn[1].val2),
                marg_data.magn[2].val1, abs(marg_data.magn[2].val2));

        LOG_INF("Pressure:%3d.%06d", pressure.val1, abs(pressure.val2));

        // Calculate altitude
        float height =
            44330.0f * (1.0f - pow(sensor_value_to_double(&pressure) / 101.325f, 0.1902949));
        // Make altitude printable without printf float
        int height_decimal = (int)height;
        int height_fractional = (int)((height - floorf(height)) * 100000);
        LOG_INF("Altitude:%3d.%06d", height_decimal, height_fractional);
    }
}