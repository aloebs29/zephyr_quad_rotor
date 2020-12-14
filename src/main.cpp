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

#include "altitude.hpp"
#include "fxas21002.hpp"
#include "fxos8700.hpp"
#include "marg_sensor.hpp"
#include "orientation.hpp"
#include "pressure_sensor.hpp"

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// MARG sensor
static MargSensor marg_sensor;

// Pressure sensor
static PressureSensor pressure_sensor;
static k_thread pressure_sensor_thread;
K_THREAD_STACK_DEFINE(pressure_sensor_stack, PressureSensor::THREAD_STACK_SIZE);
PRESSURE_DEFINE_THREAD_ENTRY_FUNC(pressure_sensor, pressure_entry_func);

// Orientation filter
static const RotationMatrix remap({1.0f, 0.0f, 0.0f}, // Identity matrix (TODO: create actual remap)
                                  {0.0f, 1.0f, 0.0f}, // -
                                  {0.0f, 0.0f, 1.0f});
static Orientation<MadgwickFusion6> orientation(remap);

// Altitude filter
static Altitude altitude;

// TODO: delete -- for testing
// timer to initiate orientation updates
K_TIMER_DEFINE(orientation_update_timer, NULL, NULL);
static const uint32_t FUSION_UPDATE_RATE = 10; // ms

// TODO: delete -- for testing
static struct sensor_value float_to_sensor_value(float f)
{
    return {(int)f, (int)((f - floorf(f)) * 1000000)};
}

void main(void)
{
    // enable USB for shell backend
    usb_enable(NULL);

    // setup sensors
    int err = fxos8700::setup(DT_LABEL(DT_INST(0, nxp_fxos8700)), &marg_sensor);
    if (!err) {
        err = fxas21002::setup(DT_LABEL(DT_INST(0, nxp_fxas21002)), &marg_sensor);
    }
    if (!err) {
        err = pressure_sensor.init(DT_LABEL(DT_INST(0, infineon_dps310)), pressure_entry_func,
                                   &pressure_sensor_thread, pressure_sensor_stack,
                                   K_THREAD_STACK_SIZEOF(pressure_sensor_stack), 10);
    }

    uint32_t count = 0;
    // start timer and perform fusion updates on sync
    k_timer_start(&orientation_update_timer, K_MSEC(FUSION_UPDATE_RATE),
                  K_MSEC(FUSION_UPDATE_RATE));
    for (;;) {
        k_timer_status_sync(&orientation_update_timer);
        // update orientation
        MargData marg_data = marg_sensor.get_marg();
        orientation.update(marg_data, FUSION_UPDATE_RATE);
        // update altitude
        struct sensor_value pressure = pressure_sensor.get_pressure();
        altitude.update(pressure);
        // use count to log values every 1 second
        if (100 == count) {
            // LOG_INF("AX:%3d.%06d AY:%3d.%06d AZ:%3d.%06d", marg_data.accel[0].val1,
            //         abs(marg_data.accel[0].val2), marg_data.accel[1].val1,
            //         abs(marg_data.accel[1].val2), marg_data.accel[2].val1,
            //         abs(marg_data.accel[2].val2));

            // LOG_INF("GX:%3d.%06d GY:%3d.%06d GZ:%3d.%06d", marg_data.gyro[0].val1,
            //         abs(marg_data.gyro[0].val2), marg_data.gyro[1].val1,
            //         abs(marg_data.gyro[1].val2), marg_data.gyro[2].val1,
            //         abs(marg_data.gyro[2].val2));

            // LOG_INF("MX:%3d.%06d MY:%3d.%06d MZ:%3d.%06d", marg_data.magn[0].val1,
            //         abs(marg_data.magn[0].val2), marg_data.magn[1].val1,
            //         abs(marg_data.magn[1].val2), marg_data.magn[2].val1,
            //         abs(marg_data.magn[2].val2));
            EulerAngle euler_angle = orientation.get_euler_angle() * RAD_TO_DEG;
            struct sensor_value roll = float_to_sensor_value(euler_angle.x);
            struct sensor_value pitch = float_to_sensor_value(euler_angle.y);
            struct sensor_value yaw = float_to_sensor_value(euler_angle.z);
            LOG_INF("Roll:%3d.%06d Pitch:%3d.%06d Yaw:%3d.%06d", roll.val1, abs(roll.val2),
                    pitch.val1, abs(pitch.val2), yaw.val1, abs(yaw.val2));

            float height_f = altitude.get_altitude();
            struct sensor_value height = float_to_sensor_value(height_f);
            LOG_INF("Altitude:%3d.%06d", height.val1, height.val2);

            count = 0; // reset
        }
        count++;
    }
}