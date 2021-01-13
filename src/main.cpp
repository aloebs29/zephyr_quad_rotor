/**
 * @file	main.cpp
 * @author	Andrew Loebs
 * @brief	Main application
 *
 */

#include <cmath>
#include <cstdlib>

#include <device.h>
#include <drivers/adc.h>   // TODO: Delete -- for testing
#include <hal/nrf_saadc.h> // TODO: Delete -- for testing
#include <logging/log.h>
#include <usb/usb_device.h>
#include <zephyr.h>

#include "altitude.hpp"
#include "dps310.hpp"
#include "fxas21002.hpp"
#include "fxos8700.hpp"
#include "marg_sensor.hpp"
#include "orientation.hpp"
#include "pressure_sensor.hpp"

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// constants
static constexpr size_t DPS310_SAMPLING_STACK_SIZE = 1024;
static constexpr int DPS310_SAMPLING_THREAD_PRIO = 10;

// static objects
static MargSensor marg_sensor;
static PressureSensor pressure_sensor;

static const RotationMatrix remap({1.0f, 0.0f, 0.0f}, // Identity matrix (TODO: create actual remap)
                                  {0.0f, 1.0f, 0.0f}, // -
                                  {0.0f, 0.0f, 1.0f});
static Orientation<MadgwickFusion6> orientation(remap);
static Altitude altitude;

// threads
static k_thread dps310_sampling_thread;
K_THREAD_STACK_DEFINE(dps310_sampling_stack, DPS310_SAMPLING_STACK_SIZE);

// TODO: delete -- for testing
// timer to initiate orientation updates
K_TIMER_DEFINE(orientation_update_timer, NULL, NULL);
static const uint32_t FUSION_UPDATE_RATE = 10; // ms

// TODO: delete -- for testing
static struct sensor_value float_to_sensor_value(float f)
{
    return {(int)f, (int)((f - floorf(f)) * 1000000)};
}

// dps310 sampling thread
void dps310_sampling_thread_func(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (;;) {
        // sample at max rate (limited by conversion time)
        dps310::read_pressure(&pressure_sensor);
    }
}

static const struct adc_channel_cfg ccfg = {
    .gain = ADC_GAIN_2,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 0,
    .input_positive = NRF_SAADC_INPUT_AIN5,
};
static int16_t raw;
static const struct adc_sequence seq = {
    .channels = BIT(0),
    .buffer = &raw,
    .buffer_size = sizeof(raw),
    .resolution = 14,
    .oversampling = 4,
    .calibrate = true,
};

// main thread
void main(void)
{
    // enable USB for shell backend
    usb_enable(NULL);

    // setup adc
    const struct device *adc = device_get_binding(DT_LABEL(DT_INST(0, nordic_nrf_saadc)));
    if (!adc) {
        LOG_ERR("ADC binding failed.");
    }
    else {
        int temp = adc_channel_setup(adc, &ccfg);
        if (temp) {
            LOG_ERR("ADC channel setup error: %d", temp);
        }
    }

    // setup sensors
    int err = fxos8700::setup(DT_LABEL(DT_INST(0, nxp_fxos8700)), &marg_sensor);
    if (!err) {
        err = fxas21002::setup(DT_LABEL(DT_INST(0, nxp_fxas21002)), &marg_sensor);
    }
    if (!err) {
        err = dps310::setup(DT_LABEL(DT_INST(0, infineon_dps310)));
    }

    // startup threads
    if (!err) {
        k_tid_t dps310_sampling_tid = k_thread_create(
            &dps310_sampling_thread, dps310_sampling_stack,
            K_THREAD_STACK_SIZEOF(dps310_sampling_stack), dps310_sampling_thread_func, NULL, NULL,
            NULL, DPS310_SAMPLING_THREAD_PRIO, 0, K_NO_WAIT);
        err = k_thread_name_set(dps310_sampling_tid, "dps310 sampling");
        if (err) {
            LOG_ERR("Failed to set dps310 sampling thread name.");
        }
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
            // EulerAngle euler_angle = orientation.get_euler_angle() * RAD_TO_DEG;
            // struct sensor_value roll = float_to_sensor_value(euler_angle.x);
            // struct sensor_value pitch = float_to_sensor_value(euler_angle.y);
            // struct sensor_value yaw = float_to_sensor_value(euler_angle.z);
            // LOG_INF("Roll:%3d.%06d Pitch:%3d.%06d Yaw:%3d.%06d", roll.val1, abs(roll.val2),
            //         pitch.val1, abs(pitch.val2), yaw.val1, abs(yaw.val2));

            // float height_f = altitude.get_altitude();
            // struct sensor_value height = float_to_sensor_value(height_f);
            // LOG_INF("Altitude:%3d.%06d", height.val1, height.val2);

            if (adc) {
                err = adc_read(adc, &seq);
                if (err) {
                    LOG_ERR("Error reading ADC: %d", err);
                }
                else {
                    int32_t val = raw;
                    err = adc_raw_to_millivolts(adc_ref_internal(adc), ccfg.gain, seq.resolution,
                                                &val);
                    if (err) {
                        LOG_ERR("Error converting ADC measurement: %d", err);
                    }
                    else {
                        LOG_INF("V Batt: %d", val);
                    }
                }
            }

            count = 0; // reset
        }
        count++;
    }
}