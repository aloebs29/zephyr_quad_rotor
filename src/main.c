/**
 * @file		main.c
 * @author		Andrew Loebs
 * @brief		Main application
 *
 */

#include <zephyr.h>
#include <usb/usb_device.h>
#include <logging/log.h>
#include <drivers/sensor.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

K_SEM_DEFINE(sem_fxos8700_drdy, 0, 1);

static void fxos8700_trig_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    ARG_UNUSED(trigger);

    if (0 != sensor_sample_fetch(dev)) {
        // error
        LOG_ERR("FXOS8700 sensor_sample_fetch() failed.");
        return; // exit without giving semaphore
    }

    k_sem_give(&sem_fxos8700_drdy);
}

void main(void)
{
    // Enable USB for shell backend
    usb_enable(NULL);

    LOG_INF("z_quad_rotor firmware running..");

    struct sensor_value accel[3];
    struct sensor_value mag[3];

    const struct device *fxos8700 = device_get_binding(DT_LABEL(DT_INST(0, nxp_fxos8700)));
    if (NULL == fxos8700) {
        LOG_ERR("unable to bind FXOS8700 device.");
        return;
    }

    struct sensor_value fxos8700_sample_rate = {
		.val1 = 6,
		.val2 = 250000,
	};

    if (0 != sensor_attr_set(fxos8700, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &fxos8700_sample_rate)) {
        LOG_ERR("unable to set FXOS8700 sample rate.");
        return;
    }

    struct sensor_trigger fxos8700_trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    if (0 != sensor_trigger_set(fxos8700, &fxos8700_trig, fxos8700_trig_handler)) {
        LOG_ERR("unable to set FXOS8700 trigger.");
    }

    // log samples as they are ready
    for (;;) {
        // wait for trigger
        k_sem_take(&sem_fxos8700_drdy, K_FOREVER);

        sensor_channel_get(fxos8700, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(fxos8700, SENSOR_CHAN_MAGN_XYZ, mag);

        LOG_INF("AX:%3d.%06d AY:%3d.%06d AZ:%3d.%06d", 
            accel[0].val1, accel[0].val2,
            accel[1].val1, accel[1].val2,
            accel[2].val1, accel[2].val2);

        LOG_INF("MX:%3d.%06d MY:%3d.%06d MZ:%3d.%06d", 
            mag[0].val1, mag[0].val2,
            mag[1].val1, mag[1].val2,
            mag[2].val1, mag[2].val2);
    }
}