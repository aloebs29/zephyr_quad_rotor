/**
 * @file		main.cpp
 * @author		Andrew Loebs
 * @brief		Main application
 *
 */

#include <device.h>
#include <logging/log.h>
#include <usb/usb_device.h>
#include <zephyr.h>

#include "marg_sensor.hpp"

using namespace z_quad_rotor;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// static variables
static MargSensor marg_sensor;

MARG_DEFINE_FXOS8700_TRIG_HANDLER(marg_sensor, fxos8700_trig_handler);
MARG_DEFINE_FXAS21002_TRIG_HANDLER(marg_sensor, fxas21002_trig_handler);

void main(void)
{
    // enable USB for shell backend
    usb_enable(NULL);

    // setup orientation subsystem
    int err = marg_sensor.init(DT_LABEL(DT_INST(0, nxp_fxos8700)), "TODO", fxos8700_trig_handler,
                               fxas21002_trig_handler);
    if (err) {
        LOG_ERR("MARG sensor initialization error: %d", err);
    }

    // log samples as they are ready
    for (;;) {
        k_sleep(K_MSEC(1000));

        MargData data = marg_sensor.get_data();

        LOG_INF("AX:%3d.%06d AY:%3d.%06d AZ:%3d.%06d", data.accel[0].val1, data.accel[0].val2,
                data.accel[1].val1, data.accel[1].val2, data.accel[2].val1, data.accel[2].val2);

        LOG_INF("MX:%3d.%06d MY:%3d.%06d MZ:%3d.%06d", data.magn[0].val1, data.magn[0].val2,
                data.magn[1].val1, data.magn[1].val2, data.magn[2].val1, data.magn[2].val2);
    }
}