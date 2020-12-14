/**
 * @file		fxos8700.hpp
 * @author		Andrew Loebs
 * @brief		Header file of the fxos8700 module
 *
 * Thin wrapper around zephyr's fxos8700 driver
 *
 *
 */

#ifndef __FXOS8700_H
#define __FXOS8700_H

#include "marg_sensor.hpp"

namespace z_quad_rotor {

namespace fxos8700 {

int setup(const char *dev_name, MargSensor *output_sink);

}

}

#endif // __FXOS8700_H