/**
 * @file		fxas21002.hpp
 * @author		Andrew Loebs
 * @brief		Header file of the fxas21002 module
 *
 * Thin wrapper around zephyr's fxas21002 driver
 *
 *
 */

#ifndef __FXAS21002_H
#define __FXAS21002_H

#include "marg_sensor.hpp"

namespace z_quad_rotor {

namespace fxas21002 {

int setup(const char *dev_name, MargSensor *output_sink);

}

}

#endif // __FXAS21002_H