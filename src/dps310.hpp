/**
 * @file		dps310.hpp
 * @author		Andrew Loebs
 * @brief		Header file of the dps310 module
 *
 * Thin wrapper around zephyr's dps310 driver
 *
 *
 */

#ifndef __DPS310_H
#define __DPS310_H

#include "pressure_sensor.hpp"

namespace z_quad_rotor {

namespace dps310 {

/// Initializes the sensor
int setup(const char *dev_name);

/// Reads pressure data and writes to output
/// @note Blocking call -- will yield thread while waiting on conversion & i2c transaction
int read_pressure(PressureSensor *output);

} // namespace dps310

} // namespace z_quad_rotor

#endif // __DPS310_H