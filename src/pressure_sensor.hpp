/**
 * @file		pressure_sensor.hpp
 * @author	Andrew Loebs
 * @brief		Header-only pressure sensor module
 *
 * Defines pressure sensor class for handling read/write access to MARG data.
 *
 *
 */

#ifndef __PRESSURE_SENSOR_H
#define __PRESSURE_SENSOR_H

#include <drivers/sensor.h>

#include "synced_var.hpp"

namespace z_quad_rotor {

/// Handles setup and sampling of pressure sensor
class PressureSensor {
  public:
    /// Returns the current pressure.
    /// @note Will block until pressure mutex is available
    struct sensor_value get_pressure() { return m_pressure.get_read_lock().get_var(); }
    /// Returns a write lock to the MARG sensor data
    WriteLock<struct sensor_value> get_write_lock() { return m_pressure.get_write_lock(); }

  protected:
    SyncedVar<struct sensor_value> m_pressure;
};

} // namespace z_quad_rotor

#endif // __PRESSURE_SENSOR_H