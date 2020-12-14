/**
 * @file		marg_sensor.hpp
 * @author	Andrew Loebs
 * @brief		Header-only MARG sensor module
 *
 * Defines MARG data types, sensor class for handling read/write access to MARG data.
 *
 *
 */

#ifndef __MARG_SENSOR_H
#define __MARG_SENSOR_H

#include <drivers/sensor.h>

#include "linalg.h"

#include "synced_var.hpp"

namespace z_quad_rotor {

/// Structure for 9DOF data
struct MargData {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];
};

struct MargDataFloat {
    linalg::vec<float, 3> accel;
    linalg::vec<float, 3> gyro;
    linalg::vec<float, 3> magn;
    MargDataFloat(MargData &in)
        : accel(sensor_value_to_double(&in.accel[0]), sensor_value_to_double(&in.accel[1]),
                sensor_value_to_double(&in.accel[2])),
          gyro(sensor_value_to_double(&in.gyro[0]), sensor_value_to_double(&in.gyro[1]),
               sensor_value_to_double(&in.gyro[2])),
          magn(sensor_value_to_double(&in.magn[0]), sensor_value_to_double(&in.magn[1]),
               sensor_value_to_double(&in.magn[2]))
    {
    }
};

/// Manages read/write access to MARG sensor data
class MargSensor {
  public:
    /// Returns the current MARG sensor data.
    /// @note Will block until data lock is available
    MargData get_marg() { return m_marg_data.get_read_lock().get_var(); }
    /// Returns a write lock to the MARG sensor data
    WriteLock<MargData> get_write_lock() { return m_marg_data.get_write_lock(); }

  protected:
    SyncedVar<MargData> m_marg_data;
};

} // namespace z_quad_rotor

#endif // __MARG_SENSOR_H