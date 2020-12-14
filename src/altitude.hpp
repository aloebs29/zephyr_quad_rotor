/**
 * @file		altitude.hpp
 * @author		Andrew Loebs
 * @brief		Header-only altitude calculation module
 *
 */

#ifndef __ALTITUDE_H
#define __ALTITUDE_H

#include <cmath>

#include <drivers/sensor.h>
#include <zephyr.h>

#include "synced_var.h"

namespace z_quad_rotor {

/// Stores altitude; updates based on raw pressure inputs
class Altitude {
  public:
    Altitude() : m_altitude(0.0f) {}
    /// Updates altitude based on new raw pressure
    void update(struct sensor_value pressure)
    {
        float new_alt =
            44330.0f * (1.0f - powf(sensor_value_to_double(&pressure) / 101.325f, 0.1902949f));
        WriteLock<float> write_access = m_altitude.get_write_lock();
        // if this is the first update, initialize altitude member
        if (m_init) {
            write_access.set_var(new_alt);
            m_init = false;
        }
        else {
            write_access.set_var((new_alt * SMOOTHING_RATIO) +
                                 ((1 - SMOOTHING_RATIO) * write_access.get_var()));
        }
    }
    /// Returns the current altitude in meters
    /// @note Will block until alt mutex is available (locked by update)
    float get_altitude() { return m_altitude.get_read_lock().get_var(); }

  private:
    bool m_init = true;
    SyncedVar<float> m_altitude;

    constexpr static float SMOOTHING_RATIO = 0.03f;
};

} // namespace z_quad_rotor

#endif // __ALTITUDE_H