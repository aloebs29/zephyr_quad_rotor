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

namespace z_quad_rotor {

/// Stores altitude; updates based on raw pressure inputs
class Altitude {
  public:
    /// Constructor (no-params)
    Altitude() { k_mutex_init(&m_alt_mutex); }
    /// Updates altitude based on new raw pressure
    void update(struct sensor_value pressure)
    {
        float new_alt =
            44330.0f * (1.0f - powf(sensor_value_to_double(&pressure) / 101.325f, 0.1902949f));
        // don't update while another thread is reading
        k_mutex_lock(&m_alt_mutex, K_FOREVER);
        // if this is the first update, initialize altitude member
        if (m_init) {
            m_alt = new_alt;
            m_init = false;
        }
        else {
            m_alt = (new_alt * SMOOTHING_RATIO) + ((1 - SMOOTHING_RATIO) * m_alt);
        }
        k_mutex_unlock(&m_alt_mutex);
    }
    /// Returns the current altitude in meters
    /// @note Will block until alt mutex is available (locked by update)
    float get_altitude()
    {
        // guarantee copy happens without data being updated
        k_mutex_lock(&m_alt_mutex, K_FOREVER);
        float ret = m_alt;
        k_mutex_unlock(&m_alt_mutex);

        return ret;
    }

  private:
    bool m_init = true;
    float m_alt;
    struct k_mutex m_alt_mutex;

    constexpr static float SMOOTHING_RATIO = 0.03f;
};

} // namespace z_quad_rotor

#endif // __ALTITUDE_H