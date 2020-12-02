/**
 * @file		marg_sensor.hpp
 * @author		Andrew Loebs
 * @brief		Header file of the MARG sensor module
 *
 * Manages sampling of accelerometer, gyroscope, and magnetometer sensor data.
 *
 *
 */

#ifndef __MARG_SENSOR_H
#define __MARG_SENSOR_H

#include <cstdint>
#include <device.h>
#include <drivers/sensor.h>
#include <zephyr.h>

namespace z_quad_rotor {

/// Structure for 9DOF data
struct MargData {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];
};

/// Macro for defining an fxos8700 trigger handler.
/// @note This must be defined at compile time.
#define MARG_DEFINE_FXOS8700_TRIG_HANDLER(marg_sensor, handler_name)                               \
    void handler_name(const struct device *d, struct sensor_trigger *t)                            \
    {                                                                                              \
        marg_sensor.fxos8700_trig_handler(d, t);                                                   \
    }

/// Macro for defining an fxas21002 trigger handler.
/// @note This must be defined at compile time.
#define MARG_DEFINE_FXAS21002_TRIG_HANDLER(marg_sensor, handler_name)                              \
    void handler_name(const struct device *d, struct sensor_trigger *t)                            \
    {                                                                                              \
        marg_sensor.fxas21002_trig_handler(d, t);                                                  \
    }

/// Handles setup and sampling of 9DOF sensors
class MargSensor {
  public:
    /// Initializes the MARG sensor and begins sampling.
    /// @note Setup must be done after kernel initialization, necessitating an init function (rather
    /// than doing this in constructor)
    int init(const char *fxos8700_dev_name, const char *fxas21002_dev_name,
             sensor_trigger_handler_t fxos8700_trig_handler,
             sensor_trigger_handler_t fxas21002_trig_handler);
    /// @note This must be public to allow binding of static handler function to member function,
    /// however, this function should not be called directly.
    void fxos8700_trig_handler(const struct device *dev, struct sensor_trigger *trigger);
    /// @note This must be public to allow binding of static handler function to member function,
    /// however, this function should not be called directly.
    void fxas21002_trig_handler(const struct device *dev, struct sensor_trigger *trigger);
    /// Returns the current MARG sensor data.
    /// @note Will block until MARG data mutex is available
    MargData get_data();

  protected:
    const struct device *m_fxos8700;
    const struct device *m_fxas21002;

    MargData m_data;
    struct k_mutex m_data_mutex;

  private:
    int setup_fxos8700(const char *dev_name, sensor_trigger_handler_t trig_handler);
    int setup_fxas21002(const char *dev_name, sensor_trigger_handler_t trig_handler);
};

} // namespace z_quad_rotor

#endif // __MARG_SENSOR_H