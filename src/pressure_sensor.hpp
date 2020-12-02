/**
 * @file		pressure_sensor.hpp
 * @author		Andrew Loebs
 * @brief		Header file of the pressure sensor module
 *
 * Manages sampling of a pressure sensor. Runs in a separate thread which continuously polls sensor
 * for new readings.
 *
 *
 */

#ifndef __PRESSURE_SENSOR_H
#define __PRESSURE_SENSOR_H

#include <device.h>
#include <drivers/sensor.h>
#include <zephyr.h>

namespace z_quad_rotor {

/// Macro for defining an pressure sensor thread entry function
/// @note This must be defined at compile time.
#define PRESSURE_DEFINE_THREAD_ENTRY_FUNC(pressure_sensor, entry_func_name)                        \
    void entry_func_name(void *p1, void *p2, void *p3) { pressure_sensor.entry_func(p1, p2, p3); }

/// Handles setup and sampling of pressure sensor
class PressureSensor {
  public:
    /// Initializes the pressure sensor, spawns a thread, and starts sampling
    /// @note Setup must be done after kernel initialization, necessitating an init function (rather
    /// than doing this in constructor)
    int init(const char *dev_name, k_thread_entry_t entry_func, struct k_thread *thread,
             k_thread_stack_t *stack, size_t stack_size, int priority);
    /// @note This must be public to allow binding of static handler function to member function,
    /// however, this function should not be called directly.
    void entry_func(void *p1, void *p2, void *p3);
    /// Returns the current pressure.
    /// @note Will block until pressure mutex is available
    struct sensor_value get_pressure();

    static const size_t THREAD_STACK_SIZE = 1024;

  protected:
    const struct device *m_dev;
    k_tid_t m_tid;

    struct sensor_value m_pressure;
    struct k_mutex m_pressure_mutex;

  private:
    int setup_pressure_sensor(const char *dev_name);
    int setup_thread(k_thread_entry_t entry_func, struct k_thread *thread, k_thread_stack_t *stack,
                     size_t stack_size, int priority);
};

} // namespace z_quad_rotor

#endif // __PRESSURE_SENSOR_H