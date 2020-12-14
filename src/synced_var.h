/**
 * @file		synced_var.hpp
 * @author		Andrew Loebs
 * @brief		Header-only template for mutex-protected access to a var
 *
 */

#ifndef __SYNCED_VAR_H
#define __SYNCED_VAR_H

#include <zephyr.h>

namespace z_quad_rotor {

template <class T>
class WriteLock {
  public:
    WriteLock(T &var, k_mutex &mutex) : m_var(var), m_mutex(mutex)
    {
        k_mutex_lock(&mutex, K_FOREVER);
    }
    ~WriteLock() { k_mutex_unlock(&m_mutex); }
    const T &get_var() { return m_var; }
    T &get_ref() { return m_var; }
    void set_var(T val) { m_var = val; }

  private:
    T &m_var;
    struct k_mutex &m_mutex;
};
template <class T>
class ReadLock {
  public:
    ReadLock(const T &var, k_mutex &mutex) : m_var(var), m_mutex(mutex)
    {
        k_mutex_lock(&mutex, K_FOREVER);
    }
    ~ReadLock() { k_mutex_unlock(&m_mutex); }
    const T &get_var() { return m_var; }

  private:
    const T &m_var;
    struct k_mutex &m_mutex;
};
/// Provides mutex-protection for variable access
template <class T>
class SyncedVar {
  public:
    /// Constructor -- initializes mutex as unlocked
    SyncedVar() : m_value() { k_mutex_init(&m_mutex); }
    SyncedVar(T initial_val) : m_value(initial_val) { k_mutex_init(&m_mutex); }
    SyncedVar(T &initial_val) : m_value(initial_val) { k_mutex_init(&m_mutex); }
    /// Waits for mutex to be unlocked and returns mutable access to variable.
    /// @note mutex will be unlocked on destruction
    WriteLock<T> get_write_lock() { return WriteLock<T>(m_value, m_mutex); }
    /// Waits for mutex to be unlocked and returns immutable access to variable.
    /// @note mutex will be unlocked on destruction
    ReadLock<T> get_read_lock() { return ReadLock<T>(m_value, m_mutex); }

  private:
    T m_value;
    struct k_mutex m_mutex;
};

} // namespace z_quad_rotor

#endif // __SYNCED_VAR_H