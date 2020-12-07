/**
 * @file		orientation.hpp
 * @author		Andrew Loebs
 * @brief		Header-only orientation module
 *
 * Derives orientation from raw MARG sensor values using selected sensor fusion algorithm.
 *
 *
 */

#ifndef __ORIENTATION_H
#define __ORIENTATION_H

#include <cstdint>

#include <zephyr.h>

#include "linalg.h"

#include "fusion.hpp"
#include "marg_sensor.hpp"
#include "orientation_defs.hpp"
#include "synced_var.h"

namespace z_quad_rotor {

/// Stores orientation in 3D space; updates based on raw MARG inputs
/// @tparam Fusion implementation to be used for updates
template <class T>
class Orientation {
  public:
    /// Constructor
    /// @param remap_matrix Matrix for remapping raw sensor values to right-hand coordinate system
    /// (e.g. [-1, 0, 0, 0, 0, 1, 0, 1, 0])
    Orientation(const RotationMatrix &remap_matrix)
        : m_quat(Quaternion(0.0f, 0.0f, 0.0f, 1.0f)), m_fusion_impl(), m_remap_matrix(remap_matrix)
    {
    }
    /// Updates orientation based on new raw sensor values
    void update(MargData &marg_data, uint32_t time_diff_ms)
    {
        MargDataFloat remapped = remap_marg_data(marg_data, m_remap_matrix);
        // We should not need to scale the gyro measurements (zephyr claims gyro outputs should be
        // rad/s), so this is a "temporary" fix.
        remapped.gyro *= DEG_TO_RAD;
        m_fusion_impl.update(remapped, m_quat.write_access().get_ref(), time_diff_ms);
    }
    /// Returns the current orientation in quaternion representation.
    /// @note Will block until quat mutex is available (locked by update)
    Quaternion get_quaternion() { return m_quat.read_access().get_var(); }
    /// Returns the current orientation in euler angle representation (degrees).
    /// @note Will block until quat mutex is available (locked by update)
    EulerAngle get_euler_angle() { return quat_to_euler(m_quat.read_access().get_var()); }

  protected:
    SyncedVar<Quaternion> m_quat;

  private:
    const FusionImpl<T> m_fusion_impl;
    const RotationMatrix m_remap_matrix;
    /// converts marg data from sensor value to float, remaps according to remap matrix
    static const MargDataFloat remap_marg_data(MargData &marg_data,
                                               const RotationMatrix &remap_matrix)
    {
        MargDataFloat remapped(marg_data);
        remapped.accel = linalg::mul(remap_matrix, remapped.accel);
        remapped.gyro = linalg::mul(remap_matrix, remapped.gyro);
        remapped.magn = linalg::mul(remap_matrix, remapped.magn);

        return remapped;
    }
    /// converts quaternion orientation to euler angles
    static EulerAngle quat_to_euler(const Quaternion &quat)
    {
        float roll = atan2f(2 * (quat.w * quat.x + quat.y * quat.z),
                            1 - 2 * (quat.x * quat.x + quat.y * quat.y));
        // limit pitch to +/- 90
        float sin_pitch = 2 * (quat.w * quat.y - quat.z * quat.x);
        float pitch = copysign(PI_OVER_2, sin_pitch);
        if (abs(sin_pitch) < 1) pitch = asinf(sin_pitch);

        float yaw = atan2f(2 * (quat.w * quat.z + quat.x * quat.y),
                           1 - 2 * (quat.y * quat.y + quat.z * quat.z));

        return EulerAngle(roll, pitch, yaw);
    }
};

} // namespace z_quad_rotor

#endif // __ORIENTATION_H