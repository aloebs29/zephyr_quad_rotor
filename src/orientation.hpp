/**
 * @file		orientation.hpp
 * @author		Andrew Loebs
 * @brief		Header file of the orientation module
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
#include "marg_sensor.hpp"

namespace z_quad_rotor {

using Quaternion = linalg::vec<float, 4>;
using EulerAngle = linalg::vec<float, 3>;
using RotationMatrix = linalg::mat<float, 3, 3>;

constexpr float PI = 3.1415926535f;
constexpr float PI_OVER_2 = PI / 2.0f;

/// Stores orientation in 3D space; updated with raw sensor inputs
class Orientation {
  public:
    /// Enum for selecting the fusion algorithm used by the orientation object
    enum FusionType {
        FUSION_TYPE_MAGWICK_9,
    };
    /// Constructor
    /// @param fusion_type Fusion algorithm to be used for orientation updates.
    /// @param remap_matrix Matrix for remapping raw sensor values to right-hand coordinate system
    /// (e.g. [-1, 0, 0, 0, 0, 1, 0, 1, 0])
    Orientation(FusionType fusion_type, const RotationMatrix remap_matrix);
    /// Updates orientation based on new raw sensor values
    void update(MargData &marg_data, uint32_t time_diff_ms);
    /// Returns the current orientation in quaternion representation.
    /// @note Will block until quat mutex is available (locked by update)
    Quaternion get_quaternion();
    /// Returns the current orientation in euler angle representation.
    /// @note Will block until quat mutex is available (locked by update)
    EulerAngle get_euler_angle();

  protected:
    Quaternion m_quat;
    struct k_mutex m_quat_mutex;

  private:
    const FusionType m_fusion_type;
    const RotationMatrix m_remap_matrix;
};

} // namespace z_quad_rotor

#endif // __ORIENTATION_H