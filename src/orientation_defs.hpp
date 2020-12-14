/**
 * @file	orientation_defs.hpp
 * @author	Andrew Loebs
 * @brief	Header-only orientation definitions
 *
 * Contains type aliases and constants for representing & manipulating orientations (this was needed
 * to prevent a cyclic dependency between fusion & orientation modules).
 *
 *
 */

#ifndef __ORIENTATION_DEFS_H
#define __ORIENTATION_DEFS_H

#include "linalg.h"

namespace z_quad_rotor {

using Quaternion = linalg::vec<float, 4>;
using EulerAngle = linalg::vec<float, 3>;
using RotationMatrix = linalg::mat<float, 3, 3>;

constexpr float PI = 3.1415926535f;
constexpr float PI_OVER_2 = PI / 2.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;

} // namespace z_quad_rotor

#endif // __ORIENTATION_DEFS_H