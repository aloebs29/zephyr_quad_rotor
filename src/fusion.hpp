/**
 * @file	fusion.hpp
 * @author	Andrew Loebs
 * @brief	Header file of the fusion module
 *
 * CRTP polymorphic classes for MARG/IMU sensor fusion algorithms
 *
 *
 */

#ifndef __FUSION_H
#define __FUSION_H

#include <cstdint>

#include "marg_sensor.hpp"
#include "orientation_defs.hpp"

namespace z_quad_rotor {

template <class T>
struct FusionImpl {
    void update(MargDataFloat marg_data, Quaternion &quat, uint32_t time_diff_ms) const
    {
        static_cast<const T *>(this)->update(marg_data, quat, time_diff_ms);
    }
};

struct MadgwickFusion6 : FusionImpl<MadgwickFusion6> {
    void update(MargDataFloat marg_data, Quaternion &quat, uint32_t time_diff_ms) const;
};

struct MadgwickFusion9 : FusionImpl<MadgwickFusion9> {
    void update(MargDataFloat marg_data, Quaternion &quat, uint32_t time_diff_ms) const;
};

} // namespace z_quad_rotor

#endif // __FUSION_H