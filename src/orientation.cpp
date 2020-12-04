/**
 * @file		orientation.cpp
 * @author		Andrew Loebs
 * @brief		Source file of the orientation module
 *
 */

#include "orientation.hpp"

#include <math.h>
#include <stdlib.h>

#include <logging/log.h>

namespace z_quad_rotor {

LOG_MODULE_REGISTER(orientation, LOG_LEVEL_DBG);

// constants
constexpr float BETA = 0.041f;

// private function declarations
static const MargDataFloat remap_marg_data(MargData &marg_data, const RotationMatrix &remap_matrix);
static bool try_normalize(linalg::vec<float, 3> &vec3);
static void update_magwick_9(MargDataFloat marg_data, Quaternion &quat, uint32_t time_diff_ms);
static EulerAngle quat_to_euler(const Quaternion &quat);

// private function definitions
static const MargDataFloat remap_marg_data(MargData &marg_data, const RotationMatrix &remap_matrix)
{
    MargDataFloat remapped(marg_data);
    remapped.accel = linalg::mul(remap_matrix, remapped.accel);
    remapped.gyro = linalg::mul(remap_matrix, remapped.gyro);
    remapped.magn = linalg::mul(remap_matrix, remapped.magn);

    return remapped;
}

static bool try_normalize(linalg::vec<float, 3> &vec3)
{
    // find magnitude
    float length = linalg::length(vec3);
    // fail on nan
    if (0.0f == length) return false;
    // valid normalization -- perform op and return success
    float norm = 1.0f / length;
    vec3 *= norm;
    return true;
}

static void update_magwick_9(MargDataFloat marg_data, Quaternion &quat, uint32_t time_diff_ms)
{
    // rate of change of quaternion from gyroscope
    Quaternion q_dot(
        -quat.x * marg_data.gyro.x - quat.y * marg_data.gyro.y - quat.z * marg_data.gyro.z,
        quat.w * marg_data.gyro.x + quat.y * marg_data.gyro.z - quat.z * marg_data.gyro.y,
        quat.w * marg_data.gyro.y - quat.x * marg_data.gyro.z + quat.z * marg_data.gyro.x,
        quat.w * marg_data.gyro.z + quat.x * marg_data.gyro.y - quat.y * marg_data.gyro.x);
    q_dot *= 0.5f;

    // normalize accel and mag
    if (!try_normalize(marg_data.accel)) return; // skip iteration if nan occurs
    if (!try_normalize(marg_data.magn)) return;  // skip iteration if nan occurs

    // pre-compute repeated operands
    float qw_mx_2 = 2.0f * quat.w * marg_data.magn.x;
    float qw_my_2 = 2.0f * quat.w * marg_data.magn.y;
    float qw_mz_2 = 2.0f * quat.w * marg_data.magn.z;
    float qx_mx_2 = 2.0f * quat.x * marg_data.magn.x;
    float qw_2 = 2.0f * quat.w;
    float qx_2 = 2.0f * quat.x;
    float qy_2 = 2.0f * quat.y;
    float qz_2 = 2.0f * quat.z;
    float qw_qy_2 = 2.0f * quat.w * quat.y;
    float qy_qz_2 = 2.0f * quat.y * quat.z;
    float qw_qw = quat.w * quat.w;
    float qw_qx = quat.w * quat.x;
    float qw_qy = quat.w * quat.y;
    float qw_qz = quat.w * quat.z;
    float qx_qx = quat.x * quat.x;
    float qx_qy = quat.x * quat.y;
    float qx_qz = quat.x * quat.z;
    float qy_qy = quat.y * quat.y;
    float qy_qz = quat.y * quat.z;
    float qz_qz = quat.z * quat.z;

    // reference direction of Earth's magnetic field
    float hx = marg_data.magn.x * qw_qw - qw_my_2 * quat.z + qw_mz_2 * quat.y +
               marg_data.magn.x * qx_qx + qx_2 * marg_data.magn.y * quat.y +
               qx_2 * marg_data.magn.z * quat.z - marg_data.magn.x * qy_qy -
               marg_data.magn.x * qz_qz;
    float hy = qw_mx_2 * quat.z + marg_data.magn.y * qw_qw - qw_mz_2 * quat.x + qx_mx_2 * quat.y -
               marg_data.magn.y * qx_qx + marg_data.magn.y * qy_qy +
               qy_2 * marg_data.magn.z * quat.z - marg_data.magn.y * qz_qz;
    float bx_2 = sqrt(hx * hx + hy * hy);
    float bz_2 = -qw_mx_2 * quat.y + qw_my_2 * quat.x + marg_data.magn.z * qw_qw +
                 qx_mx_2 * quat.z - marg_data.magn.z * qx_qx + qy_2 * marg_data.magn.y * quat.z -
                 marg_data.magn.z * qy_qy + marg_data.magn.z * qz_qz;
    float bx_4 = 2.0f * bx_2;
    float bz_4 = 2.0f * bz_2;

    // gradient decent algorithm corrective step
    float sw =
        -qy_2 * (2.0f * qx_qz - qw_qy_2 - marg_data.accel.x) +
        qx_2 * (2.0f * qw_qx + qy_qz_2 - marg_data.accel.y) -
        bz_2 * quat.y *
            (bx_2 * (0.5f - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - marg_data.magn.x) +
        (-bx_2 * quat.z + bz_2 * quat.x) *
            (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - marg_data.magn.y) +
        bx_2 * quat.y * (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5f - qx_qx - qy_qy) - marg_data.magn.z);
    float sx = qz_2 * (2.0f * qx_qz - qw_qy_2 - marg_data.accel.x) +
               qw_2 * (2.0f * qw_qx + qy_qz_2 - marg_data.accel.y) -
               4.0f * quat.x * (1 - 2.0f * qx_qx - 2.0f * qy_qy - marg_data.accel.z) +
               bz_2 * quat.z *
                   (bx_2 * (0.5f - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - marg_data.magn.x) +
               (bx_2 * quat.y + bz_2 * quat.w) *
                   (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - marg_data.magn.y) +
               (bx_2 * quat.z - bz_4 * quat.x) *
                   (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5f - qx_qx - qy_qy) - marg_data.magn.z);
    float sy = -qw_2 * (2.0f * qx_qz - qw_qy_2 - marg_data.accel.x) +
               qz_2 * (2.0f * qw_qx + qy_qz_2 - marg_data.accel.y) -
               4.0f * quat.y * (1 - 2.0f * qx_qx - 2.0f * qy_qy - marg_data.accel.z) +
               (-bx_4 * quat.y - bz_2 * quat.w) *
                   (bx_2 * (0.5f - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - marg_data.magn.x) +
               (bx_2 * quat.x + bz_2 * quat.z) *
                   (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - marg_data.magn.y) +
               (bx_2 * quat.w - bz_4 * quat.y) *
                   (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5f - qx_qx - qy_qy) - marg_data.magn.z);
    float sz =
        qx_2 * (2.0f * qx_qz - qw_qy_2 - marg_data.accel.x) +
        qy_2 * (2.0f * qw_qx + qy_qz_2 - marg_data.accel.y) +
        (-bx_4 * quat.z + bz_2 * quat.x) *
            (bx_2 * (0.5f - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - marg_data.magn.x) +
        (-bx_2 * quat.w + bz_2 * quat.y) *
            (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - marg_data.magn.y) +
        bx_2 * quat.x * (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5f - qx_qx - qy_qy) - marg_data.magn.z);
    // normalize
    Quaternion step(sw, sx, sy, sz);
    float norm = 1.0f / linalg::length(step);
    step *= norm;

    // apply feedback step
    q_dot -= BETA * step;
    // integrate rate of change of quaternion to yield quaternion
    quat *= q_dot * (time_diff_ms * 0.001f);
    // normalize
    norm = 1.0f / linalg::length(quat);
    quat *= norm;
}

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

// public functions
Orientation::Orientation(FusionType fusion_type, const RotationMatrix remap_matrix)
    : m_quat(0.0f, 0.0f, 0.0f, 1.0f), m_fusion_type(fusion_type), m_remap_matrix(remap_matrix)
{
    k_mutex_init(&m_quat_mutex);
}

void Orientation::update(MargData &marg_data, uint32_t time_diff_ms)
{
    const MargDataFloat remapped = remap_marg_data(marg_data, m_remap_matrix);
    // don't update while another thread is reading
    k_mutex_lock(&m_quat_mutex, K_FOREVER);
    switch (m_fusion_type) {
        case FUSION_TYPE_MAGWICK_9:
            update_magwick_9(remapped, m_quat, time_diff_ms);
            break;
        default:
            LOG_ERR("Unhandled fusion type in Orientation::update().");
            break;
    }
    k_mutex_unlock(&m_quat_mutex);
}

Quaternion Orientation::get_quaternion()
{
    // guarantee copy happens without data being updated
    k_mutex_lock(&m_quat_mutex, K_FOREVER);
    Quaternion ret = m_quat;
    k_mutex_unlock(&m_quat_mutex);

    return ret;
}

EulerAngle Orientation::get_euler_angle()
{
    // guarantee copy happens without data being updated
    k_mutex_lock(&m_quat_mutex, K_FOREVER);
    EulerAngle ret = quat_to_euler(m_quat);
    k_mutex_unlock(&m_quat_mutex);

    return ret;
}

} // namespace z_quad_rotor