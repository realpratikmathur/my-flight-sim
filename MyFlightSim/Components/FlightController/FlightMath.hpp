// ======================================================================
// \title  FlightMath.hpp
// \brief  Small, allocation-free math helpers for the flight controller
//
// Conventions: world frame x east, y north, z up (ENU); body frame x forward,
// y left, z up (FLU). Quaternions rotate body vectors into the world frame.
// ======================================================================

#ifndef MyFlightSim_FlightMath_HPP
#define MyFlightSim_FlightMath_HPP

#include <cmath>

#include "MyFlightSim/Types/QuaternionSerializableAc.hpp"
#include "MyFlightSim/Types/Vector3SerializableAc.hpp"

namespace MyFlightSim {
namespace FlightMath {

constexpr F32 PI = 3.14159265358979f;
constexpr F32 RAD_TO_DEG = 180.0f / PI;

//! Roll, pitch and yaw (radians, ZYX convention) from an attitude quaternion
inline Vector3 eulerFromQuaternion(const Quaternion& q) {
    const F32 w = q.get_w();
    const F32 x = q.get_x();
    const F32 y = q.get_y();
    const F32 z = q.get_z();
    const F32 roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    F32 sinPitch = 2.0f * (w * y - z * x);
    sinPitch = (sinPitch > 1.0f) ? 1.0f : ((sinPitch < -1.0f) ? -1.0f : sinPitch);
    const F32 pitch = std::asin(sinPitch);
    const F32 yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return Vector3(roll, pitch, yaw);
}

//! Multiply every component of a vector by a scalar
inline Vector3 scale(const Vector3& v, F32 k) {
    return Vector3(v.get_x() * k, v.get_y() * k, v.get_z() * k);
}

}  // namespace FlightMath
}  // namespace MyFlightSim

#endif
