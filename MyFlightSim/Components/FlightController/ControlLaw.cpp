// ======================================================================
// \title  ControlLaw.cpp
// \brief  Cascaded quadcopter control law
// ======================================================================

#include "MyFlightSim/Components/FlightController/ControlLaw.hpp"
#include "MyFlightSim/Components/FlightController/FlightMath.hpp"

#include <cmath>

namespace MyFlightSim {

namespace {
F32 clamp(F32 x, F32 lo, F32 hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

//! Wrap an angle to [-pi, pi)
F32 wrapAngle(F32 a) {
    while (a >= FlightMath::PI) {
        a -= 2.0f * FlightMath::PI;
    }
    while (a < -FlightMath::PI) {
        a += 2.0f * FlightMath::PI;
    }
    return a;
}

constexpr F32 VEL_Z_INTEGRAL_LIMIT = 2.0f;  //!< Anti-windup limit (m)
constexpr F32 MIN_TILT_COS = 0.5f;          //!< Never boost thrust for more than 60 degrees of tilt
}  // namespace

ControlLaw::ControlLaw() : m_velZIntegral(0.0f) {}

void ControlLaw::reset() {
    this->m_velZIntegral = 0.0f;
}

F32 ControlLaw::hoverThrottle(const ControlGains& g) {
    return std::sqrt(g.mass * g.gravity / (4.0f * g.motorMaxThrust));
}

MotorThrottles ControlLaw::step(const SensorData& s, const Setpoint& sp, const ControlGains& g, F32 dt) {
    const Vector3& p = s.get_position();
    const Vector3& v = s.get_velocity();
    const Vector3& w = s.get_gyro();
    const Vector3 euler = FlightMath::eulerFromQuaternion(s.get_attitude());
    const F32 roll = euler.get_x();
    const F32 pitch = euler.get_y();
    const F32 yaw = euler.get_z();

    // 1. Position error -> desired velocity (limited)
    F32 vdx = g.posKp * (sp.position.get_x() - p.get_x());
    F32 vdy = g.posKp * (sp.position.get_y() - p.get_y());
    const F32 horizontal = std::sqrt(vdx * vdx + vdy * vdy);
    if (horizontal > g.maxSpeed) {
        vdx *= g.maxSpeed / horizontal;
        vdy *= g.maxSpeed / horizontal;
    }
    F32 vdz = sp.useClimbRate ? sp.climbRate : g.posKp * (sp.position.get_z() - p.get_z());
    vdz = clamp(vdz, -g.maxClimb, g.maxClimb);

    // 2. Velocity error -> desired acceleration (PI on the vertical axis)
    const F32 ax = g.velKp * (vdx - v.get_x());
    const F32 ay = g.velKp * (vdy - v.get_y());
    const F32 ez = vdz - v.get_z();
    this->m_velZIntegral = clamp(this->m_velZIntegral + ez * dt, -VEL_Z_INTEGRAL_LIMIT, VEL_Z_INTEGRAL_LIMIT);
    const F32 az = g.velZKp * ez + g.velZKi * this->m_velZIntegral;

    // 3. Desired acceleration -> desired roll/pitch and collective thrust
    const F32 cy = std::cos(yaw);
    const F32 sy = std::sin(yaw);
    const F32 aForward = ax * cy + ay * sy;
    const F32 aLeft = -ax * sy + ay * cy;
    const F32 vertical = g.gravity + az;
    const F32 pitchDesired = clamp(std::atan2(aForward, vertical), -g.maxTiltRad, g.maxTiltRad);
    const F32 rollDesired = clamp(std::atan2(-aLeft, vertical), -g.maxTiltRad, g.maxTiltRad);
    F32 tiltCos = std::cos(roll) * std::cos(pitch);
    tiltCos = (tiltCos < MIN_TILT_COS) ? MIN_TILT_COS : tiltCos;
    const F32 thrust = g.mass * vertical / tiltCos;

    // 4. Attitude error -> desired body rates -> torques (rate P loop)
    const F32 pDesired = g.attKp * (rollDesired - roll);
    const F32 qDesired = g.attKp * (pitchDesired - pitch);
    const F32 rDesired = g.yawKp * wrapAngle(sp.yaw - yaw);
    const F32 tx = Vehicle::INERTIA_XX * g.rateKp * (pDesired - w.get_x());
    const F32 ty = Vehicle::INERTIA_YY * g.rateKp * (qDesired - w.get_y());
    const F32 tz = Vehicle::INERTIA_ZZ * g.yawRateKp * (rDesired - w.get_z());

    // 5. Mixer: collective thrust and three torques -> four motor thrusts
    const F32 armMoment = Vehicle::ARM_LENGTH / std::sqrt(2.0f);
    const F32 t = thrust / 4.0f;
    const F32 a = tx / (4.0f * armMoment);
    const F32 b = ty / (4.0f * armMoment);
    const F32 c = tz / (4.0f * Vehicle::YAW_TORQUE_PER_THRUST);
    const F32 motorThrust[4] = {t + a - b + c, t - a - b - c, t - a + b + c, t + a + b - c};

    // 6. Thrust is proportional to rotor speed squared: throttle = sqrt(thrust / max)
    MotorThrottles throttles;
    for (FwSizeType i = 0; i < MotorThrottles::SIZE; i++) {
        const F32 fraction = clamp(motorThrust[i] / g.motorMaxThrust, 0.0f, 1.0f);
        throttles[i] = std::sqrt(fraction);
    }
    return throttles;
}

}  // namespace MyFlightSim
