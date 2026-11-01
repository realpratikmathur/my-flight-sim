// ======================================================================
// \title  ControlLaw.hpp
// \brief  Cascaded quadcopter control law (plain C++, no F Prime ports)
//
// position -> velocity -> acceleration -> attitude -> body rates -> torques
// -> motor mixer. Keeping the algorithm in a plain class (instead of inside
// the component) makes it easy to unit test and to reuse; the F Prime
// component only feeds it sensors, setpoints and parameters.
//
// Conventions: world x east, y north, z up; body x forward, y left, z up.
// Motor order: 0 front-left, 1 front-right, 2 back-right, 3 back-left.
// ======================================================================

#ifndef MyFlightSim_ControlLaw_HPP
#define MyFlightSim_ControlLaw_HPP

#include "MyFlightSim/Types/MotorThrottlesArrayAc.hpp"
#include "MyFlightSim/Types/SensorDataSerializableAc.hpp"
#include "MyFlightSim/Types/Vector3SerializableAc.hpp"

namespace MyFlightSim {

//! Fixed vehicle geometry. Must match the airframe (here: QuadParams in sim/physics.py).
namespace Vehicle {
constexpr F32 ARM_LENGTH = 0.17f;              //!< Centre to motor (m)
constexpr F32 YAW_TORQUE_PER_THRUST = 0.016f;  //!< Reaction torque per newton of thrust (m)
constexpr F32 INERTIA_XX = 0.012f;             //!< kg m^2
constexpr F32 INERTIA_YY = 0.012f;             //!< kg m^2
constexpr F32 INERTIA_ZZ = 0.022f;             //!< kg m^2
}  // namespace Vehicle

//! Tuning values and vehicle properties, filled from component parameters
struct ControlGains {
    F32 posKp;           //!< (m/s) per m
    F32 velKp;           //!< (m/s^2) per (m/s), horizontal
    F32 velZKp;          //!< (m/s^2) per (m/s), vertical
    F32 velZKi;          //!< (m/s^2) per m, vertical integral
    F32 attKp;           //!< (rad/s) per rad
    F32 rateKp;          //!< 1/s, roll and pitch
    F32 yawKp;           //!< (rad/s) per rad
    F32 yawRateKp;       //!< 1/s
    F32 maxSpeed;        //!< m/s, horizontal
    F32 maxClimb;        //!< m/s
    F32 maxTiltRad;      //!< rad
    F32 mass;            //!< kg
    F32 motorMaxThrust;  //!< N per motor
    F32 gravity;         //!< m/s^2
};

//! What the controller should do this step
struct Setpoint {
    Vector3 position;   //!< Target position (m), world frame
    F32 yaw;            //!< Target heading (rad), 0 = east, counter-clockwise positive
    bool useClimbRate;  //!< If true, ignore position.z and use climbRate instead
    F32 climbRate;      //!< Vertical speed (m/s) when useClimbRate is true
};

class ControlLaw {
  public:
    ControlLaw();

    //! Clear the integrator (call when starting closed-loop flight)
    void reset();

    //! One control step of length dt seconds
    MotorThrottles step(const SensorData& sensors, const Setpoint& setpoint, const ControlGains& gains, F32 dt);

    //! Throttle at which the four motors together balance gravity
    static F32 hoverThrottle(const ControlGains& gains);

  private:
    F32 m_velZIntegral;  //!< Integral of the vertical velocity error (m)
};

}  // namespace MyFlightSim

#endif
