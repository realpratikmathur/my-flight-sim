// ======================================================================
// \title  FlightController.cpp
// \author my-flight-sim
// \brief  cpp file for FlightController component implementation class
// ======================================================================

#include "MyFlightSim/Components/FlightController/FlightController.hpp"
#include "MyFlightSim/Components/FlightController/FlightMath.hpp"

#include <cmath>

namespace MyFlightSim {

// Out-of-class definitions of static constexpr members (required in C++14,
// the language standard F Prime uses, when they are passed by reference)
constexpr F32 FlightController::IDLE_THROTTLE;
constexpr U32 FlightController::TELEMETRY_DECIMATION;
constexpr F32 FlightController::CONTROL_DT;
constexpr F32 FlightController::MIN_ALTITUDE;
constexpr F32 FlightController::MAX_ALTITUDE;
constexpr F32 FlightController::ARRIVAL_DISTANCE;
constexpr F32 FlightController::ARRIVAL_SPEED;
constexpr F32 FlightController::TOUCHDOWN_RANGE;
constexpr F32 FlightController::TOUCHDOWN_SPEED;
constexpr U32 FlightController::TOUCHDOWN_STEPS;

namespace {

//! Allowed range for each parameter; values outside are clamped (and reported)
struct ParamRange {
    F32 minimum;
    F32 maximum;
};
constexpr ParamRange GAIN_RANGE = {0.0f, 100.0f};
constexpr ParamRange MAX_SPEED_RANGE = {0.5f, 15.0f};
constexpr ParamRange MAX_CLIMB_RANGE = {0.2f, 5.0f};
constexpr ParamRange MAX_TILT_RANGE = {5.0f, 45.0f};
constexpr ParamRange LAND_SPEED_RANGE = {0.1f, 3.0f};
constexpr ParamRange MASS_RANGE = {0.1f, 20.0f};
constexpr ParamRange THRUST_RANGE = {0.5f, 100.0f};
constexpr ParamRange GRAVITY_RANGE = {1.0f, 20.0f};
constexpr ParamRange GEOFENCE_RANGE = {1.0f, 1000.0f};

F32 clampTo(F32 value, const ParamRange& range) {
    return (value < range.minimum) ? range.minimum : ((value > range.maximum) ? range.maximum : value);
}

F32 norm3(F32 x, F32 y, F32 z) {
    return std::sqrt(x * x + y * y + z * z);
}

}  // namespace

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

FlightController ::FlightController(const char* const compName)
    : FlightControllerComponentBase(compName),
      m_mode(FlightMode::DISARMED),
      m_testLevel(0.0f),
      m_cycle(0),
      m_sensors(),
      m_target(),
      m_targetYaw(0.0f),
      m_touchdownSteps(0),
      m_control() {}

FlightController ::~FlightController() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void FlightController ::run_handler(FwIndexType portNum, U32 context) {
    // 1. Read the sensors (a synchronous port call that returns a value).
    //    Done first so that command handlers below see fresh data.
    this->m_sensors = this->sensorsGet_out(0);

    // 2. Pull queued commands into this cycle. This is what makes the
    //    component "queued": the async command handlers below run here, on
    //    the rate group thread, at a well-defined point in the control step.
    (void)this->dispatchAvailableMessages();

    // 3. Read the parameters (they can change at any time via PRM_SET)
    const ControlGains gains = this->readGains();

    // 4. Decide the motor throttles for the current mode
    MotorThrottles motors(0.0f);
    switch (this->m_mode.e) {
        case FlightMode::ARMED_IDLE:
            motors = MotorThrottles(IDLE_THROTTLE);
            break;
        case FlightMode::MOTOR_TEST:
            motors = MotorThrottles(this->m_testLevel);
            break;
        case FlightMode::TAKEOFF:
        case FlightMode::HOLD:
        case FlightMode::GOTO:
        case FlightMode::LANDING:
            motors = this->flyClosedLoop(gains);
            break;
        case FlightMode::DISARMED:
        default:
            break;
    }

    // 5. Command the motors
    this->motorsOut_out(0, motors);

    // 6. Telemetry at 10 Hz
    if ((this->m_cycle % TELEMETRY_DECIMATION) == 0) {
        this->writeTelemetry(motors, gains);
    }
    this->m_cycle++;
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void FlightController ::ARM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    if (this->m_mode == FlightMode::DISARMED) {
        this->m_mode = FlightMode::ARMED_IDLE;
        this->log_ACTIVITY_HI_Armed();
    }
    // Arming while already armed is harmless: report success
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FlightController ::DISARM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // Always accepted, even in flight: operators must always be able to stop the motors
    if (this->m_mode != FlightMode::DISARMED) {
        this->m_mode = FlightMode::DISARMED;
        this->log_ACTIVITY_HI_Disarmed();
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FlightController ::MOTOR_TEST_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 level) {
    // Check the argument before anything else
    if (!(level >= 0.0f && level <= 1.0f)) {  // also rejects NaN
        this->log_WARNING_LO_InvalidThrottle(level);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    // Then check the vehicle state
    if (this->m_mode == FlightMode::DISARMED) {
        this->log_WARNING_LO_NotArmed();
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    if (!this->isArmedOnGround()) {
        this->log_WARNING_LO_NotOnGround(this->m_mode);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    this->m_mode = FlightMode::MOTOR_TEST;
    this->m_testLevel = level;
    this->log_ACTIVITY_HI_MotorTestStarted(level);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FlightController ::TAKEOFF_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 altitude) {
    if (!(altitude >= MIN_ALTITUDE && altitude <= MAX_ALTITUDE)) {
        this->log_WARNING_LO_InvalidAltitude(altitude);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    if (this->m_mode == FlightMode::DISARMED) {
        this->log_WARNING_LO_NotArmed();
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    if (!this->isArmedOnGround()) {
        this->log_WARNING_LO_NotOnGround(this->m_mode);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    // Climb straight up from where we are, keeping the current heading
    const Vector3& p = this->m_sensors.get_position();
    this->m_target = Vector3(p.get_x(), p.get_y(), altitude);
    this->m_targetYaw = FlightMath::eulerFromQuaternion(this->m_sensors.get_attitude()).get_z();
    this->startClosedLoop(FlightMode::TAKEOFF);
    this->log_ACTIVITY_HI_TakeoffStarted(altitude);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FlightController ::GOTO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 east, F32 north, F32 altitude) {
    if (!(altitude >= MIN_ALTITUDE && altitude <= MAX_ALTITUDE)) {
        this->log_WARNING_LO_InvalidAltitude(altitude);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    Fw::ParamValid valid;
    const F32 geofence = clampTo(this->paramGet_GEOFENCE_RADIUS(valid), GEOFENCE_RANGE);
    const F32 distance = std::sqrt(east * east + north * north);
    if (!(distance <= geofence)) {  // also rejects NaN
        this->log_WARNING_LO_OutsideGeofence(distance, geofence);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }
    if (!this->isFlying()) {
        this->log_WARNING_LO_NotFlying(this->m_mode);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    this->m_target = Vector3(east, north, altitude);
    this->m_mode = FlightMode::GOTO;
    this->log_ACTIVITY_HI_GotoStarted(east, north, altitude);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void FlightController ::LAND_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    if (!this->isFlying()) {
        this->log_WARNING_LO_NotFlying(this->m_mode);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }
    // Hold the current horizontal position while descending
    const Vector3& p = this->m_sensors.get_position();
    this->m_target = Vector3(p.get_x(), p.get_y(), 0.0f);
    this->m_mode = FlightMode::LANDING;
    this->m_touchdownSteps = 0;
    Fw::ParamValid valid;
    this->log_ACTIVITY_HI_LandingStarted(clampTo(this->paramGet_LAND_SPEED(valid), LAND_SPEED_RANGE));
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Parameter hook
// ----------------------------------------------------------------------

void FlightController ::parameterUpdated(FwPrmIdType id) {
    // Runs when a <NAME>_PRM_SET command is accepted. The new value is already
    // stored; here we only warn if it will be clamped when used.
    Fw::ParamValid valid;
    F32 value = 0.0f;
    ParamRange range = GAIN_RANGE;
    switch (id) {
        case PARAMID_MAX_SPEED:
            value = this->paramGet_MAX_SPEED(valid);
            range = MAX_SPEED_RANGE;
            break;
        case PARAMID_MAX_CLIMB:
            value = this->paramGet_MAX_CLIMB(valid);
            range = MAX_CLIMB_RANGE;
            break;
        case PARAMID_MAX_TILT_DEG:
            value = this->paramGet_MAX_TILT_DEG(valid);
            range = MAX_TILT_RANGE;
            break;
        case PARAMID_LAND_SPEED:
            value = this->paramGet_LAND_SPEED(valid);
            range = LAND_SPEED_RANGE;
            break;
        case PARAMID_VEHICLE_MASS:
            value = this->paramGet_VEHICLE_MASS(valid);
            range = MASS_RANGE;
            break;
        case PARAMID_MOTOR_MAX_THRUST:
            value = this->paramGet_MOTOR_MAX_THRUST(valid);
            range = THRUST_RANGE;
            break;
        case PARAMID_GRAVITY:
            value = this->paramGet_GRAVITY(valid);
            range = GRAVITY_RANGE;
            break;
        case PARAMID_GEOFENCE_RADIUS:
            value = this->paramGet_GEOFENCE_RADIUS(valid);
            range = GEOFENCE_RANGE;
            break;
        default:
            return;  // gains: any value in GAIN_RANGE; not reported
    }
    if (!(value >= range.minimum && value <= range.maximum)) {
        this->log_WARNING_LO_ParamOutOfRange(id, value, range.minimum, range.maximum);
    }
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

bool FlightController ::isFlying() const {
    return (this->m_mode == FlightMode::TAKEOFF) || (this->m_mode == FlightMode::HOLD) ||
           (this->m_mode == FlightMode::GOTO) || (this->m_mode == FlightMode::LANDING);
}

bool FlightController ::isArmedOnGround() const {
    return (this->m_mode == FlightMode::ARMED_IDLE) || (this->m_mode == FlightMode::MOTOR_TEST);
}

ControlGains FlightController ::readGains() {
    // paramGet_* is thread safe (the generated code locks the parameter mutex),
    // so reading every step always uses the latest PRM_SET values.
    Fw::ParamValid valid;
    ControlGains g;
    g.posKp = clampTo(this->paramGet_POS_KP(valid), GAIN_RANGE);
    g.velKp = clampTo(this->paramGet_VEL_KP(valid), GAIN_RANGE);
    g.velZKp = clampTo(this->paramGet_VEL_Z_KP(valid), GAIN_RANGE);
    g.velZKi = clampTo(this->paramGet_VEL_Z_KI(valid), GAIN_RANGE);
    g.attKp = clampTo(this->paramGet_ATT_KP(valid), GAIN_RANGE);
    g.rateKp = clampTo(this->paramGet_RATE_KP(valid), GAIN_RANGE);
    g.yawKp = clampTo(this->paramGet_YAW_KP(valid), GAIN_RANGE);
    g.yawRateKp = clampTo(this->paramGet_YAW_RATE_KP(valid), GAIN_RANGE);
    g.maxSpeed = clampTo(this->paramGet_MAX_SPEED(valid), MAX_SPEED_RANGE);
    g.maxClimb = clampTo(this->paramGet_MAX_CLIMB(valid), MAX_CLIMB_RANGE);
    g.maxTiltRad = clampTo(this->paramGet_MAX_TILT_DEG(valid), MAX_TILT_RANGE) / FlightMath::RAD_TO_DEG;
    g.mass = clampTo(this->paramGet_VEHICLE_MASS(valid), MASS_RANGE);
    g.motorMaxThrust = clampTo(this->paramGet_MOTOR_MAX_THRUST(valid), THRUST_RANGE);
    g.gravity = clampTo(this->paramGet_GRAVITY(valid), GRAVITY_RANGE);
    return g;
}

void FlightController ::startClosedLoop(FlightMode mode) {
    this->m_control.reset();
    this->m_touchdownSteps = 0;
    this->m_mode = mode;
}

MotorThrottles FlightController ::flyClosedLoop(const ControlGains& gains) {
    const Vector3& p = this->m_sensors.get_position();
    const Vector3& v = this->m_sensors.get_velocity();
    const F32 distance = norm3(this->m_target.get_x() - p.get_x(), this->m_target.get_y() - p.get_y(),
                               this->m_target.get_z() - p.get_z());
    const F32 speed = norm3(v.get_x(), v.get_y(), v.get_z());

    Setpoint setpoint;
    setpoint.position = this->m_target;
    setpoint.yaw = this->m_targetYaw;
    setpoint.useClimbRate = false;
    setpoint.climbRate = 0.0f;

    switch (this->m_mode.e) {
        case FlightMode::TAKEOFF:
            if ((distance < ARRIVAL_DISTANCE) && (speed < ARRIVAL_SPEED)) {
                this->m_mode = FlightMode::HOLD;
                this->log_ACTIVITY_HI_TakeoffComplete(p.get_z());
            }
            break;
        case FlightMode::GOTO:
            if ((distance < ARRIVAL_DISTANCE) && (speed < ARRIVAL_SPEED)) {
                this->m_mode = FlightMode::HOLD;
                this->log_ACTIVITY_HI_Arrived(p.get_x(), p.get_y(), p.get_z());
            }
            break;
        case FlightMode::LANDING: {
            Fw::ParamValid valid;
            setpoint.useClimbRate = true;
            setpoint.climbRate = -clampTo(this->paramGet_LAND_SPEED(valid), LAND_SPEED_RANGE);
            const bool touching = (this->m_sensors.get_rangeToGround() < TOUCHDOWN_RANGE) &&
                                  (std::fabs(v.get_z()) < TOUCHDOWN_SPEED);
            this->m_touchdownSteps = touching ? (this->m_touchdownSteps + 1) : 0;
            if (this->m_touchdownSteps >= TOUCHDOWN_STEPS) {
                this->m_mode = FlightMode::DISARMED;
                this->log_ACTIVITY_HI_Landed();
                return MotorThrottles(0.0f);
            }
            break;
        }
        default:
            break;
    }
    return this->m_control.step(this->m_sensors, setpoint, gains, CONTROL_DT);
}

void FlightController ::writeTelemetry(const MotorThrottles& motors, const ControlGains& gains) {
    const SensorData& sensors = this->m_sensors;
    this->tlmWrite_Mode(this->m_mode);
    this->tlmWrite_Motors(motors);
    this->tlmWrite_Altitude(sensors.get_rangeToGround());
    const Vector3 euler = FlightMath::eulerFromQuaternion(sensors.get_attitude());
    this->tlmWrite_AttitudeDeg(FlightMath::scale(euler, FlightMath::RAD_TO_DEG));
    this->tlmWrite_Position(sensors.get_position());
    this->tlmWrite_Velocity(sensors.get_velocity());
    this->tlmWrite_BatteryVoltage(sensors.get_batteryVoltage());
    this->tlmWrite_Target(this->m_target);
    const Vector3& p = sensors.get_position();
    const F32 dx = this->m_target.get_x() - p.get_x();
    const F32 dy = this->m_target.get_y() - p.get_y();
    this->tlmWrite_DistanceToTarget(std::sqrt(dx * dx + dy * dy));
    this->tlmWrite_HoverThrottle(ControlLaw::hoverThrottle(gains));
}

}  // namespace MyFlightSim
