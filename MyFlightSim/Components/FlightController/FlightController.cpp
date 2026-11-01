// ======================================================================
// \title  FlightController.cpp
// \author my-flight-sim
// \brief  cpp file for FlightController component implementation class
// ======================================================================

#include "MyFlightSim/Components/FlightController/FlightController.hpp"
#include "MyFlightSim/Components/FlightController/FlightMath.hpp"

namespace MyFlightSim {

// Out-of-class definitions of static constexpr members (required in C++14,
// the language standard F Prime uses, when they are passed by reference)
constexpr F32 FlightController::IDLE_THROTTLE;
constexpr U32 FlightController::TELEMETRY_DECIMATION;

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

FlightController ::FlightController(const char* const compName)
    : FlightControllerComponentBase(compName), m_mode(FlightMode::DISARMED), m_testLevel(0.0f), m_cycle(0) {}

FlightController ::~FlightController() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void FlightController ::run_handler(FwIndexType portNum, U32 context) {
    // 1. Pull queued commands into this cycle. This is what makes the
    //    component "queued": the async command handlers below run here, on
    //    the rate group thread, at a well-defined point in the control step.
    (void)this->dispatchAvailableMessages();

    // 2. Read the sensors (a synchronous port call that returns a value)
    const SensorData sensors = this->sensorsGet_out(0);

    // 3. Decide the motor throttles for the current mode
    MotorThrottles motors(0.0f);
    switch (this->m_mode.e) {
        case FlightMode::ARMED_IDLE:
            motors = MotorThrottles(IDLE_THROTTLE);
            break;
        case FlightMode::MOTOR_TEST:
            motors = MotorThrottles(this->m_testLevel);
            break;
        case FlightMode::DISARMED:
        default:
            break;
    }

    // 4. Command the motors
    this->motorsOut_out(0, motors);

    // 5. Telemetry at 10 Hz
    if ((this->m_cycle % TELEMETRY_DECIMATION) == 0) {
        this->writeTelemetry(sensors, motors);
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
    this->m_mode = FlightMode::MOTOR_TEST;
    this->m_testLevel = level;
    this->log_ACTIVITY_HI_MotorTestStarted(level);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

void FlightController ::writeTelemetry(const SensorData& sensors, const MotorThrottles& motors) {
    this->tlmWrite_Mode(this->m_mode);
    this->tlmWrite_Motors(motors);
    this->tlmWrite_Altitude(sensors.get_rangeToGround());
    const Vector3 euler = FlightMath::eulerFromQuaternion(sensors.get_attitude());
    this->tlmWrite_AttitudeDeg(FlightMath::scale(euler, FlightMath::RAD_TO_DEG));
    this->tlmWrite_Position(sensors.get_position());
    this->tlmWrite_Velocity(sensors.get_velocity());
    this->tlmWrite_BatteryVoltage(sensors.get_batteryVoltage());
}

}  // namespace MyFlightSim
