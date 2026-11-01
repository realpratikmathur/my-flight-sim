// ======================================================================
// \title  FlightControllerTester.cpp
// \author my-flight-sim
// \brief  cpp file for FlightController component test harness implementation class
// ======================================================================

#include "FlightControllerTester.hpp"

namespace MyFlightSim {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

FlightControllerTester ::FlightControllerTester()
    : FlightControllerGTestBase("FlightControllerTester", FlightControllerTester::MAX_HISTORY_SIZE),
      component("FlightController"),
      m_sensors() {
    this->initComponents();
    this->connectPorts();
}

FlightControllerTester ::~FlightControllerTester() {
    this->component.deinit();
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

SensorData FlightControllerTester ::from_sensorsGet_handler(FwIndexType portNum) {
    this->pushFromPortEntry_sensorsGet();
    return this->m_sensors;
}

void FlightControllerTester ::step() {
    this->invoke_to_run(0, 0);
}

MotorThrottles FlightControllerTester ::lastMotors() const {
    const FwSizeType n = this->fromPortHistory_motorsOut->size();
    EXPECT_GT(n, 0u);
    return this->fromPortHistory_motorsOut->at(n - 1).throttles;
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

void FlightControllerTester ::testDisarmedMotorsOff() {
    this->step();
    ASSERT_from_sensorsGet_SIZE(1);  // sensors are read every step
    ASSERT_from_motorsOut_SIZE(1);   // motors are commanded every step
    ASSERT_EQ(this->lastMotors(), MotorThrottles(0.0f));
}

void FlightControllerTester ::testCommandsWaitForRun() {
    this->sendCmd_ARM(0, 1);
    // The command is only on the queue: nothing has happened yet
    ASSERT_CMD_RESPONSE_SIZE(0);
    ASSERT_EVENTS_Armed_SIZE(0);

    // The next control step dispatches it
    this->step();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, FlightControllerComponentBase::OPCODE_ARM, 1, Fw::CmdResponse::OK);
    ASSERT_EVENTS_Armed_SIZE(1);
    // ...and the motors respond in that same step
    ASSERT_EQ(this->lastMotors(), MotorThrottles(FlightController::IDLE_THROTTLE));
}

void FlightControllerTester ::testArmDisarm() {
    this->sendCmd_ARM(0, 1);
    this->step();
    ASSERT_EQ(this->lastMotors(), MotorThrottles(FlightController::IDLE_THROTTLE));

    this->sendCmd_DISARM(0, 2);
    this->step();
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_DISARM, 2, Fw::CmdResponse::OK);
    ASSERT_EVENTS_Disarmed_SIZE(1);
    ASSERT_EQ(this->lastMotors(), MotorThrottles(0.0f));
}

void FlightControllerTester ::testMotorTest() {
    this->sendCmd_ARM(0, 1);
    this->sendCmd_MOTOR_TEST(0, 2, 0.6f);
    this->step();  // both queued commands are dispatched in this one step
    ASSERT_CMD_RESPONSE_SIZE(2);
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_MOTOR_TEST, 2, Fw::CmdResponse::OK);
    ASSERT_EVENTS_MotorTestStarted_SIZE(1);
    ASSERT_EVENTS_MotorTestStarted(0, 0.6f);
    ASSERT_EQ(this->lastMotors(), MotorThrottles(0.6f));
}

void FlightControllerTester ::testMotorTestRequiresArm() {
    this->sendCmd_MOTOR_TEST(0, 1, 0.6f);
    this->step();
    ASSERT_CMD_RESPONSE(0, FlightControllerComponentBase::OPCODE_MOTOR_TEST, 1, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_EVENTS_NotArmed_SIZE(1);
    ASSERT_EQ(this->lastMotors(), MotorThrottles(0.0f));
}

void FlightControllerTester ::testMotorTestValidatesLevel() {
    this->sendCmd_ARM(0, 1);
    this->sendCmd_MOTOR_TEST(0, 2, 1.5f);
    this->sendCmd_MOTOR_TEST(0, 3, -0.1f);
    this->step();
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_MOTOR_TEST, 2, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_CMD_RESPONSE(2, FlightControllerComponentBase::OPCODE_MOTOR_TEST, 3, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_InvalidThrottle_SIZE(2);
    // Still armed and idle: a rejected command changes nothing
    ASSERT_EQ(this->lastMotors(), MotorThrottles(FlightController::IDLE_THROTTLE));
}

void FlightControllerTester ::testTelemetryDecimated() {
    for (U32 i = 0; i < 2 * FlightController::TELEMETRY_DECIMATION; i++) {
        this->step();
    }
    ASSERT_TLM_Mode_SIZE(2);
    ASSERT_TLM_Mode(0, FlightMode::DISARMED);
}

}  // namespace MyFlightSim
