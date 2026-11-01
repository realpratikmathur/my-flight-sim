// ======================================================================
// \title  FlightControllerTester.cpp
// \author my-flight-sim
// \brief  cpp file for FlightController component test harness implementation class
// ======================================================================

#include "FlightControllerTester.hpp"

#include <cmath>

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
    // Like readParameters()/loadParameters() at startup in the real topology:
    // without this, parameters are uninitialized and paramGet_* returns 0.
    // The tester has no saved values, so the FPP "default" values are used.
    this->component.loadParameters();
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

void FlightControllerTester ::setSensors(F32 x, F32 y, F32 z, F32 vx, F32 vy, F32 vz) {
    this->m_sensors.set_position(Vector3(x, y, z));
    this->m_sensors.set_velocity(Vector3(vx, vy, vz));
    this->m_sensors.set_attitude(Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    this->m_sensors.set_gyro(Vector3(0.0f, 0.0f, 0.0f));
    this->m_sensors.set_rangeToGround(z);
}

void FlightControllerTester ::flyToHold() {
    this->setSensors(0.0f, 0.0f, 0.0f);
    this->sendCmd_ARM(0, 1);
    this->sendCmd_TAKEOFF(0, 2, 5.0f);
    this->step();
    this->setSensors(0.0f, 0.0f, 5.0f);  // pretend we climbed
    this->step();
    ASSERT_EVENTS_TakeoffComplete_SIZE(1);
    this->clearHistory();
}

namespace {
//! Expected hover throttle for the default vehicle parameters
F32 hover(F32 gravity) {
    return std::sqrt(1.2f * gravity / (4.0f * 7.0f));
}
}  // namespace

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

void FlightControllerTester ::testTakeoffRequiresArm() {
    this->sendCmd_TAKEOFF(0, 1, 5.0f);
    this->step();
    ASSERT_CMD_RESPONSE(0, FlightControllerComponentBase::OPCODE_TAKEOFF, 1, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_EVENTS_NotArmed_SIZE(1);
}

void FlightControllerTester ::testTakeoffValidatesAltitude() {
    this->sendCmd_ARM(0, 1);
    this->sendCmd_TAKEOFF(0, 2, 0.5f);
    this->sendCmd_TAKEOFF(0, 3, 60.0f);
    this->step();
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_TAKEOFF, 2, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_CMD_RESPONSE(2, FlightControllerComponentBase::OPCODE_TAKEOFF, 3, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_InvalidAltitude_SIZE(2);
}

void FlightControllerTester ::testTakeoffClimbsThenHolds() {
    this->setSensors(0.0f, 0.0f, 0.0f);
    this->sendCmd_ARM(0, 1);
    this->sendCmd_TAKEOFF(0, 2, 5.0f);
    this->step();
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_TAKEOFF, 2, Fw::CmdResponse::OK);
    ASSERT_EVENTS_TakeoffStarted(0, 5.0f);
    ASSERT_TLM_Mode(0, FlightMode::TAKEOFF);
    // On the ground below the target: all motors above hover throttle (climb)
    const MotorThrottles climbing = this->lastMotors();
    for (FwSizeType i = 0; i < MotorThrottles::SIZE; i++) {
        ASSERT_GT(climbing[i], hover(9.81f));
    }

    // At the target and still: TAKEOFF -> HOLD
    this->setSensors(0.0f, 0.0f, 5.0f);
    this->step();
    ASSERT_EVENTS_TakeoffComplete_SIZE(1);
}

void FlightControllerTester ::testHoverThrottleFollowsGravityParam() {
    this->flyToHold();
    this->step();
    for (FwSizeType i = 0; i < MotorThrottles::SIZE; i++) {
        ASSERT_NEAR(this->lastMotors()[i], hover(9.81f), 0.01f);
    }

    // Change the GRAVITY parameter the same way the ground would (PRM_SET)
    this->paramSet_GRAVITY(3.71f, Fw::ParamValid::VALID);
    this->paramSend_GRAVITY(0, 10);
    ASSERT_CMD_RESPONSE(0, FlightControllerComponentBase::OPCODE_GRAVITY_SET, 10, Fw::CmdResponse::OK);
    this->step();
    for (FwSizeType i = 0; i < MotorThrottles::SIZE; i++) {
        ASSERT_NEAR(this->lastMotors()[i], hover(3.71f), 0.01f);
    }
}

void FlightControllerTester ::testCommandsCheckFlyingState() {
    // On the ground (armed, idle): GOTO and LAND make no sense
    this->sendCmd_ARM(0, 1);
    this->sendCmd_GOTO(0, 2, 10.0f, 5.0f, 5.0f);
    this->sendCmd_LAND(0, 3);
    this->step();
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_GOTO, 2, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_CMD_RESPONSE(2, FlightControllerComponentBase::OPCODE_LAND, 3, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_EVENTS_NotFlying_SIZE(2);
    ASSERT_EVENTS_NotFlying(0, FlightMode::ARMED_IDLE);

    // In the air: MOTOR_TEST and TAKEOFF are not allowed
    this->flyToHold();
    this->sendCmd_MOTOR_TEST(0, 4, 0.5f);
    this->sendCmd_TAKEOFF(0, 5, 10.0f);
    this->step();
    ASSERT_CMD_RESPONSE(0, FlightControllerComponentBase::OPCODE_MOTOR_TEST, 4, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_CMD_RESPONSE(1, FlightControllerComponentBase::OPCODE_TAKEOFF, 5, Fw::CmdResponse::EXECUTION_ERROR);
    ASSERT_EVENTS_NotOnGround_SIZE(2);
    ASSERT_EVENTS_NotOnGround(0, FlightMode::HOLD);
}

void FlightControllerTester ::testGotoGeofence() {
    this->flyToHold();
    this->sendCmd_GOTO(0, 1, 200.0f, 0.0f, 5.0f);
    this->step();
    ASSERT_CMD_RESPONSE(0, FlightControllerComponentBase::OPCODE_GOTO, 1, Fw::CmdResponse::VALIDATION_ERROR);
    ASSERT_EVENTS_OutsideGeofence(0, 200.0f, 100.0f);

    // Raise the geofence with a parameter, and the same GOTO is accepted
    this->paramSet_GEOFENCE_RADIUS(500.0f, Fw::ParamValid::VALID);
    this->paramSend_GEOFENCE_RADIUS(0, 2);
    this->sendCmd_GOTO(0, 3, 200.0f, 0.0f, 5.0f);
    this->step();
    ASSERT_CMD_RESPONSE(2, FlightControllerComponentBase::OPCODE_GOTO, 3, Fw::CmdResponse::OK);
}

void FlightControllerTester ::testGotoArrives() {
    this->flyToHold();
    this->sendCmd_GOTO(0, 1, 10.0f, 0.0f, 5.0f);
    this->step();
    ASSERT_EVENTS_GotoStarted(0, 10.0f, 0.0f, 5.0f);
    // Target is east (+x) and the vehicle faces east: pitch nose-down to accelerate
    // forward, i.e. the back motors (2, 3) push harder than the front motors (0, 1)
    const MotorThrottles m = this->lastMotors();
    ASSERT_GT(m[2], m[1]);
    ASSERT_GT(m[3], m[0]);

    this->setSensors(10.0f, 0.0f, 5.0f);
    this->step();
    ASSERT_EVENTS_Arrived_SIZE(1);
}

void FlightControllerTester ::testLandTouchdownDisarms() {
    this->flyToHold();
    this->sendCmd_LAND(0, 1);
    this->step();
    ASSERT_EVENTS_LandingStarted(0, 0.6f);

    // Sitting on the ground: disarm only after TOUCHDOWN_STEPS steps
    this->setSensors(0.0f, 0.0f, 0.0f);
    for (U32 i = 0; i < FlightController::TOUCHDOWN_STEPS - 1; i++) {
        this->step();
    }
    ASSERT_EVENTS_Landed_SIZE(0);
    this->step();
    ASSERT_EVENTS_Landed_SIZE(1);
    ASSERT_EQ(this->lastMotors(), MotorThrottles(0.0f));
    this->step();
    ASSERT_EQ(this->lastMotors(), MotorThrottles(0.0f));  // stays disarmed
}

void FlightControllerTester ::testParamOutOfRangeReported() {
    this->paramSet_MAX_TILT_DEG(80.0f, Fw::ParamValid::VALID);
    this->paramSend_MAX_TILT_DEG(0, 1);
    ASSERT_EVENTS_ParamOutOfRange_SIZE(1);
    ASSERT_EVENTS_ParamOutOfRange(0, FlightControllerComponentBase::PARAMID_MAX_TILT_DEG, 80.0f, 5.0f, 45.0f);

    // A value inside the range is not reported
    this->paramSet_MAX_TILT_DEG(30.0f, Fw::ParamValid::VALID);
    this->paramSend_MAX_TILT_DEG(0, 2);
    ASSERT_EVENTS_ParamOutOfRange_SIZE(1);
}

}  // namespace MyFlightSim
