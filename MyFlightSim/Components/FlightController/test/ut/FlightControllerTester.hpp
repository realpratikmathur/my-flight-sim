// ======================================================================
// \title  FlightControllerTester.hpp
// \author my-flight-sim
// \brief  hpp file for FlightController component test harness implementation class
// ======================================================================

#ifndef MyFlightSim_FlightControllerTester_HPP
#define MyFlightSim_FlightControllerTester_HPP

#include "MyFlightSim/Components/FlightController/FlightController.hpp"
#include "MyFlightSim/Components/FlightController/FlightControllerGTestBase.hpp"

namespace MyFlightSim {

class FlightControllerTester final : public FlightControllerGTestBase {
  public:
    // ----------------------------------------------------------------------
    // Constants
    // ----------------------------------------------------------------------

    // Maximum size of histories storing events, telemetry, and port outputs
    static const FwSizeType MAX_HISTORY_SIZE = 1000;  // LandTouchdownDisarms runs > 100 steps

    // Instance ID supplied to the component instance under test
    static const FwEnumStoreType TEST_INSTANCE_ID = 0;

    // Queue depth supplied to the component instance under test
    static const FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 10;

  public:
    // ----------------------------------------------------------------------
    // Construction and destruction
    // ----------------------------------------------------------------------

    //! Construct object FlightControllerTester
    FlightControllerTester();

    //! Destroy object FlightControllerTester
    ~FlightControllerTester();

  public:
    // ----------------------------------------------------------------------
    // Tests
    // ----------------------------------------------------------------------

    //! Disarmed: motors stay at zero
    void testDisarmedMotorsOff();

    //! Commands are queued and only take effect on the next control step
    void testCommandsWaitForRun();

    //! ARM spins the motors at idle; DISARM stops them
    void testArmDisarm();

    //! MOTOR_TEST drives all motors at the requested level when armed
    void testMotorTest();

    //! MOTOR_TEST is rejected while disarmed
    void testMotorTestRequiresArm();

    //! MOTOR_TEST rejects levels outside 0..1
    void testMotorTestValidatesLevel();

    //! Telemetry is written once every TELEMETRY_DECIMATION steps
    void testTelemetryDecimated();

    // Feature 04: closed-loop flight

    //! TAKEOFF is rejected while disarmed
    void testTakeoffRequiresArm();

    //! TAKEOFF rejects altitudes outside 1..50 m
    void testTakeoffValidatesAltitude();

    //! TAKEOFF commands more than hover thrust, then switches to HOLD at the target
    void testTakeoffClimbsThenHolds();

    //! Holding still at the target needs exactly hover throttle, which follows the GRAVITY parameter
    void testHoverThrottleFollowsGravityParam();

    //! GOTO and LAND are rejected on the ground; MOTOR_TEST and TAKEOFF are rejected in the air
    void testCommandsCheckFlyingState();

    //! GOTO rejects targets outside GEOFENCE_RADIUS
    void testGotoGeofence();

    //! GOTO tilts the vehicle towards the target and switches to HOLD on arrival
    void testGotoArrives();

    //! LAND disarms after TOUCHDOWN_STEPS steps on the ground
    void testLandTouchdownDisarms();

    //! Setting a parameter outside its range is reported
    void testParamOutOfRangeReported();

  private:
    // ----------------------------------------------------------------------
    // Helper functions
    // ----------------------------------------------------------------------

    //! Connect ports
    void connectPorts();

    //! Initialize components
    void initComponents();

    //! Fake sensor driver: returns m_sensors
    SensorData from_sensorsGet_handler(FwIndexType portNum) override;

    //! Run one control step (as the 200 Hz rate group would)
    void step();

    //! Throttles sent to the motors on the most recent step
    MotorThrottles lastMotors() const;

    //! Make the fake sensors report a level vehicle at a position and velocity
    void setSensors(F32 x, F32 y, F32 z, F32 vx = 0.0f, F32 vy = 0.0f, F32 vz = 0.0f);

    //! Arm, take off to 5 m, and arrive there (mode HOLD), then clear histories
    void flyToHold();

  private:
    // ----------------------------------------------------------------------
    // Member variables
    // ----------------------------------------------------------------------

    //! The component under test
    FlightController component;

    //! Sensor data returned to the component
    SensorData m_sensors;
};

}  // namespace MyFlightSim

#endif
