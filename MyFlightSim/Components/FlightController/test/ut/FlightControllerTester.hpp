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
    static const FwSizeType MAX_HISTORY_SIZE = 100;

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
