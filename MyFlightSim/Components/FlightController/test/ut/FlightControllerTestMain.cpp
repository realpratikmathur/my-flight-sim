// ======================================================================
// \title  FlightControllerTestMain.cpp
// \author my-flight-sim
// \brief  cpp file for FlightController component test main function
// ======================================================================

#include "FlightControllerTester.hpp"

TEST(Nominal, DisarmedMotorsOff) {
    MyFlightSim::FlightControllerTester tester;
    tester.testDisarmedMotorsOff();
}

TEST(Nominal, CommandsWaitForRun) {
    MyFlightSim::FlightControllerTester tester;
    tester.testCommandsWaitForRun();
}

TEST(Nominal, ArmDisarm) {
    MyFlightSim::FlightControllerTester tester;
    tester.testArmDisarm();
}

TEST(Nominal, MotorTest) {
    MyFlightSim::FlightControllerTester tester;
    tester.testMotorTest();
}

TEST(Nominal, TelemetryDecimated) {
    MyFlightSim::FlightControllerTester tester;
    tester.testTelemetryDecimated();
}

TEST(OffNominal, MotorTestRequiresArm) {
    MyFlightSim::FlightControllerTester tester;
    tester.testMotorTestRequiresArm();
}

TEST(OffNominal, MotorTestValidatesLevel) {
    MyFlightSim::FlightControllerTester tester;
    tester.testMotorTestValidatesLevel();
}

TEST(Flight, TakeoffClimbsThenHolds) {
    MyFlightSim::FlightControllerTester tester;
    tester.testTakeoffClimbsThenHolds();
}

TEST(Flight, HoverThrottleFollowsGravityParam) {
    MyFlightSim::FlightControllerTester tester;
    tester.testHoverThrottleFollowsGravityParam();
}

TEST(Flight, GotoArrives) {
    MyFlightSim::FlightControllerTester tester;
    tester.testGotoArrives();
}

TEST(Flight, LandTouchdownDisarms) {
    MyFlightSim::FlightControllerTester tester;
    tester.testLandTouchdownDisarms();
}

TEST(FlightOffNominal, TakeoffRequiresArm) {
    MyFlightSim::FlightControllerTester tester;
    tester.testTakeoffRequiresArm();
}

TEST(FlightOffNominal, TakeoffValidatesAltitude) {
    MyFlightSim::FlightControllerTester tester;
    tester.testTakeoffValidatesAltitude();
}

TEST(FlightOffNominal, CommandsCheckFlyingState) {
    MyFlightSim::FlightControllerTester tester;
    tester.testCommandsCheckFlyingState();
}

TEST(FlightOffNominal, GotoGeofence) {
    MyFlightSim::FlightControllerTester tester;
    tester.testGotoGeofence();
}

TEST(FlightOffNominal, ParamOutOfRangeReported) {
    MyFlightSim::FlightControllerTester tester;
    tester.testParamOutOfRangeReported();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
