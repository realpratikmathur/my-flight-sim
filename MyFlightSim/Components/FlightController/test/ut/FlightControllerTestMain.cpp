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

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
