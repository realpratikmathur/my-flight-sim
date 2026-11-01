// ======================================================================
// \title  SimLinkTestMain.cpp
// \author my-flight-sim
// \brief  cpp file for SimLink component test main function
// ======================================================================

#include "SimLinkTester.hpp"

TEST(Nominal, ValidFramesDriveClock) {
    MyFlightSim::SimLinkTester tester;
    tester.testValidFramesDriveClock();
}

TEST(Nominal, MissedTicksReported) {
    MyFlightSim::SimLinkTester tester;
    tester.testMissedTicksReported();
}

TEST(Nominal, SimulatorRestart) {
    MyFlightSim::SimLinkTester tester;
    tester.testSimulatorRestart();
}

TEST(OffNominal, BadMagicRejected) {
    MyFlightSim::SimLinkTester tester;
    tester.testBadMagicRejected();
}

TEST(OffNominal, ShortDatagramRejected) {
    MyFlightSim::SimLinkTester tester;
    tester.testShortDatagramRejected();
}

TEST(OffNominal, DriverErrorIgnored) {
    MyFlightSim::SimLinkTester tester;
    tester.testDriverErrorIgnored();
}

TEST(Nominal, SensorsReturned) {
    MyFlightSim::SimLinkTester tester;
    tester.testSensorsReturned();
}

TEST(Nominal, ActuatorFrameSent) {
    MyFlightSim::SimLinkTester tester;
    tester.testActuatorFrameSent();
}

TEST(OffNominal, NoSendBeforeConnected) {
    MyFlightSim::SimLinkTester tester;
    tester.testNoSendBeforeConnected();
}

TEST(OffNominal, SendFailureReported) {
    MyFlightSim::SimLinkTester tester;
    tester.testSendFailureReported();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
