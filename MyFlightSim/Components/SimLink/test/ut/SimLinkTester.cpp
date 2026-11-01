// ======================================================================
// \title  SimLinkTester.cpp
// \author my-flight-sim
// \brief  cpp file for SimLink component test harness implementation class
// ======================================================================

#include "SimLinkTester.hpp"
#include "MyFlightSim/Types/FppConstantsAc.hpp"
#include "MyFlightSim/Types/SimHeaderSerializableAc.hpp"

namespace MyFlightSim {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

SimLinkTester ::SimLinkTester()
    : SimLinkGTestBase("SimLinkTester", SimLinkTester::MAX_HISTORY_SIZE), component("SimLink"), m_data() {
    this->initComponents();
    this->connectPorts();
}

SimLinkTester ::~SimLinkTester() {
    this->component.deinit();
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

Fw::Buffer SimLinkTester ::makeFrame(U32 magic, U32 tick, U64 simTimeUs) {
    Fw::Buffer buffer(this->m_data, sizeof(this->m_data));
    auto serializer = buffer.getSerializer();
    const SimHeader header(magic, tick, simTimeUs);
    EXPECT_EQ(header.serializeTo(serializer), Fw::FW_SERIALIZE_OK);
    buffer.setSize(serializer.getSize());
    return buffer;
}

void SimLinkTester ::receive(Fw::Buffer& buffer, Drv::ByteStreamStatus status) {
    this->invoke_to_drvReceiveIn(0, buffer, status);
}

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

void SimLinkTester ::testValidFramesDriveClock() {
    for (U32 tick = 1; tick <= 3; tick++) {
        Fw::Buffer frame = this->makeFrame(SIM_MAGIC, tick, tick * 5000);
        this->receive(frame);
    }
    // One clock tick per frame
    ASSERT_from_cycleOut_SIZE(3);
    // Every buffer was handed back to the driver
    ASSERT_from_drvReceiveReturnOut_SIZE(3);
    // "Connected" is announced only once
    ASSERT_EVENTS_SimConnected_SIZE(1);
    ASSERT_EVENTS_SimConnected(0, 1);
    // No warnings
    ASSERT_EVENTS_TicksMissed_SIZE(0);
    ASSERT_EVENTS_BadFrame_SIZE(0);
    // Telemetry tracks the latest tick and simulated time
    ASSERT_TLM_Tick_SIZE(3);
    ASSERT_TLM_Tick(2, 3);
    ASSERT_TLM_SimTime(2, 0.015);
}

void SimLinkTester ::testBadMagicRejected() {
    Fw::Buffer frame = this->makeFrame(0xDEADBEEF, 1, 5000);
    this->receive(frame);
    ASSERT_from_cycleOut_SIZE(0);
    ASSERT_from_drvReceiveReturnOut_SIZE(1);
    ASSERT_EVENTS_BadFrame_SIZE(1);
    ASSERT_EVENTS_BadFrame(0, SimHeader::SERIALIZED_SIZE, 0xDEADBEEF);
    ASSERT_TLM_BadFramesTotal(0, 1);
    ASSERT_EVENTS_SimConnected_SIZE(0);
}

void SimLinkTester ::testShortDatagramRejected() {
    Fw::Buffer frame = this->makeFrame(SIM_MAGIC, 1, 5000);
    frame.setSize(4);  // only the magic number arrived
    this->receive(frame);
    ASSERT_from_cycleOut_SIZE(0);
    ASSERT_from_drvReceiveReturnOut_SIZE(1);
    ASSERT_EVENTS_BadFrame_SIZE(1);
    ASSERT_EVENTS_BadFrame(0, 4, 0);
}

void SimLinkTester ::testMissedTicksReported() {
    const U32 ticks[] = {1, 2, 5};
    for (const U32 tick : ticks) {
        Fw::Buffer frame = this->makeFrame(SIM_MAGIC, tick, tick * 5000);
        this->receive(frame);
    }
    // The clock still ticked for every frame that did arrive
    ASSERT_from_cycleOut_SIZE(3);
    // Ticks 3 and 4 never arrived
    ASSERT_EVENTS_TicksMissed_SIZE(1);
    ASSERT_EVENTS_TicksMissed(0, 3, 5);
    ASSERT_TLM_TicksMissedTotal(0, 2);
}

void SimLinkTester ::testDriverErrorIgnored() {
    Fw::Buffer frame = this->makeFrame(SIM_MAGIC, 1, 5000);
    this->receive(frame, Drv::ByteStreamStatus::OTHER_ERROR);
    ASSERT_from_cycleOut_SIZE(0);
    ASSERT_EVENTS_SIZE(0);
    // Even a failed receive must give the buffer back
    ASSERT_from_drvReceiveReturnOut_SIZE(1);
    ASSERT_from_drvReceiveReturnOut(0, frame);
}

void SimLinkTester ::testSimulatorRestart() {
    const U32 ticks[] = {10, 11, 0, 1};
    for (const U32 tick : ticks) {
        Fw::Buffer frame = this->makeFrame(SIM_MAGIC, tick, tick * 5000);
        this->receive(frame);
    }
    ASSERT_from_cycleOut_SIZE(4);
    // Connected at tick 10, and again when the simulator restarted at tick 0
    ASSERT_EVENTS_SimConnected_SIZE(2);
    ASSERT_EVENTS_SimConnected(0, 10);
    ASSERT_EVENTS_SimConnected(1, 0);
    ASSERT_EVENTS_TicksMissed_SIZE(0);
}

}  // namespace MyFlightSim
