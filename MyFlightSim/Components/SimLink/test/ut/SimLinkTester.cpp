// ======================================================================
// \title  SimLinkTester.cpp
// \author my-flight-sim
// \brief  cpp file for SimLink component test harness implementation class
// ======================================================================

#include "SimLinkTester.hpp"
#include "MyFlightSim/Types/FppConstantsAc.hpp"
#include "MyFlightSim/Types/ActuatorFrameSerializableAc.hpp"
#include "MyFlightSim/Types/SensorFrameSerializableAc.hpp"
#include "MyFlightSim/Types/SimHeaderSerializableAc.hpp"

namespace MyFlightSim {

// ----------------------------------------------------------------------
// Construction and destruction
// ----------------------------------------------------------------------

SimLinkTester ::SimLinkTester()
    : SimLinkGTestBase("SimLinkTester", SimLinkTester::MAX_HISTORY_SIZE), component("SimLink"), m_data(), m_sendStatus(Drv::ByteStreamStatus::OP_OK) {
    this->initComponents();
    this->connectPorts();
}

SimLinkTester ::~SimLinkTester() {
    this->component.deinit();
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

Fw::Buffer SimLinkTester ::makeFrame(U32 magic, U32 tick, U64 simTimeUs, const SensorData& sensors) {
    Fw::Buffer buffer(this->m_data, sizeof(this->m_data));
    auto serializer = buffer.getSerializer();
    const SensorFrame frame(SimHeader(magic, tick, simTimeUs), sensors);
    EXPECT_EQ(frame.serializeTo(serializer), Fw::FW_SERIALIZE_OK);
    buffer.setSize(serializer.getSize());
    return buffer;
}

Drv::ByteStreamStatus SimLinkTester ::from_drvSendOut_handler(FwIndexType portNum, Fw::Buffer& sendBuffer) {
    this->pushFromPortEntry_drvSendOut(sendBuffer);
    return this->m_sendStatus;
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
    ASSERT_EVENTS_BadFrame(0, SensorFrame::SERIALIZED_SIZE, 0xDEADBEEF);
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

void SimLinkTester ::testSensorsReturned() {
    SensorData sensors;
    sensors.set_rangeToGround(2.5f);
    sensors.set_position(Vector3(1.0f, 2.0f, 2.5f));
    sensors.set_batteryVoltage(16.1f);
    Fw::Buffer frame = this->makeFrame(SIM_MAGIC, 1, 5000, sensors);
    this->receive(frame);

    const SensorData got = this->invoke_to_sensorsGet(0);
    ASSERT_EQ(got, sensors);
}

void SimLinkTester ::testActuatorFrameSent() {
    Fw::Buffer frame = this->makeFrame(SIM_MAGIC, 7, 35000);
    this->receive(frame);
    const MotorThrottles throttles({0.1f, 0.2f, 0.3f, 0.4f});
    this->invoke_to_motorsIn(0, throttles);
    this->invoke_to_actuatorSchedIn(0, 0);

    // Exactly one datagram went to the driver...
    ASSERT_from_drvSendOut_SIZE(1);
    Fw::Buffer sent = this->fromPortHistory_drvSendOut->at(0).sendBuffer;
    ASSERT_EQ(sent.getSize(), ActuatorFrame::SERIALIZED_SIZE);

    // ...and it decodes to an ActuatorFrame answering tick 7 with our throttles
    ActuatorFrame decoded;
    auto deserializer = sent.getDeserializer();
    ASSERT_EQ(decoded.deserializeFrom(deserializer), Fw::FW_SERIALIZE_OK);
    ASSERT_EQ(decoded.get_header().get_magic(), SIM_MAGIC);
    ASSERT_EQ(decoded.get_header().get_tick(), 7u);
    ASSERT_EQ(decoded.get_header().get_simTimeUs(), 35000u);
    ASSERT_EQ(decoded.get_motors(), throttles);
}

void SimLinkTester ::testNoSendBeforeConnected() {
    this->invoke_to_actuatorSchedIn(0, 0);
    ASSERT_from_drvSendOut_SIZE(0);
}

void SimLinkTester ::testSendFailureReported() {
    Fw::Buffer frame = this->makeFrame(SIM_MAGIC, 1, 5000);
    this->receive(frame);
    this->m_sendStatus = Drv::ByteStreamStatus::OTHER_ERROR;
    this->invoke_to_actuatorSchedIn(0, 0);
    ASSERT_EVENTS_SendFailed_SIZE(1);
    ASSERT_EVENTS_SendFailed(0, Drv::ByteStreamStatus::OTHER_ERROR);
    ASSERT_TLM_SendErrorsTotal(0, 1);
}

}  // namespace MyFlightSim
