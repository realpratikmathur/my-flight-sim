// ======================================================================
// \title  SimLink.cpp
// \author my-flight-sim
// \brief  cpp file for SimLink component implementation class
// ======================================================================

#include "MyFlightSim/Components/SimLink/SimLink.hpp"
#include "MyFlightSim/Types/FppConstantsAc.hpp"
#include "MyFlightSim/Types/SensorFrameSerializableAc.hpp"
#include "MyFlightSim/Types/SimHeaderSerializableAc.hpp"

namespace MyFlightSim {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

SimLink ::SimLink(const char* const compName)
    : SimLinkComponentBase(compName),
      m_connected(false),
      m_lastTick(0),
      m_lastSimTimeUs(0),
      m_ticksMissed(0),
      m_badFrames(0),
      m_sendErrors(0),
      m_sensors(),
      m_motors(0.0f),
      m_sendData(),
      m_windowStarted(false),
      m_windowStartSimUs(0),
      m_windowFrames(0) {}

SimLink ::~SimLink() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void SimLink ::drvConnected_handler(FwIndexType portNum) {
    // UDP is connectionless: "ready" only means the socket is open.
    // We consider the simulator connected when the first valid frame arrives.
}

void SimLink ::drvReceiveIn_handler(FwIndexType portNum,
                                    Fw::Buffer& buffer,
                                    const Drv::ByteStreamStatus& status) {
    if (status == Drv::ByteStreamStatus::OP_OK) {
        this->processDatagram(buffer);
    }
    // The driver lent us this buffer. Whatever happened, give it back so the
    // buffer manager can reuse it (no memory is allocated while flying).
    this->drvReceiveReturnOut_out(0, buffer);
}

void SimLink ::schedIn_handler(FwIndexType portNum, U32 context) {
    Os::RawTime now;
    (void)now.now();

    if (this->m_windowStarted) {
        U32 elapsedUs = 0;
        if ((now.getDiffUsec(this->m_windowStart, elapsedUs) == Os::RawTime::OP_OK) && (elapsedUs > 0)) {
            const F32 elapsedS = static_cast<F32>(elapsedUs) / 1.0e6f;
            const F32 simElapsedS = static_cast<F32>(this->m_lastSimTimeUs - this->m_windowStartSimUs) / 1.0e6f;
            this->tlmWrite_TickRate(static_cast<F32>(this->m_windowFrames) / elapsedS);
            this->tlmWrite_RealTimeFactor(simElapsedS / elapsedS);
        }
    }

    // Start a new statistics window
    this->m_windowStarted = true;
    this->m_windowStart = now;
    this->m_windowStartSimUs = this->m_lastSimTimeUs;
    this->m_windowFrames = 0;
}

SensorData SimLink ::sensorsGet_handler(FwIndexType portNum) {
    return this->m_sensors;
}

void SimLink ::motorsIn_handler(FwIndexType portNum, const MotorThrottles& throttles) {
    this->m_motors = throttles;
}

void SimLink ::actuatorSchedIn_handler(FwIndexType portNum, U32 context) {
    if (!this->m_connected) {
        return;  // No simulator yet: nobody to answer
    }

    // Answer the most recent tick with the most recent motor throttles
    const ActuatorFrame frame(SimHeader(SIM_MAGIC, this->m_lastTick, this->m_lastSimTimeUs), this->m_motors);

    // Serialize into our own fixed memory and send it. Drv.Udp's send port is
    // synchronous and the caller keeps ownership of the buffer.
    Fw::Buffer buffer(this->m_sendData, sizeof(this->m_sendData));
    auto serializer = buffer.getSerializer();
    const Fw::SerializeStatus serStatus = frame.serializeTo(serializer);
    FW_ASSERT(serStatus == Fw::FW_SERIALIZE_OK, static_cast<FwAssertArgType>(serStatus));
    buffer.setSize(serializer.getSize());

    const Drv::ByteStreamStatus sendStatus = this->drvSendOut_out(0, buffer);
    if (sendStatus != Drv::ByteStreamStatus::OP_OK) {
        this->m_sendErrors++;
        this->tlmWrite_SendErrorsTotal(this->m_sendErrors);
        this->log_WARNING_LO_SendFailed(sendStatus);
    }
}

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

void SimLink ::processDatagram(Fw::Buffer& buffer) {
    // 1. Decode the frame using the code FPP generated for struct SensorFrame.
    //    The deserializer reads big-endian fields in declaration order.
    SensorFrame frame;
    auto deserializer = buffer.getDeserializer();
    const Fw::SerializeStatus status = frame.deserializeFrom(deserializer);
    const SimHeader& header = frame.get_header();

    if ((status != Fw::FW_SERIALIZE_OK) || (header.get_magic() != SIM_MAGIC)) {
        this->m_badFrames++;
        this->tlmWrite_BadFramesTotal(this->m_badFrames);
        const U32 magic = (status == Fw::FW_SERIALIZE_OK) ? header.get_magic() : 0;
        this->log_WARNING_LO_BadFrame(buffer.getSize(), magic);
        return;
    }

    // 2. Track tick continuity
    const U32 tick = header.get_tick();
    if (!this->m_connected || (tick <= this->m_lastTick)) {
        // First frame ever, or the simulator restarted (tick went backwards)
        this->m_connected = true;
        this->log_ACTIVITY_HI_SimConnected(tick);
    } else if (tick != this->m_lastTick + 1) {
        this->m_ticksMissed += (tick - this->m_lastTick - 1);
        this->tlmWrite_TicksMissedTotal(this->m_ticksMissed);
        this->log_WARNING_LO_TicksMissed(this->m_lastTick + 1, tick);
    }
    this->m_lastTick = tick;
    this->m_lastSimTimeUs = header.get_simTimeUs();
    this->m_sensors = frame.get_sensors();
    this->m_windowFrames++;

    this->tlmWrite_Tick(tick);
    this->tlmWrite_SimTime(static_cast<F64>(this->m_lastSimTimeUs) / 1.0e6);

    // 3. This frame *is* the system clock tick: drive the rate group driver.
    if (this->isConnected_cycleOut_OutputPort(0)) {
        Os::RawTime cycleStart;
        (void)cycleStart.now();
        this->cycleOut_out(0, cycleStart);
    }
}

}  // namespace MyFlightSim
