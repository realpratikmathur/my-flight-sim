// ======================================================================
// \title  SimLink.hpp
// \author my-flight-sim
// \brief  hpp file for SimLink component implementation class
//
// SimLink is the boundary between the flight software and the physics
// simulator. See SimLink.fpp for the model and docs/sdd.md for the design.
// ======================================================================

#ifndef MyFlightSim_SimLink_HPP
#define MyFlightSim_SimLink_HPP

#include "MyFlightSim/Components/SimLink/SimLinkComponentAc.hpp"
#include "MyFlightSim/Types/ActuatorFrameSerializableAc.hpp"
#include "MyFlightSim/Types/SensorDataSerializableAc.hpp"
#include "Os/RawTime.hpp"

namespace MyFlightSim {

class SimLink final : public SimLinkComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct SimLink object
    SimLink(const char* const compName  //!< The component name
    );

    //! Destroy SimLink object
    ~SimLink();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for drvConnected
    //!
    //! Ready signal from the driver
    void drvConnected_handler(FwIndexType portNum  //!< The port number
                              ) override;

    //! Handler implementation for drvReceiveIn
    //!
    //! Datagrams received from the simulator (called on the Udp receive thread)
    void drvReceiveIn_handler(FwIndexType portNum,  //!< The port number
                              Fw::Buffer& buffer,
                              const Drv::ByteStreamStatus& status) override;

    //! Handler implementation for schedIn
    //!
    //! Called by the 1 Hz rate group to compute and publish link statistics
    void schedIn_handler(FwIndexType portNum,  //!< The port number
                         U32 context           //!< The call order
                         ) override;

    //! Handler implementation for sensorsGet
    //!
    //! Returns the sensor data from the most recent simulator frame
    SensorData sensorsGet_handler(FwIndexType portNum  //!< The port number
                                  ) override;

    //! Handler implementation for motorsIn
    //!
    //! Stores the motor throttles to send at the end of this cycle
    void motorsIn_handler(FwIndexType portNum,  //!< The port number
                          const MotorThrottles& throttles) override;

    //! Handler implementation for actuatorSchedIn
    //!
    //! Sends the motor throttles to the simulator, answering the current tick
    void actuatorSchedIn_handler(FwIndexType portNum,  //!< The port number
                                 U32 context           //!< The call order
                                 ) override;

  private:
    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    //! Validate one datagram and, if it is good, use it as a clock tick
    void processDatagram(Fw::Buffer& buffer);

  private:
    // ----------------------------------------------------------------------
    // Member variables (protected by the component mutex: every input port
    // that touches them is "guarded")
    // ----------------------------------------------------------------------

    bool m_connected;          //!< At least one valid frame has been received
    U32 m_lastTick;            //!< Tick number of the most recent valid frame
    U64 m_lastSimTimeUs;       //!< Simulated time of the most recent valid frame
    U32 m_ticksMissed;         //!< Total ticks that never arrived
    U32 m_badFrames;           //!< Total rejected datagrams
    U32 m_sendErrors;          //!< Total failed sends to the simulator

    SensorData m_sensors;      //!< Sensor data from the most recent valid frame
    MotorThrottles m_motors;   //!< Motor throttles to send to the simulator

    //! Memory for the outgoing ActuatorFrame (no allocation while flying)
    U8 m_sendData[ActuatorFrame::SERIALIZED_SIZE];

    // Statistics window, restarted on every schedIn call
    bool m_windowStarted;      //!< m_windowStart/m_windowStartSimUs are valid
    Os::RawTime m_windowStart; //!< Wall-clock time at the start of the window
    U64 m_windowStartSimUs;    //!< Simulated time at the start of the window
    U32 m_windowFrames;        //!< Valid frames received during the window
};

}  // namespace MyFlightSim

#endif
