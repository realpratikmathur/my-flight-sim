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

    // Statistics window, restarted on every schedIn call
    bool m_windowStarted;      //!< m_windowStart/m_windowStartSimUs are valid
    Os::RawTime m_windowStart; //!< Wall-clock time at the start of the window
    U64 m_windowStartSimUs;    //!< Simulated time at the start of the window
    U32 m_windowFrames;        //!< Valid frames received during the window
};

}  // namespace MyFlightSim

#endif
