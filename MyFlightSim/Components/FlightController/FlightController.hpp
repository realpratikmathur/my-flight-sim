// ======================================================================
// \title  FlightController.hpp
// \author my-flight-sim
// \brief  hpp file for FlightController component implementation class
// ======================================================================

#ifndef MyFlightSim_FlightController_HPP
#define MyFlightSim_FlightController_HPP

#include "MyFlightSim/Components/FlightController/FlightControllerComponentAc.hpp"

namespace MyFlightSim {

class FlightController final : public FlightControllerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Constants
    // ----------------------------------------------------------------------

    //! Throttle applied to every motor while armed and idle
    static constexpr F32 IDLE_THROTTLE = 0.08f;

    //! Telemetry is written every this many control steps (200 Hz / 20 = 10 Hz)
    static constexpr U32 TELEMETRY_DECIMATION = 20;

    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct FlightController object
    FlightController(const char* const compName  //!< The component name
    );

    //! Destroy FlightController object
    ~FlightController();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Control step: called by the 200 Hz rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command ARM
    void ARM_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                        U32 cmdSeq            //!< The command sequence number
                        ) override;

    //! Handler implementation for command DISARM
    void DISARM_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                           U32 cmdSeq            //!< The command sequence number
                           ) override;

    //! Handler implementation for command MOTOR_TEST
    void MOTOR_TEST_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                               U32 cmdSeq,           //!< The command sequence number
                               F32 level             //!< Throttle for all motors, 0.0 to 1.0
                               ) override;

  private:
    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    //! Write telemetry for this control step
    void writeTelemetry(const SensorData& sensors, const MotorThrottles& motors);

  private:
    // ----------------------------------------------------------------------
    // Member variables. Only ever touched on the 200 Hz rate group thread
    // (run_handler, and command handlers dispatched from it), so no mutex.
    // ----------------------------------------------------------------------

    FlightMode m_mode;    //!< Current flight mode
    F32 m_testLevel;      //!< Throttle for MOTOR_TEST mode
    U32 m_cycle;          //!< Control step counter (for telemetry decimation)
};

}  // namespace MyFlightSim

#endif
