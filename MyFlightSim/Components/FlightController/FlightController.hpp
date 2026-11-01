// ======================================================================
// \title  FlightController.hpp
// \author my-flight-sim
// \brief  hpp file for FlightController component implementation class
// ======================================================================

#ifndef MyFlightSim_FlightController_HPP
#define MyFlightSim_FlightController_HPP

#include "MyFlightSim/Components/FlightController/ControlLaw.hpp"
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

    //! Control step length: the 200 Hz rate group period (s)
    static constexpr F32 CONTROL_DT = 0.005f;

    //! Allowed TAKEOFF / GOTO altitudes (m)
    static constexpr F32 MIN_ALTITUDE = 1.0f;
    static constexpr F32 MAX_ALTITUDE = 50.0f;

    //! A target counts as reached within this distance (m) and below this speed (m/s)
    static constexpr F32 ARRIVAL_DISTANCE = 0.25f;
    static constexpr F32 ARRIVAL_SPEED = 0.25f;

    //! Touchdown: rangefinder below this (m) and vertical speed below this (m/s)...
    static constexpr F32 TOUCHDOWN_RANGE = 0.10f;
    static constexpr F32 TOUCHDOWN_SPEED = 0.20f;
    //! ...for this many consecutive control steps (100 steps = 0.5 s)
    static constexpr U32 TOUCHDOWN_STEPS = 100;

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
    void ARM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    //! Handler implementation for command DISARM
    void DISARM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    //! Handler implementation for command MOTOR_TEST
    void MOTOR_TEST_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 level) override;

    //! Handler implementation for command TAKEOFF
    void TAKEOFF_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 altitude) override;

    //! Handler implementation for command GOTO
    void GOTO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 east, F32 north, F32 altitude) override;

    //! Handler implementation for command LAND
    void LAND_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

  private:
    // ----------------------------------------------------------------------
    // Parameter hook
    // ----------------------------------------------------------------------

    //! Called by the generated code after a parameter is set (PRM_SET command)
    void parameterUpdated(FwPrmIdType id) override;

  private:
    // ----------------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------------

    //! True in the closed-loop flying modes
    bool isFlying() const;

    //! True when the vehicle can accept a ground-only command (armed and not flying)
    bool isArmedOnGround() const;

    //! Read all parameters (clamped to their allowed ranges) into a ControlGains
    ControlGains readGains();

    //! Closed-loop step for TAKEOFF / HOLD / GOTO / LANDING, including mode transitions
    MotorThrottles flyClosedLoop(const ControlGains& gains);

    //! Start closed-loop flight towards m_target in the given mode
    void startClosedLoop(FlightMode mode);

    //! Write telemetry for this control step
    void writeTelemetry(const MotorThrottles& motors, const ControlGains& gains);

  private:
    // ----------------------------------------------------------------------
    // Member variables. Only ever touched on the 200 Hz rate group thread
    // (run_handler, and command handlers dispatched from it), so no mutex.
    // ----------------------------------------------------------------------

    FlightMode m_mode;       //!< Current flight mode
    F32 m_testLevel;         //!< Throttle for MOTOR_TEST mode
    U32 m_cycle;             //!< Control step counter (for telemetry decimation)
    SensorData m_sensors;    //!< Sensor data read at the start of this step
    Vector3 m_target;        //!< Position target for closed-loop modes
    F32 m_targetYaw;         //!< Heading target (rad)
    U32 m_touchdownSteps;    //!< Consecutive steps that looked like touchdown
    ControlLaw m_control;    //!< The control algorithm
};

}  // namespace MyFlightSim

#endif
