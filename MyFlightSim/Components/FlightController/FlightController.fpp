module MyFlightSim {

    @ What the flight controller is currently doing
    enum FlightMode {
        DISARMED @< Motors stopped
        ARMED_IDLE @< Motors spinning slowly; not enough thrust to lift off
        MOTOR_TEST @< All motors at the throttle given by MOTOR_TEST (open loop)
    }

    @ The quadcopter's flight controller.
    @
    @ A *queued* component: it has a message queue but no thread. Commands are
    @ async, so the command dispatcher only puts them on the queue. The 200 Hz
    @ rate group calls `run`, which first dispatches any queued commands and
    @ then runs the control step. Commands therefore always take effect at the
    @ same point in the control cycle, and no mutex is needed.
    queued component FlightController {

        # ----------------------------------------------------------------------
        # Ports
        # ----------------------------------------------------------------------

        @ Control step: called by the 200 Hz rate group
        sync input port run: Svc.Sched

        @ Reads the latest sensor data
        output port sensorsGet: SensorsGet

        @ Sets the motor throttles
        output port motorsOut: MotorsSet

        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Arm the motors. They spin at idle throttle, too slowly to lift off.
        async command ARM

        @ Disarm: stop all motors immediately. Always accepted.
        async command DISARM

        @ Open-loop motor test: drive all four motors at the same throttle.
        @ The vehicle must be armed. There is no stabilization in this mode!
        async command MOTOR_TEST(
            level: F32 @< Throttle for all motors, 0.0 to 1.0
        )

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ The motors were armed
        event Armed severity activity high format "Motors armed"

        @ The motors were disarmed
        event Disarmed severity activity high format "Motors disarmed"

        @ A motor test started
        event MotorTestStarted(
            level: F32 @< Throttle applied to all motors
        ) severity activity high \
          format "Motor test: all motors at {.2f}"

        @ A command was rejected because the motors are not armed
        event NotArmed severity warning low format "Rejected: motors are not armed"

        @ A command was rejected because the throttle is out of range
        event InvalidThrottle(
            level: F32 @< The rejected throttle
        ) severity warning low \
          format "Rejected: throttle {.2f} is outside 0.0 to 1.0"

        # ----------------------------------------------------------------------
        # Telemetry (written at 10 Hz, every 20th control step)
        # ----------------------------------------------------------------------

        @ Current flight mode
        telemetry Mode: FlightMode

        @ Throttles being sent to the motors
        telemetry Motors: MotorThrottles

        @ Altitude above the ground from the rangefinder (m)
        telemetry Altitude: F32 format "{.2f}"

        @ Roll (x), pitch (y) and yaw (z) in degrees
        telemetry AttitudeDeg: Vector3

        @ Position in the world frame (m): x east, y north, z up
        telemetry Position: Vector3

        @ Velocity in the world frame (m/s)
        telemetry Velocity: Vector3

        @ Battery voltage (V)
        telemetry BatteryVoltage: F32 format "{.2f}"

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

    }
}
