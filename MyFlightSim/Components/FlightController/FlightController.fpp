module MyFlightSim {

    @ What the flight controller is currently doing
    enum FlightMode {
        DISARMED @< Motors stopped
        ARMED_IDLE @< Motors spinning slowly; not enough thrust to lift off
        MOTOR_TEST @< All motors at the throttle given by MOTOR_TEST (open loop)
        TAKEOFF @< Climbing to the TAKEOFF altitude (closed loop)
        HOLD @< Holding position at the current target (closed loop)
        GOTO @< Flying to the GOTO target (closed loop)
        LANDING @< Descending at LAND_SPEED until touchdown (closed loop)
    }

    @ The quadcopter's flight controller: arming, an open-loop motor test,
    @ and closed-loop takeoff, hold, go-to and landing.
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
        @ The vehicle must be armed and on the ground. There is no
        @ stabilization in this mode!
        async command MOTOR_TEST(
            level: F32 @< Throttle for all motors, 0.0 to 1.0
        )

        @ Closed-loop takeoff: climb straight up to the given altitude, then hold.
        @ The vehicle must be armed and on the ground.
        async command TAKEOFF(
            altitude: F32 @< Altitude above the takeoff point (m), 1 to 50
        )

        @ Fly to a point and hold there. The vehicle must be flying.
        async command GOTO(
            east: F32 @< Target position east of the helipad (m)
            north: F32 @< Target position north of the helipad (m)
            altitude: F32 @< Target altitude (m), 1 to 50
        )

        @ Descend straight down at LAND_SPEED, then disarm on touchdown.
        @ The vehicle must be flying.
        async command LAND

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

        @ A command was rejected because the altitude is out of range
        event InvalidAltitude(
            altitude: F32 @< The rejected altitude
        ) severity warning low \
          format "Rejected: altitude {.1f} m is outside 1 to 50 m"

        @ A command was rejected because the vehicle is flying
        event NotOnGround(
            mode: FlightMode @< Current mode
        ) severity warning low \
          format "Rejected: the vehicle must be on the ground (mode {})"

        @ A command was rejected because the vehicle is not flying
        event NotFlying(
            mode: FlightMode @< Current mode
        ) severity warning low \
          format "Rejected: the vehicle must be flying (mode {})"

        @ A GOTO was rejected because the target is outside the geofence
        event OutsideGeofence(
            distance: F32 @< Horizontal distance of the target from the helipad (m)
            limit: F32 @< GEOFENCE_RADIUS (m)
        ) severity warning low \
          format "Rejected: target is {.1f} m from the helipad; the geofence is {.1f} m"

        @ Takeoff started
        event TakeoffStarted(
            altitude: F32 @< Target altitude (m)
        ) severity activity high \
          format "Taking off to {.1f} m"

        @ Takeoff altitude reached; now holding
        event TakeoffComplete(
            altitude: F32 @< Altitude reached (m)
        ) severity activity high \
          format "Takeoff complete at {.1f} m; holding"

        @ Flying to a new target
        event GotoStarted(
            east: F32 @< Target east (m)
            north: F32 @< Target north (m)
            altitude: F32 @< Target altitude (m)
        ) severity activity high \
          format "Flying to E {.1f} m, N {.1f} m, alt {.1f} m"

        @ Target reached; now holding
        event Arrived(
            east: F32 @< Position east (m)
            north: F32 @< Position north (m)
            altitude: F32 @< Altitude (m)
        ) severity activity high \
          format "Arrived at E {.1f} m, N {.1f} m, alt {.1f} m; holding"

        @ Landing started
        event LandingStarted(
            speed: F32 @< Descent speed (m/s)
        ) severity activity high \
          format "Landing at {.2f} m/s"

        @ Touchdown detected; motors disarmed
        event Landed severity activity high format "Touchdown; motors disarmed"

        @ A parameter was set to a value outside its allowed range and will be clamped
        event ParamOutOfRange(
            paramId: FwPrmIdType @< Parameter id (relative to the component)
            value: F32 @< The value that was set
            minimum: F32 @< Lowest allowed value
            maximum: F32 @< Highest allowed value
        ) severity warning low \
          format "Parameter {} = {.3f} is outside [{.3f}, {.3f}]; it will be clamped"

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

        @ Position the controller is flying to or holding (m)
        telemetry Target: Vector3

        @ Horizontal distance to the target (m)
        telemetry DistanceToTarget: F32 format "{.2f}"

        @ Throttle that balances gravity, computed from the vehicle parameters
        telemetry HoverThrottle: F32 format "{.3f}"

        # ----------------------------------------------------------------------
        # Parameters: set with <NAME>_PRM_SET, make permanent with <NAME>_PRM_SAVE
        # then FileHandling.prmDb.PRM_SAVE_FILE
        # ----------------------------------------------------------------------

        @ Position loop gain: desired speed (m/s) per metre of position error
        param POS_KP: F32 default 1.0

        @ Horizontal velocity loop gain: desired acceleration (m/s^2) per m/s of error
        param VEL_KP: F32 default 2.0

        @ Vertical velocity loop gain (m/s^2 per m/s)
        param VEL_Z_KP: F32 default 3.0

        @ Vertical velocity integral gain (m/s^2 per m)
        param VEL_Z_KI: F32 default 1.0

        @ Attitude loop gain: desired body rate (rad/s) per radian of roll/pitch error
        param ATT_KP: F32 default 6.0

        @ Body rate loop gain for roll and pitch (1/s)
        param RATE_KP: F32 default 20.0

        @ Heading loop gain (rad/s per rad)
        param YAW_KP: F32 default 3.0

        @ Yaw rate loop gain (1/s)
        param YAW_RATE_KP: F32 default 4.0

        @ Maximum horizontal speed (m/s)
        param MAX_SPEED: F32 default 3.0

        @ Maximum climb and descent speed (m/s)
        param MAX_CLIMB: F32 default 1.5

        @ Maximum tilt angle (degrees)
        param MAX_TILT_DEG: F32 default 25.0

        @ Descent speed when landing (m/s)
        param LAND_SPEED: F32 default 0.6

        @ Vehicle mass (kg)
        param VEHICLE_MASS: F32 default 1.2

        @ Thrust of one motor at full throttle (N)
        param MOTOR_MAX_THRUST: F32 default 7.0

        @ Gravitational acceleration (m/s^2)
        param GRAVITY: F32 default 9.81

        @ Maximum horizontal distance of a GOTO target from the helipad (m)
        param GEOFENCE_RADIUS: F32 default 100.0

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

        @ Parameter get port (reads values from the component's local copy)
        param get port prmGetOut

        @ Parameter set port (sends values to the parameter database on PRM_SAVE)
        param set port prmSetOut

    }
}
