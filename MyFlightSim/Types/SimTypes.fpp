module MyFlightSim {

  # ----------------------------------------------------------------------
  # Types shared by the flight software and the physics simulator.
  #
  # Every UDP datagram between the simulator (sim/) and the flight software
  # is the F Prime serialized form of one of these structs: fields in order,
  # big-endian, no padding. See FPP User's Guide section 18.4
  # "Serialization of FPP Values". The Python simulator packs the same layout
  # with struct.pack(">...") -- see sim/protocol.py.
  #
  # Frames: world = x east, y north, z up (ENU)
  #         body  = x forward, y left, z up (FLU)
  # ----------------------------------------------------------------------

  @ Magic number at the start of every simulator datagram: ASCII "QSIM"
  constant SIM_MAGIC = 0x5153494D

  @ Header at the start of every datagram exchanged with the simulator
  struct SimHeader {
    magic: U32 @< Must equal SIM_MAGIC
    tick: U32 @< Simulator step number; increases by 1 every simulator step
    simTimeUs: U64 @< Simulated time since the simulator started (microseconds)
  }

  @ A vector with three components
  struct Vector3 {
    x: F32
    y: F32
    z: F32
  }

  @ A rotation quaternion, scalar part first
  struct Quaternion {
    w: F32
    x: F32
    y: F32
    z: F32
  } default { w = 1.0 }

  @ Throttle for each motor, 0.0 (stopped) to 1.0 (full).
  @ Order: 0 front-left, 1 front-right, 2 back-right, 3 back-left.
  array MotorThrottles = [4] F32 format "{.3f}"

  @ Everything the vehicle's sensors measured during one simulator step
  struct SensorData {
    gyro: Vector3 @< Angular rate (rad/s), body axes
    accel: Vector3 @< Specific force measured by the accelerometer (m/s^2), body axes
    attitude: Quaternion @< Attitude from the inertial navigation unit (body to world)
    position: Vector3 @< Position (m), world frame
    velocity: Vector3 @< Velocity (m/s), world frame
    rangeToGround: F32 @< Downward rangefinder distance (m)
    batteryVoltage: F32 @< Battery voltage (V)
  }

  @ Datagram from the simulator to the flight software, once per simulator step
  struct SensorFrame {
    header: SimHeader
    sensors: SensorData
  }

  @ Datagram from the flight software to the simulator, once per 200 Hz cycle
  struct ActuatorFrame {
    header: SimHeader @< tick and simTimeUs of the SensorFrame being answered
    motors: MotorThrottles
  }

}
