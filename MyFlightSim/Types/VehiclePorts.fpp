module MyFlightSim {

  # ----------------------------------------------------------------------
  # Hardware-abstraction ports.
  #
  # Flight software components (e.g. FlightController) use only these ports
  # to read sensors and drive motors. They never know whether the other end
  # is the simulator (SimLink) or real hardware drivers.
  # ----------------------------------------------------------------------

  @ Read the most recent sensor data
  port SensorsGet -> SensorData

  @ Set the throttle of all four motors
  port MotorsSet(
    throttles: MotorThrottles @< One throttle per motor, 0.0 to 1.0
  )

}
