module MyFlightSim {

  # ----------------------------------------------------------------------
  # Types shared by the flight software and the physics simulator.
  #
  # Every UDP datagram between the simulator (sim/) and the flight software
  # is the F Prime serialized form of one of these structs: fields in order,
  # big-endian, no padding. See FPP User's Guide section 18.4
  # "Serialization of FPP Values". The Python simulator packs the same layout
  # with struct.pack(">...").
  # ----------------------------------------------------------------------

  @ Magic number at the start of every simulator datagram: ASCII "QSIM"
  constant SIM_MAGIC = 0x5153494D

  @ Header at the start of every datagram exchanged with the simulator
  struct SimHeader {
    magic: U32 @< Must equal SIM_MAGIC
    tick: U32 @< Simulator step number; increases by 1 every simulator step
    simTimeUs: U64 @< Simulated time since the simulator started (microseconds)
  }

}
