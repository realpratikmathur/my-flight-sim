module MyFlightSim {

    @ Boundary between the flight software and the physics simulator.
    @
    @ The simulator plays the role of the vehicle's hardware. Every simulator
    @ step it sends one UDP datagram (received by a Drv.Udp driver and passed
    @ to this component). SimLink treats each datagram as one tick of the
    @ system clock and drives the rate group driver with it, so the flight
    @ software runs in lock-step with simulated time.
    passive component SimLink {

        # ----------------------------------------------------------------------
        # Byte stream driver client ports (connected to a Drv.Udp instance)
        #
        # These mirror the Drv.ByteStreamDriverClient interface, except that the
        # receive and connect ports are *guarded* instead of *sync*: they run on
        # the Udp driver's receive thread, while schedIn runs on a rate group
        # thread, and both touch the same member variables.
        # ----------------------------------------------------------------------

        @ Ready signal from the driver
        guarded input port drvConnected: Drv.ByteStreamReady

        @ Datagrams received from the simulator (called on the Udp receive thread)
        guarded input port drvReceiveIn: Drv.ByteStreamData

        @ Returns ownership of each received buffer to the driver
        output port drvReceiveReturnOut: Fw.BufferSend

        @ Sends datagrams to the simulator (used starting in feature 03)
        output port drvSendOut: Drv.ByteStreamSend

        # ----------------------------------------------------------------------
        # Clock and scheduling
        # ----------------------------------------------------------------------

        @ System clock output: invoked once per simulator tick. Connect to
        @ Svc.RateGroupDriver.CycleIn.
        output port cycleOut: Svc.Cycle

        @ Called by the 1 Hz rate group to compute and publish link statistics
        guarded input port schedIn: Svc.Sched

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        @ The first valid frame arrived from the simulator
        event SimConnected(
            tick: U32 @< First tick received
        ) severity activity high \
          format "Simulator connected; first tick {}"

        @ A datagram was rejected
        event BadFrame(
            numBytes: FwSizeType @< Datagram size in bytes
            magic: U32 @< Magic number found in the datagram
        ) severity warning low \
          format "Rejected simulator datagram: size {} bytes, magic 0x{x}" \
          throttle 5 every { seconds = 10 }

        @ One or more simulator ticks never arrived
        event TicksMissed(
            expected: U32 @< The tick number we expected
            received: U32 @< The tick number that arrived
        ) severity warning low \
          format "Simulator ticks missed: expected {}, received {}" \
          throttle 5 every { seconds = 10 }

        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ Most recent simulator tick number
        telemetry Tick: U32

        @ Most recent simulated time (seconds)
        telemetry SimTime: F64 format "{.3f}"

        @ Ticks received per second of *wall-clock* time (200 when running in real time)
        telemetry TickRate: F32 format "{.1f}"

        @ Simulated seconds per wall-clock second (1.0 = real time)
        telemetry RealTimeFactor: F32 format "{.2f}"

        @ Total number of ticks that never arrived
        telemetry TicksMissedTotal: U32 update on change

        @ Total number of rejected datagrams
        telemetry BadFramesTotal: U32 update on change

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

    }
}
