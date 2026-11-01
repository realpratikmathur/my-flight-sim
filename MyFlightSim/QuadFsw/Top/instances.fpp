module MyFlightSim {

  # ----------------------------------------------------------------------
  # Base ID Convention
  # ----------------------------------------------------------------------
  #
  # All Base IDs follow the 8-digit hex format: 0xDSSCCxxx
  #
  # Where:
  #   D   = Deployment digit (1 for this deployment)
  #   SS  = Subtopology digits (00 for main topology, 01-05 for subtopologies)
  #   CC  = Component digits (00, 01, 02, etc.)
  #   xxx = Reserved for internal component items (events, commands, telemetry)
  #

  # ----------------------------------------------------------------------
  # Defaults
  # ----------------------------------------------------------------------

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ----------------------------------------------------------------------
  # Active component instances
  # ----------------------------------------------------------------------

  # The base clock is the simulator: one tick every 5 ms (200 Hz). See
  # rateGroupDivisorsSet in QuadFswTopology.cpp for the divisors.

  # 200 Hz rate group (divisor 1): flight control will run here (features 03-04)
  instance rateGroup_200Hz: Svc.ActiveRateGroup base id 0x10001000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 43

  # 10 Hz rate group (divisor 20): telemetry, communications, sequencing
  instance rateGroup_10Hz: Svc.ActiveRateGroup base id 0x10002000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 42

  # 1 Hz rate group (divisor 200): health, housekeeping, statistics
  instance rateGroup_1Hz: Svc.ActiveRateGroup base id 0x10003000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 41

  instance cmdSeq: Svc.CmdSequencer base id 0x10004000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 40

  # ----------------------------------------------------------------------
  # Queued component instances
  # ----------------------------------------------------------------------


  # ----------------------------------------------------------------------
  # Passive component instances
  # ----------------------------------------------------------------------

  instance chronoTime: Svc.ChronoTime base id 0x10010000

  instance rateGroupDriver: Svc.RateGroupDriver base id 0x10011000

  instance systemResources: Svc.SystemResources base id 0x10012000

  # (Feature 02) The Svc.LinuxTimer that used to be here is gone: the
  # simulator is now the system clock. See simLink below.

  instance comDriver: Drv.TcpClient base id 0x10014000

  # ----------------------------------------------------------------------
  # Simulator link (feature 02)
  #
  # These instances use FPP *init specifiers* (FPP User's Guide 13.3) to put
  # their C++ setup code right next to the instance. The topology autocoder
  # pastes each snippet into the matching generated function, e.g. the
  # "startTasks" snippet ends up inside startTasks().
  # ----------------------------------------------------------------------

  @ UDP driver that exchanges datagrams with the physics simulator
  instance simUdp: Drv.Udp base id 0x10015000 {
    phase Fpp.ToCpp.Phases.configComponents """
    // Receive simulator frames on 127.0.0.1:FSW_RECV_PORT
    (void) simUdp.configureRecv("127.0.0.1", SimConfig::FSW_RECV_PORT, SimConfig::DATAGRAM_BUFFER_SIZE);
    // Send to the simulator on 127.0.0.1:SIM_RECV_PORT (used from feature 03)
    (void) simUdp.configureSend("127.0.0.1", SimConfig::SIM_RECV_PORT);
    """

    phase Fpp.ToCpp.Phases.startTasks """
    // Start the UDP receive thread
    simUdp.start(Os::TaskString("SimUdpRecv"), SimConfig::RECV_PRIORITY, Default::STACK_SIZE);
    """

    phase Fpp.ToCpp.Phases.stopTasks """
    simUdp.stop();
    """

    phase Fpp.ToCpp.Phases.freeThreads """
    (void) simUdp.join();
    """
  }

  @ Fixed pool of receive buffers for simUdp, allocated once at startup
  instance simBufferManager: Svc.BufferManager base id 0x10016000 {
    phase Fpp.ToCpp.Phases.configObjects """
    Svc::BufferManager::BufferBins bins;
    """

    phase Fpp.ToCpp.Phases.configComponents """
    memset(&ConfigObjects::MyFlightSim_simBufferManager::bins, 0, sizeof(ConfigObjects::MyFlightSim_simBufferManager::bins));
    ConfigObjects::MyFlightSim_simBufferManager::bins.bins[0].bufferSize = SimConfig::DATAGRAM_BUFFER_SIZE;
    ConfigObjects::MyFlightSim_simBufferManager::bins.bins[0].numBuffers = SimConfig::DATAGRAM_BUFFER_COUNT;
    simBufferManager.setup(SimConfig::BUFFER_MANAGER_ID, 0, mallocator, ConfigObjects::MyFlightSim_simBufferManager::bins);
    """

    phase Fpp.ToCpp.Phases.tearDownComponents """
    simBufferManager.cleanup();
    """
  }

  @ Boundary between the flight software and the simulator; the system clock
  instance simLink: MyFlightSim.SimLink base id 0x10017000

  @ Settings for the simulator link
  module SimConfig {
    @ UDP port the flight software listens on (simulator -> flight software)
    constant FSW_RECV_PORT = 50100
    @ UDP port the simulator listens on (flight software -> simulator)
    constant SIM_RECV_PORT = 50101
    @ Size of each receive buffer; larger than any simulator datagram
    constant DATAGRAM_BUFFER_SIZE = 1024
    @ Number of receive buffers in the pool
    constant DATAGRAM_BUFFER_COUNT = 4
    @ ID used by simBufferManager to tag its buffers
    constant BUFFER_MANAGER_ID = 200
    @ Priority of the UDP receive thread (higher than the rate groups)
    constant RECV_PRIORITY = 44
  }

}
