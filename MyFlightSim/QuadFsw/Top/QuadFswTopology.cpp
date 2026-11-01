// ======================================================================
// \title  QuadFswTopology.cpp
// \brief cpp file containing the topology instantiation code
//
// ======================================================================
// Provides access to autocoded functions
#include <MyFlightSim/QuadFsw/Top/QuadFswTopologyAc.hpp>
// Note: Uncomment when using Svc:TlmPacketizer
//#include <MyFlightSim/QuadFsw/Top/QuadFswPacketsAc.hpp>

// Necessary project-specified types
#include <Fw/Types/MallocAllocator.hpp>

// Public functions for use in main program are namespaced with deployment module MyFlightSim
// This is also the namespace where the topology components are instantiated by FPP.
namespace MyFlightSim {

// Instantiate a malloc allocator for cmdSeq buffer allocation
Fw::MallocAllocator mallocator;

// Rate group timing (feature 02): the base clock is no longer a timer. It is the
// simulator, which sends one datagram every 5 ms of simulated time (200 Hz).
// SimLink turns each datagram into one call of rateGroupDriver.CycleIn.
// Divisors are in the same order as enum Ports_RateGroups in topology.fpp.
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{{{1, 0}, {20, 0}, {200, 0}}};
// Divisors: 200 Hz / 1 = 200 Hz, 200 Hz / 20 = 10 Hz, 200 Hz / 200 = 1 Hz

// Context tokens for rate group members (unused, set to zero)
Svc::ActiveRateGroup::ContextArray rateGroup_200HzContext(0);
Svc::ActiveRateGroup::ContextArray rateGroup_10HzContext(0);
Svc::ActiveRateGroup::ContextArray rateGroup_1HzContext(0);

// Set by requestShutdown() (called from the Ctrl-C signal handler)
static volatile bool s_shutdownRequested = false;

enum TopologyConstants {
    COMM_PRIORITY = 34,
};

/**
 * \brief configure/setup components in project-specific way
 *
 * This is a *helper* function which configures/sets up each component requiring project specific input. This includes
 * allocating resources, passing-in arguments, etc. This function may be inlined into the topology setup function if
 * desired, but is extracted here for clarity.
 */
void configureTopology() {
    // Rate group driver needs a divisor list
    rateGroupDriver.configure(rateGroupDivisorsSet);

    // Rate groups require context arrays.
    rateGroup_200Hz.configure(rateGroup_200HzContext);
    rateGroup_10Hz.configure(rateGroup_10HzContext);
    rateGroup_1Hz.configure(rateGroup_1HzContext);

    // Command sequencer needs to allocate memory to hold contents of command sequences
    cmdSeq.allocateBuffer(0, mallocator, 5 * 1024);

    // PrmDb file name must be supplied by the using topology.
    // (Feature 04) The flight software runs with its bin/ directory as the
    // working directory. Keep the parameter file one level up, so fprime-gds
    // still finds exactly one program in bin/ to launch.
    FileHandling::prmDb.configure("../PrmDb.dat");
}

void setupTopology(const TopologyState& state) {
    // Autocoded initialization. Function provided by autocoder.
    initComponents(state);
    // Autocoded id setup. Function provided by autocoder.
    setBaseIds();
    // Autocoded connection wiring. Function provided by autocoder.
    connectComponents();
    // Autocoded command registration. Function provided by autocoder.
    regCommands();
    // Autocoded configuration. Function provided by autocoder.
    configComponents(state);
    if (state.hostname != nullptr && state.port != 0) {
        comDriver.configure(state.hostname, state.port);
    }
    // Project-specific component configuration. Function provided above. May be inlined, if desired.
    configureTopology();
    // Autocoded parameter read from file. Function provided by autocoder.
    readParameters();
    // Autocoded parameter loading. Function provided by autocoder.
    loadParameters();
    // Autocoded task kick-off (active components). Function provided by autocoder.
    startTasks(state);
    // Initialize socket communication if and only if there is a valid specification
    if (state.hostname != nullptr && state.port != 0) {
        Os::TaskString name("ReceiveTask");
        // Uplink is configured for receive so a socket task is started
        comDriver.start(name, COMM_PRIORITY, Default::STACK_SIZE);
    }
}

void waitForShutdown() {
    // Nothing to start here: the rate groups run whenever the simulator sends
    // a tick. The main thread just waits for Ctrl-C.
    while (!s_shutdownRequested) {
        (void)Os::Task::delay(Fw::TimeInterval(0, 100000));  // 100 ms
    }
}

void requestShutdown() {
    s_shutdownRequested = true;
}

void teardownTopology(const TopologyState& state) {
    // Autocoded (active component) task clean-up. Functions provided by topology autocoder.
    stopTasks(state);
    freeThreads(state);

    // Other task clean-up.
    comDriver.stop();
    (void)comDriver.join();

    // Resource deallocation
    cmdSeq.deallocateBuffer(mallocator);

    tearDownComponents(state);
    deinitComponents(state);
}
};  // namespace MyFlightSim
