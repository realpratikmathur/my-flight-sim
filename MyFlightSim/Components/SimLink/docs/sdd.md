# MyFlightSim::SimLink

Boundary between the flight software and the physics simulator.

## Introduction

In a real vehicle, the flight software talks to hardware: an IMU, motor
controllers, and a hardware timer that produces the system clock. In this
project the hardware is a physics simulator running in a separate process.
`SimLink` is the only component that knows about it.

`SimLink` receives one UDP datagram per simulator step through a `Drv.Udp`
driver. It validates each datagram and **uses it as the system clock tick**: it
calls `rateGroupDriver.CycleIn` once per datagram. The flight software therefore
runs in lock-step with simulated time. Pause the simulator and the flight
software pauses; run it at 2× and the flight software runs at 2×.

This follows the advice in the F´ docs, *Rate Groups and Timeliness* ›
*System Clock Sources*: "Most projects implement a clock component that
translates between the system clock and the port call to rate group driver's
`CycleIn` port."

## Requirements

| Name | Description | Rationale | Validation |
|---|---|---|---|
| SIMLINK-001 | SimLink shall invoke `cycleOut` exactly once for each valid datagram received. | The simulator is the system clock. | Unit test `ValidFramesDriveClock` |
| SIMLINK-002 | SimLink shall reject datagrams that are too short or whose magic number is not `SIM_MAGIC`, emit `BadFrame`, and not invoke `cycleOut`. | Corrupt data must not drive the system. | Unit tests `BadMagicRejected`, `ShortDatagramRejected` |
| SIMLINK-003 | SimLink shall emit `TicksMissed` and count missed ticks when the tick number skips forward. | Make lost simulator steps visible to operators. | Unit test `MissedTicksReported` |
| SIMLINK-004 | SimLink shall emit `SimConnected` on the first valid datagram and whenever the tick number goes backwards (simulator restart). | Operators need to know when the simulator (re)connects. | Unit tests `ValidFramesDriveClock`, `SimulatorRestart` |
| SIMLINK-005 | SimLink shall return every received buffer to the driver, whatever the receive status. | Buffers come from a fixed pool; leaking one would eventually stop reception. | Unit test `DriverErrorIgnored` |
| SIMLINK-006 | SimLink shall report the tick rate and real-time factor measured against the wall clock once per second. | Shows whether the simulation keeps up with real time. | Manual (GDS) |

## Design

### Ports

| Port | Kind | Type | Connected to | Purpose |
|---|---|---|---|---|
| `drvReceiveIn` | guarded input | `Drv.ByteStreamData` | `simUdp.$recv` | Datagrams from the simulator (runs on the Udp receive thread) |
| `drvReceiveReturnOut` | output | `Fw.BufferSend` | `simUdp.recvReturnIn` | Gives each buffer back to the driver |
| `drvConnected` | guarded input | `Drv.ByteStreamReady` | `simUdp.ready` | Socket opened (informational) |
| `drvSendOut` | output | `Drv.ByteStreamSend` | `simUdp.$send` | Datagrams to the simulator (from feature 03) |
| `cycleOut` | output | `Svc.Cycle` | `rateGroupDriver.CycleIn` | The system clock tick |
| `schedIn` | guarded input | `Svc.Sched` | `rateGroup_1Hz` | Publishes link statistics |

`drvReceiveIn` and `schedIn` run on **different threads** (the Udp receive thread
and the 1 Hz rate group thread) and share member variables. Both are therefore
`guarded`: the generated code locks one component-wide mutex around each
handler.

### Wire format

Every datagram starts with a `SimHeader` (`MyFlightSim/Types/SimTypes.fpp`),
serialized by F´: fields in order, big-endian, no padding.

| Field | Type | Meaning |
|---|---|---|
| `magic` | U32 | `0x5153494D` ("QSIM") |
| `tick` | U32 | Simulator step number |
| `simTimeUs` | U64 | Simulated time in microseconds |

### Statistics

`TickRate` and `RealTimeFactor` are measured against the **wall clock**
(`Os::RawTime`), not against the rate groups. The rate groups are driven by
the very ticks being measured, so counting ticks per rate-group second
would always give 200.

## Configuration

Set in `MyFlightSim/QuadFsw/Top/instances.fpp`, module `SimConfig`:

| Constant | Default | Meaning |
|---|---|---|
| `FSW_RECV_PORT` | 50100 | UDP port the flight software listens on |
| `SIM_RECV_PORT` | 50101 | UDP port the simulator listens on |
| `DATAGRAM_BUFFER_SIZE` | 1024 | Bytes per receive buffer |
| `DATAGRAM_BUFFER_COUNT` | 4 | Receive buffers in `simBufferManager` |
