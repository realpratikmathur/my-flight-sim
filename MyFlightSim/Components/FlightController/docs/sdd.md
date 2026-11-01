# MyFlightSim::FlightController

The quadcopter's flight controller.

## Introduction

`FlightController` decides what the four motors do. In feature 03 it can arm,
disarm, and run an **open-loop motor test**: every motor at the same throttle,
with no stabilization. Feature 04 adds closed-loop flight (take off, hover, go
to a point, land).

It is a **queued** component: it has a message queue but no thread. The
200 Hz rate group calls its `run` port. `run` first dispatches any queued
commands, then performs the control step. This follows *Queued Components for
Cyclic Work* in the F´ docs (`lib/fprime/docs/user-manual/framework/component-and-port-selection.md`).

The component never talks to the simulator. It only uses two
hardware-abstraction ports (`SensorsGet`, `MotorsSet`, defined in
`MyFlightSim/Types/VehiclePorts.fpp`), which `SimLink` implements here. Real
sensor and motor drivers could implement them on a real vehicle.

## Requirements

| Name | Description | Rationale | Validation |
|---|---|---|---|
| FC-001 | On every `run` call, FlightController shall read the sensors once and command the motors once. | Fixed-rate control loop. | Unit test `DisarmedMotorsOff` |
| FC-002 | Commands shall take effect at the start of the next `run` call, not when they are received. | Commands change state at a known point in the control cycle; no mutex needed. | Unit test `CommandsWaitForRun` |
| FC-003 | When disarmed, all motor throttles shall be 0. | Safety. | Unit tests `DisarmedMotorsOff`, `ArmDisarm` |
| FC-004 | `ARM` shall set all motors to `IDLE_THROTTLE`; `DISARM` shall always be accepted and stop all motors. | Operators can always stop the motors. | Unit test `ArmDisarm` |
| FC-005 | `MOTOR_TEST` shall be rejected with `VALIDATION_ERROR` if the level is outside 0.0–1.0, and with `EXECUTION_ERROR` if the vehicle is disarmed. | Validate arguments, then state. | Unit tests `MotorTestValidatesLevel`, `MotorTestRequiresArm` |
| FC-006 | Telemetry shall be written every `TELEMETRY_DECIMATION` (20) control steps, i.e. 10 Hz. | 200 Hz telemetry would be wasted downlink. | Unit test `TelemetryDecimated` |

## Design

### Ports

| Port | Kind | Type | Connected to |
|---|---|---|---|
| `run` | sync input | `Svc.Sched` | `rateGroup_200Hz.RateGroupMemberOut[0]` |
| `sensorsGet` | output | `MyFlightSim.SensorsGet` (returns `SensorData`) | `simLink.sensorsGet` |
| `motorsOut` | output | `MyFlightSim.MotorsSet` | `simLink.motorsIn` |

### Commands

| Command | Kind | Arguments | Effect |
|---|---|---|---|
| `ARM` | async | — | `DISARMED` → `ARMED_IDLE` |
| `DISARM` | async | — | any → `DISARMED` |
| `MOTOR_TEST` | async | `level: F32` (0.0–1.0) | armed → `MOTOR_TEST` |

### Modes (`enum FlightMode`)

`DISARMED` (motors 0) → `ARMED_IDLE` (motors at `IDLE_THROTTLE` = 0.08) →
`MOTOR_TEST` (motors at `level`).

## Configuration

`IDLE_THROTTLE` and `TELEMETRY_DECIMATION` are constants in
`FlightController.hpp`.
