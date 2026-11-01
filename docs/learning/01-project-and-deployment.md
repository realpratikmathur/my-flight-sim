# 01 — An F´ project and our first deployment (`QuadFsw`)

**Branch:** `feature/01-quadfsw-deployment` (builds on `main`)

**Goal:** get a real, unmodified F´ flight software program building and talking
to the F´ ground system. No quadcopter yet. This is the empty "spacecraft
computer" that the later features fill in.

**No hand-written code in this branch.** Everything here was produced by the
two official F´ tools, so you can see what F´ gives you out of the box:

```bash
fprime-bootstrap project --tag v4.4.1                # -> branch main
cd MyFlightSim && fprime-util new --deployment       # -> this branch
#   Deployment name: QuadFsw
#   Deployment namespace: MyFlightSim
#   Communication driver type: 1 - TcpClient
```

---

## 1. The F´ concepts this branch uses

### 1.1 Project → deployment → topology

> "A project may consist of one or more related deployments. Each deployment is
> one instance of F´ software."
>
> — F´ docs, *Projects and Deployments* › **Projects**
> (`lib/fprime/docs/user-manual/overview/proj-dep.md`,
> online: <https://fprime.jpl.nasa.gov/latest/docs/user-manual/overview/proj-dep/>)

> "Deployments are tied to builds; for each build of the code there is a single
> deployment."
>
> — same page, **Deployments**

The same page uses the real Mars Helicopter as its example, which is our
inspiration:

> "The Mars Helicopter is the project, but has two deployments: one designated
> for the helicopter, and one designated for the base station."

In this repository:

| F´ term | Here | What it is |
|---|---|---|
| Project | the repo root (`settings.ini`, `CMakeLists.txt`) | Holds everything; `settings.ini` says where F´ is (`framework_path: ./lib/fprime`) |
| Deployment | `MyFlightSim/QuadFsw/` | One executable: our quadcopter's flight software |
| Topology | `MyFlightSim/QuadFsw/Top/topology.fpp` | Which component instances exist, and how their ports connect |

> "In F Prime, a *topology* or connection graph is the highest level of software
> architecture in a *deployment*, or executable binary program. A topology
> specifies what component instances are used in the application and how their
> port instances are connected."
>
> — FPP User's Guide, **§14 Defining Topologies**
> (<https://nasa.github.io/fpp/fpp-users-guide.html#Defining-Topologies>)

### 1.2 Subtopologies: reusable chunks of flight software

Open `MyFlightSim/QuadFsw/Top/topology.fpp`. The first thing in the topology is:

```fpp
import CdhCore.Subtopology
import ComCcsds.Subtopology
import DataProducts.Subtopology
import FileHandling.Subtopology
```

> "A subtopology in FPP is a grouping of component instances and connections
> between them. It can be imported into a larger topology, allowing for modular,
> reusable design, and easier management of complex systems."
>
> — F´ docs, *Subtopologies*
> (`lib/fprime/docs/user-manual/design-patterns/subtopologies.md`,
> online: <https://fprime.jpl.nasa.gov/latest/docs/user-manual/design-patterns/subtopologies/>)

The FPP mechanism behind `import` is described in **FPP User's Guide §14.4
Importing Topologies**:

> "To import a topology `A` into a topology `B`, you do the following: …
> Inside topology `B`, write `import A`."

So with four lines we already have:

* **CdhCore** (command and data handling): command dispatcher (`CdhCore.cmdDisp`),
  event manager, telemetry channel database (`CdhCore.tlmSend`), health monitor,
  fatal error handling, version reporting.
* **ComCcsds**: the full radio link stack, using the CCSDS space standards: framing,
  deframing, a downlink queue, and buffer management.
* **FileHandling**: file uplink and downlink, the parameter database (`prmDb`),
  and the file manager.
* **DataProducts**: storage for large science and engineering data.

This is a big part of *why F´ exists*: a team does not rewrite command
dispatching or telemetry plumbing for every vehicle. They reuse flight-proven
components and spend their effort on what is unique to their mission.

### 1.3 Pattern graph specifiers: the automatic wiring

Also in `topology.fpp`:

```fpp
command connections instance CdhCore.cmdDisp
event connections instance CdhCore.events
telemetry connections instance CdhCore.tlmSend
...
time connections instance chronoTime
```

> "A few connection patterns are so common in F Prime that they get special
> treatment in FPP. … This works, but it is tedious and repetitive. So FPP
> provides a better way: you can use a *pattern graph specifier* to specify this
> common pattern."
>
> — FPP User's Guide, **§14.2 Connection Graphs** › *Pattern Graph Specifiers*

Every component we add later that has commands, events, or telemetry gets
connected to these services **automatically**. You will see this in feature 02.

### 1.4 Rate groups: the heartbeat

At the bottom of `topology.fpp`, in `connections RateGroups`:

```fpp
timer.CycleOut -> rateGroupDriver.CycleIn
rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup_1Hz] -> rateGroup_1Hz.CycleIn
rateGroup_1Hz.RateGroupMemberOut[0] -> CdhCore.tlmSend.Run
...
```

> "F´ provides a mechanism to trigger time-based actions called "rate groups".
> The `ActiveRateGroup` component contains multiple output `Sched` ports that it
> sends a message to at a repeated rate. Thus, components having an input `Sched`
> port can run a repeated action at this rate."
>
> — F´ docs, *Rate Groups and Timeliness* (intro)
> (`lib/fprime/docs/user-manual/design-patterns/rate-group.md`,
> online: <https://fprime.jpl.nasa.gov/latest/docs/user-manual/design-patterns/rate-group/>)

In the generated deployment, `timer` (a `Svc.LinuxTimer`) ticks at **1 Hz**. In
`Top/QuadFswTopology.cpp` you can see the divisors that make the 1 Hz, 0.5 Hz
and 0.25 Hz rate groups:

```cpp
const Fw::TimeInterval rateGroupInterval(1, 0);  // 1Hz base clock
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{{{1, 0}, {2, 0}, {4, 0}}};
```

1 Hz is far too slow to fly a quadcopter. In **feature 02** we replace this
wall-clock timer with a clock driven by the **simulator**. Remember this
spot.

### 1.5 The startup sequence is generated for you

`Top/QuadFswTopology.cpp` → `setupTopology()` calls functions that FPP
**generated** from the topology model:

```cpp
initComponents(state);   // construct + init every instance
setBaseIds();            // give each instance its ID range
connectComponents();     // wire every port connection in topology.fpp
regCommands();           // every component registers its commands with cmdDisp
configComponents(state);
configureTopology();     // <- the only hand-written part
readParameters();        // load PrmDb.dat
loadParameters();        // push parameter values into components
startTasks(state);       // start a thread for every active component
```

FPP User's Guide **§18.3 Implementing Deployments** describes these
functions. The point to remember: **you never hand-write the wiring.** If it's in
`topology.fpp`, the generated `connectComponents()` does it.

### 1.6 IDs: how the ground knows what's what

`Top/instances.fpp` begins with the ID convention the template chose:

```
# All Base IDs follow the 8-digit hex format: 0xDSSCCxxx
```

Every command, event, and telemetry channel gets an ID equal to its component
instance's **base ID** plus a small offset. F´ docs, *Data Constructs* ›
**Commands** (`lib/fprime/docs/user-manual/overview/04-cmd-evt-chn-prm.md`):

> "opcode: a numerical value uniquely representing the command. This opcode is
> automatically adjusted to be relative to a component's base id, and thus will
> not collide with other commands defined by other components."

Example: `CdhCore.cmdDisp` has base ID `0x01000000`, and `CMD_NO_OP` is
`opcode 0`, so the ground sends opcode `0x01000000` (= 16777216). You'll
see that number in the test below.

Names are qualified by the FPP **module** they are defined in. Instances from
`instances.fpp` live in `module MyFlightSim`, so in the GDS they appear as
`MyFlightSim.systemResources.CPU`. Subtopology instances appear as
`CdhCore.cmdDisp.CMD_NO_OP`.

### 1.7 The Ground Data System (GDS)

> "A GDS is the "Ground Data System" that supports large-scale embedded systems
> by allowing operators to interact with the embedded system. … The F´ GDS
> provides this capability out-of-the-box to developers and projects using F´."
>
> — F´ docs, *The F´ Ground Data System* › **What is the GDS?**
> (`lib/fprime/docs/user-manual/overview/gds-introduction.md`,
> online: <https://fprime.jpl.nasa.gov/latest/docs/user-manual/overview/gds-introduction/>)

The GDS learns every command, event, and channel name from the **dictionary**
the build generates:
`build-artifacts/Linux/MyFlightSim_QuadFsw/dict/QuadFswTopologyDictionary.json`.

---

## 2. Files to read, in this order

1. `settings.ini`: where F´ lives, and default build options.
2. `MyFlightSim/QuadFsw/Top/topology.fpp`: the whole system on one page.
3. `MyFlightSim/QuadFsw/Top/instances.fpp`: the instances that aren't from
   subtopologies, with their base IDs, queue sizes and thread priorities.
4. `MyFlightSim/QuadFsw/Top/QuadFswTopology.cpp`: the startup sequence (§1.5).
5. `MyFlightSim/QuadFsw/Main.cpp`: `main()`, which reads `-a`/`-p`, calls
   `setupTopology`, then `startRateGroups()` (which blocks).
6. `lib/fprime/Svc/Subtopologies/CdhCore/CdhCore.fpp`: what one subtopology
   looks like inside.

---

## 3. Build it

```bash
. fprime-venv/bin/activate
```

```bash
cd MyFlightSim/QuadFsw
```

```bash
fprime-util generate
```

```bash
fprime-util build -j4
```

`generate` runs CMake once to make the build cache (`build-fprime-automatic-native/`
at the project root). `build` runs the FPP autocoder and compiles. It's slow
the first time (all of F´ is compiled) and fast after that.

**What to check:** the executable and dictionary exist:

```bash
ls ../../build-artifacts/Linux/MyFlightSim_QuadFsw/bin/ ../../build-artifacts/Linux/MyFlightSim_QuadFsw/dict/
```

Expect `MyFlightSim_QuadFsw` and `QuadFswTopologyDictionary.json`.

Optional: look at the generated code. It's under
`../../build-fprime-automatic-native/MyFlightSim/QuadFsw/Top/`; try
`QuadFswTopologyAc.cpp` and search for `connectComponents`.

---

## 4. Run it and test it

From `MyFlightSim/QuadFsw` (venv active):

```bash
fprime-gds
```

This starts the GDS **and** launches our flight software (see GDS docs,
*Running the GDS*: "This will launch both the `Ref` flight software and the GDS
layer along with the browser pointed at the GDS."). A browser tab opens at
<http://localhost:5000>.

### Test 1: the flight software is alive (telemetry)

1. Open the **Channels** tab.
2. **Expect** `MyFlightSim.systemResources.*` channels (CPU, memory) updating about once a
   second. That's the 1 Hz rate group calling `systemResources.run`, then
   `CdhCore.tlmSend.Run` sending the values down.

### Test 2: send a command, watch the events

1. Open the **Commanding** tab.
2. Mnemonic: `CdhCore.cmdDisp.CMD_NO_OP`. Click **Send Command**.
3. Open the **Events** tab. **Expect**, in this order:
   * `CdhCore.cmdDisp.OpCodeDispatched`: "Opcode 0x1000000 dispatched to port 0"
     (that's the base ID + opcode from §1.6)
   * `CdhCore.cmdDisp.NoOpReceived`
   * `CdhCore.cmdDisp.OpCodeCompleted`
4. In **Channels**, `CdhCore.cmdDisp.CommandsDispatched` increased by 1.

### Test 3: the same thing from the command line (no browser)

The GDS can run headless (GDS docs, *Run the GDS Without a UI*: `fprime-gds -g none`).
Terminal 1:

```bash
fprime-gds -g none
```

Terminal 2 (venv active, same directory):

```bash
fprime-cli command-send CdhCore.cmdDisp.CMD_NO_OP
```

```bash
fprime-cli events -t 5
```

**Expect** the same `OpCodeDispatched` / `NoOpReceived` / `OpCodeCompleted` events.

Press **Ctrl-C** in the `fprime-gds` terminal to stop everything.

### Where are the logs?

`MyFlightSim/QuadFsw/logs/<timestamp>/` holds the flight software's own stdout
(`MyFlightSim_QuadFsw.log`) and the GDS's recorded events, channels, and commands.

---

## 5. Things to try

* Send `CdhCore.cmdDisp.CMD_NO_OP_STRING` with the argument `hello` and find the
  `NoOpStringReceived` event.
* Open `QuadFswTopologyDictionary.json` and search for `CMD_NO_OP`. Compare the
  opcode with §1.6.
* Change `rateGroupInterval` in `QuadFswTopology.cpp` to `(0, 500000)` (2 Hz),
  rebuild, and watch the Channels tab update twice as fast. (Undo it afterwards.)

---

## 6. What's next

**Feature 02** writes our first component, `SimLink`. It is the boundary between
the flight software and the "hardware" (the simulator), and it replaces the
1 Hz `LinuxTimer`, so **the simulator's clock drives the rate groups**.
