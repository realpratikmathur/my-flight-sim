# sim/: the "hardware" for the flight software

| File | What it is |
|---|---|
| `server.py` | Quadcopter physics simulator and 3D-viewer web server (feature 03+) |
| `physics.py` | Rigid-body quadcopter model (6 DOF, rotors, drag, ground contact, battery) |
| `protocol.py` | Byte layouts matching the FPP structs in `MyFlightSim/Types/SimTypes.fpp` |
| `clock_sim.py` | Feature 02's clock-only simulator (no physics, stdlib only) |

Install the one extra dependency into the project venv:

```bash
pip install -r sim/requirements.txt
```

Run (after starting `fprime-gds` in `MyFlightSim/QuadFsw`):

```bash
python3 sim/server.py
```

Open the viewer at <http://localhost:8080>. The physics is only here to give the
F´ flight software something realistic to control. See `docs/learning/` for
the F´ side.
