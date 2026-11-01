#!/usr/bin/env python3
"""The simplest possible "simulator": just a clock, no physics.

Introduced in feature 02. It sends one SensorFrame every simulator step
(5 ms = 200 Hz) to the flight software's SimLink component, always reporting
a vehicle sitting still on the ground. Since feature 03 the full physics
simulator is sim/server.py; this script is kept as a tiny, dependency-free
way to see the "simulator is the clock" idea on its own.

Each datagram is an F Prime serialized ``SensorFrame`` struct (see
MyFlightSim/Types/SimTypes.fpp and sim/protocol.py). F Prime serializes
struct fields in order, big-endian, with no padding.

Usage (no extra Python packages needed):

    python3 sim/clock_sim.py                # real time
    python3 sim/clock_sim.py --speed 2      # twice real time
    python3 sim/clock_sim.py --drop-every 50   # lose a tick now and then
    python3 sim/clock_sim.py --bad-every 400   # send a corrupt datagram now and then
"""
import argparse
import socket
import time

import protocol

STEP_US = 5000                  # 5 ms per step -> 200 Hz (see QuadFswTopology.cpp)
AT_REST = protocol.Sensors()    # vehicle sitting level on the ground


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1", help="flight software address")
    parser.add_argument("--port", type=int, default=50100,
                        help="flight software UDP port (SimConfig.FSW_RECV_PORT in instances.fpp)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="real-time factor: 1.0 = real time, 2.0 = twice as fast")
    parser.add_argument("--drop-every", type=int, default=0, metavar="N",
                        help="do not send every Nth tick (shows SimLink.TicksMissed)")
    parser.add_argument("--bad-every", type=int, default=0, metavar="N",
                        help="send a corrupt datagram every Nth tick (shows SimLink.BadFrame)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Receive (and ignore) the motor commands the flight software sends back,
    # so they don't bounce off a closed port
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 50101))
    sink.setblocking(False)
    target = (args.host, args.port)
    print(f"Sending {1e6 / STEP_US:.0f} Hz ticks to udp://{args.host}:{args.port} "
          f"at {args.speed}x real time. Ctrl-C to stop.")

    tick = 0
    start = time.monotonic()
    next_report = start + 1.0
    try:
        while True:
            sim_time_us = tick * STEP_US
            frame = protocol.pack_sensor_frame(tick, sim_time_us, AT_REST)
            if args.bad_every and tick and tick % args.bad_every == 0:
                sock.sendto(b"\xba\xdb\xad\x00" + frame[4:], target)  # corrupt the magic number
            elif not (args.drop_every and tick and tick % args.drop_every == 0):
                sock.sendto(frame, target)
            tick += 1
            try:
                while True:
                    sink.recv(64)
            except BlockingIOError:
                pass

            # Pace ourselves against the wall clock
            wake = start + (tick * STEP_US / 1e6) / args.speed
            delay = wake - time.monotonic()
            if delay > 0:
                time.sleep(delay)

            now = time.monotonic()
            if now >= next_report:
                print(f"tick {tick:8d}   sim time {sim_time_us / 1e6:9.2f} s", flush=True)
                next_report += 1.0
    except KeyboardInterrupt:
        print("\nStopped. The flight software's rate groups have stopped too.")


if __name__ == "__main__":
    main()
