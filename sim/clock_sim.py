#!/usr/bin/env python3
"""Feature 02: the simplest possible "simulator" -- just a clock.

It sends one datagram every simulator step (5 ms = 200 Hz) to the flight
software's SimLink component. Each datagram is an F Prime serialized
``SimHeader`` struct (see MyFlightSim/Types/SimTypes.fpp):

    magic     U32  0x5153494D ("QSIM")
    tick      U32  step number
    simTimeUs U64  simulated time in microseconds

F Prime serializes struct fields in order, big-endian, with no padding, so
the Python equivalent is struct.Struct(">IIQ").

There is no physics yet. The point is to show that **the simulator is the
flight software's clock**: stop this script and the rate groups stop.

Usage (no extra Python packages needed):

    python3 sim/clock_sim.py                # real time
    python3 sim/clock_sim.py --speed 2      # twice real time
    python3 sim/clock_sim.py --drop-every 50   # lose a tick now and then
    python3 sim/clock_sim.py --bad-every 400   # send a corrupt datagram now and then
"""
import argparse
import socket
import struct
import time

SIM_MAGIC = 0x5153494D          # must match SIM_MAGIC in SimTypes.fpp
SIM_HEADER = struct.Struct(">IIQ")  # SimHeader: magic, tick, simTimeUs
STEP_US = 5000                  # 5 ms per step -> 200 Hz (see QuadFswTopology.cpp)


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
    target = (args.host, args.port)
    print(f"Sending {1e6 / STEP_US:.0f} Hz ticks to udp://{args.host}:{args.port} "
          f"at {args.speed}x real time. Ctrl-C to stop.")

    tick = 0
    start = time.monotonic()
    next_report = start + 1.0
    try:
        while True:
            sim_time_us = tick * STEP_US
            if args.bad_every and tick and tick % args.bad_every == 0:
                sock.sendto(SIM_HEADER.pack(0xBADBAD00, tick, sim_time_us), target)
            elif not (args.drop_every and tick and tick % args.drop_every == 0):
                sock.sendto(SIM_HEADER.pack(SIM_MAGIC, tick, sim_time_us), target)
            tick += 1

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
