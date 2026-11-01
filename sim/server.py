#!/usr/bin/env python3
"""Quadcopter physics simulator + 3D viewer server.

This process plays the role of the vehicle hardware for the F Prime flight
software (the QuadFsw deployment):

* Every 5 ms of simulated time it sends a SensorFrame datagram to SimLink
  (UDP 127.0.0.1:50100). SimLink uses each one as the flight software's clock
  tick and as its sensor readings.
* It then waits (lock-step) for the ActuatorFrame answering that tick
  (UDP 127.0.0.1:50101), applies the four motor throttles, and advances the
  physics by 5 ms.
* It serves the 3D viewer at http://localhost:8080 and streams the vehicle
  state to it over a WebSocket.

If the flight software is not running, the simulator keeps running in real time
with the motors off, so you can start the two in any order.

Usage:  python3 sim/server.py [--speed 1.0] [--http-port 8080]
"""
import argparse
import asyncio
import json
import math
import pathlib
import time

from aiohttp import web, WSMsgType

import protocol
from physics import Quadcopter, tilt_of, yaw_of

STEP_S = 0.005             # one simulator step = 5 ms -> 200 Hz (matches QuadFswTopology.cpp)
VIEWER_PERIOD_S = 1 / 30   # state updates to the browser
LOCKSTEP_TIMEOUT_S = 0.25  # how long to wait for the flight software to answer a tick
OFFLINE_AFTER_S = 1.0      # no answers for this long -> stop waiting (flight software offline)

WEB_DIR = pathlib.Path(__file__).resolve().parent.parent / "web"


class FswLink(asyncio.DatagramProtocol):
    """Receives ActuatorFrames from SimLink and matches them to the tick we sent."""

    def __init__(self):
        self.waiting_tick = None
        self.answer = None
        self.bad_frames = 0

    def expect(self, tick):
        self.waiting_tick = tick
        self.answer = asyncio.get_running_loop().create_future()
        return self.answer

    def datagram_received(self, data, addr):
        frame = protocol.unpack_actuator_frame(data)
        if frame is None:
            self.bad_frames += 1
            return
        tick, _, motors = frame
        if tick == self.waiting_tick and self.answer is not None and not self.answer.done():
            self.answer.set_result(motors)
        # Answers for older ticks (late) are ignored


class Simulator:
    def __init__(self, args):
        self.args = args
        self.quad = Quadcopter()
        self.tick = 0
        self.speed = args.speed
        self.paused = False
        self.motors_cmd = [0.0] * 4
        self.fsw_online = False
        self.last_answer_wall = 0.0
        self.late_answers = 0
        self.viewers = set()
        self.link = FswLink()
        self.transport = None

    # ------------------------------------------------------------ main loop
    async def run(self):
        loop = asyncio.get_running_loop()
        self.transport, _ = await loop.create_datagram_endpoint(
            lambda: self.link, local_addr=("127.0.0.1", self.args.listen_port))
        fsw_addr = (self.args.fsw_host, self.args.fsw_port)
        print(f"Physics: sending sensor frames to udp://{fsw_addr[0]}:{fsw_addr[1]}, "
              f"listening for motor commands on udp://127.0.0.1:{self.args.listen_port}")
        print(f"Hover throttle on {self.quad.env.name}: {self.quad.hover_throttle():.3f}")

        wall_start = time.monotonic()
        sim_start_tick = 0
        next_view = 0.0
        while True:
            if self.paused:
                await asyncio.sleep(0.05)
                wall_start, sim_start_tick = time.monotonic(), self.tick
                continue

            # 1. Send this tick's sensor readings (and clock tick) to the flight software
            sim_time_us = self.tick * int(STEP_S * 1e6)
            answer = self.link.expect(self.tick)
            self.transport.sendto(protocol.pack_sensor_frame(self.tick, sim_time_us, self.quad.sensors()), fsw_addr)

            # 2. Lock-step: wait for the motor throttles answering this tick
            timeout = LOCKSTEP_TIMEOUT_S if self.fsw_online else 0.002
            try:
                self.motors_cmd = await asyncio.wait_for(answer, timeout)
                self.fsw_online = True
                self.last_answer_wall = time.monotonic()
            except asyncio.TimeoutError:
                if self.fsw_online:
                    self.late_answers += 1
                if time.monotonic() - self.last_answer_wall > OFFLINE_AFTER_S:
                    if self.fsw_online:
                        print("Flight software stopped answering: motors off, running free")
                    self.fsw_online = False
                    self.motors_cmd = [0.0] * 4

            # 3. Advance the physics by one step
            self.quad.step(self.motors_cmd, STEP_S)
            self.tick += 1

            # 4. Keep pace with the wall clock (times the speed factor)
            target = wall_start + (self.tick - sim_start_tick) * STEP_S / self.speed
            delay = target - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            elif delay < -0.5:
                wall_start, sim_start_tick = time.monotonic(), self.tick  # fell behind: re-anchor

            # 5. Update the 3D viewers
            now = time.monotonic()
            if now >= next_view:
                next_view = now + VIEWER_PERIOD_S
                await self.broadcast()

    # ------------------------------------------------------------ viewers
    def snapshot(self):
        s = self.quad.state
        sensors_battery = self.quad.sensors().battery_voltage
        return {
            "type": "state",
            "tick": self.tick,
            "simTime": self.tick * STEP_S,
            "speed": self.speed,
            "paused": self.paused,
            "env": self.quad.env.name,
            "position": s.position,
            "velocity": s.velocity,
            "attitude": s.attitude,
            "rotorSpeed": s.rotor_speed,
            "motorCmd": self.motors_cmd,
            "tiltDeg": math.degrees(tilt_of(s.attitude)),
            "yawDeg": math.degrees(yaw_of(s.attitude)),
            "onGround": s.on_ground,
            "crashed": s.crashed,
            "crashReason": s.crash_reason,
            "battery": sensors_battery,
            "fswOnline": self.fsw_online,
            "lateAnswers": self.late_answers,
            "hoverThrottle": self.quad.hover_throttle(),
        }

    async def broadcast(self):
        if not self.viewers:
            return
        message = json.dumps(self.snapshot())
        for ws in list(self.viewers):
            try:
                await ws.send_str(message)
            except Exception:
                self.viewers.discard(ws)

    def handle_viewer_command(self, msg):
        cmd = msg.get("cmd")
        if cmd == "reset":
            self.quad.reset()
            print("Viewer: vehicle reset to the helipad")
        elif cmd == "pause":
            self.paused = not self.paused
        elif cmd == "speed":
            self.speed = max(0.1, min(4.0, float(msg.get("value", 1.0))))


async def make_app(sim: Simulator):
    async def index(_request):
        return web.FileResponse(WEB_DIR / "index.html")

    async def websocket(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        sim.viewers.add(ws)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        sim.handle_viewer_command(json.loads(msg.data))
                    except (ValueError, TypeError):
                        pass
        finally:
            sim.viewers.discard(ws)
        return ws

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/", WEB_DIR, show_index=False)
    return app


async def main():
    parser = argparse.ArgumentParser(description="Quadcopter physics simulator for the QuadFsw F Prime deployment")
    parser.add_argument("--speed", type=float, default=1.0, help="real-time factor (1.0 = real time)")
    parser.add_argument("--http-port", type=int, default=8080, help="port for the 3D viewer")
    parser.add_argument("--fsw-host", default="127.0.0.1", help="flight software address")
    parser.add_argument("--fsw-port", type=int, default=50100, help="SimConfig.FSW_RECV_PORT in instances.fpp")
    parser.add_argument("--listen-port", type=int, default=50101, help="SimConfig.SIM_RECV_PORT in instances.fpp")
    args = parser.parse_args()

    sim = Simulator(args)
    runner = web.AppRunner(await make_app(sim))
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", args.http_port).start()
    print(f"3D viewer: http://localhost:{args.http_port}")
    await sim.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nSimulator stopped. The flight software's rate groups have stopped too.")
