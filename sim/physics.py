"""Rigid-body physics for a quadcopter (pure Python, no dependencies).

Frames
    world: x east, y north, z up (ENU), origin on the ground at the helipad
    body : x forward, y left, z up (FLU)
    The attitude quaternion q = (w, x, y, z) rotates body vectors into the world.

Model
    * 6 degrees of freedom: Newton (translation) and Euler (rotation) equations
    * rotors: thrust = max_thrust * (air density ratio) * speed^2, with a
      first-order spin-up lag; reaction (yaw) torque proportional to thrust
    * linear + quadratic air drag, angular drag
    * small per-motor thrust mismatch ("manufacturing tolerance"), so an
      unstabilized vehicle does not fly perfectly straight
    * ground contact with landing gear; hard impacts or tipping over = crash
    * simple battery model
Integration: semi-implicit Euler at 1 kHz (5 sub-steps per 5 ms simulator step).
"""
import math
import random
from dataclasses import dataclass, field
from typing import List

from protocol import Sensors

SEA_LEVEL_AIR_DENSITY = 1.225  # kg/m^3


@dataclass
class Environment:
    name: str = "Earth"
    gravity: float = 9.81                  # m/s^2
    air_density: float = SEA_LEVEL_AIR_DENSITY  # kg/m^3


EARTH = Environment()


@dataclass
class QuadParams:
    mass: float = 1.2                  # kg
    arm_length: float = 0.17           # m, centre to each motor
    inertia: tuple = (0.012, 0.012, 0.022)  # kg m^2 about body x, y, z
    max_thrust: float = 7.0            # N per motor at full throttle, sea-level air
    yaw_torque_per_thrust: float = 0.016  # N m of reaction torque per N of thrust
    motor_time_constant: float = 0.03  # s
    linear_drag: float = 0.30          # N per m/s
    quadratic_drag: float = 0.05       # N per (m/s)^2
    angular_drag: float = 0.003        # N m per rad/s
    thrust_mismatch: float = 0.006     # +/- fraction, fixed per motor
    crash_speed: float = 3.0           # m/s vertical impact speed that breaks the vehicle
    battery_full: float = 16.8         # V
    battery_empty: float = 14.0        # V
    battery_capacity: float = 1000.0   # "throttle-seconds" of all motors combined


# Motor order matches MotorThrottles in SimTypes.fpp:
# 0 front-left, 1 front-right, 2 back-right, 3 back-left (X configuration)
_A = 1.0 / math.sqrt(2.0)
MOTOR_LAYOUT = [(+_A, +_A), (+_A, -_A), (-_A, -_A), (-_A, +_A)]  # unit (x, y) in body frame
# Reaction torque direction about body z for each motor. Diagonal pairs match.
MOTOR_YAW_DIR = [+1.0, -1.0, +1.0, -1.0]


# ---------------------------------------------------------------- vector math
def _add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _scale(a, k):
    return [a[0] * k, a[1] * k, a[2] * k]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def quat_rotate(q, v):
    """Rotate vector v from body to world by unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    # v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    u = [x, y, z]
    t = _scale(_cross(u, v), 2.0)
    return _add(_add(v, _scale(t, w)), _cross(u, t))


def quat_rotate_inverse(q, v):
    """Rotate vector v from world to body."""
    w, x, y, z = q
    return quat_rotate([w, -x, -y, -z], v)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q]


def quat_from_yaw(yaw):
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_of(q):
    """Angle (rad) between the body z axis and the world z axis."""
    body_up = quat_rotate(q, [0.0, 0.0, 1.0])
    return math.acos(max(-1.0, min(1.0, body_up[2])))


# ---------------------------------------------------------------- vehicle
@dataclass
class QuadState:
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    attitude: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    rates: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # body rad/s
    rotor_speed: List[float] = field(default_factory=lambda: [0.0] * 4)   # 0..1
    accel_world: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    on_ground: bool = True
    crashed: bool = False
    crash_reason: str = ""
    battery_used: float = 0.0


class Quadcopter:
    def __init__(self, params: QuadParams = None, env: Environment = EARTH, seed: int = 7):
        self.params = params or QuadParams()
        self.env = env
        self.rng = random.Random(seed)
        self.mismatch = [1.0 + self.rng.uniform(-1, 1) * self.params.thrust_mismatch for _ in range(4)]
        self.state = QuadState()

    def reset(self):
        self.state = QuadState()

    # --------------------------------------------------------------- physics
    def step(self, throttle_cmd: List[float], dt: float, substeps: int = 5):
        h = dt / substeps
        for _ in range(substeps):
            self._substep(throttle_cmd, h)

    def _thrusts(self):
        p = self.params
        density_ratio = self.env.air_density / SEA_LEVEL_AIR_DENSITY
        return [p.max_thrust * density_ratio * w * w * m
                for w, m in zip(self.state.rotor_speed, self.mismatch)]

    def _substep(self, throttle_cmd, h):
        p, s, env = self.params, self.state, self.env
        if s.crashed:
            s.rotor_speed = [0.0] * 4
            s.accel_world = [0.0, 0.0, 0.0]
            return

        # Rotors spin up/down with a first-order lag toward the command
        alpha = min(1.0, h / p.motor_time_constant)
        for i in range(4):
            cmd = max(0.0, min(1.0, throttle_cmd[i]))
            s.rotor_speed[i] += (cmd - s.rotor_speed[i]) * alpha
        thrusts = self._thrusts()
        total_thrust = sum(thrusts)
        s.battery_used += sum(s.rotor_speed) * h

        # Torques in the body frame: tau = r x F for each rotor, plus yaw reaction
        tau = [0.0, 0.0, 0.0]
        for (ux, uy), yaw_dir, t in zip(MOTOR_LAYOUT, MOTOR_YAW_DIR, thrusts):
            tau[0] += uy * p.arm_length * t
            tau[1] -= ux * p.arm_length * t
            tau[2] += yaw_dir * p.yaw_torque_per_thrust * t
        tau = _add(tau, _scale(s.rates, -p.angular_drag))

        # Euler's equation: I w' = tau - w x (I w)
        inertia = p.inertia
        iw = [inertia[0] * s.rates[0], inertia[1] * s.rates[1], inertia[2] * s.rates[2]]
        gyro_term = _cross(s.rates, iw)
        rate_dot = [(tau[i] - gyro_term[i]) / inertia[i] for i in range(3)]

        # Newton's second law in the world frame
        thrust_world = quat_rotate(s.attitude, [0.0, 0.0, total_thrust])
        speed = _norm(s.velocity)
        drag = _scale(s.velocity, -(p.linear_drag + p.quadratic_drag * speed))
        force = _add(_add(thrust_world, drag), [0.0, 0.0, -p.mass * env.gravity])
        accel = _scale(force, 1.0 / p.mass)

        weight = p.mass * env.gravity
        if s.on_ground and thrust_world[2] < weight:
            # Resting on the landing gear: the ground pushes back, nothing moves
            s.velocity = [0.0, 0.0, 0.0]
            s.rates = [0.0, 0.0, 0.0]
            s.accel_world = [0.0, 0.0, 0.0]
            s.attitude = quat_from_yaw(yaw_of(s.attitude))  # gear keeps it level
            s.position[2] = 0.0
            return

        # Free flight: integrate (semi-implicit Euler)
        s.rates = _add(s.rates, _scale(rate_dot, h))
        wx, wy, wz = s.rates
        dq = quat_multiply(s.attitude, [0.0, wx, wy, wz])
        s.attitude = quat_normalize([s.attitude[i] + 0.5 * dq[i] * h for i in range(4)])
        s.velocity = _add(s.velocity, _scale(accel, h))
        s.position = _add(s.position, _scale(s.velocity, h))
        s.accel_world = accel
        s.on_ground = False

        # Ground contact
        if s.position[2] <= 0.0:
            impact = -s.velocity[2]
            tilt = math.degrees(tilt_of(s.attitude))
            s.position[2] = 0.0
            if impact > p.crash_speed:
                self._crash(f"hit the ground at {impact:.1f} m/s")
            elif tilt > 45.0:
                self._crash(f"touched down tilted {tilt:.0f} degrees and tipped over")
            else:
                s.on_ground = True
                s.velocity = [0.0, 0.0, 0.0]
                s.rates = [0.0, 0.0, 0.0]
                s.accel_world = [0.0, 0.0, 0.0]

    def _crash(self, reason):
        s = self.state
        s.crashed = True
        s.crash_reason = reason
        s.velocity = [0.0, 0.0, 0.0]
        s.rates = [0.0, 0.0, 0.0]
        s.rotor_speed = [0.0] * 4

    # --------------------------------------------------------------- sensors
    def sensors(self) -> Sensors:
        s, p, env, rng = self.state, self.params, self.env, self.rng
        # Accelerometers measure specific force: acceleration minus gravity
        specific_force_world = _add(s.accel_world, [0.0, 0.0, env.gravity])
        accel_body = quat_rotate_inverse(s.attitude, specific_force_world)
        cos_tilt = max(0.2, math.cos(tilt_of(s.attitude)))
        battery = p.battery_full - (p.battery_full - p.battery_empty) * min(1.0, s.battery_used / p.battery_capacity)
        return Sensors(
            gyro=[w + rng.gauss(0.0, 0.003) for w in s.rates],
            accel=[a + rng.gauss(0.0, 0.05) for a in accel_body],
            attitude=list(s.attitude),
            position=[x + rng.gauss(0.0, 0.01) for x in s.position],
            velocity=[v + rng.gauss(0.0, 0.01) for v in s.velocity],
            range_to_ground=max(0.0, min(40.0, s.position[2] / cos_tilt + rng.gauss(0.0, 0.005))),
            battery_voltage=battery,
        )

    def hover_throttle(self) -> float:
        """Throttle at which the four motors together balance gravity."""
        p = self.params
        density_ratio = self.env.air_density / SEA_LEVEL_AIR_DENSITY
        return math.sqrt(p.mass * self.env.gravity / (4.0 * p.max_thrust * density_ratio))
