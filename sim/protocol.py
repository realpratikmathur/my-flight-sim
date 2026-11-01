"""Wire format shared with the flight software.

Every datagram is the F Prime serialization of a struct defined in
``MyFlightSim/Types/SimTypes.fpp``: fields in declaration order, big-endian,
no padding (FPP User's Guide, section 18.4 "Serialization of FPP Values").
If you change a struct in SimTypes.fpp, change the matching format here.
"""
import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

SIM_MAGIC = 0x5153494D  # "QSIM", constant SIM_MAGIC in SimTypes.fpp

# struct SensorFrame { header: SimHeader, sensors: SensorData }
#   SimHeader  : magic U32, tick U32, simTimeUs U64            -> I I Q
#   SensorData : gyro Vector3, accel Vector3, attitude Quaternion,
#                position Vector3, velocity Vector3,
#                rangeToGround F32, batteryVoltage F32         -> 3f 3f 4f 3f 3f f f
SENSOR_FRAME = struct.Struct(">IIQ" + "3f3f4f3f3fff")

# struct ActuatorFrame { header: SimHeader, motors: MotorThrottles ([4] F32) }
ACTUATOR_FRAME = struct.Struct(">IIQ4f")


@dataclass
class Sensors:
    """Mirror of struct SensorData in SimTypes.fpp."""
    gyro: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel: List[float] = field(default_factory=lambda: [0.0, 0.0, 9.81])
    attitude: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])  # w, x, y, z
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    range_to_ground: float = 0.0
    battery_voltage: float = 16.8


def pack_sensor_frame(tick: int, sim_time_us: int, s: Sensors) -> bytes:
    return SENSOR_FRAME.pack(SIM_MAGIC, tick, sim_time_us,
                             *s.gyro, *s.accel, *s.attitude, *s.position, *s.velocity,
                             s.range_to_ground, s.battery_voltage)


def unpack_actuator_frame(data: bytes) -> Optional[Tuple[int, int, List[float]]]:
    """Returns (tick, sim_time_us, [m0, m1, m2, m3]) or None if the datagram is invalid."""
    if len(data) != ACTUATOR_FRAME.size:
        return None
    magic, tick, sim_time_us, *motors = ACTUATOR_FRAME.unpack(data)
    if magic != SIM_MAGIC:
        return None
    return tick, sim_time_us, motors
