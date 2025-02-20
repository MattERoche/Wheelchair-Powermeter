import struct
import _bleio
from collections import namedtuple
from adafruit_ble.services import Service
from adafruit_ble.uuid import StandardUUID
from adafruit_ble.characteristics import Characteristic, ComplexCharacteristic
from adafruit_ble.characteristics.int import Uint32Characteristic, Uint16Characteristic

try:
    from typing import Optional, NamedTuple
except ImportError:
    pass


CyclingPowerMeasurementValues = namedtuple(
    "CyclingPowerMeasurementValues",
    (
    "flags",
    "instantaneous_power",
    "pedal_power_balance",
    "accumulated_torque",
    "wheel_revs",
    "wheel_time",
    "crank_revs",
    "crank_time",
    ),
)

class _CyclingPowerMeasurement(ComplexCharacteristic):
    def bind(self, service: "CyclingPowerService") -> _bleio.PacketBuffer:
        bound_characteristic = super().bind(service)
        uuid = StandardUUID(0x2A63)
        # Ensure notifications are enabled
        bound_characteristic.set_cccd(notify=True)
        print(f"Notifications enabled: {bound_characteristic._cccd.notify}")  # DEBUG
        return _bleio.PacketBuffer(bound_characteristic, buffer_size=1)

class CyclingPowerService(Service):
    uuid = StandardUUID(0x1818)

    # Correct characteristic declaration
    measurement = _CyclingPowerMeasurement()

    def __init__(self, service: Optional["CyclingPowerService"] = None) -> None:
        super().__init__(service=service)
        self._measurement_buf = bytearray(20)  # Pre-allocate buffer

    @property
    def values(self) -> Optional[CyclingPowerMeasurementValues]:
        try:
            # Use fixed buffer instead of dynamic allocation
            bytes_read = self.measurement.readinto(self._measurement_buf)
            if bytes_read < 4:  # Minimum valid packet size
                return None

            buf = self._measurement_buf
            offset = 0

            # Correct flag parsing (16-bit little-endian)
            flags = struct.unpack_from("<H", buf, offset)[0]
            offset += 2

            # Mandatory: Instantaneous Power (sint16)
            instantaneous_power = struct.unpack_from("<h", buf, offset)[0]
            offset += 2
            print(f"[POWER] Base Power: {instantaneous_power}W")

            # Parse optional fields based on flags
            pedal_power_balance = None
            accumulated_torque = None
            wheel_revs = None
            wheel_time = None
            crank_revs = None
            crank_time = None

            # Pedal Power Balance (bit 0)
            if flags & 0x0001:
                pedal_power_balance = buf[offset]
                offset += 1
                print(f"[POWER] Pedal Balance: {pedal_power_balance}%")

            # Accumulated Torque (bit 2)
            if flags & 0x0004:
                accumulated_torque = struct.unpack_from("<H", buf, offset)[0]
                offset += 2
                print(f"[POWER] Torque: {accumulated_torque} Nm")

            # Wheel Revolution Data (bit 4)
            if flags & 0x0010:
                wheel_revs = struct.unpack_from("<L", buf, offset)[0]
                wheel_time = struct.unpack_from("<H", buf, offset + 4)[0]
                offset += 6
                print(f"[POWER] Wheel Revs: {wheel_revs} Time: {wheel_time/1024}s")

            # Crank Revolution Data (bit 5)
            if flags & 0x0020:
                crank_revs = struct.unpack_from("<H", buf, offset)[0]
                crank_time = struct.unpack_from("<H", buf, offset + 2)[0]
                offset += 4
                print(f"[POWER] Crank Revs: {crank_revs} Time: {crank_time/1024}s")

            return CyclingPowerMeasurementValues(
                flags=flags,
                instantaneous_power=instantaneous_power,
                pedal_power_balance=pedal_power_balance,
                accumulated_torque=accumulated_torque,
                wheel_revs=wheel_revs,
                wheel_time=wheel_time,
                crank_revs=crank_revs,
                crank_time=crank_time
            )

        except Exception as e:
            print(f"[POWER] Parsing Error: {e}")
            return None
