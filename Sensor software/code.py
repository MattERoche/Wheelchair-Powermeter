import time
import board
import busio
import adafruit_icm20x
import adafruit_mpl3115a2
import digitalio
import os
import neopixel
from micropython import const
import adafruit_ble
from adafruit_ble_cycling_speed_and_cadence import CyclingSpeedAndCadenceService
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.uuid import StandardUUID
from adafruit_ble.characteristics import Characteristic, ComplexCharacteristic
from adafruit_ble.characteristics.int import Uint8Characteristic
import ble_cycling_power_service


led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
pixels = neopixel.NeoPixel(board.NEOPIXEL, 1)

i2c = busio.I2C(board.SCL, board.SDA)
icm = adafruit_icm20x.ICM20948(i2c)
sensor = adafruit_mpl3115a2.MPL3115A2(i2c)

ble = adafruit_ble.BLERadio()
csc_connection = None
power_meter_connection = None
last_power_update = 0
last_power = 0
previous_crank_revs = 0
previous_crank_time = 0
last_cadence = 0.0


CYCLING_POWER_SERVICE_UUID = StandardUUID(0x1818)
CYCLING_POWER_MEASUREMENT_UUID = StandardUUID(0x2A63)

previous_wheel_revolutions = 0
previous_event_time = 0
WHEEL_CIRCUMFERENCE = 2.105

button = digitalio.DigitalInOut(board.D32)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

buttonZero = digitalio.DigitalInOut(board.D14)
buttonZero.direction = digitalio.Direction.INPUT
buttonZero.pull = digitalio.Pull.UP

State_Record = 0
previous_button_state = False
log_file = None
directory = "/sd"
file_name = None
# Sampling intervals
MAIN_SAMPLING_INTERVAL = 0.01  # 100Hz main loop
MPL3115_SAMPLING_INTERVAL = 1.0  # 1Hz for temperature/pressure
AIRSPEED_SAMPLING_INTERVAL = 0.02  # 50Hz for airspeed
BUFFER_SIZE = 50  # Number of samples to buffer before writing
last_CSC_update = 0
last_sensor_update = 0
CSC_SAMPLING_INTERVAL = 0.1
SENSOR_SAMPLING_INTERVAL = 0.001
Last_Log = 0

# Last update timestamps
last_mpl3115_update = 0
last_airspeed_update = 0

# Cached sensor values
cached_temperature = 0
cached_pressure = 0
cached_density = 0
cached_pressure_PA = 0
cached_wind_speed = 0

last_vSpeed = 0.0
last_rps = 0.0

class SensorData:
    def __init__(self):
        self.temperature = 0
        self.pressure = 0
        self.density = 0
        self.pressure_PA = 0
        self.wind_speed = 0
        self.accell = (0, 0, 0)
        self.gyro = (0, 0, 0)

sensor_data = SensorData()
sensor_buffer = []

def BLE_CSCConn():
    global csc_connection
    print("Scanning for CSC sensor...")
    while not csc_connection or not csc_connection.connected:
        pixels[0] = (10, 0, 0)
        for adv in ble.start_scan(ProvideServicesAdvertisement, timeout=2):
            if CyclingSpeedAndCadenceService in adv.services:
                print("Found CSC sensor. Connecting...")
                try:
                    csc_connection = ble.connect(adv)
                    print("Connected to CSC sensor!")
                    pixels[0] = (0, 0, 10)
                    ble.stop_scan()
                    csc_service = csc_connection[CyclingSpeedAndCadenceService]
                    return
                except Exception as e:
                    print("Connection failed:", e)
        print("CSC sensor not found, retrying...")
        ble.stop_scan()
        time.sleep(5)

def CSC_Data():
    global previous_event_time, previous_wheel_revolutions, last_vSpeed, last_rps, last_CSC_update
    if csc_connection and csc_connection.connected:
        try:
            csc_service = csc_connection[CyclingSpeedAndCadenceService]
            measurement_data = bytearray(7)
            csc_service.csc_measurement.readinto(measurement_data)
            flags = measurement_data[0]
            if not (flags & 0x01):
                return
            wheel_revolutions = int.from_bytes(measurement_data[1:5], "little")
            event_time = int.from_bytes(measurement_data[5:7], "little")
            time_diff = (event_time - previous_event_time) % 65536 / 1024.0
            rev_diff = wheel_revolutions - previous_wheel_revolutions
            if time_diff > 0 and rev_diff >= 0:
                last_vSpeed = (WHEEL_CIRCUMFERENCE * rev_diff) / time_diff
                last_rps = rev_diff / time_diff
                previous_wheel_revolutions = wheel_revolutions
                previous_event_time = event_time
                last_CSC_update = time.monotonic()
        except (IndexError, AttributeError, OSError) as e:
            print("CSC error:", e)
            if isinstance(e, OSError):
                csc_connection.disconnect()
                BLE_CSCConn()
    if time.monotonic() - last_CSC_update > 1.0:
        last_vSpeed = 0.0
        last_rps = 0.0


def BLE_PowerMeterConn():
    global power_meter_connection
    print("Scanning for Power Meter...")
    while not power_meter_connection or not power_meter_connection.connected:
        for adv in ble.start_scan(ProvideServicesAdvertisement, timeout=2):
            # Check for specific service class
            if ble_cycling_power_service.CyclingPowerService in adv.services:
                print("Found compatible power meter")
                try:
                    power_meter_connection = ble.connect(adv)
                    print("Power meter connected!")
                    return
                except Exception as e:
                    print("Connection failed:", e)
        print("Retrying...")
        time.sleep(2)


def Power_Data():
    global last_power, power_meter_connection

    if not power_meter_connection or not power_meter_connection.connected:
        return

    try:
        # Correct service access
        power_service = power_meter_connection[ble_cycling_power_service.CyclingPowerService]

        # Get power value
        current_power = power_service.power_Value
        if current_power is not None:
            last_power = current_power
            print(f"Power: {last_power}W")
        else:
            print("No power data received")

    except Exception as e:
        pixels = (10,0,0)
        print(f"Power data error: {e}")
        power_meter_connection = None


def get_inc_High_Val(directory):
    files = os.listdir(directory)
    high_Val = max((int(file[:6]) for file in files if file[:6].isdigit()), default=0)
    return '{:06d}.csv'.format(high_Val + 1)

def Start_file():
    global file_name, log_file
    if State_Record == 1 and log_file is None:
        file_name = get_inc_High_Val(directory)
        print(f"New file created: {file_name}")
        log_file = open(f"/sd/{file_name}", "a")
        headers(log_file)

def Stop_logging():
    global log_file
    if log_file:
        log_file.close()
        log_file = None
        print("Logging stopped.")

def headers(datalog):
    if datalog.tell() == 0:
        datalog.write('Time,A_X,A_Y,A_Z,G_X,G_Y,G_Z,Temperature,AirDensity,vSpeed,RPS,Delta_Pressure,wind_speed_ms,power,cadence\n')


def record():
    global State_Record, previous_button_state
    Button_Record = button.value
    if not Button_Record and not previous_button_state:
        State_Record = 1 - State_Record
        print(f"Logging {'Started' if State_Record else 'Stopped'}")
    previous_button_state = Button_Record

def LED_Rec():
    if State_Record ==1:
        pixels[0] = (10,0,10)
        time.sleep(1)
        pixels[0] = (10,5,10)
    elif csc_connection and power_meter_connection and State_Record ==0:
        pixels[0] = (0,10,0)

def read_pressure_ms4525do():
    try:
        while not i2c.try_lock():
            pass
        data = bytearray(2)
        i2c.readfrom_into(0x28, data)
        i2c.unlock()
        raw_pressure = ((data[0] & 0x3F) << 8) | data[1]
        pressure_psi = ((raw_pressure - 8192) / 16384) * 15
        pressure_PA = round(-1*((pressure_psi*6894.76)-284),1)
        return pressure_PA
    except Exception as e:
        print("MS4525DO Read Error:", e)
        return None

def wind_Speed():
    if pressure_PA >= 0:
        wind_speed_ms = ((2*pressure_PA)/density)**(1/2)
    else:
        wind_speed_ms = 0
    return wind_speed_ms


def update_mpl3115():
    """Update MPL3115 readings (1Hz)"""
    global cached_temperature, cached_pressure, cached_density
    try:
        cached_temperature = sensor.temperature
        cached_pressure = sensor.pressure
        pascals = cached_pressure * 100
        cached_density = pascals / ((cached_temperature + 273) * 287)
        return True
    except Exception as e:
        print("MPL3115 read error:", e)
        return False

def update_airspeed():
    """Update airspeed sensor readings (50Hz)"""
    global cached_pressure_PA, cached_wind_speed
    try:
        cached_pressure_PA = read_pressure_ms4525do()
        if cached_pressure_PA >= 0:
            cached_wind_speed = ((2 * cached_pressure_PA) / cached_density) ** (1/2)
        else:
            cached_wind_speed = 0
        return True
    except Exception as e:
        print("Airspeed sensor read error:", e)
        return False

def update_icm20x():
    """Update ICM20X readings (100Hz)"""
    try:
        sensor_data.accell = icm.acceleration
        sensor_data.gyro = icm.gyro
        return True
    except Exception as e:
        print("ICM20X read error:", e)
        return False

def write_buffer_to_file():
    """Write buffered data to file"""
    global sensor_buffer
    if log_file and sensor_buffer:
        try:
            log_file.write(''.join(sensor_buffer))
            sensor_buffer.clear()
        except OSError as e:
            print("File write error:", e)

# Modified main loop
next_sensor_update = time.monotonic()


BLE_CSCConn()
BLE_PowerMeterConn()

while True:
    current_time = time.monotonic()

    # Main 100Hz loop
    if current_time >= next_sensor_update:
        next_sensor_update = current_time + MAIN_SAMPLING_INTERVAL

        # Update ICM20X at 100Hz
        icm_updated = update_icm20x()

        # Update airspeed sensor at 50Hz
        if current_time - last_airspeed_update >= AIRSPEED_SAMPLING_INTERVAL:
            update_airspeed()
            last_airspeed_update = current_time

        # Update MPL3115 at 1Hz
        if current_time - last_mpl3115_update >= MPL3115_SAMPLING_INTERVAL:
            update_mpl3115()
            last_mpl3115_update = current_time

        # Log data at 100Hz if recording
        if State_Record and icm_updated:
            A_X, A_Y, A_Z = sensor_data.accell
            G_X, G_Y, G_Z = sensor_data.gyro

# Format data string with latest sensor values
            data_str = f'{current_time:.3f},{A_X:.2f},{A_Y:.2f},{A_Z:.2f},{G_X:.2f},{G_Y:.2f},{G_Z:.2f},{cached_temperature:.1f},{cached_density:.2f},{last_vSpeed:.2f},{last_rps:.2f},{cached_pressure_PA:.2f},{cached_wind_speed:.1f},{last_power:.2f},{last_cadence:.1f}\n'

            sensor_buffer.append(data_str)

            # Write buffer when full
            if len(sensor_buffer) >= BUFFER_SIZE:
                write_buffer_to_file()

    # BLE updates (keep existing BLE timing)
    if current_time - last_CSC_update >= CSC_SAMPLING_INTERVAL:
        CSC_Data()
        Power_Data()
        last_CSC_update = current_time

    # Button handling and LED updates
    record()A
    LED_Rec()

    # File management
    if State_Record and log_file is None:
        Start_file()
    elif State_Record == 0 and log_file:
        write_buffer_to_file()
        Stop_logging()


