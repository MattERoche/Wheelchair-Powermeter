import time
import board
import busio
import adafruit_icm20x
import adafruit_mpl3115a2
import digitalio
import os
import neopixel
from micropython import const
import gc
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

last_CSC_update = 0
last_sensor_update = 0
CSC_SAMPLING_INTERVAL = 0.1
SENSOR_SAMPLING_INTERVAL = 0.001
Last_Log = 0

# Sampling intervals
MAIN_SAMPLING_INTERVAL = 0.01  # 100Hz main loop
MPL3115_SAMPLING_INTERVAL = 1.0  # 1Hz for temperature/pressure
AIRSPEED_SAMPLING_INTERVAL = 0.02  # 50Hz for airspeed
BUFFER_SIZE = 50  # Number of samples to buffer before writing

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


# BLE Constants
MAX_CONNECTION_ATTEMPTS = 3
RECONNECT_DELAY = 5
BLE_RETRY_DELAY = 2
SCAN_TIMEOUT = const(2)

def cleanup_ble():
    """Clean up BLE connections and force garbage collection"""
    global csc_connection, power_meter_connection
    try:
        if csc_connection and csc_connection.connected:
            csc_connection.disconnect()
        if power_meter_connection and power_meter_connection.connected:
            power_meter_connection.disconnect()
    except Exception as e:
        print("Cleanup error:", e)
    finally:
        csc_connection = None
        power_meter_connection = None
        gc.collect()  # Force garbage collection

def BLE_CSCConn():
    """Connect to CSC sensor with improved memory management"""
    global csc_connection

    attempt_count = 0
    while attempt_count < MAX_CONNECTION_ATTEMPTS:
        try:
            if csc_connection and csc_connection.connected:
                return True

            pixels[0] = (10, 0, 0)
            print(f"Scanning for CSC sensor... (Attempt {attempt_count + 1})")

            # Clear any existing connection
            if csc_connection:
                try:
                    csc_connection.disconnect()
                except:
                    pass
                csc_connection = None
                gc.collect()

            for adv in ble.start_scan(ProvideServicesAdvertisement, timeout=SCAN_TIMEOUT):
                if CyclingSpeedAndCadenceService in adv.services:
                    print("Found CSC sensor. Connecting...")

                    # Stop scan before connecting
                    ble.stop_scan()

                    try:
                        csc_connection = ble.connect(adv)
                        print("Connected to CSC sensor!")
                        pixels[0] = (0, 0, 10)
                        return True
                    except Exception as e:
                        print("Connection failed:", e)
                        time.sleep(BLE_RETRY_DELAY)
                        break

            ble.stop_scan()
            attempt_count += 1

            if attempt_count < MAX_CONNECTION_ATTEMPTS:
                print(f"CSC sensor not found, retrying in {RECONNECT_DELAY} seconds...")
                pixels[0] = (10, 10, 0)
                time.sleep(RECONNECT_DELAY)
                gc.collect()  # Force garbage collection between attempts

        except Exception as e:
            print("BLE CSC Error:", e)
            cleanup_ble()
            attempt_count += 1
            time.sleep(RECONNECT_DELAY)

    return False

def BLE_PowerMeterConn():
    """Connect to Power Meter with improved memory management"""
    global power_meter_connection

    attempt_count = 0
    while attempt_count < MAX_CONNECTION_ATTEMPTS:
        try:
            if power_meter_connection and power_meter_connection.connected:
                return True

            print(f"Scanning for Power Meter... (Attempt {attempt_count + 1})")

            # Clear any existing connection
            if power_meter_connection:
                try:
                    power_meter_connection.disconnect()
                except:
                    pass
                power_meter_connection = None
                gc.collect()

            for adv in ble.start_scan(ProvideServicesAdvertisement, timeout=SCAN_TIMEOUT):
                if ble_cycling_power_service.CyclingPowerService in adv.services:
                    print("Found compatible power meter")

                    # Stop scan before connecting
                    ble.stop_scan()

                    try:
                        power_meter_connection = ble.connect(adv)
                        print("Power meter connected!")
                        return True
                    except Exception as e:
                        print("Connection failed:", e)
                        time.sleep(BLE_RETRY_DELAY)
                        break

            ble.stop_scan()
            attempt_count += 1

            if attempt_count < MAX_CONNECTION_ATTEMPTS:
                print(f"Retrying in {RECONNECT_DELAY} seconds...")
                time.sleep(RECONNECT_DELAY)
                gc.collect()  # Force garbage collection between attempts

        except Exception as e:
            print("BLE Power Meter Error:", e)
            cleanup_ble()
            attempt_count += 1
            time.sleep(RECONNECT_DELAY)

    return False

def CSC_Data():
    """Read CSC data with memory management"""
    global previous_event_time, previous_wheel_revolutions, last_vSpeed, last_rps, last_CSC_update

    if not csc_connection or not csc_connection.connected:
        return

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
            pixels[0] = (0, 10, 0)

    except (IndexError, AttributeError, OSError) as e:
        print("CSC error:", e)
        if isinstance(e, OSError):
            cleanup_ble()
            BLE_CSCConn()

def Power_Data():
    """Read Power data with memory management"""
    global last_power, power_meter_connection

    if not power_meter_connection or not power_meter_connection.connected:
        return

    try:
        power_service = power_meter_connection[ble_cycling_power_service.CyclingPowerService]
        current_power = power_service.power_Value

        if current_power is not None:
            last_power = current_power

    except Exception as e:
        print(f"Power data error: {e}")
        cleanup_ble()
        power_meter_connection = None

# Modified main loop BLE handling
def handle_ble_operations(current_time):
    """Handle BLE operations with memory management"""
    global last_CSC_update

    if current_time - last_CSC_update >= CSC_SAMPLING_INTERVAL:
        # Check connections and reconnect if needed
        if not csc_connection or not csc_connection.connected:
            BLE_CSCConn()
        if not power_meter_connection or not power_meter_connection.connected:
            BLE_PowerMeterConn()

        # Read data if connected
        if csc_connection and csc_connection.connected:
            CSC_Data()
        if power_meter_connection and power_meter_connection.connected:
            Power_Data()

        last_CSC_update = current_time



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
        pressure_PA = round(((-1*(pressure_psi*6894.76))-284),0)
        return pressure_PA
    except Exception as e:
        print("MS4525DO Read Error:", e)
        return None

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

def wind_Speed():
    if pressure_PA >= 0:
        wind_speed_ms = ((2*pressure_PA)/density)**(1/2)
    else:
        wind_speed_ms = 0
    return wind_speed_ms

BLE_CSCConn()
BLE_PowerMeterConn()

next_sensor_update = time.monotonic()

while True:
    current_time = time.monotonic()
    handle_ble_operations(current_time)

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
            data_str = (
                f'{current_time:.3f},{A_X:.2f},{A_Y:.2f},{A_Z:.2f},'
                f'{G_X:.2f},{G_Y:.2f},{G_Z:.2f},{cached_temperature:.1f},'
                f'{cached_density:.2f},{last_vSpeed:.2f},{last_rps:.2f},'
                f'{cached_pressure_PA:.2f},{cached_wind_speed:.1f},'
                f'{last_power:.2f},{last_cadence:.1f}\n'
            )

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
    record()
    LED_Rec()

    # File management
    if State_Record and log_file is None:
        Start_file()
    elif State_Record == 0 and log_file:
        write_buffer_to_file()
        Stop_logging()

