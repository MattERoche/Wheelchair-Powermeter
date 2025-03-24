#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ----------------------
// PIN and CONSTANT DEFINITIONS
// ----------------------
#define LED_PIN           LED_BUILTIN      // On-board LED pin
#define NEOPIXEL_PIN      6                // Adjust to your wiring
#define NEOPIXEL_COUNT    1

#define BUTTON_RECORD_PIN 5               // Record toggle button (active LOW)
#define BUTTON_ZERO_PIN   14               // (Defined but not used in this sketch)

#define SD_CS_PIN                         // SD card chip-select (adjust as needed)

// BLE UUIDs (as strings)
#define CSC_SERVICE_UUID         "1816"    // Cycling Speed and Cadence Service UUID
#define CSC_MEASUREMENT_CHAR_UUID "2A5B"    // CSC Measurement Characteristic UUID
#define POWER_SERVICE_UUID       "1818"    // Cycling Power Service UUID
#define POWER_MEASUREMENT_CHAR_UUID "2A63"   // Power Measurement Characteristic UUID

// Sampling intervals (in milliseconds)
const unsigned long CSC_SAMPLING_INTERVAL = 100;  // 0.1 sec
const unsigned long SENSOR_SAMPLING_INTERVAL = 1;   // 0.001 sec (note: extremely fast)

const float WHEEL_CIRCUMFERENCE = 2.105;

// ----------------------
// GLOBAL VARIABLES
// ----------------------
Adafruit_ICM20948 icm;
Adafruit_MPL3115A2 mpl3115;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// BLE client pointers
NimBLEClient* cscClient = nullptr;
NimBLEClient* powerMeterClient = nullptr;

// For CSC sensor
uint32_t previous_wheel_revolutions = 0;
uint16_t previous_event_time = 0;
float last_vSpeed = 0.0;
float last_rps = 0.0;
unsigned long last_CSC_update = 0;

// For power meter
float last_power = 0.0;
float last_cadence = 0.0;  // if you later parse cadence data

// Logging state and file variables
volatile int State_Record = 0;  // 0 = not logging, 1 = logging
bool previous_button_state = false;
File logFile;
String fileName = "";
unsigned long lastLogTime = 0;

// Timing variables for loop scheduling
unsigned long lastCSCUpdate = 0;
unsigned long lastSensorUpdate = 0;

// ----------------------
// BLE CONNECTION FUNCTIONS
// ----------------------
bool connectCSC() {
  Serial.println("Scanning for CSC sensor...");
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  NimBLEScanResults results;
  pScan->start(5, &results, false); // Increase scan duration to 5 seconds

  Serial.print("Found ");
  Serial.print(results.getCount());
  Serial.println(" devices.");

  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* device = results.getDevice(i);
    Serial.print("Device ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(device->getName().c_str());
    Serial.print(" [");
    Serial.print(device->getAddress().toString().c_str());
    Serial.println("]");

    // Print the advertised service UUID (if available)
    NimBLEUUID advService = device->getServiceUUID();
    Serial.print("  Service UUID: ");
    Serial.println(advService.toString().c_str());

    // Check if it advertises the CSC service (using 16-bit UUID 0x1816)
    NimBLEUUID cscUUID = NimBLEUUID((uint16_t)0x1816);
    if (device->isAdvertisingService(cscUUID)) {
      Serial.println("Found CSC sensor. Attempting to connect...");
      cscClient = NimBLEDevice::createClient();
      if (cscClient->connect(device)) {
         Serial.println("Connected to CSC sensor!");
         pScan->clearResults();
         return true;
      } else {
         Serial.println("Failed to connect to CSC sensor.");
      }
    }
  }
  pScan->clearResults();
  return false;
}

bool connectPowerMeter() {
  Serial.println("Scanning for Power Meter...");
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  NimBLEScanResults results; 
  pScan->start(2, &results, false); // pass pointer to results
  for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice* device = results.getDevice(i);
    if (device->isAdvertisingService(NimBLEUUID(POWER_SERVICE_UUID))) {
      Serial.println("Found Power Meter. Connecting...");
      powerMeterClient = NimBLEDevice::createClient();
      if (powerMeterClient->connect(device)) {
         Serial.println("Connected to Power Meter!");
         pScan->clearResults();
         return true;
      } else {
         Serial.println("Failed to connect to Power Meter.");
      }
    }
  }
  pScan->clearResults();
  return false;
}

void readCSCData() {
  if (cscClient && cscClient->isConnected()) {
    NimBLERemoteService* pService = cscClient->getService(NimBLEUUID(CSC_SERVICE_UUID));
    if (!pService) {
      Serial.println("CSC service not found.");
      return;
    }
    NimBLERemoteCharacteristic* pChar = pService->getCharacteristic(NimBLEUUID(CSC_MEASUREMENT_CHAR_UUID));
    if (!pChar) {
      Serial.println("CSC measurement characteristic not found.");
      return;
    }
    std::string value = pChar->readValue();
    if (value.length() < 7) return;  // expecting at least 7 bytes
    const uint8_t* data = (const uint8_t*)value.data();
    uint8_t flags = data[0];
    if (!(flags & 0x01)) return;  // no wheel revolution data present
    uint32_t wheel_revolutions = (uint32_t)data[1] |
                                 ((uint32_t)data[2] << 8) |
                                 ((uint32_t)data[3] << 16) |
                                 ((uint32_t)data[4] << 24);
    uint16_t event_time = data[5] | (data[6] << 8);
    // Calculate time difference (event_time units: 1/1024 sec)
    float time_diff = ((uint16_t)(event_time - previous_event_time)) / 1024.0;
    int rev_diff = (int)(wheel_revolutions - previous_wheel_revolutions);
    if (time_diff > 0 && rev_diff >= 0) {
      last_vSpeed = (WHEEL_CIRCUMFERENCE * rev_diff) / time_diff;
      last_rps = rev_diff / time_diff;
      previous_wheel_revolutions = wheel_revolutions;
      previous_event_time = event_time;
      last_CSC_update = millis();
      // Set Neopixel to blue-green (0,10,0)
      pixels.setPixelColor(0, pixels.Color(0, 10, 0));
      pixels.show();
    }
  }
  if (millis() - last_CSC_update > 1000) {  // if no update for >1 sec
    last_vSpeed = 0.0;
    last_rps = 0.0;
    pixels.setPixelColor(0, pixels.Color(10, 10, 0));
    pixels.show();
  }
}

void readPowerData() {
  if (powerMeterClient && powerMeterClient->isConnected()) {
    NimBLERemoteService* pService = powerMeterClient->getService(NimBLEUUID(POWER_SERVICE_UUID));
    if (!pService) {
      Serial.println("Power service not found.");
      return;
    }
    NimBLERemoteCharacteristic* pChar = pService->getCharacteristic(NimBLEUUID(POWER_MEASUREMENT_CHAR_UUID));
    if (!pChar) {
      Serial.println("Power measurement characteristic not found.");
      return;
    }
    std::string value = pChar->readValue();
    if (value.length() < 2) return;
    // Assume power is a 16-bit integer in little-endian
    uint16_t powerVal = (uint8_t)value[0] | ((uint8_t)value[1] << 8);
    last_power = powerVal;
    Serial.print("Power: ");
    Serial.print(last_power);
    Serial.println(" W");
  }
}

// ----------------------
// SENSOR FUNCTIONS
// ----------------------
float readPressureMS4525DO() {
  const uint8_t addr = 0x28;
  Wire.beginTransmission(addr);
  // Request 2 bytes
  if (Wire.requestFrom(addr, (uint8_t)2) == 2) {
    uint8_t data[2];
    data[0] = Wire.read();
    data[1] = Wire.read();
    Wire.endTransmission();
    uint16_t raw_pressure = ((data[0] & 0x3F) << 8) | data[1];
    float pressure_psi = ((raw_pressure - 8192) / 16384.0) * 15.0;
    float pressure_PA = (-1.0 * (pressure_psi * 6894.76)) - 284;
    return pressure_PA;
  } else {
    Serial.println("MS4525DO Read Error");
    Wire.endTransmission();
    return 0.0;
  }
}

float computeWindSpeed(float pressure_PA, float density) {
  if (pressure_PA >= 0 && density > 0) {
    return sqrt((2 * pressure_PA) / density);
  }
  return 0.0;
}

// ----------------------
// FILE LOGGING FUNCTIONS
// ----------------------
String getNextFileName() {
  File root = SD.open("/");
  uint32_t maxVal = 0;
  File entry = root.openNextFile();
  while (entry) {
    String name = entry.name();
    if (name.length() >= 10) {  // expecting "000001.csv"
      String numStr = name.substring(0, 6);
      uint32_t val = numStr.toInt();
      if (val > maxVal) {
        maxVal = val;
      }
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  char filename[13];
  sprintf(filename, "%06d.csv", maxVal + 1);
  return String(filename);
}

void startFile() {
  if (!logFile) {
    fileName = getNextFileName();
    Serial.print("New file created: ");
    Serial.println(fileName);
    logFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (logFile) {
      // Write header line matching the CSV format
      logFile.println("Time,A_X,A_Y,A_Z,G_X,G_Y,G_Z,Temperature,AirDensity,vSpeed,RPS,Delta_Pressure,wind_speed_ms,power,cadence");
      logFile.flush();
    } else {
      Serial.println("Failed to open log file!");
    }
  }
}

void stopFile() {
  if (logFile) {
    logFile.close();
    Serial.println("Logging stopped.");
  }
}

// ----------------------
// SETUP AND LOOP
// ----------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize LED and buttons
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_RECORD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_ZERO_PIN, INPUT_PULLUP);
  
  // Initialize Neopixel
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  // Initialize I2C and sensors
  Wire.begin();
  if (!icm.begin_I2C()) {
    Serial.println("ICM20948 not found. Check wiring!");
  }
  if (!mpl3115.begin()) {
    Serial.println("MPL3115A2 not found. Check wiring!");
  }
  
  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized.");
  }
  
  // Initialize BLE
  NimBLEDevice::init("ESP32_S3_Client");
  
  // Connect to BLE devices (blocking until connected)
  while (!connectCSC()) {
    Serial.println("Retrying CSC connection in 5 seconds...");
    delay(5000);
  }
  while (!connectPowerMeter()) {
    Serial.println("Retrying Power Meter connection in 2 seconds...");
    delay(2000);
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // LED recording indicator (simulate LED_Rec())
  if (State_Record == 1) {
    pixels.setPixelColor(0, pixels.Color(10, 0, 10));
    pixels.show();
    delay(1);
    pixels.setPixelColor(0, pixels.Color(10, 5, 10));
    pixels.show();
  }
  else if (cscClient && cscClient->isConnected() && powerMeterClient && powerMeterClient->isConnected() && State_Record == 0) {
    pixels.setPixelColor(0, pixels.Color(0, 10, 0));
    pixels.show();
  }
  
  // Periodically update BLE CSC and power data
  if (currentMillis - lastCSCUpdate >= CSC_SAMPLING_INTERVAL) {
    lastCSCUpdate = currentMillis;
    readCSCData();
    readPowerData();
  }
  
  // Sensor readings and CSV logging (using SENSOR_SAMPLING_INTERVAL)
  if (currentMillis - lastSensorUpdate >= SENSOR_SAMPLING_INTERVAL) {
    lastSensorUpdate = currentMillis;
    
    // Read ICM20948 data using the Adafruit Sensor API
    sensors_event_t a, g, dummy;
    icm.getEvent(&a, &g, &dummy);
    float A_X = a.acceleration.x;
    float A_Y = a.acceleration.y;
    float A_Z = a.acceleration.z;
    float G_X = g.gyro.x;
    float G_Y = g.gyro.y;
    float G_Z = g.gyro.z;
    
    // Read MPL3115A2 values (pressure in kPa and temperature in °C)
    float pressure = mpl3115.getPressure();    // in kPa
    float temperature = mpl3115.getTemperature();
    float pascals = pressure * 100;              // convert to Pa
    float density = pascals / ((temperature + 273.15) * 287.0);
    
    // Read pressure from MS4525DO sensor and compute wind speed
    float pressure_PA = readPressureMS4525DO();
    float wind_speed_ms = computeWindSpeed(pressure_PA, density);
    
    // Start logging if in recording state and file isn’t open
    if (State_Record == 1 && !logFile) {
      startFile();
    }
    
    if (logFile) {
      float time_sec = currentMillis / 1000.0;
      char buffer[200];
      // CSV format: Time,A_X,A_Y,A_Z,G_X,G_Y,G_Z,Temperature,AirDensity,vSpeed,RPS,Delta_Pressure,wind_speed_ms,power,cadence
      snprintf(buffer, sizeof(buffer), "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f",
               time_sec, A_X, A_Y, A_Z, G_X, G_Y, G_Z, temperature, density, last_vSpeed, last_rps, pressure_PA, wind_speed_ms, last_power, last_cadence);
      logFile.println(buffer);
      logFile.flush();
      lastLogTime = currentMillis;
    }
  }
  
  // Check the record button to toggle logging state (active LOW)
  bool buttonState = (digitalRead(BUTTON_RECORD_PIN) == LOW);
  if (buttonState && !previous_button_state) {
    State_Record = 1 - State_Record;
    Serial.print("Logging ");
    Serial.println(State_Record == 1 ? "Started" : "Stopped");
  }
  previous_button_state = buttonState;
  
  // If logging has been turned off and a file is open, close it.
  if (State_Record == 0 && logFile) {
    stopFile();
  }
  
  delay(1); // small delay to avoid hogging the CPU
}
