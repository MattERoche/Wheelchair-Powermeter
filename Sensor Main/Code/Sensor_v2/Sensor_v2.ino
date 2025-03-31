// Summary:
// - Adds BLE connection to CSC (speed) and Power Meter (power/cadence)
// - Only sets STATUS_READY (green LED) when all sensors (I2C + BLE) are ready
// - Only allows recording once everything is connected

#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include "ms4525do.h"
#include <SPIFFS.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// BLE UUIDs
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_CHAR_UUID((uint16_t)0x2A5B);
static BLEUUID CYCLING_POWER_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CYCLING_POWER_MEASUREMENT_CHAR_UUID((uint16_t)0x2A63);

BLEScan* pBLEScan;
BLEAdvertisedDevice* foundCSCDevice = nullptr;
BLEAdvertisedDevice* foundPowerDevice = nullptr;
BLEClient* pClient = nullptr;
float currentSpeedKph = 0.0f;
float currentPower = 0.0f;
uint32_t lastWheelRevs = 0;
uint16_t lastWheelEventTime = 0;
unsigned long lastNonZeroRevTime = 0;
float lastPressure = 0.0f;
float lastTemp = 0.0f;
float lastAirspeed = 0.0f;
float lastAirTemp = 0.0f;

bool cscConnected = false;
bool powerConnected = false;
bool icmReady = false;
bool mplReady = false;
bool ms4525Ready = false;

// Sensor Objects
Adafruit_ICM20948 icm;
Adafruit_MPL3115A2 mpl;
bfs::Ms4525do ms4525do(&Wire, 0x28, 1.0f, -1.0f);
String filename = "/log.csv";

#define MODE_PIN 11
#define RECORD_PIN 5
#define NEOPIXEL_PIN 33
#define NEOPIXEL_COUNT 1
#define NEOPIXEL_PWR_PIN 21

Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

enum SystemStatus {
  STATUS_ERROR,
  STATUS_CONNECTING,
  STATUS_READY,
  STATUS_RECORDING
};

bool headerWritten = false;
bool recordingStarted = false;
SystemStatus lastStatus = STATUS_CONNECTING;

// LED Utils
void setPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}
void updateLEDStatus(SystemStatus status) {
  if (status == lastStatus && status != STATUS_RECORDING) return;
  lastStatus = status;
  switch (status) {
    case STATUS_ERROR:
      Serial.println("🔴 STATUS_ERROR");
      setPixelColor(255, 0, 0);
      break;
    case STATUS_CONNECTING:
      Serial.println("🔵 STATUS_CONNECTING");
      setPixelColor(0, 0, 255);
      break;
    case STATUS_READY:
      Serial.println("🟢 STATUS_READY");
      setPixelColor(0, 255, 0);
      break;
    case STATUS_RECORDING:
      setPixelColor(128, 0, 128);
      break;
  }
}
void logToCSV(const String &line) {
  Serial.print("Writing to: ");
  Serial.println(filename);  // Should say "/log.csv"

  File file = SPIFFS.open(filename, FILE_APPEND);
  if (!file) {
    Serial.println("❌ Failed to open log file!");
    updateLEDStatus(STATUS_ERROR);
    return;
  }

  if (!headerWritten) {
    Serial.println("✅ Writing header");

    headerWritten = true;

  }
  
  file.println(line);
  file.flush(); // optional but may help with persistence
  file.close();
}

// ====== Dump Logs ======
void dumpCSVOverSerial() {
  File file = SPIFFS.open("/log.csv", "r");
  if (!file) {
    Serial.println("No log file found.");
    return;
  }

  Serial.println("\n\n\n==================== CSV LOG DUMP ====================");
  Serial.println("IF YOU SEE THIS, DATA IS STARTING BELOW");
  Serial.println("=====================================================");

  while (file.available()) {
    Serial.write(file.read());
  }
  Serial.println("===== END CSV DUMP =====");
  file.close();

  while (true) delay(100);
}

void deleteAllLogFiles() {
  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open root directory.");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String filename = file.name();
    if (filename.endsWith(".csv")) {
      Serial.print("Deleting: ");
      Serial.println(filename);
      SPIFFS.remove(filename);
    }
    file = root.openNextFile();
  }
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID()) {
      if (advertisedDevice.isAdvertisingService(CSC_SERVICE_UUID)) {
        Serial.println("🚲 CSC device found!");
        foundCSCDevice = new BLEAdvertisedDevice(advertisedDevice);
        pBLEScan->stop();
      } else if (advertisedDevice.isAdvertisingService(CYCLING_POWER_SERVICE_UUID)) {
        Serial.println("⚡ Power Meter found!");
        foundPowerDevice = new BLEAdvertisedDevice(advertisedDevice);
        pBLEScan->stop();
      }
    }
  }
};  // ← this closing brace and semicolon were missing!


void powerNotifyCallback(BLERemoteCharacteristic* pCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  
  if (length < 4) return;

  uint16_t flags = pData[0] | (pData[1] << 8);
  int16_t currentPower = pData[2] | (pData[3] << 8); // signed!

}

void notifyCallback(BLERemoteCharacteristic* pCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

  uint8_t flags = pData[0];
  bool wheelDataPresent = flags & 0x01;
  bool crankDataPresent = flags & 0x02;

  int index = 1;

  static uint32_t lastWheelRevs = 0;
  static uint16_t lastWheelEventTime = 0;
  static unsigned long lastNonZeroRevTime = 0;


  if (wheelDataPresent && length >= index + 6) {
    uint32_t wheelRevs = pData[index] |
                         (pData[index + 1] << 8) |
                         (pData[index + 2] << 16) |
                         (pData[index + 3] << 24);
    uint16_t wheelEventTime = pData[index + 4] | (pData[index + 5] << 8);
    index += 6;

    uint32_t revDelta = wheelRevs - lastWheelRevs;
    uint16_t timeDelta = wheelEventTime - lastWheelEventTime;

    if (revDelta > 0 && timeDelta > 0) {
      float timeSeconds = timeDelta / 1024.0f;
      float wheelCircumference = 2.105f; // meters (adjust to your tire size)
      float speed_mps = (revDelta * wheelCircumference) / timeSeconds;
      currentSpeedKph = speed_mps * 3.6;

      lastNonZeroRevTime = millis();

    } else {
      unsigned long now = millis();
      if (now - lastNonZeroRevTime > 500) {
        currentSpeedKph = 0.0f;
      }
    }

    lastWheelRevs = wheelRevs;
    lastWheelEventTime = wheelEventTime;
  }
}

void setup() {
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(RECORD_PIN, INPUT_PULLUP);
  pinMode(NEOPIXEL_PWR_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_PWR_PIN, HIGH);
  Serial.begin(115200);
  delay(1000);
  pixel.begin();
  pixel.setBrightness(50);
  updateLEDStatus(STATUS_CONNECTING);

  Serial.println("🚲 BLE CSC Tracker Starting...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);

  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS mount failed");
    updateLEDStatus(STATUS_ERROR);
    while (1) delay(10);
  }

  if (digitalRead(MODE_PIN) == LOW) dumpCSVOverSerial();

  Wire.begin();
  Wire.setClock(100000);

  if (!icm.begin_I2C()) {
    Serial.println("❌ ICM20948 not found");
    updateLEDStatus(STATUS_ERROR);
    while (1);
  }
  icm.setAccelRateDivisor(0);
  icm.setGyroRateDivisor(0);
  icmReady = true;
  Serial.println("✅ ICM20948 ready");

  if (!mpl.begin()) {
    Serial.println("❌ MPL3115A2 not found");
    updateLEDStatus(STATUS_ERROR);
    while (1);
  }
  mplReady = true;
  Serial.println("✅ MPL3115A2 ready");

  if (!ms4525do.Begin()) {
    Serial.println("❌ MS4525DO not found");
    updateLEDStatus(STATUS_ERROR);
    while (1);
  } else {
    ms4525Ready = true;
    Serial.println("✅ MS4525DO ready");
  }
}

void loop() {
  static unsigned long lastBLEScan = 0;
  static unsigned long lastLogMicros = 0;
  const unsigned long LOG_INTERVAL_US = 10000; // 10ms for 100Hz
  unsigned long nowMicros = micros();

  static unsigned long loopStart = 0;
  unsigned long loopNow = micros();


  // === BLE Scan every 5s if needed ===
  if ((millis() - lastBLEScan > 5000) && (!cscConnected || !powerConnected)) {
    Serial.println("🔎 Scanning for BLE devices...");
    pBLEScan->start(3, false);  // short scan, non-blocking
    pBLEScan->clearResults();
    lastBLEScan = millis();
  }

  // === Deferred BLE connect ===
  static bool pendingCSCConnect = false;
  static bool pendingPowerConnect = false;

  if (!cscConnected && foundCSCDevice && !pendingCSCConnect) {
    pendingCSCConnect = true;
  }

  if (!powerConnected && foundPowerDevice && !pendingPowerConnect) {
    pendingPowerConnect = true;
  }

  // === Button & Ready Check ===
  bool allSensorsReady = icmReady && mplReady && ms4525Ready;
  bool allBleReady = cscConnected && powerConnected;

  static bool buttonPressed = false;
  int buttonState = digitalRead(RECORD_PIN);

  if (!recordingStarted) {
    if (!buttonPressed && buttonState == LOW) {
      buttonPressed = true;
      Serial.println("🟡 Record button pressed...");
    }

    if (buttonPressed && buttonState == HIGH && allSensorsReady && allBleReady) {
      buttonPressed = false;
      SPIFFS.remove("/log.csv");
      headerWritten = false;
      recordingStarted = true;
      Serial.println("📦 Recording started");
    }
  }

  if (allSensorsReady && allBleReady && !recordingStarted) updateLEDStatus(STATUS_READY);
  if (recordingStarted) updateLEDStatus(STATUS_RECORDING);

  // === Logging block (runs every 10ms if ready) ===
  if (recordingStarted && (nowMicros - lastLogMicros >= LOG_INTERVAL_US)) {
    lastLogMicros = nowMicros;
    float timeSec = nowMicros / 1000000.0f;

    sensors_event_t a, g, t;
    icm.getEvent(&a, &g, &t);

    // Read and store current values from MPL3115A2 and MS4525DO
    lastPressure = mpl.getPressure();
    lastTemp = mpl.getTemperature();
    ms4525do.Read();
    lastAirspeed = ms4525do.pres_pa();
    lastAirTemp = ms4525do.die_temp_c();

    String csv = String(timeSec, 3) + "," +
                 String(a.acceleration.x, 2) + "," +
                 String(a.acceleration.y, 2) + "," +
                 String(a.acceleration.z, 2) + "," +
                 String(lastPressure, 1) + "," +
                 String(lastTemp, 1) + "," +
                 String(lastAirspeed, 1) + "," +
                 String(lastAirTemp, 1) + "," +
                 String(currentSpeedKph, 2) + "," +
                 String(currentPower, 0);

    File file = SPIFFS.open(filename, FILE_APPEND);
    if (!file) {
      Serial.println("❌ Failed to open log file");
      updateLEDStatus(STATUS_ERROR);
      return;
    }

    if (!headerWritten) {
      file.println("Time (s),AccelX,AccelY,AccelZ,Pressure (Pa),Temp (C),Airspeed (Pa),AirspeedTemp (C),Speed (kph),Power (W)");
      headerWritten = true;
    }

    file.println(csv);
    file.flush();
    file.close();
    Serial.println(csv);
  }

  // === Perform deferred BLE connections (after logging) ===
  if (pendingCSCConnect) {
    Serial.print("🔗 Connecting to CSC: ");
    Serial.println(foundCSCDevice->getAddress().toString().c_str());

    BLEClient* cscClient = BLEDevice::createClient();
    if (cscClient->connect(foundCSCDevice)) {
      BLERemoteService* service = cscClient->getService(CSC_SERVICE_UUID);
      if (service) {
        BLERemoteCharacteristic* charac = service->getCharacteristic(CSC_MEASUREMENT_CHAR_UUID);
        if (charac && charac->canNotify()) {
          charac->registerForNotify(notifyCallback);
          Serial.println("✅ Subscribed to CSC notifications!");
          cscConnected = true;
        }
      }
    } else {
      Serial.println("❌ CSC connection failed");
    }

    delete foundCSCDevice;
    foundCSCDevice = nullptr;
    pendingCSCConnect = false;
  }

  if (pendingPowerConnect) {
    Serial.print("🔗 Connecting to Power Meter: ");
    Serial.println(foundPowerDevice->getAddress().toString().c_str());

    BLEClient* powerClient = BLEDevice::createClient();
    if (powerClient->connect(foundPowerDevice)) {
      BLERemoteService* service = powerClient->getService(CYCLING_POWER_SERVICE_UUID);
      if (service) {
        BLERemoteCharacteristic* charac = service->getCharacteristic(CYCLING_POWER_MEASUREMENT_CHAR_UUID);
        if (charac && charac->canNotify()) {
          charac->registerForNotify(powerNotifyCallback);
          Serial.println("✅ Subscribed to Power Meter notifications!");
          powerConnected = true;
        }
      }
    } else {
      Serial.println("❌ Power connection failed");
    }

    delete foundPowerDevice;
    foundPowerDevice = nullptr;
    pendingPowerConnect = false;
  }

  Serial.print("Loop interval (us): ");
  Serial.println(loopNow - loopStart);
  loopStart = loopNow;
}

