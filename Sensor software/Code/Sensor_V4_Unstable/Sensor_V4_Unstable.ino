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
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <math.h> // Include for isnan, sqrtf

// =============================================================
// ==                     Configuration                     ==
// =============================================================

// --- Pin Definitions ---
#define MODE_PIN 6
#define RECORD_PIN 5
#define NEOPIXEL_PIN 33
#define NEOPIXEL_PWR_PIN 21
#define TFT_CS    10
#define TFT_DC    12
#define TFT_RST   11
#define TFT_BLK   13

// --- NeoPixel Settings ---
#define NEOPIXEL_COUNT 1
#define NEOPIXEL_BRIGHTNESS 50

// --- I2C Settings ---
#define I2C_CLOCK_SPEED 400000

// --- Sensor Settings ---
#define ICM_ACCEL_RATE_DIV 10
#define ICM_GYRO_RATE_DIV 10
#define MPL_OSR_SETTING 0
#define AIR_DENSITY 1.22f // kg/m^3

// --- Logging Settings ---
const char* LOG_FILENAME = "/log.csv";
const unsigned long LOG_INTERVAL_MS = 10;
const unsigned long FLUSH_INTERVAL_MS = 1000;
const size_t MAX_LOG_BUFFER_SIZE = 1024;
const int CSV_PRECISION_ACCEL = 2;
const int CSV_PRECISION_GYRO = 2;
const int CSV_PRECISION_PRESSURE = 1; // For Ambient Pressure (MPL)
const int CSV_PRECISION_TEMP = 1;
const int CSV_PRECISION_AIRSPEED = 1; // Precision for Raw DeltaP (MS4525)
const int CSV_PRECISION_SPEED_KPH = 2; // Precision for Wheel Speed
const int CSV_PRECISION_POWER = 0;
const int CSV_PRECISION_WINDSPEED = 2; // Precision for Calculated Airspeed
const int CSV_BUFFER_LINE_LENGTH = 220;
const int CSV_PRECISION_CADENCE = 2;   // Precision for Calculated Cadence (rad/s)

// --- BLE Settings ---
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_CHAR_UUID((uint16_t)0x2A5B);
static BLEUUID CYCLING_POWER_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CYCLING_POWER_MEASUREMENT_CHAR_UUID((uint16_t)0x2A63);
const unsigned long BLE_SCAN_INTERVAL_MS = 100;
const unsigned long BLE_SCAN_WINDOW_MS = 99;
const unsigned long BLE_SCAN_DURATION_S = 3;
const unsigned long BLE_CONNECT_RETRY_INTERVAL_MS = 5000;
const float WHEEL_CIRCUMFERENCE_METERS = 2.105f;
const unsigned long BLE_SPEED_TIMEOUT_MS = 3000;

// --- Display Settings ---
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500;

// --- Optional Debug Flag ---
// #define ENABLE_VERBOSE_DEBUG // Uncomment for more serial output (may cause instability)

// =============================================================
// ==                  Global Variables                     ==
// =============================================================

// --- System State ---
enum SystemStatus { STATUS_ERROR, STATUS_CONNECTING, STATUS_READY, STATUS_RECORDING };
SystemStatus currentStatus = STATUS_CONNECTING;
SystemStatus lastDisplayedStatus = STATUS_ERROR;
bool recordingStarted = false;

// --- BLE State ---
BLEScan* pBLEScan = nullptr;
BLEClient* cscClient = nullptr;
BLEClient* powerClient = nullptr;
BLEAdvertisedDevice* foundCSCDevice = nullptr;
BLEAdvertisedDevice* foundPowerDevice = nullptr;
bool cscConnected = false;
bool powerConnected = false;
bool bleInitAttempted = false;
static bool bleScanInProgress = false;

// --- Sensor Raw Data ---
float currentSpeedKph = 0.0f; // From BLE CSC
float currentPower = 0.0f;    // From BLE Power
float lastPressure = 0.0f;    // Ambient Pressure (MPL3115A2)
float lastTemp = 0.0f;        // Ambient Temp (MPL3115A2)
float DeltaAirPress = 0.0f;   // Raw Delta Pressure (MS4525DO)
float lastAirTemp = 0.0f;     // Die Temp (MS4525DO)
sensors_event_t accel, gyro, temp_imu; // Structures for IMU data (Raw values)

// --- Speed Calculation ---
uint32_t lastWheelRevs = 0;
uint16_t lastWheelEventTime = 0;
unsigned long lastNonZeroRevTime = 0;

uint16_t lastCrankRevs = 0;
uint16_t lastCrankEventTime = 0; // 1/1024s
unsigned long lastNonZeroCrankTime = 0; // ms
float currentCadenceRadPerSec = 0.0f; // Calculated cadence in rad/s
const unsigned long BLE_CADENCE_TIMEOUT_MS = 3000; // Timeout for cadence -> 0

// --- Sensor Readiness ---
bool icmReady = false;
bool mplReady = false;
bool ms4525Ready = false;

// --- Sensor Offsets (Zeroing) --- REMOVED ---

// --- Display Object ---
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Sensor Objects ---
Adafruit_ICM20948 icm;
Adafruit_MPL3115A2 mpl;
bfs::Ms4525do ms4525do(&Wire, 0x28, 1.0f, -1.0f);

// --- NeoPixel Object ---
Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- Logging & Timing ---
static String logBuffer = "";
static unsigned long lastFlushMillis = 0;
static unsigned long lastLogMillis = 0;
static unsigned long lastDisplayUpdateMillis = 0;

// =============================================================
// ==            Forward Declarations & Constants           ==
// =============================================================
void updateLEDStatus(SystemStatus status, bool forceUpdate = false);
void dumpCSVOverSerial();
void connectBLEDevices();
void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
void powerNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
void flushLogBuffer();
bool setMplOsr(uint8_t osr);
bool setupCSCNotifications(BLEClient* client);
bool setupPowerNotifications(BLEClient* client);
float airspeedPaToKph(float pressure_pa);
void initDisplay();
void updateDisplay(float currentRawDeltaP); // Pass RAW pressure for display calculation

#ifndef MPL3115A2_ADDRESS
#define MPL3115A2_ADDRESS (0x60)
#endif
#ifndef MPL3115A2_CTRL_REG1
#define MPL3115A2_CTRL_REG1 (0x26)
#endif

// =============================================================
// ==             BLE Callback Class Definition             ==
// =============================================================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
     bool changed = false;
     if (!cscConnected && foundCSCDevice == nullptr && advertisedDevice.isAdvertisingService(CSC_SERVICE_UUID)) {
        #ifdef ENABLE_VERBOSE_DEBUG
        Serial.println("VERBOSE: 🚲 CSC candidate found");
        #endif
        foundCSCDevice = new BLEAdvertisedDevice(advertisedDevice);
        changed = true;
      }
      else if (!powerConnected && foundPowerDevice == nullptr && advertisedDevice.isAdvertisingService(CYCLING_POWER_SERVICE_UUID)) {
        #ifdef ENABLE_VERBOSE_DEBUG
        Serial.println("VERBOSE: ⚡ Power candidate found");
        #endif
        foundPowerDevice = new BLEAdvertisedDevice(advertisedDevice);
        changed = true;
      }

    bool gotCscCandidate = cscConnected || (foundCSCDevice != nullptr);
    bool gotPowerCandidate = powerConnected || (foundPowerDevice != nullptr);
    if (changed && gotCscCandidate && gotPowerCandidate) {
      if(pBLEScan != nullptr && bleScanInProgress) {
           Serial.println("Found candidates for all needed devices. Stopping scan.");
           pBLEScan->stop();
           bleScanInProgress = false;
      }
    }
  }
};

// =============================================================
// ==                     Setup Function                      ==
// =============================================================
void setup() {
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(RECORD_PIN, INPUT_PULLUP);
  pinMode(NEOPIXEL_PWR_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_PWR_PIN, HIGH);

  Serial.begin(115200);
  unsigned long bootStart = millis();
  while (!Serial && (millis() - bootStart < 2000)) {;}
  Serial.println("\n\n--- System Boot ---");

  pixel.begin();
  pixel.setBrightness(NEOPIXEL_BRIGHTNESS);
  updateLEDStatus(STATUS_CONNECTING, true);

  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS mount failed!");
    updateLEDStatus(STATUS_ERROR, true);
    while (1) delay(10);
  }
  Serial.println("✅ SPIFFS Mounted.");
  if (digitalRead(MODE_PIN) == LOW) {
    Serial.println(" DUMP MODE DETECTED ");
    dumpCSVOverSerial();
  }

  initDisplay();
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 5);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.println("Booting...");

  Serial.println("Initializing I2C Bus...");
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  Serial.println("Initializing I2C Sensors...");
  if (!icm.begin_I2C()) {
    Serial.println("❌ ICM20948 not found"); updateLEDStatus(STATUS_ERROR, true); while (1) delay(10);
  }
  icm.setAccelRateDivisor(ICM_ACCEL_RATE_DIV);
  icm.setGyroRateDivisor(ICM_GYRO_RATE_DIV);
  icmReady = true;
  Serial.println("✅ ICM20948 ready");

  if (!mpl.begin()) {
    Serial.println("❌ MPL3115A2 not found"); updateLEDStatus(STATUS_ERROR, true); while (1) delay(10);
  }
  mplReady = true;
  Serial.println("✅ MPL3115A2 ready");
  if (!setMplOsr(MPL_OSR_SETTING)) {
      Serial.println("⚠️ Failed to set MPL3115A2 OSR.");
  } else {
      #ifdef ENABLE_VERBOSE_DEBUG
      Serial.print("VERBOSE: MPL3115A2 OSR set to: "); Serial.println(MPL_OSR_SETTING);
      #endif
  }

  if (!ms4525do.Begin()) {
    Serial.println("❌ MS4525DO not found"); updateLEDStatus(STATUS_ERROR, true); while (1) delay(10);
  }
  ms4525Ready = true;
  Serial.println("✅ MS4525DO ready");

  Serial.println("Initializing BLE...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(BLE_SCAN_INTERVAL_MS);
  pBLEScan->setWindow(BLE_SCAN_WINDOW_MS);
  pBLEScan->setActiveScan(true);

  Serial.println("--- Setup Complete ---");
  tft.setCursor(5, 15);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("Setup Complete!");
  delay(1000);
}

// =============================================================
// ==                    Main Loop                          ==
// =============================================================
void loop() {
  unsigned long nowMillis = millis();

  // --- 1. Handle BLE Connection State ---
  bool needScanOrConnect = false;
  if (!bleInitAttempted) needScanOrConnect = true;
  if (!cscConnected && foundCSCDevice == nullptr) needScanOrConnect = true;
  if (!powerConnected && foundPowerDevice == nullptr) needScanOrConnect = true;
  if (foundCSCDevice != nullptr && !cscConnected) needScanOrConnect = true;
  if (foundPowerDevice != nullptr && !powerConnected) needScanOrConnect = true;

  if (needScanOrConnect) {
    static unsigned long lastBleActionAttempt = 0;
    if (nowMillis - lastBleActionAttempt >= BLE_CONNECT_RETRY_INTERVAL_MS) {
      #ifdef ENABLE_VERBOSE_DEBUG
      Serial.println("VERBOSE: Checking BLE state (Scan/Connect)...");
      #endif
      connectBLEDevices();
      lastBleActionAttempt = nowMillis;
      bleInitAttempted = true;
    }
  }
  if (cscConnected && (cscClient == nullptr || !cscClient->isConnected())) {
    Serial.println("⚠️ CSC Disconnected Unexpectedly");
    cscConnected = false;
  }
  if (powerConnected && (powerClient == nullptr || !powerClient->isConnected())) {
    Serial.println("⚠️ Power Meter Disconnected Unexpectedly");
    powerConnected = false;
  }

  // --- 2. Determine System Readiness & Status ---
  bool allSensorsReady = icmReady && mplReady && ms4525Ready;
  bool allBleReady = cscConnected && powerConnected;
  bool systemReady = allSensorsReady && allBleReady;

  SystemStatus previousStatus = currentStatus;
  if (recordingStarted) {
    currentStatus = STATUS_RECORDING;
  } else if (systemReady) {
    currentStatus = STATUS_READY;
  } else if (!allSensorsReady) {
    currentStatus = STATUS_ERROR;
  } else {
    currentStatus = STATUS_CONNECTING;
  }
  updateLEDStatus(currentStatus, currentStatus != previousStatus);


  // --- 3. Handle Recording Start/Stop ---
  if (!recordingStarted) {
    static bool buttonWasPressed = false;
    int buttonState = digitalRead(RECORD_PIN);

    if (buttonState == LOW && !buttonWasPressed) {
      buttonWasPressed = true;
    } else if (buttonState == HIGH && buttonWasPressed) {
      buttonWasPressed = false;
      if (systemReady) {
        Serial.println("Attempting to start recording...");
        if(SPIFFS.exists(LOG_FILENAME)) {
            SPIFFS.remove(LOG_FILENAME);
            Serial.println("Old log file removed.");
        }
        File file = SPIFFS.open(LOG_FILENAME, FILE_WRITE);
        if (!file) {
           Serial.println("❌ Failed to create log file!");
           updateLEDStatus(STATUS_ERROR, true);
        } else {
           // Changed Header: DeltaP (raw), Airspeed (calculated)
           file.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure,Temp,DeltaP,AirspeedTemp,Speed,Power,Airspeed,Cadence(rad/s)"); 
           file.close();
           Serial.println("✅ Recording starting.");

           recordingStarted = true;
           // No offset variables to reset

           logBuffer = "";
           lastLogMillis = nowMillis;
           lastFlushMillis = nowMillis;
        }
      } else {
         Serial.println("❌ Cannot start recording: System not ready!");
         #ifdef ENABLE_VERBOSE_DEBUG
         if (!allSensorsReady) Serial.println(" -> I2C sensors not ready.");
         if (!allBleReady) Serial.println(" -> BLE sensors not ready.");
         #endif
      }
    }
  } else {
     // Optional stop logic
  }

  // --- 4. Read I2C Sensors ---
  if (icmReady) icm.getEvent(&accel, &gyro, &temp_imu); // Reads raw accel/gyro
  if (mplReady) {
      lastPressure = mpl.getPressure(); // Reads raw ambient pressure
      lastTemp = mpl.getTemperature();   // Reads raw ambient temp
  }
  if (ms4525Ready) {
      if (ms4525do.Read()) {
          DeltaAirPress = ms4525do.pres_pa(); // Reads raw pressure difference
          lastAirTemp = ms4525do.die_temp_c();
      } else {
          DeltaAirPress = NAN; lastAirTemp = NAN;
      }
  }

  // --- 4.5 Offset Logic REMOVED ---

  // --- 5. Log Data to Buffer (if recording) ---
  if (recordingStarted && (nowMillis - lastLogMillis >= LOG_INTERVAL_MS)) {
    lastLogMillis += LOG_INTERVAL_MS;
    if (lastLogMillis < nowMillis - LOG_INTERVAL_MS) {
        lastLogMillis = nowMillis;
    }

    // Calculate airspeed using the RAW pressure value
    float currentAirspeedKph = airspeedPaToKph(DeltaAirPress);

    char csvLine[CSV_BUFFER_LINE_LENGTH];
    snprintf(csvLine, sizeof(csvLine),
             "%.3f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.f",
             nowMillis / 1000.0f,
             CSV_PRECISION_ACCEL, accel.acceleration.x, // Log raw Accel X
             CSV_PRECISION_ACCEL, accel.acceleration.y, // Log raw Accel Y
             CSV_PRECISION_ACCEL, accel.acceleration.z, // Log raw Accel Z
             CSV_PRECISION_GYRO, gyro.gyro.x,
             CSV_PRECISION_GYRO, gyro.gyro.y,
             CSV_PRECISION_GYRO, gyro.gyro.z,
             CSV_PRECISION_PRESSURE, lastPressure,      // Log raw Ambient Pressure
             CSV_PRECISION_TEMP, lastTemp,              // Log raw Ambient Temp
             CSV_PRECISION_AIRSPEED, DeltaAirPress,     // Log raw Pressure Difference
             CSV_PRECISION_TEMP, lastAirTemp,
             CSV_PRECISION_SPEED_KPH, currentSpeedKph,  // Wheel speed from BLE
             CSV_PRECISION_POWER, currentPower,         // Power from BLE
             CSV_PRECISION_WINDSPEED, currentAirspeedKph, // Calculated airspeed
             CSV_PRECISION_CADENCE, currentCadenceRadPerSec //
            );

    logBuffer += csvLine;
    logBuffer += "\n";
  }

  // --- 6. Flush Log Buffer Periodically ---
  bool timeToFlush = (nowMillis - lastFlushMillis >= FLUSH_INTERVAL_MS);
  bool bufferGettingFull = (logBuffer.length() >= MAX_LOG_BUFFER_SIZE);

  if (recordingStarted && (bufferGettingFull || timeToFlush)) {
       #ifdef ENABLE_VERBOSE_DEBUG
       if (bufferGettingFull) Serial.println("VERBOSE: Flushing buffer (full)...");
       else Serial.println("VERBOSE: Flushing buffer (interval)...");
       #endif
       flushLogBuffer();
  }

  // --- 7. Update Display Periodically ---
  if (nowMillis - lastDisplayUpdateMillis >= DISPLAY_UPDATE_INTERVAL_MS) {
      updateDisplay(DeltaAirPress); // Pass RAW pressure for display calculation
      lastDisplayUpdateMillis = nowMillis;
  }

  // --- 8. Yield ---
  delay(1);
}


// =============================================================
// ==                  Helper Functions                     ==
// =============================================================

void updateLEDStatus(SystemStatus status, bool forceUpdate) {
  if (!forceUpdate && status == lastDisplayedStatus) return;

  if (status != lastDisplayedStatus) {
      Serial.print("System Status -> ");
      switch (status) {
          case STATUS_ERROR: Serial.println("🔴 ERROR"); break;
          case STATUS_CONNECTING: Serial.println("🔵 CONNECTING"); break;
          case STATUS_READY: Serial.println("🟢 READY"); break;
          case STATUS_RECORDING: Serial.println("🟣 RECORDING"); break;
          default: Serial.println("⚪ UNKNOWN"); break;
      }
  }
  lastDisplayedStatus = status;

  if (status == STATUS_RECORDING) {
    bool ledOn = (millis() / 500) % 2 == 0;
    pixel.setPixelColor(0, ledOn ? pixel.Color(128, 0, 128) : pixel.Color(0, 0, 0));
  } else {
      switch (status) {
          case STATUS_ERROR: pixel.setPixelColor(0, pixel.Color(255, 0, 0)); break;
          case STATUS_CONNECTING: pixel.setPixelColor(0, pixel.Color(0, 0, 255)); break;
          case STATUS_READY: pixel.setPixelColor(0, pixel.Color(0, 255, 0)); break;
          default: pixel.setPixelColor(0, pixel.Color(50, 50, 50)); break;
      }
  }
  pixel.show();
}

bool setupCSCNotifications(BLEClient* client) {
    if (!client || !client->isConnected()) return false;
    #ifdef ENABLE_VERBOSE_DEBUG
    Serial.println("VERBOSE: Setting up CSC Notifications...");
    #endif
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
    try { pRemoteService = client->getService(CSC_SERVICE_UUID); } catch (...) { return false; }
    if (!pRemoteService) { Serial.println("  ❌ Failed to find CSC service"); return false; }
    #ifdef ENABLE_VERBOSE_DEBUG
    Serial.println("VERBOSE:  ✅ Found CSC service.");
    #endif
    try { pRemoteCharacteristic = pRemoteService->getCharacteristic(CSC_MEASUREMENT_CHAR_UUID); } catch (...) { return false; }
    if (!pRemoteCharacteristic) { Serial.println("  ❌ Failed to find CSC characteristic"); return false; }
    #ifdef ENABLE_VERBOSE_DEBUG
    Serial.println("VERBOSE:  ✅ Found CSC characteristic.");
    #endif
    if (pRemoteCharacteristic->canNotify()) {
        #ifdef ENABLE_VERBOSE_DEBUG
        Serial.println("VERBOSE:  Attempting to register for CSC notifications...");
        #endif
        try { pRemoteCharacteristic->registerForNotify(notifyCallback, true); } catch (...) { return false; }
        #ifdef ENABLE_VERBOSE_DEBUG
        Serial.println("VERBOSE:  ✅ CSC Notification registration attempted.");
        #endif
        return true;
    } else { Serial.println("  ❌ CSC characteristic does not support notifications!"); return false; }
}

bool setupPowerNotifications(BLEClient* client) {
     if (!client || !client->isConnected()) return false;
    #ifdef ENABLE_VERBOSE_DEBUG
    Serial.println("VERBOSE: Setting up Power Notifications...");
    #endif
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
    try { pRemoteService = client->getService(CYCLING_POWER_SERVICE_UUID); } catch (...) { return false; }
    if (!pRemoteService) { Serial.println("  ❌ Failed to find Power service"); return false; }
    #ifdef ENABLE_VERBOSE_DEBUG
    Serial.println("VERBOSE:  ✅ Found Power service.");
    #endif
    try { pRemoteCharacteristic = pRemoteService->getCharacteristic(CYCLING_POWER_MEASUREMENT_CHAR_UUID); } catch (...) { return false; }
    if (!pRemoteCharacteristic) { Serial.println("  ❌ Failed to find Power characteristic"); return false; }
    #ifdef ENABLE_VERBOSE_DEBUG
    Serial.println("VERBOSE:  ✅ Found Power characteristic.");
    #endif
    if (pRemoteCharacteristic->canNotify()) {
         #ifdef ENABLE_VERBOSE_DEBUG
         Serial.println("VERBOSE:  Attempting to register for Power notifications...");
         #endif
         try { pRemoteCharacteristic->registerForNotify(powerNotifyCallback, true); } catch (...) { return false; }
         #ifdef ENABLE_VERBOSE_DEBUG
         Serial.println("VERBOSE:  ✅ Power Notification registration attempted.");
         #endif
         return true;
    } else { Serial.println("  ❌ Power characteristic does not support notifications!"); return false; }
}

void connectBLEDevices() {
    bool needScan = (!cscConnected && foundCSCDevice == nullptr) ||
                    (!powerConnected && foundPowerDevice == nullptr);

    if (needScan && pBLEScan != nullptr && !bleScanInProgress) {
        Serial.print("🔍 Starting BLE scan...");
        if (!cscConnected && foundCSCDevice) { delete foundCSCDevice; foundCSCDevice = nullptr; }
        if (!powerConnected && foundPowerDevice) { delete foundPowerDevice; foundPowerDevice = nullptr; }
        bleScanInProgress = true;
        pBLEScan->start(BLE_SCAN_DURATION_S, false);
        bleScanInProgress = false;
        Serial.println(" Scan Finished.");
    }
    #ifdef ENABLE_VERBOSE_DEBUG
    else if (bleScanInProgress) { Serial.println("VERBOSE: (Scan already in progress...)"); }
    #endif

    if (!cscConnected && foundCSCDevice != nullptr) {
        if (cscClient == nullptr) cscClient = BLEDevice::createClient();
        if (!cscClient) { Serial.println("❌ Failed to create CSC BLE Client!"); return; }
        if (!cscClient->isConnected()) {
            #ifdef ENABLE_VERBOSE_DEBUG
            Serial.print("VERBOSE: Attempting CSC Connect to: "); Serial.println(foundCSCDevice->getAddress().toString().c_str());
            #endif
            bool connectSuccess = false;
            try { connectSuccess = cscClient->connect(foundCSCDevice); } catch (...) { connectSuccess = false; }
            if (connectSuccess) {
                Serial.println("✅ CSC Connected.");
                if (setupCSCNotifications(cscClient)) { cscConnected = true; delete foundCSCDevice; foundCSCDevice = nullptr; }
                else { Serial.println("❌ CSC Setup Failed. Disconnecting."); cscClient->disconnect(); }
            } else { Serial.println("❌ CSC connect() call failed."); }
        }
    }

    if (!powerConnected && foundPowerDevice != nullptr) {
         if (powerClient == nullptr) powerClient = BLEDevice::createClient();
         if (!powerClient) { Serial.println("❌ Failed to create Power BLE Client!"); return; }
         if (!powerClient->isConnected()) {
             #ifdef ENABLE_VERBOSE_DEBUG
            Serial.print("VERBOSE: Attempting Power Connect to: "); Serial.println(foundPowerDevice->getAddress().toString().c_str());
            #endif
            bool connectSuccess = false;
            try { connectSuccess = powerClient->connect(foundPowerDevice); } catch (...) { connectSuccess = false; }
            if (connectSuccess) {
                Serial.println("✅ Power Connected.");
                if (setupPowerNotifications(powerClient)) { powerConnected = true; delete foundPowerDevice; foundPowerDevice = nullptr; }
                else { Serial.println("❌ Power Setup Failed. Disconnecting."); powerClient->disconnect(); }
            } else { Serial.println("❌ Power connect() call failed."); }
        }
    }
}

void flushLogBuffer() {
  if (logBuffer.length() == 0) return;
  File file = SPIFFS.open(LOG_FILENAME, FILE_APPEND);
  if (!file) {
    Serial.println("❌ Failed to open log file for flushing!");
    updateLEDStatus(STATUS_ERROR, true);
    return;
  }
  size_t written = file.print(logBuffer);
  file.close();
  if (written == logBuffer.length()) {
     logBuffer = "";
  } else {
      Serial.print("❌ Log buffer flush failed! Expected "); Serial.print(logBuffer.length());
      Serial.print(" wrote "); Serial.println(written);
      updateLEDStatus(STATUS_ERROR, true);
  }
  lastFlushMillis = millis();
}

bool setMplOsr(uint8_t osr) {
  if (osr > 7) return false;
  Wire.beginTransmission(MPL3115A2_ADDRESS); Wire.write(MPL3115A2_CTRL_REG1);
  if (Wire.endTransmission(false) != 0) { return false; }
  if (Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)1) != 1) { return false; }
  uint8_t currentCtrl1 = Wire.read();
  uint8_t newCtrl1 = (currentCtrl1 & ~0b00111000) | ((osr & 0x07) << 3);
  Wire.beginTransmission(MPL3115A2_ADDRESS); Wire.write(MPL3115A2_CTRL_REG1); Wire.write(newCtrl1);
  if (Wire.endTransmission() != 0) { return false; }
  return true;
}

void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    // No Serial printing
    if (length < 7) return;
    uint8_t flags = pData[0];
    bool wheelDataPresent = flags & 0x01;
    if (wheelDataPresent) {
        uint32_t cumulativeRevs = pData[1] | (pData[2] << 8) | (pData[3] << 16) | (pData[4] << 24);
        uint16_t wheelEventTime = pData[5] | (pData[6] << 8);
        uint32_t timeDelta_1024 = (wheelEventTime >= lastWheelEventTime) ? (wheelEventTime - lastWheelEventTime) : ((0xFFFF - lastWheelEventTime) + wheelEventTime + 1);
        uint32_t revDelta = (cumulativeRevs >= lastWheelRevs) ? (cumulativeRevs - lastWheelRevs) : ((0xFFFFFFFF - lastWheelRevs) + cumulativeRevs + 1);
        if (revDelta > 0 && timeDelta_1024 > 0) {
            float timeDeltaSec = (float)timeDelta_1024 / 1024.0f;
            float distanceMeters = (float)revDelta * WHEEL_CIRCUMFERENCE_METERS;
            float speedMps = distanceMeters / timeDeltaSec;
            currentSpeedKph = speedMps * 3.6f;
            lastNonZeroRevTime = millis();
        } else if (cumulativeRevs == lastWheelRevs) {
             if (millis() - lastNonZeroRevTime > BLE_SPEED_TIMEOUT_MS) {
                 currentSpeedKph = 0.0f;
             }
        }
        lastWheelRevs = cumulativeRevs;
        lastWheelEventTime = wheelEventTime;
    }
}

float airspeedPaToKph(float pressure_pa) {
    // Input pressure_pa is the RAW sensor value
    if (isnan(pressure_pa)) { return NAN; }
    float effective_pressure = pressure_pa;

    /* // --- OPTION 2: Use if RAW sensor reading is NEGATIVE for positive airflow ---
    effective_pressure = -pressure_pa; // Apply negation if raw sensor is inverted
    if (effective_pressure < 0) { effective_pressure = 0.0f; }
    */

    // --- OPTION 3: Use if RAW sensor reading is POSITIVE for positive airflow ---
    if (effective_pressure < 0) { effective_pressure = 0.0f; }
    // --- END OPTION 3 ---

    // Check for extremely small pressures that might be noise after removing offset logic
    // if (fabsf(effective_pressure) < 0.1) { // Adjust threshold as needed
    //    return 0.0f;
    // }

    float speed_mps = sqrtf((2.0f * effective_pressure) / AIR_DENSITY);
    return speed_mps * 3.6f;
}

void powerNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length < 4) return; // Minimum length for Flags(2) + Power(2)
    uint16_t flags = pData[0] | (pData[1] << 8);
    int16_t watts = pData[2] | (pData[3] << 8);
    currentPower = (float)watts;
    bool crankDataPresent = (flags & 0x0200);

    if (crankDataPresent) {
        if (length < 8) {
            #ifdef ENABLE_VERBOSE_DEBUG
            Serial.println("VERBOSE: Power callback - crank flag set but length too short.");
            #endif
            return; // Not enough data for cadence
        }
        uint16_t cumulativeCrankRevs = pData[4] | (pData[5] << 8);
        uint16_t crankEventTime = pData[6] | (pData[7] << 8);

        uint32_t timeDelta_1024 = (crankEventTime >= lastCrankEventTime) ?
                                  (crankEventTime - lastCrankEventTime) :
                                  ((0xFFFF - lastCrankEventTime) + crankEventTime + 1);
        uint16_t crankRevDelta = (cumulativeCrankRevs >= lastCrankRevs) ?
                                 (cumulativeCrankRevs - lastCrankRevs) :
                                 ((0xFFFF - lastCrankRevs) + cumulativeCrankRevs + 1);

        if (crankRevDelta > 0 && timeDelta_1024 > 0) {
            float timeDeltaSec = (float)timeDelta_1024 / 1024.0f;
            // Cadence (rad/s) = (Revolutions / TimeDeltaSeconds) * (2*PI Radians / Revolution)
            currentCadenceRadPerSec = ((float)crankRevDelta / timeDeltaSec) * (2.0f * PI);
            lastNonZeroCrankTime = millis(); // Update time of last movement
        } else if (cumulativeCrankRevs == lastCrankRevs) {
            if (millis() - lastNonZeroCrankTime > BLE_CADENCE_TIMEOUT_MS) {
                currentCadenceRadPerSec = 0.0f; // Set cadence to zero after timeout
            }
        } else {
             #ifdef ENABLE_VERBOSE_DEBUG
             Serial.println("VERBOSE: Cadence calc - unexpected delta state.");
             #endif
        }

        // Update state for next calculation
        lastCrankRevs = cumulativeCrankRevs;
        lastCrankEventTime = crankEventTime;

    } else {
        if (millis() - lastNonZeroCrankTime > BLE_CADENCE_TIMEOUT_MS) {
            currentCadenceRadPerSec = 0.0f;
        }
    }
}

void dumpCSVOverSerial() {
  Serial.print("Attempting to dump log file: "); Serial.println(LOG_FILENAME);
  File file = SPIFFS.open(LOG_FILENAME, "r");
  if (!file || file.isDirectory()) {
    Serial.println("❌ Log file not found or is directory.");
    updateLEDStatus(STATUS_ERROR, true); delay(3000); return;
  }
  Serial.println("\n\n--- START LOG DUMP ---");
  char buf[64];
  while (file.available()) {
    int bytesRead = file.readBytes(buf, sizeof(buf));
    Serial.write((const uint8_t*)buf, bytesRead);
  }
  Serial.println("\n--- END LOG DUMP ---");
  file.close();
  Serial.println("Dump complete. System halted. Reset device to continue.");
  updateLEDStatus(STATUS_ERROR, true);
  while (true) {
       pixel.setPixelColor(0, pixel.Color(255, 255, 0)); delay(500);
       pixel.setPixelColor(0, pixel.Color(0, 0, 0)); delay(500);
       pixel.show();
  }
}

void initDisplay() {
    Serial.println("Initializing TFT Display...");
    if (TFT_RST >= 0) {
        pinMode(TFT_RST, OUTPUT); digitalWrite(TFT_RST, LOW); delay(10); digitalWrite(TFT_RST, HIGH); delay(10);
    }
    tft.initR(INITR_MINI160x80);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    Serial.println("✅ TFT Initialized.");
    if (TFT_BLK >= 0) {
        pinMode(TFT_BLK, OUTPUT); digitalWrite(TFT_BLK, HIGH); Serial.println("✅ TFT Backlight ON.");
    } else { Serial.println("-> TFT Backlight pin not defined or controlled."); }
}

// Update display function now accepts RAW pressure
void updateDisplay(float currentRawDeltaP) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Status: ");
    switch (currentStatus) {
        case STATUS_ERROR:      tft.setTextColor(ST77XX_RED);    tft.print("ERROR"); break;
        case STATUS_CONNECTING: tft.setTextColor(ST77XX_BLUE);   tft.print("CONNECTING"); break;
        case STATUS_READY:      tft.setTextColor(ST77XX_GREEN);  tft.print("READY"); break;
        case STATUS_RECORDING:  tft.setTextColor(ST77XX_MAGENTA);tft.print("RECORDING"); break;
        default:                tft.setTextColor(ST77XX_WHITE);  tft.print("UNKNOWN"); break;
    }

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);

    tft.setCursor(5, 25);
    tft.print("P:");
    if (powerConnected) { tft.print(currentPower, 0); } else { tft.print("---"); }
    tft.print("W");

    tft.setCursor(5, 50);
    tft.print("S:");
    if (cscConnected) {
       char speedStr[8]; dtostrf(currentSpeedKph, 4, 1, speedStr); tft.print(speedStr);
    } else { tft.print("----"); }

    tft.setTextSize(1);
    tft.setCursor(tft.width() - 30, 60);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("kph");
}