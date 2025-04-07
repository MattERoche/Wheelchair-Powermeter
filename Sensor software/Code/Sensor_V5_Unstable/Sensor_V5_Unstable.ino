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

// --- Logging Settings ---
const char* LOG_FILENAME = "/log.csv";
const unsigned long LOG_INTERVAL_MS = 10; // Log data every 10ms (100 Hz)
const unsigned long FLUSH_INTERVAL_MS = 1000;
const size_t MAX_LOG_BUFFER_SIZE = 1024;
const int CSV_PRECISION_ACCEL = 2;
const int CSV_PRECISION_GYRO = 2;
const int CSV_PRECISION_PRESSURE = 1;
const int CSV_PRECISION_TEMP = 1;
const int CSV_PRECISION_AIRSPEED = 1;
const int CSV_PRECISION_SPEED_KPH = 2;
const int CSV_PRECISION_POWER = 0;
const int CSV_PRECISION_WINDSPEED = 2;
const int CSV_BUFFER_LINE_LENGTH = 220;
const int CSV_PRECISION_CADENCE = 0; // Cadence often reported/displayed as integer RPM

// --- Sensor Settings ---
#define ICM_ACCEL_RATE_DIV 10
#define ICM_GYRO_RATE_DIV 10
#define MPL_OSR_SETTING 0
#define AIR_DENSITY 1.22f

// --- BLE Settings ---
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816); // Used by Speed and Cadence
static BLEUUID CSC_MEASUREMENT_CHAR_UUID((uint16_t)0x2A5B); // Used by Speed and Cadence
static BLEUUID CYCLING_POWER_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CYCLING_POWER_MEASUREMENT_CHAR_UUID((uint16_t)0x2A63);
const unsigned long BLE_SCAN_INTERVAL_MS = 100;
const unsigned long BLE_SCAN_WINDOW_MS = 99;
const unsigned long BLE_SCAN_DURATION_S = 5; // Slightly longer scan maybe needed
const unsigned long BLE_CONNECT_RETRY_INTERVAL_MS = 5000;
const float WHEEL_CIRCUMFERENCE_METERS = 2.105f;
const unsigned long BLE_SPEED_TIMEOUT_MS = 3000;
const unsigned long BLE_CADENCE_TIMEOUT_MS = 3000; // Timeout for dedicated cadence sensor

// --- Display Settings ---
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500;

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
BLEClient* cscClient = nullptr;         // Client for SPEED sensor
BLEClient* powerClient = nullptr;       // Client for POWER meter
BLEClient* cadenceClient = nullptr;     // Client for CADENCE sensor
BLEAdvertisedDevice* foundCSCDevice = nullptr; // Found SPEED sensor candidate
BLEAdvertisedDevice* foundPowerDevice = nullptr; // Found POWER meter candidate
BLEAdvertisedDevice* foundCadenceDevice = nullptr; // Found CADENCE sensor candidate
bool cscConnected = false;              // Speed sensor connected state
bool powerConnected = false;            // Power meter connected state
bool cadenceConnected = false;          // Cadence sensor connected state
bool bleInitAttempted = false;
static bool bleScanInProgress = false;

// --- Sensor Raw Data ---
float currentSpeedKph = 0.0f;       // From BLE Speed Sensor
float currentPower = 0.0f;          // From BLE Power Meter
float lastPressure = 0.0f;
float lastTemp = 0.0f;
float DeltaAirPress = 0.0f;
float lastAirTemp = 0.0f;
sensors_event_t accel, gyro, temp_imu;

// --- Speed Calculation (from Speed Sensor) ---
uint32_t lastWheelRevs = 0;
uint16_t lastWheelEventTime = 0;    // 1/1024s
unsigned long lastNonZeroRevTime = 0; // ms

// --- Cadence Calculation (from Cadence Sensor) ---
uint16_t lastCrankRevs = 0;         // From cadence sensor data
uint16_t lastCrankEventTime = 0;    // 1/1024s, from cadence sensor data
unsigned long lastNonZeroCrankTime = 0; // ms, updated by cadence callback
float currentCadenceRadPerSec = 0.0f; // Calculated cadence in rad/s

// --- Sensor Readiness ---
bool icmReady = false;
bool mplReady = false;
bool ms4525Ready = false;

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
void speedNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify); // Renamed
void powerNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify); // Reverted
void cadenceNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify); // New
void flushLogBuffer();
bool setMplOsr(uint8_t osr);
bool setupSpeedNotifications(BLEClient* client);    // Renamed
bool setupPowerNotifications(BLEClient* client);
bool setupCadenceNotifications(BLEClient* client);  // New
float airspeedPaToKph(float pressure_pa);
void initDisplay();
void updateDisplay(float currentRawDeltaP);

#ifndef MPL3115A2_ADDRESS
#define MPL3115A2_ADDRESS (0x60)
#endif
#ifndef MPL3115A2_CTRL_REG1
#define MPL3115A2_CTRL_REG1 (0x26)
#endif

#define KNOWN_SPEED_ADDRESS   "cd:0e:f8:71:4b:6f"   // Confirmed by user
#define KNOWN_CADENCE_ADDRESS "cc:22:bf:b6:3b:02"   // From previous log
#define KNOWN_POWER_ADDRESS   "fa:95:b1:c2:fb:5e"   // From previous log
// =============================================================
// ==             BLE Callback Class Definition             ==
// =============================================================
// =============================================================
// ==             BLE Callback Class Definition             ==
// =============================================================
// =============================================================
// ==             BLE Callback Class Definition             ==
// =============================================================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  // --- onResult ---
  void onResult(BLEAdvertisedDevice advertisedDevice) {
     bool changed = false;
     String currentDeviceAddress = advertisedDevice.getAddress().toString(); // Use Arduino 

     // --- Check against KNOWN addresses FIRST ---

     #ifdef KNOWN_SPEED_ADDRESS
     if (!cscConnected && foundCSCDevice == nullptr &&
         advertisedDevice.isAdvertisingService(CSC_SERVICE_UUID) &&
         currentDeviceAddress == KNOWN_SPEED_ADDRESS)
     {
         Serial.print("Found Known Speed Sensor: "); Serial.println(currentDeviceAddress.c_str());
         foundCSCDevice = new BLEAdvertisedDevice(advertisedDevice);
         changed = true;
     }
     #endif

     #ifdef KNOWN_CADENCE_ADDRESS
     if (!cadenceConnected && foundCadenceDevice == nullptr &&
         advertisedDevice.isAdvertisingService(CSC_SERVICE_UUID) &&
         currentDeviceAddress == KNOWN_CADENCE_ADDRESS)
     {
         Serial.print("Found Known Cadence Sensor: "); Serial.println(currentDeviceAddress.c_str());
         foundCadenceDevice = new BLEAdvertisedDevice(advertisedDevice);
         changed = true;
     }
     #endif

     #ifdef KNOWN_POWER_ADDRESS
     if (!powerConnected && foundPowerDevice == nullptr &&
         advertisedDevice.isAdvertisingService(CYCLING_POWER_SERVICE_UUID) &&
         currentDeviceAddress == KNOWN_POWER_ADDRESS)
     {
         Serial.print("Found Known Power Meter: "); Serial.println(currentDeviceAddress.c_str());
         foundPowerDevice = new BLEAdvertisedDevice(advertisedDevice);
         changed = true;
     }
     #endif

     // --- Fallback to Discovery Order (if known addresses not defined/found) ---

     // Power Meter (Discovery Fallback)
     #ifndef KNOWN_POWER_ADDRESS
     if (!powerConnected && foundPowerDevice == nullptr && advertisedDevice.isAdvertisingService(CYCLING_POWER_SERVICE_UUID)) {
        Serial.print("Found Power Candidate (Discovery): "); Serial.println(currentDeviceAddress.c_str());
        foundPowerDevice = new BLEAdvertisedDevice(advertisedDevice);
        changed = true;
      }
     #endif

     // CSC Devices (Discovery Fallback)
     if (advertisedDevice.isAdvertisingService(CSC_SERVICE_UUID)) {
         // Speed Sensor (Discovery Fallback)
         #ifndef KNOWN_SPEED_ADDRESS
         if (!cscConnected && foundCSCDevice == nullptr) {
             // Ensure it's not the already assigned known cadence/power sensor
             #ifdef KNOWN_CADENCE_ADDRESS
               if (currentDeviceAddress == KNOWN_CADENCE_ADDRESS) goto skip_csc_discovery; // Already assigned
             #endif
             #ifdef KNOWN_POWER_ADDRESS
               if (currentDeviceAddress == KNOWN_POWER_ADDRESS) goto skip_csc_discovery; // Already assigned
             #endif

             Serial.print("Found CSC Candidate 1 (Assuming Speed - Discovery): "); Serial.println(currentDeviceAddress.c_str());
             foundCSCDevice = new BLEAdvertisedDevice(advertisedDevice);
             changed = true;
         }
         #endif

         // Cadence Sensor (Discovery Fallback)
         #ifndef KNOWN_CADENCE_ADDRESS
         if (!cadenceConnected && foundCadenceDevice == nullptr) {
             // Make sure it's not the already assigned known speed/power sensor
             // AND not the already assigned *discovered* speed sensor
             bool assigned_as_known_speed = false;
             bool assigned_as_known_power = false;
             bool assigned_as_discovered_speed = false;

             #ifdef KNOWN_SPEED_ADDRESS
               if (currentDeviceAddress == KNOWN_SPEED_ADDRESS) assigned_as_known_speed = true;
             #endif
             #ifdef KNOWN_POWER_ADDRESS
               if (currentDeviceAddress == KNOWN_POWER_ADDRESS) assigned_as_known_power = true;
             #endif
             if (foundCSCDevice != nullptr && currentDeviceAddress == foundCSCDevice->getAddress().toString()) {
                 assigned_as_discovered_speed = true;
             }

             if (!assigned_as_known_speed && !assigned_as_known_power && !assigned_as_discovered_speed) {
                 Serial.print("Found CSC Candidate 2 (Assuming Cadence - Discovery): "); Serial.println(currentDeviceAddress.c_str());
                 foundCadenceDevice = new BLEAdvertisedDevice(advertisedDevice);
                 changed = true;
             }
         }
         #endif
     }
     skip_csc_discovery:; // Label for goto jump

    // --- Stop Scan Logic (Same as before) ---
    bool gotSpeedCandidate = cscConnected || (foundCSCDevice != nullptr);
    bool gotPowerCandidate = powerConnected || (foundPowerDevice != nullptr);
    bool gotCadenceCandidate = cadenceConnected || (foundCadenceDevice != nullptr);

    if (changed && gotSpeedCandidate && gotPowerCandidate && gotCadenceCandidate) {
      if(pBLEScan != nullptr && bleScanInProgress) {
           Serial.println("Found candidates/matches for all needed devices. Stopping scan early.");
           pBLEScan->stop();
           bleScanInProgress = false; // Set flag immediately when stopping early
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
  // Check if we need to find or connect to any of the three sensors
  if (!cscConnected && foundCSCDevice == nullptr) needScanOrConnect = true;
  if (!powerConnected && foundPowerDevice == nullptr) needScanOrConnect = true;
  if (!cadenceConnected && foundCadenceDevice == nullptr) needScanOrConnect = true;
  if (foundCSCDevice != nullptr && !cscConnected) needScanOrConnect = true;
  if (foundPowerDevice != nullptr && !powerConnected) needScanOrConnect = true;
  if (foundCadenceDevice != nullptr && !cadenceConnected) needScanOrConnect = true;


  if (needScanOrConnect) {
    static unsigned long lastBleActionAttempt = 0;
    // Don't try connecting immediately after a failed scan/connect attempt
    if (nowMillis - lastBleActionAttempt >= BLE_CONNECT_RETRY_INTERVAL_MS) {
      connectBLEDevices(); // This function handles both scanning and connecting
      lastBleActionAttempt = nowMillis;
      bleInitAttempted = true; // Mark that we've tried at least once
    }
  }

  // --- Check for unexpected disconnections ---
  if (cscConnected && (cscClient == nullptr || !cscClient->isConnected())) {
    Serial.println("⚠️ Speed Sensor Disconnected Unexpectedly");
    cscConnected = false;
    // Should we try to reconnect? The logic above will trigger a scan/connect next cycle
  }
  if (powerConnected && (powerClient == nullptr || !powerClient->isConnected())) {
    Serial.println("⚠️ Power Meter Disconnected Unexpectedly");
    powerConnected = false;
  }
   if (cadenceConnected && (cadenceClient == nullptr || !cadenceClient->isConnected())) {
    Serial.println("⚠️ Cadence Sensor Disconnected Unexpectedly");
    cadenceConnected = false;
  }

  // --- 2. Determine System Readiness & Status ---
  bool allSensorsReady = icmReady && mplReady && ms4525Ready;
  // Now requires all *three* BLE devices to be connected
  bool allBleReady = cscConnected && powerConnected && cadenceConnected;
  bool systemReady = allSensorsReady && allBleReady;

  SystemStatus previousStatus = currentStatus;
  if (recordingStarted) {
    currentStatus = STATUS_RECORDING;
  } else if (systemReady) {
    currentStatus = STATUS_READY;
  } else if (!allSensorsReady) {
    currentStatus = STATUS_ERROR; // Prioritize sensor HW error
  } else { // I2C sensors okay, but BLE not ready
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
           // Header updated to reflect cadence source is now separate sensor
           file.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure,Temp,DeltaP,AirspeedTemp,SpeedKPH,PowerW,AirspeedKPH,CadenceRPM");
           file.close();
           Serial.println("✅ Recording starting.");

           recordingStarted = true;
           logBuffer = "";
           lastLogMillis = nowMillis;
           lastFlushMillis = nowMillis;
        }
      } else {
         Serial.println("❌ Cannot start recording: System not ready!");
         if (!allSensorsReady) Serial.println(" -> I2C sensors not ready.");
         if (!allBleReady) {
             Serial.print(" -> BLE not ready: Speed="); Serial.print(cscConnected);
             Serial.print(", Power="); Serial.print(powerConnected);
             Serial.print(", Cadence="); Serial.println(cadenceConnected);
         }
      }
    }
  }

  // --- 4. Read I2C Sensors ---
  if (icmReady) icm.getEvent(&accel, &gyro, &temp_imu);
  if (mplReady) {
      lastPressure = mpl.getPressure();
      lastTemp = mpl.getTemperature();
  }
  if (ms4525Ready) {
      if (ms4525do.Read()) {
          DeltaAirPress = ms4525do.pres_pa();
          lastAirTemp = ms4525do.die_temp_c();
      } else {
          DeltaAirPress = NAN; lastAirTemp = NAN;
      }
  }

  // --- 5. Log Data to Buffer (if recording) ---
  if (recordingStarted && (nowMillis - lastLogMillis >= LOG_INTERVAL_MS)) {
    lastLogMillis += LOG_INTERVAL_MS;
    if (lastLogMillis < nowMillis - LOG_INTERVAL_MS) {
        lastLogMillis = nowMillis; // Prevent excessive catch-up if loop is slow
    }

    float currentAirspeedKph = airspeedPaToKph(DeltaAirPress);
    // Convert cadence from rad/s to RPM for logging
    float currentCadenceRPM = currentCadenceRadPerSec * (60.0f / (2.0f * PI));

    char csvLine[CSV_BUFFER_LINE_LENGTH];
    snprintf(csvLine, sizeof(csvLine),
             "%.3f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f",
             nowMillis / 1000.0f,
             CSV_PRECISION_ACCEL, accel.acceleration.x,
             CSV_PRECISION_ACCEL, accel.acceleration.y,
             CSV_PRECISION_ACCEL, accel.acceleration.z,
             CSV_PRECISION_GYRO, gyro.gyro.x,
             CSV_PRECISION_GYRO, gyro.gyro.y,
             CSV_PRECISION_GYRO, gyro.gyro.z,
             CSV_PRECISION_PRESSURE, lastPressure,
             CSV_PRECISION_TEMP, lastTemp,
             CSV_PRECISION_AIRSPEED, DeltaAirPress,
             CSV_PRECISION_TEMP, lastAirTemp,
             CSV_PRECISION_SPEED_KPH, currentSpeedKph,  // Wheel speed
             CSV_PRECISION_POWER, currentPower,         // Power
             CSV_PRECISION_WINDSPEED, currentAirspeedKph, // Calculated airspeed
             CSV_PRECISION_CADENCE, currentCadenceRPM   // Cadence in RPM
            );

    logBuffer += csvLine;
    logBuffer += "\n";
  }

  // --- 6. Flush Log Buffer Periodically ---
  bool timeToFlush = (nowMillis - lastFlushMillis >= FLUSH_INTERVAL_MS);
  bool bufferGettingFull = (logBuffer.length() >= MAX_LOG_BUFFER_SIZE);

  if (recordingStarted && (bufferGettingFull || timeToFlush)) {
       flushLogBuffer();
  }

  // --- 7. Update Display Periodically ---
  if (nowMillis - lastDisplayUpdateMillis >= DISPLAY_UPDATE_INTERVAL_MS) {
      updateDisplay(DeltaAirPress);
      lastDisplayUpdateMillis = nowMillis;
  }

  // --- 8. Yield ---
  delay(1);
}


// =============================================================
// ==                  Helper Functions                     ==
// =============================================================

// --- updateLEDStatus ---
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

// --- setupSpeedNotifications --- (Renamed from setupCSCNotifications)
bool setupSpeedNotifications(BLEClient* client) {
    if (!client || !client->isConnected()) return false;
    Serial.println("Setting up Speed Notifications...");
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
    try { pRemoteService = client->getService(CSC_SERVICE_UUID); } catch (...) { return false; }
    if (!pRemoteService) { Serial.println("  ❌ Failed to find CSC service (for Speed)"); return false; }
    try { pRemoteCharacteristic = pRemoteService->getCharacteristic(CSC_MEASUREMENT_CHAR_UUID); } catch (...) { return false; }
    if (!pRemoteCharacteristic) { Serial.println("  ❌ Failed to find CSC characteristic (for Speed)"); return false; }
    if (pRemoteCharacteristic->canNotify()) {
        try {
             // Register the specific callback for speed
             pRemoteCharacteristic->registerForNotify(speedNotifyCallback, true);
             Serial.println("  ✅ Speed Notification registration attempted.");
             return true;
        } catch (...) {
             Serial.println("  ❌ Exception during speed notify registration.");
             return false;
        }
    } else { Serial.println("  ❌ CSC characteristic (for Speed) does not support notifications!"); return false; }
}

// --- setupPowerNotifications ---
bool setupPowerNotifications(BLEClient* client) {
     if (!client || !client->isConnected()) return false;
    Serial.println("Setting up Power Notifications...");
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
    try { pRemoteService = client->getService(CYCLING_POWER_SERVICE_UUID); } catch (...) { return false; }
    if (!pRemoteService) { Serial.println("  ❌ Failed to find Power service"); return false; }
    try { pRemoteCharacteristic = pRemoteService->getCharacteristic(CYCLING_POWER_MEASUREMENT_CHAR_UUID); } catch (...) { return false; }
    if (!pRemoteCharacteristic) { Serial.println("  ❌ Failed to find Power characteristic"); return false; }
    if (pRemoteCharacteristic->canNotify()) {
         try {
             pRemoteCharacteristic->registerForNotify(powerNotifyCallback, true);
             Serial.println("  ✅ Power Notification registration attempted.");
             return true;
         } catch (...) {
             Serial.println("  ❌ Exception during power notify registration.");
             return false;
         }
    } else { Serial.println("  ❌ Power characteristic does not support notifications!"); return false; }
}

// --- setupCadenceNotifications --- (New Function)
bool setupCadenceNotifications(BLEClient* client) {
    if (!client || !client->isConnected()) return false;
    Serial.println("Setting up Cadence Notifications...");
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
    try { pRemoteService = client->getService(CSC_SERVICE_UUID); } catch (...) { return false; }
    if (!pRemoteService) { Serial.println("  ❌ Failed to find CSC service (for Cadence)"); return false; }
    try { pRemoteCharacteristic = pRemoteService->getCharacteristic(CSC_MEASUREMENT_CHAR_UUID); } catch (...) { return false; }
    if (!pRemoteCharacteristic) { Serial.println("  ❌ Failed to find CSC characteristic (for Cadence)"); return false; }
    if (pRemoteCharacteristic->canNotify()) {
        try {
             // Register the specific callback for cadence
             pRemoteCharacteristic->registerForNotify(cadenceNotifyCallback, true);
             Serial.println("  ✅ Cadence Notification registration attempted.");
             return true;
         } catch (...) {
             Serial.println("  ❌ Exception during cadence notify registration.");
            return false;
        }
    } else { Serial.println("  ❌ CSC characteristic (for Cadence) does not support notifications!"); return false; }
}


// --- connectBLEDevices ---
void connectBLEDevices() {
    // --- Start Scan if necessary ---
    bool needScan = (!cscConnected && foundCSCDevice == nullptr) ||
                    (!powerConnected && foundPowerDevice == nullptr) ||
                    (!cadenceConnected && foundCadenceDevice == nullptr);

    if (needScan && pBLEScan != nullptr && !bleScanInProgress) {
        Serial.print("🔍 Starting BLE scan for missing devices...");
        // Clear old candidates only if not connected
        if (!cscConnected && foundCSCDevice) { delete foundCSCDevice; foundCSCDevice = nullptr; }
        if (!powerConnected && foundPowerDevice) { delete foundPowerDevice; foundPowerDevice = nullptr; }
        if (!cadenceConnected && foundCadenceDevice) { delete foundCadenceDevice; foundCadenceDevice = nullptr; }

        bleScanInProgress = true;
        // Scan completion callback sets bleScanInProgress = false
        pBLEScan->start(BLE_SCAN_DURATION_S, [](BLEScanResults r){ bleScanInProgress = false; Serial.println(" Scan Finished."); }, false);
        // Note: Scan runs asynchronously. Connection attempts happen below *after* the scan might have found devices.
    } else if (bleScanInProgress) {
        Serial.println("(Scan already in progress...)");
        return; // Don't try to connect while actively scanning
    }

    // --- Attempt Connections (if candidates found and not already connected) ---

    // 1. Speed Sensor
    if (!cscConnected && foundCSCDevice != nullptr) {
        if (cscClient == nullptr) cscClient = BLEDevice::createClient();
        if (cscClient && !cscClient->isConnected()) { // Check client exists and is not connected
            Serial.print("Attempting Speed Connect to: "); Serial.println(foundCSCDevice->getAddress().toString().c_str());
            bool connectSuccess = false;
            try { connectSuccess = cscClient->connect(foundCSCDevice); } catch (...) { connectSuccess = false; }
            if (connectSuccess) {
                Serial.println("✅ Speed Sensor Connected.");
                if (setupSpeedNotifications(cscClient)) {
                    cscConnected = true;
                    delete foundCSCDevice; // Success, clean up
                    foundCSCDevice = nullptr;
                } else { Serial.println("❌ Speed Setup Failed. Disconnecting."); cscClient->disconnect(); }
            } else { Serial.println("❌ Speed connect() call failed."); /* Don't delete candidate, try again later */ }
        } else if (cscClient && cscClient->isConnected()){
             // Already connected, just ensure state is right and clean up candidate
             delete foundCSCDevice; foundCSCDevice = nullptr; cscConnected = true;
        }
    }

    // 2. Power Meter
    if (!powerConnected && foundPowerDevice != nullptr) {
         if (powerClient == nullptr) powerClient = BLEDevice::createClient();
         if (powerClient && !powerClient->isConnected()) {
            Serial.print("Attempting Power Connect to: "); Serial.println(foundPowerDevice->getAddress().toString().c_str());
            bool connectSuccess = false;
            try { connectSuccess = powerClient->connect(foundPowerDevice); } catch (...) { connectSuccess = false; }
            if (connectSuccess) {
                Serial.println("✅ Power Meter Connected.");
                if (setupPowerNotifications(powerClient)) {
                     powerConnected = true; delete foundPowerDevice; foundPowerDevice = nullptr;
                 } else { Serial.println("❌ Power Setup Failed. Disconnecting."); powerClient->disconnect(); }
            } else { Serial.println("❌ Power connect() call failed."); }
        } else if (powerClient && powerClient->isConnected()){
             delete foundPowerDevice; foundPowerDevice = nullptr; powerConnected = true;
        }
    }

     // 3. Cadence Sensor
    if (!cadenceConnected && foundCadenceDevice != nullptr) {
        if (cadenceClient == nullptr) cadenceClient = BLEDevice::createClient();
        if (cadenceClient && !cadenceClient->isConnected()) {
            Serial.print("Attempting Cadence Connect to: "); Serial.println(foundCadenceDevice->getAddress().toString().c_str());
            bool connectSuccess = false;
            try { connectSuccess = cadenceClient->connect(foundCadenceDevice); } catch (...) { connectSuccess = false; }
            if (connectSuccess) {
                Serial.println("✅ Cadence Sensor Connected.");
                if (setupCadenceNotifications(cadenceClient)) {
                     cadenceConnected = true; delete foundCadenceDevice; foundCadenceDevice = nullptr;
                 } else { Serial.println("❌ Cadence Setup Failed. Disconnecting."); cadenceClient->disconnect(); }
            } else { Serial.println("❌ Cadence connect() call failed."); }
        } else if (cadenceClient && cadenceClient->isConnected()){
             delete foundCadenceDevice; foundCadenceDevice = nullptr; cadenceConnected = true;
        }
    }
}


// --- flushLogBuffer ---
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
      if (written > 0) logBuffer.remove(0, written); // Remove partial write
  }
  lastFlushMillis = millis();
}

// --- setMplOsr ---
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

// --- speedNotifyCallback --- (Renamed from notifyCallback)
// Parses CSC Measurement data, focusing on Wheel Revolution Data
void speedNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (pData == nullptr || length == 0) return;

    uint8_t flags = pData[0];
    bool wheelDataPresent = flags & 0x01; // Bit 0
    size_t offset = 1;

    if (wheelDataPresent) {
        if (length < offset + 6) return; // Need 4 bytes revs, 2 bytes time

        uint32_t cumulativeRevs = pData[offset] | (pData[offset+1] << 8) | (pData[offset+2] << 16) | (pData[offset+3] << 24);
        uint16_t wheelEventTime_1024 = pData[offset+4] | (pData[offset+5] << 8); // Unit 1/1024s

        uint32_t timeDelta_1024 = (wheelEventTime_1024 >= lastWheelEventTime) ?
                                  (wheelEventTime_1024 - lastWheelEventTime) :
                                  ((0xFFFF - lastWheelEventTime) + wheelEventTime_1024 + 1);

        uint32_t revDelta = (cumulativeRevs >= lastWheelRevs) ?
                             (cumulativeRevs - lastWheelRevs) :
                             ((0xFFFFFFFF - lastWheelRevs) + cumulativeRevs + 1);

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
        lastWheelEventTime = wheelEventTime_1024;
        // offset += 6; // Only needed if parsing crank data *after* wheel data in same callback
    } else {
        // Wheel data not present, check timeout
        if (millis() - lastNonZeroRevTime > BLE_SPEED_TIMEOUT_MS) {
             currentSpeedKph = 0.0f;
         }
    }
    // Ignore Crank data (Bit 1) in this callback
}

// --- airspeedPaToKph ---
float airspeedPaToKph(float pressure_pa) {
    if (isnan(pressure_pa)) { return NAN; }
    float effective_pressure = pressure_pa;
    if (effective_pressure < 0) { effective_pressure = 0.0f; }
    float speed_mps = sqrtf((2.0f * effective_pressure) / AIR_DENSITY);
    return speed_mps * 3.6f;
}

// --- powerNotifyCallback --- (Reverted - Only parses mandatory power)
void powerNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    // Parse Cycling Power Measurement characteristic data (UUID 0x2A63)
    // Only parsing mandatory fields: Flags and Instantaneous Power

    if (pData == nullptr || length < 4) return; // Minimum length for Flags(2) + Power(2)

    // uint16_t flags = pData[0] | (pData[1] << 8); // Flags needed if parsing optional fields
    int16_t watts = pData[2] | (pData[3] << 8); // Instantaneous Power
    currentPower = (float)watts;

    // No parsing of optional fields (like cadence) here anymore.
}

// --- cadenceNotifyCallback --- (New Function)
// Parses CSC Measurement data, focusing on Crank Revolution Data
void cadenceNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (pData == nullptr || length == 0) return;

    uint8_t flags = pData[0];
    bool wheelDataPresent = flags & 0x01; // Bit 0
    bool crankDataPresent = flags & 0x02; // Bit 1
    size_t offset = 1;

    // Advance offset if wheel data is present *before* crank data
    if (wheelDataPresent) {
        if (length < offset + 6) return; // Check length before advancing
        offset += 6;
    }

    // Now check for crank data at the current offset
    if (crankDataPresent) {
        if (length < offset + 4) return; // Need 2 bytes revs, 2 bytes time

        uint16_t cumulativeCrankRevs = pData[offset] | (pData[offset+1] << 8);
        uint16_t crankEventTime_1024 = pData[offset+2] | (pData[offset+3] << 8); // Unit 1/1024s

        // --- Calculation logic (same as before, but using dedicated sensor data) ---
        uint32_t timeDelta_1024 = (crankEventTime_1024 >= lastCrankEventTime) ?
                                  (crankEventTime_1024 - lastCrankEventTime) :
                                  ((0xFFFF - lastCrankEventTime) + crankEventTime_1024 + 1);

        uint16_t crankRevDelta = (cumulativeCrankRevs >= lastCrankRevs) ?
                                 (cumulativeCrankRevs - lastCrankRevs) :
                                 ((0xFFFF - lastCrankRevs) + cumulativeCrankRevs + 1);

        if (crankRevDelta > 0 && timeDelta_1024 > 0) {
            float timeDeltaSec = (float)timeDelta_1024 / 1024.0f;
            currentCadenceRadPerSec = ((float)crankRevDelta / timeDeltaSec) / (2.0f * PI);
            lastNonZeroCrankTime = millis();
        } else if (cumulativeCrankRevs == lastCrankRevs) {
            if (millis() - lastNonZeroCrankTime > BLE_CADENCE_TIMEOUT_MS) {
                currentCadenceRadPerSec = 0.0f;
            }
        }
        lastCrankRevs = cumulativeCrankRevs;
        lastCrankEventTime = crankEventTime_1024;
        // offset += 4; // Only needed if parsing data *after* crank data

    } else {
        // Crank data flag not set, check timeout
        if (millis() - lastNonZeroCrankTime > BLE_CADENCE_TIMEOUT_MS) {
            currentCadenceRadPerSec = 0.0f;
        }
    }
}


// --- dumpCSVOverSerial ---
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
    if(bytesRead > 0) Serial.write((const uint8_t*)buf, bytesRead);
    else break;
  }
  Serial.println("\n--- END LOG DUMP ---");
  file.close();
  Serial.println("Dump complete. System halted. Reset device to continue.");
  updateLEDStatus(STATUS_ERROR, true);
  while (true) {
       pixel.setPixelColor(0, pixel.Color(255, 255, 0)); pixel.show(); delay(500);
       pixel.setPixelColor(0, pixel.Color(0, 0, 0)); pixel.show(); delay(500);
  }
}

// --- initDisplay ---
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

// --- updateDisplay ---
void updateDisplay(float currentRawDeltaP) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);

    // Status Line
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

    // Power Display
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 25);
    tft.print("P:");
    if (powerConnected) { tft.print(currentPower, 0); } else { tft.print("---"); }
    tft.print("W ");

    // Speed Display
    int16_t x_speed_start = tft.width() / 2 + 5;
    tft.setCursor(x_speed_start, 25);
    tft.print("S:");
     if (cscConnected) { // Speed sensor connected
       char speedStr[6]; dtostrf(currentSpeedKph, 4, 1, speedStr); tft.print(speedStr);
    } else { tft.print("--.-"); }

    // Airspeed Display
    float currentAirspeedKph = airspeedPaToKph(currentRawDeltaP);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(5, 50);
    tft.print("Air: ");
     if (ms4525Ready && !isnan(currentAirspeedKph)) {
       char airspeedStr[6]; dtostrf(currentAirspeedKph, 4, 1, airspeedStr); tft.print(airspeedStr);
    } else { tft.print("--.-"); }
    tft.print("kph");

    // Cadence Display (using dedicated sensor value)
     tft.setCursor(x_speed_start, 50);
     tft.setTextColor(ST77XX_ORANGE);
     tft.print("Cad: ");
     if (cadenceConnected) { // Cadence sensor connected
         float cadenceRPM = currentCadenceRadPerSec * (60.0f / (2.0f * PI));
         tft.print(cadenceRPM, 0); // Display RPM as integer
     } else {
         tft.print("---");
     }
     tft.print("rpm");

     // Speed Units
     tft.setTextSize(1);
     tft.setTextColor(ST77XX_WHITE);
     tft.setCursor(x_speed_start + 50, 35);
     tft.print("kph");
}