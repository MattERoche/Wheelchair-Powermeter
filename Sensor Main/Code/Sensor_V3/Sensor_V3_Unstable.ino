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
#include <Adafruit_ST7735.h> // Library for ST7735S controller

// =============================================================
// ==                     Configuration                     ==
// =============================================================

// --- Pin Definitions ---
#define MODE_PIN 6          // Input for Dump/Delete Mode
#define RECORD_PIN 5         // Input for Start/Stop Recording
#define NEOPIXEL_PIN 33      // NeoPixel Data Pin
#define NEOPIXEL_PWR_PIN 21  // Pin to enable power to NeoPixel

// --- NeoPixel Settings ---
#define NEOPIXEL_COUNT 1
#define NEOPIXEL_BRIGHTNESS 50 // Brightness (0-255)


// --- I2C Settings ---
#define I2C_CLOCK_SPEED 400000 // 400kHz

// --- Sensor Settings ---
#define ICM_ACCEL_RATE_DIV 10 // Target ~100Hz (1.125kHz / (1 + 10))
#define ICM_GYRO_RATE_DIV 10  // Target ~100Hz (1.1kHz / (1 + 10))
// MPL3115A2 OSR setting (0=OS1, 1=OS2, ... 7=OS128) - Lower is Faster
#define MPL_OSR_SETTING 0 // Use OS1 (0b000) for fastest reads (~4-10ms)

// --- Logging Settings ---
const char* LOG_FILENAME = "/log.csv";
const unsigned long LOG_INTERVAL_MS = 100; // Target logging frequency (10 Hz)
const unsigned long FLUSH_INTERVAL_MS = 1000; // How often to force flush buffer (ms)
const size_t MAX_LOG_BUFFER_SIZE = 1024; // Max bytes before forcing flush
const int CSV_PRECISION_ACCEL = 2;
const int CSV_PRECISION_GYRO = 2;
const int CSV_PRECISION_PRESSURE = 1;
const int CSV_PRECISION_TEMP = 1;
const int CSV_PRECISION_AIRSPEED = 1;
const int CSV_PRECISION_SPEED_KPH = 2;
const int CSV_PRECISION_POWER = 0;
const int CSV_windspeed = 2;
const int CSV_BUFFER_LINE_LENGTH = 200; // Estimated max length for snprintf buffer

// --- BLE Settings ---
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_CHAR_UUID((uint16_t)0x2A5B);
static BLEUUID CYCLING_POWER_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CYCLING_POWER_MEASUREMENT_CHAR_UUID((uint16_t)0x2A63);
const unsigned long BLE_SCAN_INTERVAL_MS = 100;
const unsigned long BLE_SCAN_WINDOW_MS = 99;
const unsigned long BLE_SCAN_DURATION_S = 3;
const unsigned long BLE_CONNECT_RETRY_INTERVAL_MS = 5000; // How often to attempt scans/connects
const float WHEEL_CIRCUMFERENCE_METERS = 2.105f; // Adjust for specific wheel size
const unsigned long BLE_SPEED_TIMEOUT_MS = 3000; // Time without movement before speed -> 0

// =============================================================
// ==                  Global Variables                     ==
// =============================================================

// --- System State ---
enum SystemStatus { STATUS_ERROR, STATUS_CONNECTING, STATUS_READY, STATUS_RECORDING };
SystemStatus currentStatus = STATUS_CONNECTING;
SystemStatus lastDisplayedStatus = STATUS_ERROR; // Force initial update
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

// --- Sensor Data ---
float currentSpeedKph = 0.0f;
float currentPower = 0.0f;
float lastPressure = 0.0f;
float lastTemp = 0.0f;
float lastAirspeed = 0.0f;
float lastAirTemp = 0.0f;
// For speed calculation
uint32_t lastWheelRevs = 0;
uint16_t lastWheelEventTime = 0; // 1/1024s resolution
unsigned long lastNonZeroRevTime = 0; // ms timestamp

// --- Sensor Readiness ---
bool icmReady = false;
bool mplReady = false;
bool ms4525Ready = false;


// Pins for the DFRobot 0.96" 160x80 Color SPI TFT (SKU: DFR0847)
#define TFT_CS    10  // TFT Chip Select Pin (Goes to CS)
#define TFT_DC    12  // TFT Data/Command Pin (Goes to DC / A0)
#define TFT_RST   11  // TFT Reset Pin (Goes to RES). Can often be tied to ESP32 RST pin, or set to -1 if not used and tied HIGH.
#define TFT_BLK   13  // TFT Backlight Control Pin (Goes to BLK). Set to -1 if not used or tied HIGH.
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 1000;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- Sensor Objects ---
Adafruit_ICM20948 icm;
Adafruit_MPL3115A2 mpl;
bfs::Ms4525do ms4525do(&Wire, 0x28, 1.0f, -1.0f);

// --- NeoPixel Object ---
Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- Logging Buffer ---
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
bool setMplOsr(uint8_t osr); // Helper for MPL3115A2 OSR setting

// Constants for MPL3115A2 registers (if not exposed by library)
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
    // ... (Existing logic to check service UUIDs and store candidates) ...
     if (!cscConnected && foundCSCDevice == nullptr && advertisedDevice.isAdvertisingService(CSC_SERVICE_UUID)) {
        Serial.println("🚲 CSC device candidate found! Storing...");
        foundCSCDevice = new BLEAdvertisedDevice(advertisedDevice);
      }
      else if (!powerConnected && foundPowerDevice == nullptr && advertisedDevice.isAdvertisingService(CYCLING_POWER_SERVICE_UUID)) {
        Serial.println("⚡ Power Meter candidate found! Storing...");
        foundPowerDevice = new BLEAdvertisedDevice(advertisedDevice);
      }


    // Stop scanning if we have candidates for all needed devices
    bool gotCscCandidate = cscConnected || (foundCSCDevice != nullptr);
    bool gotPowerCandidate = powerConnected || (foundPowerDevice != nullptr);
    if (gotCscCandidate && gotPowerCandidate) {
      // FIX: Remove isScanning check, just stop if needed
      if(pBLEScan != nullptr && bleScanInProgress) { // Check our flag
           Serial.println("Found candidates for all needed devices. Stopping scan.");
           pBLEScan->stop();
           bleScanInProgress = false; // <<<--- UPDATE FLAG: Scan stopped
      }
    }
  }
};
bool setupCSCNotifications(BLEClient* client); // Helper for CSC setup
bool setupPowerNotifications(BLEClient* client); // Helper for Power setup
#define AIR_DENSITY 1.22

// =============================================================
// ==                     Setup Function                      ==
// =============================================================
void setup() {
  // --- Basic Initialization ---
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(RECORD_PIN, INPUT_PULLUP);
  pinMode(NEOPIXEL_PWR_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_PWR_PIN, HIGH); // Power on NeoPixel

  Serial.begin(115200);
  unsigned long bootStart = millis();
  while (!Serial && (millis() - bootStart < 2000)) {;} // Wait max 2s for serial
  Serial.println("\n\n--- System Boot ---");

  // --- NeoPixel Init ---
  pixel.begin();
  pixel.setBrightness(NEOPIXEL_BRIGHTNESS);
  updateLEDStatus(STATUS_CONNECTING, true); // Initial status

  // --- SPIFFS Init & Mode Check ---
  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS mount failed!");
    updateLEDStatus(STATUS_ERROR, true);
    while (1) delay(10);
  }
  Serial.println("✅ SPIFFS Mounted.");
  if (digitalRead(MODE_PIN) == LOW) {
    Serial.println(" DUMP MODE DETECTED ");
    dumpCSVOverSerial(); // This function halts execution
  }

  initDisplay();
  tft.fillScreen(ST77XX_BLACK); // Initial clear screen
  tft.setCursor(5, 5);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.println("Booting...");


  // --- I2C Bus Init ---
  Serial.println("Initializing I2C Bus...");
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  // --- I2C Sensors Init ---
  Serial.println("Initializing I2C Sensors...");
  // ICM20948 (IMU)
  if (!icm.begin_I2C()) {
    Serial.println("❌ ICM20948 not found"); updateLEDStatus(STATUS_ERROR, true); while (1) delay(10);
  }
  icm.setAccelRateDivisor(ICM_ACCEL_RATE_DIV);
  icm.setGyroRateDivisor(ICM_GYRO_RATE_DIV);
  icmReady = true;
  Serial.println("✅ ICM20948 ready");

  // MPL3115A2 (Pressure/Temp)
  if (!mpl.begin()) {
    Serial.println("❌ MPL3115A2 not found"); updateLEDStatus(STATUS_ERROR, true); while (1) delay(10);
  }
  mplReady = true;
  Serial.println("✅ MPL3115A2 ready (Default OSR=128)");
  // *** Set faster OSR for MPL3115A2 ***
  if (!setMplOsr(MPL_OSR_SETTING)) {
      Serial.println("⚠️ Failed to set MPL3115A2 OSR. Readings will be slow!");
      // Continue, but performance will be bad
  } else {
      Serial.print("✅ MPL3115A2 OSR set to: "); Serial.println(MPL_OSR_SETTING);
  }


  // MS4525DO (Airspeed)
  if (!ms4525do.Begin()) {
    Serial.println("❌ MS4525DO not found"); updateLEDStatus(STATUS_ERROR, true); while (1) delay(10);
  }
  ms4525Ready = true;
  Serial.println("✅ MS4525DO ready");

  // --- BLE Init ---
  Serial.println("Initializing BLE...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(BLE_SCAN_INTERVAL_MS);
  pBLEScan->setWindow(BLE_SCAN_WINDOW_MS);
  pBLEScan->setActiveScan(true);
  // Scanning/Connection attempts are handled in the main loop

  
  Serial.println("--- Setup Complete ---");
  tft.setCursor(5, 15);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("Setup Complete!");
  delay(1000); // Show setup complete message briefly
}


// =============================================================
// ==                    Main Loop                          ==
// =============================================================
void loop() {
  unsigned long nowMillis = millis();

  // --- 1. Handle BLE Connection State ---
  bool needScanOrConnect = false;
  if (!bleInitAttempted) needScanOrConnect = true; // Always try on first loop
  if (!cscConnected && foundCSCDevice == nullptr) needScanOrConnect = true;
  if (!powerConnected && foundPowerDevice == nullptr) needScanOrConnect = true;
  if (foundCSCDevice != nullptr && !cscConnected) needScanOrConnect = true; // Have candidate, need connect
  if (foundPowerDevice != nullptr && !powerConnected) needScanOrConnect = true;// Have candidate, need connect

  if (needScanOrConnect) {
    static unsigned long lastBleActionAttempt = 0;
    if (nowMillis - lastBleActionAttempt >= BLE_CONNECT_RETRY_INTERVAL_MS) {
      Serial.println("Checking BLE state (Scan/Connect)...");
      connectBLEDevices(); // This function handles scanning OR connecting
      lastBleActionAttempt = nowMillis;
      bleInitAttempted = true; // Mark that at least one attempt cycle has occurred
    }
  }
  // Check for unexpected disconnects
  if (cscConnected && (cscClient == nullptr || !cscClient->isConnected())) {
    Serial.println("⚠️ CSC Disconnected Unexpectedly");
    cscConnected = false; // Reset state
    // Optionally: delete cscClient; cscClient = nullptr; // Clean up client object
  }
  if (powerConnected && (powerClient == nullptr || !powerClient->isConnected())) {
    Serial.println("⚠️ Power Meter Disconnected Unexpectedly");
    powerConnected = false; // Reset state
    // Optionally: delete powerClient; powerClient = nullptr;
  }


  // --- 2. Determine System Readiness & Status ---
  bool allSensorsReady = icmReady && mplReady && ms4525Ready;
  bool allBleReady = cscConnected && powerConnected;
  bool systemReady = allSensorsReady && allBleReady;

  if (recordingStarted) {
    currentStatus = STATUS_RECORDING;
  } else if (systemReady) {
    currentStatus = STATUS_READY;
  } else if (!allSensorsReady) {
    currentStatus = STATUS_ERROR; // Prioritize sensor error over connecting state
  } else {
    currentStatus = STATUS_CONNECTING;
  }
  updateLEDStatus(currentStatus); // Update LED based on calculated status


  // --- 3. Handle Recording Start/Stop ---
  if (!recordingStarted) {
    // Check for button press to START recording
    static bool buttonWasPressed = false;
    int buttonState = digitalRead(RECORD_PIN);

    if (buttonState == LOW && !buttonWasPressed) { // Button pressed edge
      buttonWasPressed = true;
      Serial.println("Record button pressed...");
    } else if (buttonState == HIGH && buttonWasPressed) { // Button released edge
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
           updateLEDStatus(STATUS_ERROR, true); // Show error immediately
        } else {
           Serial.println("✅ New log file created.");
           // Write header
           file.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure,Temp,DeltaP,AirspeedTemp,Speed,Power,windspeed");
           file.close();
           Serial.println("✅ Header written. Recording starting.");
           recordingStarted = true;
           logBuffer = ""; // Clear buffer
           lastLogMillis = nowMillis; // Reset log timer
           lastFlushMillis = nowMillis; // Reset flush timer
        }
      } else {
         Serial.println("❌ Cannot start recording: System not ready!");
         if (!allSensorsReady) Serial.println(" -> I2C sensors not ready.");
         if (!allBleReady) Serial.println(" -> BLE sensors not ready.");
         // LED will show Connecting/Error status automatically
      }
    }
  } else {
    // Optional: Add logic here to STOP recording if button is pressed again
    // if (digitalRead(RECORD_PIN) == LOW) { ... stop logic ...; flushLogBuffer(); recordingStarted = false; }
  }


  // --- 4. Read I2C Sensors ---
  sensors_event_t accel, gyro, temp_imu; // Separate IMU temp object
  if (icmReady) icm.getEvent(&accel, &gyro, &temp_imu);
  if (mplReady) {
      lastPressure = mpl.getPressure();
      lastTemp = mpl.getTemperature();
  }
  if (ms4525Ready) {
      if (ms4525do.Read()) {
          lastAirspeed = ms4525do.pres_pa();
          lastAirTemp = ms4525do.die_temp_c();
      } else {
          lastAirspeed = NAN; lastAirTemp = NAN; // Indicate read failure
      }
  }
  // Note: BLE sensor data (speed, power) is updated asynchronously via callbacks


  // --- 5. Log Data to Buffer (if recording) ---
  if (recordingStarted && (nowMillis - lastLogMillis >= LOG_INTERVAL_MS)) {
    // Increment log time robustly against loop variations
    lastLogMillis += LOG_INTERVAL_MS;
    // Ensure lastLogMillis doesn't fall behind excessively if loop stalls
    if(lastLogMillis < nowMillis - LOG_INTERVAL_MS) {
        lastLogMillis = nowMillis;
    }
    float currentAirspeedKph = airspeedPaToKph(lastAirspeed); // Calculate the value


    // Format data string using snprintf for efficiency
    char csvLine[CSV_BUFFER_LINE_LENGTH];
    snprintf(csvLine, sizeof(csvLine),
             "%.3f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f",
             nowMillis / 1000.0f,           
             CSV_PRECISION_ACCEL, accel.acceleration.x,
             CSV_PRECISION_ACCEL, accel.acceleration.y,
             CSV_PRECISION_ACCEL, accel.acceleration.z,
             CSV_PRECISION_GYRO, gyro.gyro.x,
             CSV_PRECISION_GYRO, gyro.gyro.y,
             CSV_PRECISION_GYRO, gyro.gyro.z,
             CSV_PRECISION_PRESSURE, lastPressure,
             CSV_PRECISION_TEMP, lastTemp,           
             CSV_PRECISION_AIRSPEED, lastAirspeed,
             CSV_PRECISION_TEMP, lastAirTemp,        
             CSV_PRECISION_SPEED_KPH, currentSpeedKph,
             CSV_PRECISION_POWER, currentPower,
             CSV_windspeed, currentAirspeedKph
            );

    logBuffer += csvLine;
    logBuffer += "\n";
  }


  // --- 6. Flush Log Buffer Periodically ---
  bool timeToFlush = (nowMillis - lastFlushMillis >= FLUSH_INTERVAL_MS);
  bool bufferGettingFull = (logBuffer.length() >= MAX_LOG_BUFFER_SIZE); // Use >= for safety

  if (recordingStarted && (bufferGettingFull || timeToFlush)) {
       flushLogBuffer(); // Handles writing and clearing buffer
  }

  // *** NEW: 7. Update Display Periodically ***
  if (nowMillis - lastDisplayUpdateMillis >= DISPLAY_UPDATE_INTERVAL_MS) {
      updateDisplay();
      lastDisplayUpdateMillis = nowMillis;
  }

  // --- 8. Yield ---
  delay(1);
}


// =============================================================
// ==                  Helper Functions                     ==
// =============================================================

// --- LED Status Update ---
void updateLEDStatus(SystemStatus status, bool forceUpdate) {
  if (!forceUpdate && status == lastDisplayedStatus) return; // Avoid unnecessary updates

  lastDisplayedStatus = status; // Store the latest status shown

  // Special handling for blinking recording LED
  if (status == STATUS_RECORDING) {
    bool ledOn = (millis() / 500) % 2 == 0; // Blink state changes every 500ms
    pixel.setPixelColor(0, ledOn ? pixel.Color(128, 0, 128) : pixel.Color(0, 0, 0)); // Purple/Off
    pixel.show();
    return; // Skip further processing for recording blink
  }

  // Handle non-blinking states
  Serial.print("System Status -> ");
  switch (status) {
    case STATUS_ERROR:
      Serial.println("🔴 ERROR");
      pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // Solid Red
      break;
    case STATUS_CONNECTING:
      Serial.println("🔵 CONNECTING");
      pixel.setPixelColor(0, pixel.Color(0, 0, 255)); // Solid Blue
      break;
    case STATUS_READY:
      Serial.println("🟢 READY");
      pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // Solid Green
      break;
    default: // Should not happen
      pixel.setPixelColor(0, pixel.Color(50, 50, 50)); // White?
      break;
  }
  pixel.show();
}

// --- BLE Connection Logic ---
// --- BLE Connection Logic ---
// =============================================================
// ==                  Helper Functions                     ==
// =============================================================

// ... (updateLEDStatus, flushLogBuffer, setMplOsr, etc.)


// --- Helper: Setup Notifications for CSC Sensor ---
bool setupCSCNotifications(BLEClient* client) {
    if (!client || !client->isConnected()) {
        Serial.println("  ❌ setupCSCNotifications: Client is null or not connected.");
        return false;
    }

    Serial.println("  Setting up CSC Notifications...");
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

    // 1. Get the Service
    try {
        pRemoteService = client->getService(CSC_SERVICE_UUID);
    } catch (const std::exception& e) {
        Serial.print("  ❌ Exception getting CSC service: "); Serial.println(e.what());
        return false;
    }
    if (pRemoteService == nullptr) {
        Serial.print("  ❌ Failed to find CSC service UUID: ");
        Serial.println(CSC_SERVICE_UUID.toString().c_str());
        return false;
    }
    Serial.println("  ✅ Found CSC service.");

    // 2. Get the Characteristic
    try {
       pRemoteCharacteristic = pRemoteService->getCharacteristic(CSC_MEASUREMENT_CHAR_UUID);
    } catch (const std::exception& e) {
        Serial.print("  ❌ Exception getting CSC characteristic: "); Serial.println(e.what());
        return false;
    }
    if (pRemoteCharacteristic == nullptr) {
        Serial.print("  ❌ Failed to find CSC characteristic UUID: ");
        Serial.println(CSC_MEASUREMENT_CHAR_UUID.toString().c_str());
        return false;
    }
    Serial.println("  ✅ Found CSC characteristic.");

    // 3. Register for Notifications
    if (pRemoteCharacteristic->canNotify()) {
        Serial.println("  Attempting to register for CSC notifications...");
        try {
            // The 'true' argument enables notifications on the server side (CCCD write)
            pRemoteCharacteristic->registerForNotify(notifyCallback, true);
            Serial.println("  ✅ CSC Notification registration attempted.");
            // Note: Actual success confirmed when callback receives data.
            return true; // Registration *attempt* made successfully
        } catch (const std::exception& e) {
            Serial.print("  ❌ Exception during CSC registerForNotify: "); Serial.println(e.what());
            return false;
        } catch (...) {
            Serial.println("  ❌ Unknown exception during CSC registerForNotify!");
            return false;
        }
    } else {
        Serial.println("  ❌ CSC characteristic does not support notifications!");
        return false; // Cannot proceed if notify isn't supported
    }
}

// --- Helper: Setup Notifications for Power Meter ---
bool setupPowerNotifications(BLEClient* client) {
     if (!client || !client->isConnected()) {
        Serial.println("  ❌ setupPowerNotifications: Client is null or not connected.");
        return false;
    }

    Serial.println("  Setting up Power Notifications...");
    BLERemoteService* pRemoteService = nullptr;
    BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

    // 1. Get the Service
    try {
        pRemoteService = client->getService(CYCLING_POWER_SERVICE_UUID);
    } catch (const std::exception& e) {
        Serial.print("  ❌ Exception getting Power service: "); Serial.println(e.what());
        return false;
    }
    if (pRemoteService == nullptr) {
        Serial.print("  ❌ Failed to find Power service UUID: ");
        Serial.println(CYCLING_POWER_SERVICE_UUID.toString().c_str());
        return false;
    }
    Serial.println("  ✅ Found Power service.");

    // 2. Get the Characteristic
    try {
        pRemoteCharacteristic = pRemoteService->getCharacteristic(CYCLING_POWER_MEASUREMENT_CHAR_UUID);
    } catch (const std::exception& e) {
        Serial.print("  ❌ Exception getting Power characteristic: "); Serial.println(e.what());
        return false;
    }
     if (pRemoteCharacteristic == nullptr) {
        Serial.print("  ❌ Failed to find Power characteristic UUID: ");
        Serial.println(CYCLING_POWER_MEASUREMENT_CHAR_UUID.toString().c_str());
        return false;
    }
    Serial.println("  ✅ Found Power characteristic.");

    // 3. Register for Notifications
    if (pRemoteCharacteristic->canNotify()) {
        Serial.println("  Attempting to register for Power notifications...");
         try {
             // The 'true' argument enables notifications on the server side (CCCD write)
            pRemoteCharacteristic->registerForNotify(powerNotifyCallback, true); // Use the correct callback!
            Serial.println("  ✅ Power Notification registration attempted.");
            // Note: Actual success confirmed when callback receives data.
            return true; // Registration *attempt* made successfully
         } catch (const std::exception& e) {
            Serial.print("  ❌ Exception during Power registerForNotify: "); Serial.println(e.what());
            return false;
        } catch (...) {
            Serial.println("  ❌ Unknown exception during Power registerForNotify!");
            return false;
        }
    } else {
         Serial.println("  ❌ Power characteristic does not support notifications!");
         return false; // Cannot proceed if notify isn't supported
    }
}


// --- Main BLE Connection Logic (Revised) ---
void connectBLEDevices() {
    // 1. Scan if needed (Scan logic remains the same as before)
    bool needScan = (!cscConnected && foundCSCDevice == nullptr) ||
                    (!powerConnected && foundPowerDevice == nullptr);

    if (needScan && pBLEScan != nullptr && !bleScanInProgress) {
        Serial.print("🔍 Starting BLE scan (");
        Serial.print(BLE_SCAN_DURATION_S); Serial.println("s)...");
        // Clear old candidates *before* scan, only if not connected
        if (!cscConnected && foundCSCDevice) { delete foundCSCDevice; foundCSCDevice = nullptr; }
        if (!powerConnected && foundPowerDevice) { delete foundPowerDevice; foundPowerDevice = nullptr; }

        bleScanInProgress = true;
        pBLEScan->start(BLE_SCAN_DURATION_S, false); // Assuming blocking or callback stops scan

        // --- Assuming scan finishes or is stopped by callback ---
        bleScanInProgress = false;
        Serial.println("...BLE Scan Finished or stopped.");
        // --- End Scan Assumption ---

    } else if (bleScanInProgress) {
        Serial.println("(Scan already in progress...)");
    }

    // 2. Attempt Connection and Setup for CSC
    if (!cscConnected && foundCSCDevice != nullptr) {
        if (cscClient == nullptr) {
            cscClient = BLEDevice::createClient();
            if (!cscClient) {
                Serial.println("❌ Failed to create CSC BLE Client! Retrying later.");
                // Keep foundCSCDevice for next attempt cycle
                return; // Exit for now, will retry creating client next time
            }
            Serial.println("CSC BLE Client created.");
            // Optional: cscClient->setClientCallbacks(new MyClientCallbacks());
        }

        if (!cscClient->isConnected()) { // Attempt connect only if not already connected
            Serial.print("Attempting CSC Connect to: "); Serial.println(foundCSCDevice->getAddress().toString().c_str());
            bool connectSuccess = false;
            try {
                 connectSuccess = cscClient->connect(foundCSCDevice);
            } catch (const std::exception& e) {
                 Serial.print("  ❌ Exception during CSC connect: "); Serial.println(e.what());
                 connectSuccess = false;
            } catch (...) {
                 Serial.println("  ❌ Unknown exception during CSC connect!");
                 connectSuccess = false;
            }


            if (connectSuccess) {
                Serial.println("✅ CSC Base Connection Successful. Setting up notifications...");
                // Now try to set up notifications using the helper
                if (setupCSCNotifications(cscClient)) {
                    Serial.println("✅✅ CSC Fully Ready (Connected & Notifications Setup)");
                    cscConnected = true; // Mark as fully ready
                    delete foundCSCDevice; // Success, clean up candidate
                    foundCSCDevice = nullptr;
                } else {
                    Serial.println("❌ CSC Setup Failed (Notifications). Disconnecting.");
                    cscClient->disconnect(); // Disconnect if setup failed
                    // Keep foundCSCDevice for next attempt cycle
                }
            } else {
                Serial.println("❌ CSC connect() call failed.");
                // Keep foundCSCDevice for next attempt cycle
            }
        } else {
           // This case shouldn't normally happen if cscConnected flag is managed correctly
           Serial.println("(CSC Client exists and thinks it's connected, but cscConnected=false? Flag mismatch?)");
           // Consider verifying connection: if(!cscClient->isConnected()) { /* try connect */ }
        }
    } // End CSC attempt

    // 3. Attempt Connection and Setup for Power (Mirror CSC logic)
    if (!powerConnected && foundPowerDevice != nullptr) {
         if (powerClient == nullptr) {
            powerClient = BLEDevice::createClient();
             if (!powerClient) {
                Serial.println("❌ Failed to create Power BLE Client! Retrying later.");
                return;
            }
            Serial.println("Power BLE Client created.");
             // Optional: powerClient->setClientCallbacks(new MyClientCallbacks());
        }

        if (!powerClient->isConnected()) { // Attempt connect only if not already connected
            Serial.print("Attempting Power Connect to: "); Serial.println(foundPowerDevice->getAddress().toString().c_str());
             bool connectSuccess = false;
            try {
                 connectSuccess = powerClient->connect(foundPowerDevice);
            } catch (const std::exception& e) {
                 Serial.print("  ❌ Exception during Power connect: "); Serial.println(e.what());
                 connectSuccess = false;
            } catch (...) {
                 Serial.println("  ❌ Unknown exception during Power connect!");
                 connectSuccess = false;
            }

            if (connectSuccess) {
                Serial.println("✅ Power Base Connection Successful. Setting up notifications...");
                if (setupPowerNotifications(powerClient)) {
                    Serial.println("✅✅ Power Meter Fully Ready (Connected & Notifications Setup)");
                    powerConnected = true; // Mark as fully ready
                    delete foundPowerDevice; // Success, clean up candidate
                    foundPowerDevice = nullptr;
                } else {
                    Serial.println("❌ Power Setup Failed (Notifications). Disconnecting.");
                    powerClient->disconnect(); // Disconnect if setup failed
                }
            } else {
                Serial.println("❌ Power connect() call failed.");
            }
        } else {
            Serial.println("(Power Client exists and thinks it's connected, but powerConnected=false? Flag mismatch?)");
        }
    } // End Power attempt

} // End connectBLEDevices

// --- Log Buffer Flushing ---
void flushLogBuffer() {
  if (logBuffer.length() == 0) return;

  // Serial.print("Flushing buffer, size: "); Serial.println(logBuffer.length()); // Debug
  unsigned long writeStart = millis();
  File file = SPIFFS.open(LOG_FILENAME, FILE_APPEND);

  if (!file) {
    Serial.println("❌ Failed to open log file for flushing!");
    updateLEDStatus(STATUS_ERROR, true);
    // Consider clearing buffer? logBuffer = ""; Might lose data.
    return;
  }

  size_t written = file.print(logBuffer);
  file.close(); // Implicitly flushes

  if (written == logBuffer.length()) {
     logBuffer = ""; // Clear buffer ONLY on successful write
     // Serial.print("Buffer flushed successfully in "); Serial.print(millis() - writeStart); Serial.println(" ms"); // Debug
  } else {
      Serial.print("❌ Log buffer flush failed! Expected "); Serial.print(logBuffer.length());
      Serial.print(" wrote "); Serial.println(written);
      updateLEDStatus(STATUS_ERROR, true);
      // Don't clear buffer on failure, maybe retry next time?
  }
  lastFlushMillis = millis(); // Update time regardless to prevent rapid retries on fail
}

// --- Set MPL3115A2 Oversampling Rate ---
// osr: 0=OS1, 1=OS2, 2=OS4, 3=OS8, 4=OS16, 5=OS32, 6=OS64, 7=OS128
bool setMplOsr(uint8_t osr) {
  if (osr > 7) return false; // Invalid OSR setting

  Serial.print("Setting MPL3115A2 OSR via Wire (0=OS1..7=OS128): "); Serial.println(osr);

  // Step 1: Read current CTRL_REG1
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(MPL3115A2_CTRL_REG1);
  if (Wire.endTransmission(false) != 0) { // Send address, reg pointer, NO STOP
     Serial.println("  ❌ Error sending register pointer"); return false;
  }
  uint8_t bytesRead = Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)1);
  if (bytesRead != 1) {
      Serial.println("  ❌ Error reading current CTRL_REG1"); return false;
  }
  uint8_t currentCtrl1 = Wire.read();
  // Serial.print("  Read CTRL_REG1: 0x"); Serial.println(currentCtrl1, HEX); // Debug

  // Step 2: Modify the OSR bits
  uint8_t newCtrl1 = currentCtrl1;
  newCtrl1 &= ~0b00111000;    // Clear current OSR bits (bits 3, 4, 5)
  newCtrl1 |= (osr & 0x07) << 3; // Set new OSR bits

  // Step 3: Write modified value back
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(MPL3115A2_CTRL_REG1); // Register address
  Wire.write(newCtrl1);            // New value
  if (Wire.endTransmission() != 0) { // Send data, STOP
      Serial.println("  ❌ Error writing new CTRL_REG1"); return false;
  }

  // Serial.print("  Successfully wrote CTRL_REG1: 0x"); Serial.println(newCtrl1, HEX); // Debug
  return true; // Success
}


// --- BLE Notification Callbacks ---
void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length < 7) return; // Need flags, revs(4), time(2)

    uint8_t flags = pData[0];
    bool wheelDataPresent = flags & 0x01;

    if (wheelDataPresent) {
        uint32_t cumulativeRevs = pData[1] | (pData[2] << 8) | (pData[3] << 16) | (pData[4] << 24);
        uint16_t wheelEventTime = pData[5] | (pData[6] << 8); // 1/1024s

        // Handle time wrap-around (uint16_t)
        uint32_t timeDelta_1024 = (wheelEventTime >= lastWheelEventTime) ?
                                  (wheelEventTime - lastWheelEventTime) :
                                  ((0xFFFF - lastWheelEventTime) + wheelEventTime + 1);

        // Handle revs wrap-around (uint32_t)
        uint32_t revDelta = (cumulativeRevs >= lastWheelRevs) ?
                             (cumulativeRevs - lastWheelRevs) :
                             ((0xFFFFFFFF - lastWheelRevs) + cumulativeRevs + 1);


        if (revDelta > 0 && timeDelta_1024 > 0) {
            float timeDeltaSec = (float)timeDelta_1024 / 1024.0f;
            float distanceMeters = (float)revDelta * WHEEL_CIRCUMFERENCE_METERS;
            float speedMps = distanceMeters / timeDeltaSec;
            currentSpeedKph = speedMps * 3.6f;
            lastNonZeroRevTime = millis();
        } else if (cumulativeRevs == lastWheelRevs) { // No change
             if (millis() - lastNonZeroRevTime > BLE_SPEED_TIMEOUT_MS) {
                 currentSpeedKph = 0.0f;
             }
        }
        // Update state for next calculation
        lastWheelRevs = cumulativeRevs;
        lastWheelEventTime = wheelEventTime;
    }
}

// --- Airspeed Calculation ---
float airspeedPaToKph(float pressure_pa) {
    if (isnan(pressure_pa) || pressure_pa < 0) {
        return NAN; // Cannot calculate speed from invalid pressure
    }
    // Speed (m/s) = sqrt(2 * PressureDiff / AirDensity)
    float speed_mps = sqrtf((2.0f * pressure_pa) / AIR_DENSITY);
    // Convert m/s to kph
    return speed_mps * 3.6f;
}
void powerNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (length < 4) return; // Need flags(2) and power(2)
  // Power is signed 16-bit int at index 2
  int16_t watts = pData[2] | (pData[3] << 8);
  currentPower = (float)watts;
}


// --- Dump Logs Function ---
void dumpCSVOverSerial() {
  Serial.print("Attempting to dump log file: "); Serial.println(LOG_FILENAME);
  File file = SPIFFS.open(LOG_FILENAME, "r");
  if (!file || file.isDirectory()) {
    Serial.println("❌ Log file not found or is directory.");
    updateLEDStatus(STATUS_ERROR, true);
    delay(3000);
    return; // Allow potential restart or normal boot
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
  updateLEDStatus(STATUS_ERROR, true); // Show error state
  while (true) { // Halt execution
       pixel.setPixelColor(0, pixel.Color(255, 255, 0)); delay(500); // Blink Yellow
       pixel.setPixelColor(0, pixel.Color(0, 0, 0)); delay(500);
       pixel.show();
  }
}

// --- Optional: Delete Logs Function ---
void deleteAllLogFiles() {
  Serial.println("Deleting all *.csv files...");
  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) { Serial.println("Failed to open root directory."); return; }

  File file = root.openNextFile();
  int count = 0;
  while (file) {
    String currentFilename = file.name();
    if (currentFilename.endsWith(".csv")) {
      Serial.print("Deleting: "); Serial.print(currentFilename);
      if (SPIFFS.remove(currentFilename)) { Serial.println(" -> Success"); count++; }
      else { Serial.println(" -> FAILED"); }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
  Serial.print("Deleted "); Serial.print(count); Serial.println(" file(s).");
}

// --- Display Init ---
// --- Display Init ---
void initDisplay() {
    Serial.println("Initializing TFT Display...");

    // Control Reset pin (optional but good practice if defined)
    if (TFT_RST >= 0) {
        pinMode(TFT_RST, OUTPUT);
        digitalWrite(TFT_RST, LOW);
        delay(10);
        digitalWrite(TFT_RST, HIGH);
        delay(10);
    }

    // Initialize ST7735S chip for 160x80 display
    tft.initR(INITR_MINI160x80); // Use this for 0.96" 160x80

    // Set rotation (Adjust based on how you mount the display)
    tft.setRotation(1); // Landscape (Screen ribbon cable likely on left or right)
                        // Use 3 for landscape the other way.

    tft.fillScreen(ST77XX_BLACK);
    Serial.println("✅ TFT Initialized.");

    // Turn on backlight via GPIO if defined and connected
    if (TFT_BLK >= 0) { // Check if a valid pin number is defined
        pinMode(TFT_BLK, OUTPUT);
        digitalWrite(TFT_BLK, HIGH); // Turn backlight ON (assuming HIGH is ON)
        Serial.println("✅ TFT Backlight ON.");
    } else {
        Serial.println("-> TFT Backlight pin not defined or controlled.");
    }
}

void updateDisplay() {
    tft.fillScreen(ST77XX_BLACK); // Clear screen before drawing
    tft.setTextWrap(false);

    // --- 1. Display Status ---
    tft.setTextSize(1);
    tft.setCursor(5, 5); // Top-left corner
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Status: ");
    switch (currentStatus) {
        case STATUS_ERROR:      tft.setTextColor(ST77XX_RED);    tft.print("ERROR"); break;
        case STATUS_CONNECTING: tft.setTextColor(ST77XX_BLUE);   tft.print("CONNECTING"); break;
        case STATUS_READY:      tft.setTextColor(ST77XX_GREEN);  tft.print("READY"); break;
        case STATUS_RECORDING:  tft.setTextColor(ST77XX_MAGENTA);tft.print("RECORDING"); break;
        default:                tft.setTextColor(ST77XX_WHITE);  tft.print("UNKNOWN"); break;
    }

    // --- 2. Display Power Data ---
    tft.setTextSize(1.5); // Larger font for values
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 20); // Line below status
    tft.print("CP:----W"); // Placeholder for Calculated Power

    tft.setCursor(5, 40); // Line below CP
    tft.print("P:");
    if (powerConnected) {
        tft.print(currentPower, 0); // Display actual power meter reading
    } else {
        tft.print("---"); // Indicate not connected/available
    }
    tft.print("W");

    // --- 3. Display Speed Data ---
    tft.setCursor(5, 65); // Line below Power
    tft.print("S:");
    if (cscConnected) {
       char speedStr[8];
       dtostrf(currentSpeedKph, 4, 1, speedStr); // Format speed (width 4, 1 decimal)
       tft.print(speedStr);
    } else {
        tft.print("----");
    }
    tft.print("AS:");
    float airspeedKph = airspeedPaToKph(lastAirspeed);
    if (ms4525Ready && !isnan(airspeedKph)) {
        char airspeedStr[8];
        dtostrf(airspeedKph, 4, 1, airspeedStr); // Format airspeed
        tft.print(airspeedStr);
    } else {
        tft.print("----");
    }
    // Add unit label aligned roughly to the right
    //tft.setTextSize(1);
    //tft.setCursor(tft.width() - 30, 70); // Position " kph" label
    //tft.setTextColor(ST77XX_CYAN);
    //tft.print("kph");

}
