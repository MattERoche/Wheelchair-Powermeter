#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>

// BLE UUIDs for Cycling Speed and Cadence (CSC) service and measurement characteristic
static BLEUUID CSC_SERVICE_UUID((uint16_t)0x1816);
static BLEUUID CSC_MEASUREMENT_CHAR_UUID((uint16_t)0x2A5B);

static BLEUUID CYCLING_POWER_SERVICE_UUID((uint16_t)0x1818);
static BLEUUID CYCLING_POWER_MEASUREMENT_CHAR_UUID((uint16_t)0x2A63);


BLEScan* pBLEScan;
BLEAdvertisedDevice* foundCSCDevice = nullptr;
BLEAdvertisedDevice* foundPowerDevice = nullptr;
BLEClient* pClient = nullptr;
float currentSpeedKph = 0.0f;
uint32_t lastWheelRevs = 0;
uint16_t lastWheelEventTime = 0;
unsigned long lastNonZeroRevTime = 0;

bool cscConnected = false;
bool powerConnected = false;


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
  Serial.print("⚡ Raw Power Data: ");
  for (size_t i = 0; i < length; i++) {
    Serial.printf("%02X ", pData[i]);
  }
  Serial.println();

  if (length < 4) return;

  uint16_t flags = pData[0] | (pData[1] << 8);
  int16_t instantaneousPower = pData[2] | (pData[3] << 8); // signed!

  Serial.print("⚡ Power: ");
  Serial.print(instantaneousPower);
  Serial.println(" W");

  // Optional: parse cadence from crank revolutions here
}

// Global notify callback function
void notifyCallback(BLERemoteCharacteristic* pCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("🔔 Raw: ");
  for (size_t i = 0; i < length; i++) {
    Serial.printf("%02X ", pData[i]);
  }
  Serial.println();

  uint8_t flags = pData[0];
  bool wheelDataPresent = flags & 0x01;
  bool crankDataPresent = flags & 0x02;

  int index = 1;

  static uint32_t lastWheelRevs = 0;
  static uint16_t lastWheelEventTime = 0;
  static unsigned long lastNonZeroRevTime = 0;
  static float currentSpeedKph = 0.0f;

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

      Serial.print("🚴 Speed: ");
      Serial.print(currentSpeedKph, 2);
      Serial.println(" km/h");
    } else {
      unsigned long now = millis();
      if (now - lastNonZeroRevTime > 500) {
        currentSpeedKph = 0.0f;
        Serial.println("🚴 Speed: 0.00 km/h (no revs for 2s)");
      }
    }

    lastWheelRevs = wheelRevs;
    lastWheelEventTime = wheelEventTime;
  }

  // (Optional) You can parse cadence from crankDataPresent if you want later
}



void setup() {
  Serial.begin(115200);
  Serial.println("🚲 BLE CSC Tracker Starting...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
}

void loop() {
  // Scan if not connected to both
  if (!cscConnected || !powerConnected) {
    Serial.println("🔎 Scanning for BLE devices...");
    pBLEScan->start(5, false);
    pBLEScan->clearResults();
    delay(500);
  }

  // Connect to CSC
  if (!cscConnected && foundCSCDevice != nullptr) {
    Serial.print("🔗 Connecting to CSC: ");
    Serial.println(foundCSCDevice->getAddress().toString().c_str());

    BLEClient* cscClient = BLEDevice::createClient();
    if (!cscClient->connect(foundCSCDevice)) {
      Serial.println("❌ CSC connection failed");
      delete foundCSCDevice;
      foundCSCDevice = nullptr;
    } else {
      BLERemoteService* service = cscClient->getService(CSC_SERVICE_UUID);
      if (service) {
        BLERemoteCharacteristic* charac = service->getCharacteristic(CSC_MEASUREMENT_CHAR_UUID);
        if (charac && charac->canNotify()) {
          charac->registerForNotify(notifyCallback);
          Serial.println("✅ Subscribed to CSC notifications!");
          cscConnected = true;
        }
      }
    }

    delete foundCSCDevice;
    foundCSCDevice = nullptr;
  }

  // Connect to Power
  if (!powerConnected && foundPowerDevice != nullptr) {
    Serial.print("🔗 Connecting to Power Meter: ");
    Serial.println(foundPowerDevice->getAddress().toString().c_str());

    BLEClient* powerClient = BLEDevice::createClient();
    if (!powerClient->connect(foundPowerDevice)) {
      Serial.println("❌ Power connection failed");
      delete foundPowerDevice;
      foundPowerDevice = nullptr;
    } else {
      BLERemoteService* service = powerClient->getService(CYCLING_POWER_SERVICE_UUID);
      if (service) {
        BLERemoteCharacteristic* charac = service->getCharacteristic(CYCLING_POWER_MEASUREMENT_CHAR_UUID);
        if (charac && charac->canNotify()) {
          charac->registerForNotify(powerNotifyCallback);
          Serial.println("✅ Subscribed to Power Meter notifications!");
          powerConnected = true;
        }
      }
    }

    delete foundPowerDevice;
    foundPowerDevice = nullptr;
  }

  delay(1000);
}


