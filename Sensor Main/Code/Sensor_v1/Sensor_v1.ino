// ====== Includes ======
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include "ms4525do.h"
#include <SPIFFS.h>
#include <Adafruit_NeoPixel.h>

// ====== Sensor Objects ======
Adafruit_ICM20948 icm;
Adafruit_MPL3115A2 mpl;
bfs::Ms4525do ms4525do(&Wire, 0x28, 1.0f, -1.0f);

// ====== Button and NeoPixel ======
#define MODE_PIN 11  // D11 = GPIO11 to dump logs
#define RECORD_PIN 5 // D5 = start recording
#define NEOPIXEL_PIN 21
#define NEOPIXEL_COUNT 1

Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool headerWritten = false;
bool recordingStarted = false;

String filename;

void setPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

void flashPixel(uint8_t r, uint8_t g, uint8_t b) {
  static bool on = false;
  on = !on;
  setPixelColor(on ? r : 0, on ? g : 0, on ? b : 0);
}

void logToCSV(const String &line) {
  File file = SPIFFS.open(filename, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open log file!");
    setPixelColor(255, 0, 0); // red for error
    return;
  }

  if (!headerWritten) {
    file.println("Time (s),AccelX,AccelY,AccelZ,Pressure (Pa),Temp (C),Airspeed (Pa),AirspeedTemp (C)");
    headerWritten = true;
  }

  file.println(line);
  file.close();
}

void dumpCSVOverSerial() {
  File file = SPIFFS.open(filename, "r");
  if (!file) {
    Serial.println("No log file found.");
    return;
  }

  Serial.println("\n\n\n==================== CSV LOG DUMP ====================");
  Serial.println("IF YOU SEE THIS, DATA IS STARTING BELOW");
  Serial.println("=====================================================");
  Serial.println("Time (s),AccelX,AccelY,AccelZ,Pressure (Pa),Temp (C),Airspeed (Pa),AirspeedTemp (C)");

  while (file.available()) {
    Serial.write(file.read());
  }
  Serial.println("===== END CSV DUMP =====");
  file.close();

  while (true) delay(100);
}

// ====== Setup ======
void setup() {
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(RECORD_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting Feather ESP32-S3 logger...");

  pixel.begin();
  pixel.setBrightness(20);
  setPixelColor(0, 255, 0); // green: initializing

  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS.");
    setPixelColor(255, 0, 0);
    while (1) delay(10);
  }
  Serial.println("SPIFFS mounted.");

  if (digitalRead(MODE_PIN) == LOW) {
    filename = "/log.csv";
    Serial.println("Button held: dumping CSV to Serial...");
    dumpCSVOverSerial();
  }

  int logIndex = 0;
  do {
    filename = "/log" + String(logIndex++) + ".csv";
  } while (SPIFFS.exists(filename));

  headerWritten = false;

  Wire.begin();
  Wire.setClock(100000);

  if (!icm.begin_I2C()) {
    Serial.println("ICM20948 not found!");
    setPixelColor(255, 0, 0);
    while (1) delay(10);
  }
  icm.setAccelRateDivisor(0);
  icm.setGyroRateDivisor(0);
  Serial.println("ICM20948 ready");

  if (!mpl.begin()) {
    Serial.println("MPL3115A2 not found!");
    setPixelColor(255, 0, 0);
    while (1) delay(10);
  }
  Serial.println("MPL3115A2 ready");

  if (!ms4525do.Begin()) {
    Serial.println("MS4525DO not found!");
    setPixelColor(255, 0, 0);
    while (1) delay(10);
  }
  Serial.println("MS4525DO ready");

  setPixelColor(0, 255, 0); // green: ready to record
}

// ====== Main Loop ======
void loop() {
  static unsigned long lastIMU = 0;
  static unsigned long lastMPL = 0;
  static unsigned long lastAirspeed = 0;
  static unsigned long lastLog = 0;
  static unsigned long lastFlash = 0;

  unsigned long now = millis();
  float timeSec = now / 1000.0f;

  if (!recordingStarted && digitalRead(RECORD_PIN) == LOW) {
    delay(50);
    if (digitalRead(RECORD_PIN) == HIGH) {
      recordingStarted = true;
      Serial.println("Recording started.");
    }
  }

  if (!recordingStarted) return;

  if (now - lastFlash >= 500) {
    lastFlash = now;
    flashPixel(128, 0, 128); // purple flash
  }

  sensors_event_t a, g, t;
  if (now - lastIMU >= 10) {
    lastIMU = now;
    icm.getEvent(&a, &g, &t);
  }

  static float mplPressure = 0, mplTemp = 0;
  if (now - lastMPL >= 1000) {
    lastMPL = now;
    mplPressure = mpl.getPressure();
    mplTemp = mpl.getTemperature();
  }

  static float airspeedPa = 0, airspeedTemp = 0;
  if (now - lastAirspeed >= 10) {
    lastAirspeed = now;
    if (ms4525do.Read()) {
      airspeedPa = ms4525do.pres_pa();
      airspeedTemp = ms4525do.die_temp_c();
    }
  }

  if (now - lastLog >= 100) {
    lastLog = now;
    String csv = String(timeSec, 3) + "," +
                 String(a.acceleration.x, 2) + "," +
                 String(a.acceleration.y, 2) + "," +
                 String(a.acceleration.z, 2) + "," +
                 String(mplPressure, 1) + "," +
                 String(mplTemp, 1) + "," +
                 String(airspeedPa, 1) + "," +
                 String(airspeedTemp, 1);
    logToCSV(csv);
    Serial.println(csv);
  }

  delay(1);
}
