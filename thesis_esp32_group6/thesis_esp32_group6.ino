#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "HX711.h"
#include "DHT.h"

// ===================== WiFi =====================
const char* ssid = "Raspi-Group6-Hotspot";
const char* password = "raspberry123";

const char* configURL = "http://10.42.0.1:5002/api/system/config";
const char* sensorURL = "http://10.42.0.1:5001/api/sensor/data";

// ===================== RELAYS =====================
const int RELAY_BLOWER  = 25;
const int RELAY_EXHAUST = 26;

// ===================== HX711 =====================
const int HX_DOUT = 16;
const int HX_SCK  = 4;
HX711 scale;
float calibrationFactor = 211830.0;

// ===================== DHT =====================
#define DHTPIN 17
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ===================== Moisture =====================
const int MOISTURE1_PIN = 32;
const int MOISTURE2_PIN = 33;
const int MOISTURE3_PIN = 34;
const int MOISTURE4_PIN = 35;
const int MOISTURE5_PIN = 36;
const int MOISTURE6_PIN = 39;

// ===== Calibration =====
float dryPercent = 14.0;
float wetPercent = 22.0;

float dry1 = 2529;
float wet1 = 2447;

float dry2 = 2550;
float wet2 = 2473;

float dry3 = 2533;
float wet3 = 2464;

float dry4 = 3155;
float wet4 = 2926;

float dry5 = 3173;
float wet5 = 2930;

float dry6 = 3119;
float wet6 = 2904;

// ===== System Targets =====
float targetMoisture = 0;
float targetTemp = 0;

int selectedTrays[6] = {0, 0, 0, 0, 0, 0};
int trayCount = 0;

bool systemRunning = false;

// ===== Relay states =====
bool blowerState = false;
bool exhaustState = false;

// ===================== FUNCTIONS =====================

int readAnalogAvg(int pin, int samples) {
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
    yield();
  }

  return sum / samples;
}

float calibrateMoisture(int raw, float rawDry, float rawWet) {
  float mc = dryPercent +
             (rawDry - raw) * (wetPercent - dryPercent) / (rawDry - rawWet);

  if (mc < 0) mc = 0;
  if (mc > 50) mc = 50;

  return mc;
}

float stableMoisture(int pin, float rawDry, float rawWet) {
  float sum = 0;

  for (int i = 0; i < 5; i++) {
    int raw = readAnalogAvg(pin, 5);
    float mc = calibrateMoisture(raw, rawDry, rawWet);
    sum += mc;
    delay(5);
  }

  return sum / 5.0;
}

float readWeightKg() {
  if (!scale.is_ready()) return 0;

  float kg = scale.get_units(2);

  if (isnan(kg) || kg < 0) kg = 0;

  return kg;
}

float trayAverage(float m1, float m2, float m3, float m4, float m5, float m6) {
  float values[6] = {m1, m2, m3, m4, m5, m6};

  float sum = 0;
  int count = 0;

  for (int i = 0; i < trayCount; i++) {
    int trayIndex = selectedTrays[i] - 1;

    if (trayIndex >= 0 && trayIndex < 6) {
      sum += values[trayIndex];
      count++;
    }
  }

  if (count == 0) return 0;

  return sum / count;
}

// ===================== WIFI =====================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ===================== GET CONFIG =====================

void getSystemConfig() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.setTimeout(3000);
  http.begin(configURL);

  int code = http.GET();
  Serial.print("HTTP Config Code: ");
  Serial.println(code);

  if (code == 200) {
    String payload = http.getString();
    Serial.println(payload);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (err) {
      Serial.println("JSON parse failed");
      http.end();
      return;
    }

    targetTemp = doc["config"]["selectedTemperature"] | 0;
    targetMoisture = doc["config"]["selectedMoisture"] | 0;

    trayCount = 0;
    for (int i = 0; i < 6; i++) {
      selectedTrays[i] = 0;
    }

    JsonArray trays = doc["config"]["selectedTrays"];

    if (!trays.isNull()) {
      for (JsonVariant v : trays) {
        int tray = v.as<int>();

        if (tray >= 1 && tray <= 6 && trayCount < 6) {
          selectedTrays[trayCount] = tray;
          trayCount++;
        }
      }
    }

    systemRunning = doc["running"] | false;

    Serial.println("========= DASHBOARD SETTINGS =========");
    Serial.print("Selected Temperature: ");
    Serial.println(targetTemp);

    Serial.print("Selected Moisture: ");
    Serial.println(targetMoisture);

    Serial.print("Selected Trays: ");
    for (int i = 0; i < trayCount; i++) {
      Serial.print(selectedTrays[i]);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print("System Running: ");
    Serial.println(systemRunning);
    Serial.println("======================================");
  }

  http.end();
}

// ===================== SEND SENSOR DATA =====================

void sendSensorData(float tempC, float hum, float weightKg,
                    float mc1, float mc2, float mc3,
                    float mc4, float mc5, float mc6,
                    float avgMoisture) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.setTimeout(3000);
  http.begin(sensorURL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<512> doc;

  doc["device_id"] = "esp32_01";
  doc["temperature"] = tempC;
  doc["humidity"] = hum;

  doc["moisture1"] = mc1;
  doc["moisture2"] = mc2;
  doc["moisture3"] = mc3;
  doc["moisture4"] = mc4;
  doc["moisture5"] = mc5;
  doc["moisture6"] = mc6;

  doc["moistureavg"] = avgMoisture;
  doc["weight1"] = weightKg;
  doc["status"] = systemRunning ? "Drying" : "Completed";

  String body;
  serializeJson(doc, body);

  Serial.println("Sending JSON:");
  Serial.println(body);

  int httpCode = http.POST(body);

  Serial.print("POST Response Code: ");
  Serial.println(httpCode);

  http.end();
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(2000);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RELAY_BLOWER, OUTPUT);
  pinMode(RELAY_EXHAUST, OUTPUT);

  digitalWrite(RELAY_BLOWER, HIGH);
  digitalWrite(RELAY_EXHAUST, HIGH);

  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(calibrationFactor);
  scale.tare();

  dht.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  connectWiFi();

  Serial.println("System Ready");
}

// ===================== LOOP =====================

void loop() {
  connectWiFi();
  getSystemConfig();

  float mc1 = stableMoisture(MOISTURE1_PIN, dry1, wet1);
  float mc2 = stableMoisture(MOISTURE2_PIN, dry2, wet2);
  float mc3 = stableMoisture(MOISTURE3_PIN, dry3, wet3);
  float mc4 = stableMoisture(MOISTURE4_PIN, dry4, wet4);
  float mc5 = stableMoisture(MOISTURE5_PIN, dry5, wet5);
  float mc6 = stableMoisture(MOISTURE6_PIN, dry6, wet6);

  float avgMoisture = trayAverage(mc1, mc2, mc3, mc4, mc5, mc6);

  float weightKg = readWeightKg();
  float tempC = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(tempC)) tempC = 0;
  if (isnan(hum)) hum = 0;

  if (systemRunning) {
    blowerState = (avgMoisture > targetMoisture);
    exhaustState = (tempC >= targetTemp);

    digitalWrite(RELAY_BLOWER, blowerState ? LOW : HIGH);
    digitalWrite(RELAY_EXHAUST, exhaustState ? LOW : HIGH);
  } else {
    blowerState = false;
    exhaustState = false;

    digitalWrite(RELAY_BLOWER, HIGH);
    digitalWrite(RELAY_EXHAUST, HIGH);
  }

  sendSensorData(tempC, hum, weightKg,
                 mc1, mc2, mc3, mc4, mc5, mc6,
                 avgMoisture);

  Serial.println("=========== SENSOR DATA ===========");
  Serial.print("Tray Count: ");
  Serial.println(trayCount);

  Serial.print("Selected Trays: ");
  for (int i = 0; i < trayCount; i++) {
    Serial.print(selectedTrays[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Moisture1: "); Serial.println(mc1);
  Serial.print("Moisture2: "); Serial.println(mc2);
  Serial.print("Moisture3: "); Serial.println(mc3);
  Serial.print("Moisture4: "); Serial.println(mc4);
  Serial.print("Moisture5: "); Serial.println(mc5);
  Serial.print("Moisture6: "); Serial.println(mc6);

  Serial.print("Average Moisture: "); Serial.println(avgMoisture);
  Serial.print("Temperature: "); Serial.println(tempC);
  Serial.print("Humidity: "); Serial.println(hum);
  Serial.print("Weight (kg): "); Serial.println(weightKg);
  Serial.print("Blower: "); Serial.println(blowerState ? "ON" : "OFF");
  Serial.print("Exhaust: "); Serial.println(exhaustState ? "ON" : "OFF");
  Serial.println("===================================");

  yield();
  delay(5000);
}