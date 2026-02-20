#include <WiFi.h>
#include <HTTPClient.h>
#include "HX711.h"
#include "DHT.h"

// ===================== WiFi =====================
const char* ssid = "Raspi-Group6-Hotspot";
const char* password = "raspberry123";
const char* serverURL = "http://10.42.0.1:3000/sensor_readings";
const char* deviceId = "esp32_01";

// ===================== RELAYS =====================
const int RELAY_BLOWER  = 25; // Blue
const int RELAY_EXHAUST = 33; // Green

// ===================== HX711 =====================
const int HX1_DOUT = 16;
const int HX1_SCK  = 4;
HX711 scale1;
float calibrationFactor1 = 211830.0;

const int HX2_DOUT = 27;
const int HX2_SCK  = 26;
HX711 scale2;
float calibrationFactor2 = 211830.0;

// ===================== DHT22 =====================
#define DHTPIN 17
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ===================== Moisture Sensors =====================
const int MOISTURE1_PIN = 34;
const int MOISTURE2_PIN = 35;

int moisture1Dry = 3200;
int moisture1Wet = 1500;

int moisture2Dry = 3200;
int moisture2Wet = 1500;

// ===================== Timing =====================
unsigned long lastSend = 0;
const unsigned long sendEveryMs = 5000;

// ===================== Helper Functions =====================

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int readAnalogAvg(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / samples;
}

float moisturePercent(int raw, int dryVal, int wetVal) {
  if (dryVal == wetVal) return 0.0f;
  float pct = (raw - dryVal) * 100.0f / (wetVal - dryVal);
  return clampFloat(pct, 0.0f, 100.0f);
}

float readWeightKg(HX711& scale, float maxKg) {
  if (!scale.is_ready()) return 0.0f;
  float kg = scale.get_units(10);
  if (kg < 0) kg = 0;
  if (kg > maxKg) kg = maxKg;
  return kg;
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Relay setup (ACTIVE LOW)
  pinMode(RELAY_BLOWER, OUTPUT);
  pinMode(RELAY_EXHAUST, OUTPUT);
  digitalWrite(RELAY_BLOWER, HIGH);   // OFF
  digitalWrite(RELAY_EXHAUST, HIGH);  // OFF

  // HX711
  scale1.begin(HX1_DOUT, HX1_SCK);
  scale1.set_scale(calibrationFactor1);
  scale1.tare();

  scale2.begin(HX2_DOUT, HX2_SCK);
  scale2.set_scale(calibrationFactor2);
  scale2.tare();

  dht.begin();

  pinMode(MOISTURE1_PIN, INPUT);
  pinMode(MOISTURE2_PIN, INPUT);

  connectWiFi();
}

// ===================== Loop =====================
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (millis() - lastSend < sendEveryMs) return;
  lastSend = millis();

  // ===== Read Sensors =====
  float weight1Kg = readWeightKg(scale1, 10.0f);
  float weight2Kg = readWeightKg(scale2, 10.0f);

  float hum = dht.readHumidity();
  float tempC = dht.readTemperature();

  if (isnan(tempC)) tempC = 0;
  if (isnan(hum)) hum = 0;

  int moist1Raw = readAnalogAvg(MOISTURE1_PIN, 20);
  int moist2Raw = readAnalogAvg(MOISTURE2_PIN, 20);

  float moist1Pct = moisturePercent(moist1Raw, moisture1Dry, moisture1Wet);
  float moist2Pct = moisturePercent(moist2Raw, moisture2Dry, moisture2Wet);

  float avgMoisture = (moist1Pct + moist2Pct) / 2.0;

  // ===================== RELAY CONTROL =====================

  // Exhaust: ON if temperature > 28°C
  if (tempC > 40) {
    digitalWrite(RELAY_EXHAUST, LOW);   // ON
  } else {
    digitalWrite(RELAY_EXHAUST, HIGH);  // OFF
  }

  // Blower: OFF if moisture > 30%
  if (avgMoisture <= 14) {
    digitalWrite(RELAY_BLOWER, HIGH);   // OFF
  } else {
    digitalWrite(RELAY_BLOWER, LOW);    // ON
  }

  // ===================== Status =====================
  String status;

  if (avgMoisture <= 14.0) {
    status = "Completed";
  } 
  else if (avgMoisture <= 20.0) {
    status = "Warning";
  } 
  else {
    status = "Drying";
  }

  // ===================== Serial Debug =====================
  Serial.println("================================");
  Serial.print("Temp: "); Serial.println(tempC);
  Serial.print("Humidity: "); Serial.println(hum);
  Serial.print("Avg Moisture: "); Serial.println(avgMoisture);
  Serial.print("Blower: ");
  Serial.println(avgMoisture > 30.0 ? "OFF" : "ON");
  Serial.print("Exhaust: ");
  Serial.println(tempC > 28.0 ? "ON" : "OFF");

  // ===================== Send to Server =====================
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"device_id\":\"" + String(deviceId) + "\",";
  json += "\"temperature\":" + String(tempC) + ",";
  json += "\"humidity\":" + String(hum) + ",";
  json += "\"moisture1\":" + String(moist1Pct) + ",";
  json += "\"moisture2\":" + String(moist2Pct) + ",";
  json += "\"weight1\":" + String(weight1Kg) + ",";
  json += "\"weight2\":" + String(weight2Kg) + ",";
  json += "\"status\":\"" + status + "\"";
  json += "}";

  int httpResponseCode = http.POST(json);
  Serial.print("HTTP Response: ");
  Serial.println(httpResponseCode);

  http.end();
}