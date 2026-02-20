#include <WiFi.h>
#include <HTTPClient.h>
#include "HX711.h"
#include "DHT.h"

// ===================== WiFi =====================
const char* ssid = "Raspi-Group6-Hotspot";
const char* password = "raspberry123";

const char* serverURL = "http://10.42.0.1:3000/sensor_readings";
const char* deviceId = "esp32_01";

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

  // HX711
  scale1.begin(HX1_DOUT, HX1_SCK);
  scale1.set_scale(calibrationFactor1);
  scale1.tare();

  scale2.begin(HX2_DOUT, HX2_SCK);
  scale2.set_scale(calibrationFactor2);
  scale2.tare();

  // DHT
  dht.begin();

  pinMode(MOISTURE1_PIN, INPUT);
  pinMode(MOISTURE2_PIN, INPUT);

  connectWiFi();
}

// ===================== Loop =====================
void loop() {

  // Reconnect WiFi if needed
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

  int moist1Raw = readAnalogAvg(MOISTURE1_PIN, 20);
  int moist2Raw = readAnalogAvg(MOISTURE2_PIN, 20);

  float moist1Pct = moisturePercent(moist1Raw, moisture1Dry, moisture1Wet);
  float moist2Pct = moisturePercent(moist2Raw, moisture2Dry, moisture2Wet);

  float avgMoisture = (moist1Pct + moist2Pct) / 2.0;

  // ===== Serial Debug =====
  Serial.println("================================");
  Serial.print("Temp: "); Serial.print(tempC); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(hum); Serial.println(" %");

  Serial.print("Weight1: "); Serial.print(weight1Kg); Serial.println(" kg");
  Serial.print("Weight2: "); Serial.print(weight2Kg); Serial.println(" kg");

  Serial.print("Moist1 Raw: "); Serial.println(moist1Raw);
  Serial.print("Moist2 Raw: "); Serial.println(moist2Raw);

  Serial.print("Moist1 %: "); Serial.println(moist1Pct);
  Serial.print("Moist2 %: "); Serial.println(moist2Pct);

  Serial.print("Avg Moisture: ");
  Serial.print(avgMoisture);
  Serial.println(" %");

  // ===== Send to Server =====
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"device_id\":\"" + String(deviceId) + "\",";
  json += "\"temperature\":" + String(tempC) + ",";
  json += "\"humidity\":" + String(hum) + ",";
  json += "\"weight1\":" + String(weight1Kg) + ",";
  json += "\"weight2\":" + String(weight2Kg) + ",";
  json += "\"moisture1\":" + String(moist1Pct) + ",";
  json += "\"moisture2\":" + String(moist2Pct) + ",";
  json += "\"avg_moisture\":" + String(avgMoisture);
  json += "}";

  int httpResponseCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpResponseCode);

  http.end();
}