#include <WiFi.h>
#include <HTTPClient.h>
#include "HX711.h"
#include "DHT.h"

// ===================== WiFi =====================
const char* ssid = "Raspi-Group6-Hotspot";
const char* password = "raspberry123";
const char* serverURL = "http://10.42.0.1:3000/sensor_readings";
const char* deviceId = "esp32_01";

// ===================== RELAYS (ACTIVE LOW) =====================
const int RELAY_BLOWER  = 25;
const int RELAY_EXHAUST = 33;

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

// ===================== 6 Moisture Sensors (ADC1 SAFE PINS) =====================
const int MOISTURE1_PIN = 34;  // Tray 1
const int MOISTURE2_PIN = 35;  // Tray 2
const int MOISTURE3_PIN = 32;  // Tray 3
const int MOISTURE4_PIN = 36;  // Tray 4 (input only)
const int MOISTURE5_PIN = 39;  // Tray 5 (input only)
//const int MOISTURE6_PIN = 33;  // ❌ WAIT — 33 is relay used

// IMPORTANT: Since 33 is used by RELAY_EXHAUST,
// we must NOT reuse it.

// So change this instead:
#undef MOISTURE6_PIN
const int MOISTURE6_PIN = 13;  // Tray 6 (ADC2 — use carefully)

// ====== REAL CALIBRATION VALUES ======
float rawAt14 = 2542.0;   // raw when moisture meter = 14%
float rawAt39 = 1700.0;   // raw when moisture meter = 39%

// ===================== Timing =====================
unsigned long lastSend = 0;
const unsigned long sendEveryMs = 5000;

// ===================== FUNCTIONS =====================

int readAnalogAvg(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / samples;
}

float grainMoisturePercent(int raw) {
  float a = (39.0 - 14.0) / (rawAt39 - rawAt14);
  float b = 14.0 - (a * rawAt14);
  return (a * raw) + b;
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

// ===================== SETUP =====================
void setup() {

  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RELAY_BLOWER, OUTPUT);
  pinMode(RELAY_EXHAUST, OUTPUT);
  digitalWrite(RELAY_BLOWER, HIGH);
  digitalWrite(RELAY_EXHAUST, HIGH);

  scale1.begin(HX1_DOUT, HX1_SCK);
  scale1.set_scale(calibrationFactor1);
  scale1.tare();

  scale2.begin(HX2_DOUT, HX2_SCK);
  scale2.set_scale(calibrationFactor2);
  scale2.tare();

  dht.begin();

  connectWiFi();
}

// ===================== LOOP =====================
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (millis() - lastSend < sendEveryMs) return;
  lastSend = millis();

  // ===== Read Weights =====
  float weight1Kg = readWeightKg(scale1, 10.0f);
  float weight2Kg = readWeightKg(scale2, 10.0f);

  // ===== Read Temperature & Humidity =====
  float hum = dht.readHumidity();
  float tempC = dht.readTemperature();

  if (isnan(tempC)) tempC = 0;
  if (isnan(hum)) hum = 0;

  // ===== Read 6 Moisture Sensors =====
  int raw1 = readAnalogAvg(MOISTURE1_PIN, 80);
  int raw2 = readAnalogAvg(MOISTURE2_PIN, 80);
  int raw3 = readAnalogAvg(MOISTURE3_PIN, 80);
  int raw4 = readAnalogAvg(MOISTURE4_PIN, 80);
  int raw5 = readAnalogAvg(MOISTURE5_PIN, 80);
  int raw6 = readAnalogAvg(MOISTURE6_PIN, 80);

  float mc1 = grainMoisturePercent(raw1);
  float mc2 = grainMoisturePercent(raw2);
  float mc3 = grainMoisturePercent(raw3);
  float mc4 = grainMoisturePercent(raw4);
  float mc5 = grainMoisturePercent(raw5);
  float mc6 = grainMoisturePercent(raw6);

  float avgMoisture = (mc1 + mc2 + mc3 + mc4 + mc5 + mc6) / 6.0;

  // ===================== RELAY CONTROL =====================

  if (tempC > 40.0)
    digitalWrite(RELAY_EXHAUST, LOW);
  else
    digitalWrite(RELAY_EXHAUST, HIGH);

  if (avgMoisture <= 14.0)
    digitalWrite(RELAY_BLOWER, HIGH);
  else
    digitalWrite(RELAY_BLOWER, LOW);

  // ===================== STATUS =====================
  String status;
  if (avgMoisture <= 14.0) status = "Completed";
  else if (avgMoisture <= 20.0) status = "Warning";
  else status = "Drying";

  // ===================== DEBUG =====================
  Serial.println("================================");
  Serial.print("Tray1 MC: "); Serial.println(mc1);
  Serial.print("Tray2 MC: "); Serial.println(mc2);
  Serial.print("Tray3 MC: "); Serial.println(mc3);
  Serial.print("Tray4 MC: "); Serial.println(mc4);
  Serial.print("Tray5 MC: "); Serial.println(mc5);
  Serial.print("Tray6 MC: "); Serial.println(mc6);
  Serial.print("Average MC: "); Serial.println(avgMoisture);
  Serial.print("Temperature: "); Serial.println(tempC);
  Serial.print("Status: "); Serial.println(status);

  // ===================== SEND TO SERVER =====================
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"device_id\":\"" + String(deviceId) + "\",";
  json += "\"temperature\":" + String(tempC) + ",";
  json += "\"humidity\":" + String(hum) + ",";
  json += "\"moisture1\":" + String(mc1) + ",";
  json += "\"moisture2\":" + String(mc2) + ",";
  json += "\"moisture3\":" + String(mc3) + ",";
  json += "\"moisture4\":" + String(mc4) + ",";
  json += "\"moisture5\":" + String(mc5) + ",";
  json += "\"moisture6\":" + String(mc6) + ",";
  json += "\"weight1\":" + String(weight1Kg) + ",";
  json += "\"weight2\":" + String(weight2Kg) + ",";
  json += "\"status\":\"" + status + "\"";
  json += "}";

  int httpResponseCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpResponseCode);

  http.end();
}