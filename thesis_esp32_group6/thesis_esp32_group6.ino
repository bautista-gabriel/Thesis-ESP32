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
const int RELAY_BLOWER  = 25;
const int RELAY_EXHAUST = 26;

// ===================== HX711 =====================
const int HX_DOUT = 16;
const int HX_SCK  = 4;
HX711 scale;
float calibrationFactor = 211830.0;

// ===================== DHT22 =====================
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

// ===== Calibration Anchor =====
float dryPercent = 14.0;
float wetPercent = 15.4;

float dry1 = 2561;
float wet1 = 2508;

float dry2 = 2571;
float wet2 = 2520;

// ===================== Timing =====================
unsigned long lastSend = 0;
const unsigned long sendInterval = 5000;

// ===================== FUNCTIONS =====================

int readAnalogAvg(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(3);
  }
  return sum / samples;
}

float calibrateMoisture(int raw, float rawDry, float rawWet) {

  float mc = dryPercent +
             (rawDry - raw) * (wetPercent - dryPercent) /
             (rawDry - rawWet);

  if (mc < 0) mc = 0;
  if (mc > 50) mc = 50;

  return mc;
}

float readWeightKg() {
  if (!scale.is_ready()) return 0.0f;
  float kg = scale.get_units(10);
  if (kg < 0) kg = 0;
  return kg;
}

float trimmedMean(float m1, float m2, float m3,
                  float m4, float m5, float m6) {

  float arr[6] = {m1, m2, m3, m4, m5, m6};

  for (int i = 0; i < 5; i++) {
    for (int j = i + 1; j < 6; j++) {
      if (arr[j] < arr[i]) {
        float t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }

  float sum = 0;

  for (int i = 1; i < 5; i++) {
    sum += arr[i];
  }

  return sum / 4.0;
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

  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(calibrationFactor);
  scale.tare();

  dht.begin();

  connectWiFi();
}

// ===================== LOOP =====================
void loop() {

  if (millis() - lastSend < sendInterval) return;
  lastSend = millis();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // ===== Raw Moisture =====
  int raw1 = readAnalogAvg(MOISTURE1_PIN, 60);
  int raw2 = readAnalogAvg(MOISTURE2_PIN, 60);
  int raw3 = readAnalogAvg(MOISTURE3_PIN, 60);
  int raw4 = readAnalogAvg(MOISTURE4_PIN, 60);
  int raw5 = readAnalogAvg(MOISTURE5_PIN, 60);
  int raw6 = readAnalogAvg(MOISTURE6_PIN, 60);

  // ===== Calibrated Moisture =====
  float mc1 = calibrateMoisture(raw1, dry1, wet1);
  float mc2 = calibrateMoisture(raw2, dry2, wet2);

  // duplicate so trimmed mean and server remain unchanged
  float mc3 = mc1;
  float mc4 = mc2;
  float mc5 = mc1;
  float mc6 = mc2;

  float avgMoisture = trimmedMean(mc1, mc2, mc3, mc4, mc5, mc6);

  // ===== Other Sensors =====
  float weightKg = readWeightKg();

  float tempC = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(tempC)) tempC = 0;
  if (isnan(hum)) hum = 0;

  // ===== STATUS LOGIC =====
  String status;

  if (avgMoisture >= 14.0 && avgMoisture <= 14.2) {
    status = "Completed";
  }
  else if (avgMoisture < 14.0 || tempC >= 40.0) {
    status = "Warning";
  }
  else {
    status = "Drying";
  }

  // ===== RELAY CONTROL =====

  if (avgMoisture == 14)
    digitalWrite(RELAY_BLOWER, HIGH);
  else
    digitalWrite(RELAY_BLOWER, LOW);

  if (tempC > 40.0)
    digitalWrite(RELAY_EXHAUST, LOW);
  else
    digitalWrite(RELAY_EXHAUST, HIGH);

  // ===== SERIAL MONITOR =====
  Serial.println("====================================");

  Serial.print("Raw1: "); Serial.print(raw1); Serial.print("  Moisture1: "); Serial.println(mc1);
  Serial.print("Raw2: "); Serial.print(raw2); Serial.print("  Moisture2: "); Serial.println(mc2);
  Serial.print("Raw3: "); Serial.print(raw3); Serial.print("  Moisture3: "); Serial.println(mc3);
  Serial.print("Raw4: "); Serial.print(raw4); Serial.print("  Moisture4: "); Serial.println(mc4);
  Serial.print("Raw5: "); Serial.print(raw5); Serial.print("  Moisture5: "); Serial.println(mc5);
  Serial.print("Raw6: "); Serial.print(raw6); Serial.print("  Moisture6: "); Serial.println(mc6);

  Serial.print("Avg Moisture: "); Serial.println(avgMoisture);
  Serial.print("Temperature: "); Serial.println(tempC);
  Serial.print("Humidity: "); Serial.println(hum);
  Serial.print("Weight (kg): "); Serial.println(weightKg);
  Serial.print("Status: "); Serial.println(status);

  Serial.println("====================================");

  // ===== HTTP SEND =====
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
  json += "\"moistureavg\":" + String(avgMoisture) + ",";
  json += "\"weight1\":" + String(weightKg) + ",";
  json += "\"status\":\"" + status + "\"";
  json += "}";

  int httpResponseCode = http.POST(json);

  Serial.print("HTTP Response: ");
  Serial.println(httpResponseCode);

  http.end();
}