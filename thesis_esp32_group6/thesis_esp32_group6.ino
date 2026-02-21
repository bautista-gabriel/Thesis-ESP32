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

// ===================== Moisture Sensors =====================
const int MOISTURE1_PIN = 34;
const int MOISTURE2_PIN = 35;

// ===== REAL CALIBRATION VALUES (CHANGE THESE) =====
int rawDry14 = 2542;   // raw when grain = 14%
int rawWet39 = 1700;   // raw when grain = 39%

// ===================== Timing =====================
unsigned long lastSend = 0;
const unsigned long sendEveryMs = 5000;

// ===================== Helper =====================

int readAnalogAvg(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / samples;
}

// ===== REAL GRAIN MOISTURE CALCULATION =====
float grainMoisturePercent(int raw) {

  float moisture =
    14.0 +
    ((float)(rawDry14 - raw) * (39.0 - 14.0)) /
    (rawDry14 - rawWet39);

  return moisture;
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

  float moist1Pct = grainMoisturePercent(moist1Raw);
  float moist2Pct = grainMoisturePercent(moist2Raw);

  float avgMoisture = (moist1Pct + moist2Pct) / 2.0;

  // ===================== RELAY CONTROL =====================

  // Exhaust ON if temperature > 40°C
  if (tempC > 40.0) {
    digitalWrite(RELAY_EXHAUST, LOW);
  } else {
    digitalWrite(RELAY_EXHAUST, HIGH);
  }

  // Blower OFF when grain <= 14%
  if (avgMoisture <= 14.0) {
    digitalWrite(RELAY_BLOWER, HIGH); 
  } else {
    digitalWrite(RELAY_BLOWER, LOW);
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

  // ===================== Debug =====================
  Serial.println("================================");
  Serial.print("Temp: "); Serial.println(tempC);
  Serial.print("Humidity: "); Serial.println(hum);
  Serial.print("Moist1 Raw: "); Serial.println(moist1Raw);
  Serial.print("Moist1 %: "); Serial.println(moist1Pct);
  Serial.print("Moist2 %: "); Serial.println(moist2Pct);
  Serial.print("Average Moisture: "); Serial.println(avgMoisture);
  Serial.print("Status: "); Serial.println(status);

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