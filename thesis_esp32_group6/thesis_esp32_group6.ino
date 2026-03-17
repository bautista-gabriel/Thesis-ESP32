#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <algorithm>
#include "HX711.h"
#include "DHT.h"

// ===================== WiFi =====================
const char* ssid     = "Raspi-Group6-Hotspot";
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
#define DHTPIN   17
#define DHTTYPE  DHT22
DHT dht(DHTPIN, DHTTYPE);

// ===================== Moisture Pins =====================
const int MOISTURE_PINS[6] = {32, 33, 34, 35, 36, 39};

// ===================== Calibration Per Sensor =====================
float dryPercent = 14.0;
float wetPercent = 22.0;

float dryRaw[6] = {2529, 2550, 2533, 3155, 3173, 3119};
float wetRaw[6] = {2447, 2473, 2464, 2926, 2930, 2904};

// ===================== System Targets =====================
float targetMoisture = 0;
float targetTemp     = 0;

int  selectedTrays[6];
int  trayCount = 0;
bool systemRunning = false;

// ===================== Relay States =====================
bool blowerState  = false;
bool exhaustState = false;

// ===================== EMA State =====================
float emaVal[6]  = {0, 0, 0, 0, 0, 0};
bool  emaInit[6] = {false, false, false, false, false, false};
const float ALPHA = 0.2; // 0.1 = smoother/slower, 0.3 = faster/noisier

// ===================== Outlier Guard State =====================
float lastMC[6]      = {-1, -1, -1, -1, -1, -1};
const float MAX_JUMP = 3.0; // max allowed % MC change per cycle


// ===================== FUNCTIONS =====================

// ----- Median of N raw ADC readings -----
int medianRaw(int pin, int samples = 15) {
  int buf[15];
  for (int i = 0; i < samples; i++) {
    buf[i] = analogRead(pin);
    delay(4);
    yield();
  }
  std::sort(buf, buf + samples);
  return buf[samples / 2];
}

// ----- Map raw ADC to moisture % -----
float calibrateMoisture(int raw, float rawDry, float rawWet) {
  float mc = dryPercent +
             (rawDry - raw) * (wetPercent - dryPercent) / (rawDry - rawWet);
  if (mc < 0)  mc = 0;
  if (mc > 50) mc = 50;
  return mc;
}

// ----- Stable single-sensor moisture reading -----
float stableMoisture(int pin, float rawDry, float rawWet) {
  int raw = medianRaw(pin, 15);
  return calibrateMoisture(raw, rawDry, rawWet);
}

// ----- Exponential Moving Average per sensor index -----
float applyEMA(int index, float newReading) {
  if (!emaInit[index]) {
    emaVal[index] = newReading;
    emaInit[index] = true;
  } else {
    emaVal[index] = ALPHA * newReading + (1.0 - ALPHA) * emaVal[index];
  }
  return emaVal[index];
}

// ----- Temperature compensation (capacitive drift ~0.15%MC per °C) -----
float compensateMoisture(float mc, float tempC, float refTemp = 28.0) {
  float correction = (tempC - refTemp) * 0.15;
  float compensated = mc - correction;
  if (compensated < 0)  compensated = 0;
  if (compensated > 50) compensated = 50;
  return compensated;
}

// ----- Outlier guard: reject spikes larger than MAX_JUMP -----
float guardedReading(int index, float newMC) {
  if (lastMC[index] < 0) {
    lastMC[index] = newMC;
    return newMC;
  }
  if (abs(newMC - lastMC[index]) > MAX_JUMP) {
    Serial.print("  [GUARD] Spike rejected on sensor ");
    Serial.print(index + 1);
    Serial.print(": ");
    Serial.print(newMC);
    Serial.print(" -> keeping ");
    Serial.println(lastMC[index]);
    return lastMC[index]; // reject spike, keep previous
  }
  lastMC[index] = newMC;
  return newMC;
}

// ----- Full pipeline: median -> calibrate -> EMA -> tempComp -> guard -----
float readMoisture(int index, float tempC) {
  float raw_mc   = stableMoisture(MOISTURE_PINS[index], dryRaw[index], wetRaw[index]);
  float ema_mc   = applyEMA(index, raw_mc);
  float comp_mc  = compensateMoisture(ema_mc, tempC);
  float final_mc = guardedReading(index, comp_mc);
  return final_mc;
}

// ----- Weight -----
float readWeightKg() {
  if (!scale.is_ready()) return 0;
  float kg = scale.get_units(2);
  if (isnan(kg) || kg < 0) kg = 0;
  return kg;
}

// ----- Tray-based average -----
float trayAverage(float mc[6]) {
  float sum  = 0;
  int   count = 0;
  for (int i = 0; i < trayCount; i++) {
    int tray = selectedTrays[i] - 1;
    if (tray >= 0 && tray < 6) {
      sum += mc[tray];
      count++;
    }
  }
  if (count == 0) return 0;
  return sum / count;
}


// ===================== WiFi =====================

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

    targetTemp     = doc["config"]["selectedTemperature"];
    targetMoisture = doc["config"]["selectedMoisture"];
    trayCount      = 0;

    for (int i = 0; i < 6; i++) selectedTrays[i] = 0;

    JsonArray trays = doc["config"]["selectedTrays"];
    if (!trays.isNull()) {
      for (JsonVariant v : trays) {
        int tray = v.as<int>();
        if (tray >= 1 && tray <= 6 && trayCount < 6) {
          selectedTrays[trayCount++] = tray;
        }
      }
    }

    systemRunning = doc["running"];

    Serial.println("========= DASHBOARD SETTINGS =========");
    Serial.print("Selected Temperature: "); Serial.println(targetTemp);
    Serial.print("Selected Moisture: ");    Serial.println(targetMoisture);
    Serial.print("Selected Trays: ");
    for (int i = 0; i < trayCount; i++) {
      Serial.print(selectedTrays[i]);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("System Running: "); Serial.println(systemRunning);
    Serial.println("======================================");
  }

  http.end();
}


// ===================== SEND SENSOR DATA =====================

void sendSensorData(float tempC, float hum, float weightKg,
                    float mc[], float avgMoisture) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.setTimeout(3000);
  http.begin(sensorURL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<512> doc;
  doc["device_id"]   = "esp32_01";
  doc["temperature"] = tempC;
  doc["humidity"]    = hum;
  doc["moisture1"]   = mc[0];
  doc["moisture2"]   = mc[1];
  doc["moisture3"]   = mc[2];
  doc["moisture4"]   = mc[3];
  doc["moisture5"]   = mc[4];
  doc["moisture6"]   = mc[5];
  doc["moistureavg"] = avgMoisture;
  doc["weight1"]     = weightKg;
  doc["status"]      = systemRunning ? "Drying" : "Completed";

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

  pinMode(RELAY_BLOWER,  OUTPUT);
  pinMode(RELAY_EXHAUST, OUTPUT);
  digitalWrite(RELAY_BLOWER,  HIGH); // relays off (active LOW)
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

  // Read temp/humidity first so compensation is ready
  float tempC = dht.readTemperature();
  float hum   = dht.readHumidity();
  if (isnan(tempC)) tempC = 28.0; // fallback to ref temp if sensor fails
  if (isnan(hum))   hum   = 0;

  // Full pipeline: median -> calibrate -> EMA -> tempComp -> guard
  float mc[6];
  for (int i = 0; i < 6; i++) {
    mc[i] = readMoisture(i, tempC);
  }

  float avgMoisture = trayAverage(mc);
  float weightKg    = readWeightKg();


  // ===== BLOWER CONTROL =====
  if (systemRunning) {
    blowerState = (avgMoisture > targetMoisture);
    digitalWrite(RELAY_BLOWER, blowerState ? LOW : HIGH);
  } else {
    blowerState = false;
    digitalWrite(RELAY_BLOWER, HIGH);
  }
  delay(50); // let relay transient settle before next ADC read


  // ===== EXHAUST CONTROL =====
  if (systemRunning) {
    exhaustState = (tempC >= targetTemp);
    digitalWrite(RELAY_EXHAUST, exhaustState ? LOW : HIGH);
  } else {
    exhaustState = false;
    digitalWrite(RELAY_EXHAUST, HIGH);
  }
  delay(50); // let relay transient settle


  // ===== SEND DATA =====
  sendSensorData(tempC, hum, weightKg, mc, avgMoisture);


  // ===== SERIAL PRINT =====
  Serial.println("=========== SENSOR DATA ===========");
  Serial.print("Selected Trays: ");
  for (int i = 0; i < trayCount; i++) {
    Serial.print(selectedTrays[i]);
    Serial.print(" ");
  }
  Serial.println();
  for (int i = 0; i < 6; i++) {
    Serial.print("  MC");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(mc[i]);
  }
  Serial.print("Average Moisture : "); Serial.println(avgMoisture);
  Serial.print("Temperature (°C) : "); Serial.println(tempC);
  Serial.print("Humidity (%)     : "); Serial.println(hum);
  Serial.print("Weight (kg)      : "); Serial.println(weightKg);
  Serial.print("Blower           : "); Serial.println(blowerState  ? "ON" : "OFF");
  Serial.print("Exhaust          : "); Serial.println(exhaustState ? "ON" : "OFF");
  Serial.println("===================================");

  yield();
  delay(5000);
}
