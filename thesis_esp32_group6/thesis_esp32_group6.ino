#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "HX711.h"
#include "DHT.h"

// ===================== WiFi =====================
const char* ssid = "Raspi-Group6-Hotspot";
const char* password = "raspberry123";

const char* configURL = "http://10.42.0.1:5001/api/system/config";
const char* sensorURL = "http://10.42.0.1:5001/api/sensor/data";

unsigned long lastWifiAttempt = 0;

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
float wetPercent = 15.4;

float dry1 = 2561;
float wet1 = 2508;

float dry2 = 2571;
float wet2 = 2520;

// ===== System Targets =====
float targetMoisture = 13.5;
float targetTemp = 42;
int selectedTray = 1;

// ===== Relay states =====
bool blowerState = true;
bool exhaustState = false;

// ===================== Timing =====================
unsigned long lastSend = 0;
const unsigned long sendInterval = 5000;

// ===================== FUNCTIONS =====================

int readAnalogAvg(int pin, int samples) {

  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
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

  if (!scale.is_ready()) return 0;

  float kg = scale.get_units(5);

  if (isnan(kg) || kg < 0) kg = 0;

  return kg;
}

float trimmedMean(float m1,float m2,float m3,float m4,float m5,float m6){

  float arr[6] = {m1,m2,m3,m4,m5,m6};

  for(int i=0;i<5;i++){
    for(int j=i+1;j<6;j++){
      if(arr[j] < arr[i]){
        float t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }

  float sum = 0;

  for(int i=1;i<5;i++)
    sum += arr[i];

  return sum / 4.0;
}

// ===================== WiFi =====================

void connectWiFi(){

  if(WiFi.status()==WL_CONNECTED) return;

  if(millis()-lastWifiAttempt < 15000) return;

  lastWifiAttempt = millis();

  Serial.println("Connecting WiFi...");

  WiFi.disconnect();
  WiFi.begin(ssid,password);
}

void maintainWiFi(){

  if(WiFi.status()!=WL_CONNECTED){
    connectWiFi();
  }
}

// ===================== GET SYSTEM CONFIG =====================

void getSystemConfig(){

  if(WiFi.status()!=WL_CONNECTED) return;

  HTTPClient http;

  http.setTimeout(3000);
  http.begin(configURL);

  int code = http.GET();

  if(code==200){

    String payload = http.getString();

    StaticJsonDocument<256> doc;

    DeserializationError err = deserializeJson(doc,payload);

    if(!err){

      targetTemp = doc["config"]["selectedTemperature"] | targetTemp;
      targetMoisture = doc["config"]["selectedMoisture"] | targetMoisture;
      selectedTray = doc["config"]["selectedTray"] | selectedTray;

      Serial.println("Dashboard config updated");
    }
  }

  http.end();
}

// ===================== SEND SENSOR DATA =====================

void sendSensorData(float tempC,float hum,float weightKg,
                    float mc1,float mc2,float mc3,
                    float mc4,float mc5,float mc6,
                    float avgMoisture){

  if(WiFi.status()!=WL_CONNECTED) return;

  HTTPClient http;

  http.setTimeout(3000);
  http.begin(sensorURL);
  http.addHeader("Content-Type","application/json");

  StaticJsonDocument<512> doc;

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

  doc["status"] = "Drying";

  String body;

  serializeJson(doc,body);

  http.POST(body);

  http.end();
}

// ===================== SETUP =====================

void setup(){

  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RELAY_BLOWER,OUTPUT);
  pinMode(RELAY_EXHAUST,OUTPUT);

  digitalWrite(RELAY_BLOWER,HIGH);
  digitalWrite(RELAY_EXHAUST,HIGH);

  scale.begin(HX_DOUT,HX_SCK);
  scale.set_scale(calibrationFactor);
  scale.tare();

  dht.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  connectWiFi();

  Serial.println("System Ready");
}

// ===================== LOOP =====================

void loop(){

  maintainWiFi();

  if(millis()-lastSend < sendInterval){
    delay(5);
    return;
  }

  lastSend = millis();

  getSystemConfig();

  // ===== Raw Moisture =====
  int raw1 = readAnalogAvg(MOISTURE1_PIN,10);
  int raw2 = readAnalogAvg(MOISTURE2_PIN,10);
  int raw3 = readAnalogAvg(MOISTURE3_PIN,10);
  int raw4 = readAnalogAvg(MOISTURE4_PIN,10);
  int raw5 = readAnalogAvg(MOISTURE5_PIN,10);
  int raw6 = readAnalogAvg(MOISTURE6_PIN,10);

  // ===== Calibrated Moisture =====
  float mc1 = calibrateMoisture(raw1,dry1,wet1);
  float mc2 = calibrateMoisture(raw2,dry2,wet2);

  float mc3 = mc1;
  float mc4 = mc2;
  float mc5 = mc1;
  float mc6 = mc2;

  float avgMoisture = trimmedMean(mc1,mc2,mc3,mc4,mc5,mc6);

  float weightKg = readWeightKg();

  float tempC = dht.readTemperature();
  float hum = dht.readHumidity();

  if(isnan(tempC)) tempC = 0;
  if(isnan(hum)) hum = 0;

  // ===== BLOWER CONTROL =====
  if(avgMoisture > targetMoisture)
    blowerState = true;
  else if(avgMoisture >= targetMoisture-1 && avgMoisture <= targetMoisture)
    blowerState = false;
  else
    blowerState = true;

  digitalWrite(RELAY_BLOWER, blowerState ? HIGH : LOW);

  // ===== EXHAUST CONTROL =====
  if(tempC >= targetTemp)
    exhaustState = true;
  else
    exhaustState = false;

  digitalWrite(RELAY_EXHAUST, exhaustState ? LOW : HIGH);

  // ===== SEND DATA =====
  sendSensorData(tempC,hum,weightKg,
                 mc1,mc2,mc3,
                 mc4,mc5,mc6,
                 avgMoisture);

  // ===== SERIAL DEBUG =====
  Serial.println("====================================");

  Serial.print("Temp: "); Serial.println(tempC);
  Serial.print("Humidity: "); Serial.println(hum);
  Serial.print("Weight: "); Serial.println(weightKg);
  Serial.print("Avg Moisture: "); Serial.println(avgMoisture);

  Serial.println("====================================");
}