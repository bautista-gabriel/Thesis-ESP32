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
int selectedTray = 0;

bool systemRunning = false;

// ===== Relay states =====
bool blowerState = false;
bool exhaustState = false;

// ===================== FUNCTIONS =====================

int readAnalogAvg(int pin,int samples){

long sum=0;

for(int i=0;i<samples;i++){
sum+=analogRead(pin);
delay(2);
yield();
}

return sum/samples;
}

float calibrateMoisture(int raw,float rawDry,float rawWet){

float mc = dryPercent +
(rawDry - raw)*(wetPercent - dryPercent)/(rawDry - rawWet);

if(mc<0) mc=0;
if(mc>50) mc=50;

return mc;
}



float readWeightKg(){

if(!scale.is_ready()) return 0;

float kg = scale.get_units(2);

if(isnan(kg)||kg<0) kg=0;

return kg;
}

// ===== Tray Based Average =====

float trayAverage(float m1,float m2,float m3,float m4,float m5,float m6,int trays){

float sum = 0;
int count = 0;

if(trays >= 1){ sum += m1; count++; }
if(trays >= 2){ sum += m2; count++; }
if(trays >= 3){ sum += m3; count++; }
if(trays >= 4){ sum += m4; count++; }
if(trays >= 5){ sum += m5; count++; }
if(trays >= 6){ sum += m6; count++; }

if(count == 0) return 0;

return sum / count;
}

// ===================== WIFI =====================

void connectWiFi(){

if(WiFi.status()==WL_CONNECTED) return;

Serial.println("Connecting to WiFi...");

WiFi.begin(ssid,password);

while(WiFi.status()!=WL_CONNECTED){
delay(500);
Serial.print(".");
}

Serial.println("\nWiFi Connected");
Serial.print("IP Address: ");
Serial.println(WiFi.localIP());

}

// ===================== GET CONFIG =====================

void getSystemConfig(){

if(WiFi.status()!=WL_CONNECTED) return;

HTTPClient http;
http.setTimeout(3000);
http.begin(configURL);

int code=http.GET();
Serial.print("HTTP Config Code: ");
Serial.println(code);

if(code==200){

String payload=http.getString();
Serial.println(payload);

StaticJsonDocument<256> doc;

DeserializationError err = deserializeJson(doc,payload);

if(err){
Serial.println("JSON parse failed");
http.end();
return;
}

targetTemp=doc["config"]["selectedTemperature"];
targetMoisture=doc["config"]["selectedMoisture"];
selectedTray=doc["config"]["selectedTray"];

systemRunning = doc["running"];

Serial.println("========= DASHBOARD SETTINGS =========");

Serial.print("Selected Temperature: ");
Serial.println(targetTemp);

Serial.print("Selected Moisture: ");
Serial.println(targetMoisture);

Serial.print("Selected Tray: ");
Serial.println(selectedTray);

Serial.print("System Running: ");
Serial.println(systemRunning);

Serial.println("======================================");

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

doc["device_id"]="esp32_01";

doc["temperature"]=tempC;
doc["humidity"]=hum;

doc["moisture1"]=mc1;
doc["moisture2"]=mc2;
doc["moisture3"]=mc3;
doc["moisture4"]=mc4;
doc["moisture5"]=mc5;
doc["moisture6"]=mc6;

doc["moistureavg"]=avgMoisture;

doc["weight1"]=weightKg;

doc["status"]=systemRunning?"Drying":"Completed";

String body;

serializeJson(doc,body);

Serial.println("Sending JSON:");
Serial.println(body);

int httpCode=http.POST(body);

Serial.print("POST Response Code: ");
Serial.println(httpCode);

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

connectWiFi();
getSystemConfig();

// ===== RAW ADC =====

int raw1=readAnalogAvg(MOISTURE1_PIN,10);
int raw2=readAnalogAvg(MOISTURE2_PIN,10);
int raw3=readAnalogAvg(MOISTURE3_PIN,10);
int raw4=readAnalogAvg(MOISTURE4_PIN,10);
int raw5=readAnalogAvg(MOISTURE5_PIN,10);
int raw6=readAnalogAvg(MOISTURE6_PIN,10);

// ===== MOISTURE =====

float mc1 = calibrateMoisture(raw1,dry1,wet1);
float mc2 = calibrateMoisture(raw2,dry2,wet2);
float mc3 = calibrateMoisture(raw3,dry3,wet3);
float mc4 = calibrateMoisture(raw4,dry4,wet4);
float mc5 = calibrateMoisture(raw5,dry5,wet5);
float mc6 = calibrateMoisture(raw6,dry6,wet6);

// ===== Tray Based Average =====

float avgMoisture = trayAverage(mc1,mc2,mc3,mc4,mc5,mc6,selectedTray);

// ===== Sensors =====

float weightKg=readWeightKg();

float tempC=dht.readTemperature();
float hum=dht.readHumidity();

if(isnan(tempC)) tempC=0;
if(isnan(hum)) hum=0;

// ===== BLOWER CONTROL =====

if(systemRunning){

if(avgMoisture > targetMoisture)
blowerState = true;
else
blowerState = false;

digitalWrite(RELAY_BLOWER, blowerState ? LOW : HIGH);

}else{

digitalWrite(RELAY_BLOWER, HIGH);

}

// ===== EXHAUST CONTROL =====

if(systemRunning){

if(tempC >= targetTemp)
exhaustState = true;
else
exhaustState = false;

digitalWrite(RELAY_EXHAUST, exhaustState ? LOW : HIGH);

}else{

digitalWrite(RELAY_EXHAUST, HIGH);

}

// ===== SEND DATA =====

sendSensorData(tempC,hum,weightKg,
mc1,mc2,mc3,
mc4,mc5,mc6,
avgMoisture);

// ===== SERIAL PRINT =====

Serial.println("=========== SENSOR DATA ===========");

Serial.print("Tray Count: "); Serial.println(selectedTray);

Serial.print("RAW ADC 1: "); Serial.println(raw1);
Serial.print("RAW ADC 2: "); Serial.println(raw2);
Serial.print("RAW ADC 3: "); Serial.println(raw3);
Serial.print("RAW ADC 4: "); Serial.println(raw4);
Serial.print("RAW ADC 5: "); Serial.println(raw5);
Serial.print("RAW ADC 6: "); Serial.println(raw6);

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

Serial.print("Blower: "); Serial.println(blowerState?"ON":"OFF");
Serial.print("Exhaust: "); Serial.println(exhaustState?"ON":"OFF");

Serial.println("===================================");

yield();
delay(5000);

}