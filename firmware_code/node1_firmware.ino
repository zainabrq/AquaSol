/*
 * AquaSol Smart Irrigation — NODE 1 FIRMWARE
 * Board: ESP32
 * MAC: E0:8C:FE:34:1B:18
 *
 * Hardware:
 *  LoRa SX1278     : SCK=18, MISO=19, MOSI=23, NSS=5, RST=27, DIO0=26
 *  Soil Moisture   : AO=GPIO34
 *  DHT22           : DATA=GPIO4
 *  Battery Voltage : GPIO32
 *  Solar Voltage   : GPIO35
 *  MOSFET Valve    : Gate=GPIO2 (via 220Ω)
 *  LED Power       : GPIO25 (always ON)
 *  LED Valve       : GPIO13 (ON when valve active)
 *  LED LoRa        : GPIO33 (blink on send)
 *  OLED SSD1306    : SDA=21, SCL=22
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_task_wdt.h"

// ─── Device Identity ───────────────────────────────────────────────────────
#define NODE_MAC    "E0:8C:FE:34:1B:18"
#define NODE_ZONE   "ZoneA"

// ─── LoRa Pins ─────────────────────────────────────────────────────────────
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   27
#define LORA_DIO0  26
#define LORA_FREQ  433E6

// ─── Sensor Pins ───────────────────────────────────────────────────────────
#define SOIL_PIN     34
#define DHT_PIN      4
#define BATTERY_PIN  32
#define SOLAR_PIN    35

// ─── Actuator / LED Pins ───────────────────────────────────────────────────
#define VALVE_PIN    2
#define LED_POWER    25
#define LED_VALVE    13
#define LED_LORA     33

// ─── DHT22 ─────────────────────────────────────────────────────────────────
#define DHT_TYPE  DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// ─── OLED ──────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─── Constants ─────────────────────────────────────────────────────────────
#define SOLAR_MAX_V    18.0f
#define BAT_MAX_V      12.6f
#define BAT_MIN_V      8.0f
#define BAT_FAULT_V    9.0f

// TDMA slot: Node1 transmits 500ms after beacon
#define TDMA_SLOT_MS   500UL
#define BEACON_TIMEOUT 10500UL  // 10s beacon + tolerance

// Watchdog
#define WDT_TIMEOUT_S  30

// Autonomous mode: if master silent for 5 consecutive beacons
#define AUTONOMOUS_ENTER_THRESHOLD  5
#define AUTONOMOUS_EXIT_THRESHOLD   5

// Autonomous fallback thresholds (used when backend offline)
#define AUTONOMOUS_VALVE_ON_PCT   15.0f
#define AUTONOMOUS_VALVE_OFF_PCT  30.0f

// ─────────────────────────────────────────────────────────────────────────
// Global State
// ─────────────────────────────────────────────────────────────────────────
Preferences prefs;

bool  valveOpen         = false;
float learnedThreshold  = 60.0f;  // Will be restored from NVS

// TDMA / beacon tracking
unsigned long lastBeaconTime     = 0;
bool          beaconReceived     = false;
int           missedBeacons      = 0;
int           consecutiveGoodBeacons = 0;
bool          autonomousMode     = false;

// Sensor values
float soilMoisture = 0.0f;
float temperature  = 0.0f;
float humidity     = 0.0f;
float batteryPct   = 0.0f;
float solarEff     = 0.0f;

// Status string for OLED
char statusMsg[32] = "Waiting POLL";

// Diagnostics
#define DIAG_HOLD_MS 1000

// ─────────────────────────────────────────────────────────────────────────
// Soil Moisture: Piecewise Calibration
// Raw ADC -> Moisture %
// ─────────────────────────────────────────────────────────────────────────
struct CalPoint { int raw; float pct; };
const CalPoint calTable[] = {
  {2500, 0.0f},
  {1900, 10.0f},
  {1500, 25.0f},
  {900,  50.0f},
  {650,  100.0f}
};
const int calTableLen = 5;

float piecewiseInterpolate(int raw) {
  // Clamp
  if (raw >= calTable[0].raw)             return calTable[0].pct;
  if (raw <= calTable[calTableLen-1].raw) return calTable[calTableLen-1].pct;
  for (int i = 0; i < calTableLen - 1; i++) {
    if (raw <= calTable[i].raw && raw >= calTable[i+1].raw) {
      float t = (float)(calTable[i].raw - raw) /
                (float)(calTable[i].raw - calTable[i+1].raw);
      return calTable[i].pct + t * (calTable[i+1].pct - calTable[i].pct);
    }
  }
  return 0.0f;
}

float readSoilMoisture() {
  int raw = analogRead(SOIL_PIN);
  return piecewiseInterpolate(raw);
}

// ─────────────────────────────────────────────────────────────────────────
// Battery %
// ─────────────────────────────────────────────────────────────────────────
float readBatteryPercent() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }
  float vOut = (sum / 10) * (3.3f / 4095.0f);
  float vIn  = vOut * 4.13f;
  float pct  = ((vIn - BAT_MIN_V) / (BAT_MAX_V - BAT_MIN_V)) * 100.0f;
  if (pct > 100.0f) pct = 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  return pct;
}

float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }
  float vOut = (sum / 10) * (3.3f / 4095.0f);
  return vOut * 4.13f;
}

// ─────────────────────────────────────────────────────────────────────────
// Solar Efficiency %
// ─────────────────────────────────────────────────────────────────────────
float readSolarEfficiency() {
  int raw = analogRead(SOLAR_PIN);
  float vOut = raw * (3.3f / 4095.0f);
  float vIn  = (6.6f * vOut) + 0.2f;
  float pct  = (vIn / SOLAR_MAX_V) * 100.0f;
  if (pct > 100.0f) pct = 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  return pct;
}

// ─────────────────────────────────────────────────────────────────────────
// DHT22
// ─────────────────────────────────────────────────────────────────────────
void readDHT(float &t, float &h) {
  t = dht.readTemperature();
  h = dht.readHumidity();
  if (isnan(t)) t = -99.0f;
  if (isnan(h)) h = -99.0f;
}

// ─────────────────────────────────────────────────────────────────────────
// Valve control
// ─────────────────────────────────────────────────────────────────────────
void setValve(bool open) {
  valveOpen = open;
  digitalWrite(VALVE_PIN, open ? HIGH : LOW);
  digitalWrite(LED_VALVE,  open ? HIGH : LOW);
  Serial.printf("[VALVE] Node1 valve -> %s\n", open ? "OPEN" : "CLOSED");

  // Save to NVS
  prefs.begin("node1", false);
  prefs.putBool("valve", valveOpen);
  prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────
// OLED helpers
// ─────────────────────────────────────────────────────────────────────────
void oledMsg(const char* line1, const char* line2) {
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,0); oled.print(line1);
  oled.setCursor(0,12); oled.print(line2);
  oled.display();
}

void oledRuntime() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Top: Temp + Humidity (small)
  oled.setTextSize(1);
  char buf[32];
  snprintf(buf, sizeof(buf), "T:%.1fC H:%.1f%%", temperature, humidity);
  oled.setCursor(0, 0); oled.print(buf);

  // Middle: Soil moisture
  snprintf(buf, sizeof(buf), "Soil: %.1f%%", soilMoisture);
  oled.setCursor(0, 14); oled.print(buf);

  // Divider
  oled.drawLine(0, 26, 127, 26, SSD1306_WHITE);

  // Large: Valve status
  oled.setTextSize(2);
  oled.setCursor(10, 32);
  oled.print(valveOpen ? "VALVE ON" : "VALVE OFF");

  // Bottom: Status
  oled.setTextSize(1);
  oled.setCursor(0, 56); oled.print(statusMsg);

  oled.display();
}

// ─────────────────────────────────────────────────────────────────────────
// Boot Diagnostics
// ─────────────────────────────────────────────────────────────────────────
void runDiagnostics() {
  oledMsg("Node 1 v1.0", "TDMA mode");
  delay(DIAG_HOLD_MS);
  Serial.println("\n[DIAG] ===== NODE 1 BOOT DIAGNOSTICS =====");

  // LoRa test
  bool loraOK = LoRa.begin(LORA_FREQ);
  oledMsg("LoRa:", loraOK ? " PASS" : " FAILED");
  Serial.printf("[DIAG] LoRa: %s\n", loraOK ? "PASS" : "FAIL");
  delay(DIAG_HOLD_MS);

  // Battery
  float batV = readBatteryVoltage();
  bool batOK = (batV >= BAT_FAULT_V);
  char buf[20]; snprintf(buf, sizeof(buf), " %.1fV %s", batV, batOK ? "OK" : "LOW!");
  oledMsg("Battery:", buf);
  Serial.printf("[DIAG] Battery: %.1fV %s\n", batV, batOK ? "OK" : "LOW");
  delay(DIAG_HOLD_MS);

  // Soil sensor
  int rawSoil = analogRead(SOIL_PIN);
  bool soilOK = (rawSoil > 0 && rawSoil < 4095);
  snprintf(buf, sizeof(buf), " raw=%d %s", rawSoil, soilOK ? "OK" : "FAULT");
  oledMsg("Soil:", buf);
  Serial.printf("[DIAG] Soil raw=%d %s\n", rawSoil, soilOK ? "OK" : "FAULT");
  delay(DIAG_HOLD_MS);

  // DHT22
  float t, h; readDHT(t, h);
  bool dhtOK = (t > -99.0f);
  snprintf(buf, sizeof(buf), " %.1fC %.1f%%", t, h);
  oledMsg("DHT22:", dhtOK ? buf : " ERROR");
  Serial.printf("[DIAG] DHT22: %s\n", dhtOK ? buf : "ERROR");
  delay(DIAG_HOLD_MS);

  bool allOK = loraOK && batOK && soilOK;
  oledMsg(allOK ? "All OK!" : "Some Issues", "Waiting BEACON");
  snprintf(statusMsg, sizeof(statusMsg), allOK ? "Waiting POLL" : "Check sensors");
  Serial.printf("[DIAG] ===== %s =====\n\n", allOK ? "ALL PASS" : "SOME FAILURES");
  delay(DIAG_HOLD_MS);
}

// ─────────────────────────────────────────────────────────────────────────
// NVS: Restore
// ─────────────────────────────────────────────────────────────────────────
void restoreState() {
  prefs.begin("node1", true);
  valveOpen        = prefs.getBool("valve", false);
  learnedThreshold = prefs.getFloat("thresh", 60.0f);
  prefs.end();
  setValve(valveOpen);
  Serial.printf("[NVS] Restored: valve=%d learnedThreshold=%.1f\n",
                valveOpen, learnedThreshold);
}

// ─────────────────────────────────────────────────────────────────────────
// Autonomous mode: decide valve based on soil moisture
// ─────────────────────────────────────────────────────────────────────────
void autonomousDecision() {
  if (soilMoisture < AUTONOMOUS_VALVE_ON_PCT && !valveOpen) {
    Serial.printf("[AUTO] Soil %.1f%% < %.1f%% -> OPEN valve\n",
                  soilMoisture, AUTONOMOUS_VALVE_ON_PCT);
    setValve(true);
    snprintf(statusMsg, sizeof(statusMsg), "AUTO: Valve ON");
  } else if (soilMoisture >= AUTONOMOUS_VALVE_OFF_PCT && valveOpen) {
    Serial.printf("[AUTO] Soil %.1f%% >= %.1f%% -> CLOSE valve\n",
                  soilMoisture, AUTONOMOUS_VALVE_OFF_PCT);
    setValve(false);
    snprintf(statusMsg, sizeof(statusMsg), "AUTO: Valve OFF");
  }
}

// ─────────────────────────────────────────────────────────────────────────
// Read all sensors
// ─────────────────────────────────────────────────────────────────────────
void readAllSensors() {
  soilMoisture = readSoilMoisture();
  batteryPct   = readBatteryPercent();
  solarEff     = readSolarEfficiency();
  readDHT(temperature, humidity);

  Serial.printf("[SENSORS] Soil=%.1f%% Temp=%.1fC Hum=%.1f%% Bat=%.1f%% Solar=%.1f%%\n",
                soilMoisture, temperature, humidity, batteryPct, solarEff);
}

// ─────────────────────────────────────────────────────────────────────────
// Transmit sensor data to Master via LoRa
// ─────────────────────────────────────────────────────────────────────────
void transmitToMaster() {
  StaticJsonDocument<256> doc;
  doc["mac"]   = NODE_MAC;
  doc["zone"]  = NODE_ZONE;
  doc["soil"]  = soilMoisture;
  doc["temp"]  = temperature;
  doc["hum"]   = humidity;
  doc["bat"]   = batteryPct;
  doc["solar"] = solarEff;
  doc["valve"] = valveOpen ? 1 : 0;

  String payload;
  serializeJson(doc, payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  // Blink LoRa LED
  digitalWrite(LED_LORA, HIGH); delay(80); digitalWrite(LED_LORA, LOW);

  Serial.printf("[LORA] >> Transmitted: %s\n", payload.c_str());
}

// ─────────────────────────────────────────────────────────────────────────
// Parse ACK from master: "ACK:E0:8C:FE:34:1B:18:1"
// ─────────────────────────────────────────────────────────────────────────
void parseACK(const String& msg) {
  // Format: ACK:<mac>:<cmd>
  if (!msg.startsWith("ACK:")) return;
  int lastColon = msg.lastIndexOf(':');
  if (lastColon < 4) return;

  String macPart = msg.substring(4, lastColon);
  int    cmd     = msg.substring(lastColon + 1).toInt();

  if (macPart == NODE_MAC) {
    Serial.printf("[ACK] Received valve command = %d\n", cmd);
    setValve(cmd == 1);
    snprintf(statusMsg, sizeof(statusMsg), cmd ? "Normal:Valve ON" : "Normal:Valve OFF");
  }
}

// ─────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[NODE1] AquaSol Node 1 Firmware v1.0 booting...");
  Serial.printf("[NODE1] MAC: %s\n", NODE_MAC);

  // GPIO
  pinMode(LED_POWER, OUTPUT); digitalWrite(LED_POWER, HIGH);
  pinMode(LED_VALVE, OUTPUT); digitalWrite(LED_VALVE, LOW);
  pinMode(LED_LORA,  OUTPUT); digitalWrite(LED_LORA,  LOW);
  pinMode(VALVE_PIN, OUTPUT); digitalWrite(VALVE_PIN, LOW);

  // DHT22
  dht.begin();

  // I2C + OLED
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] SSD1306 init failed");
  } else {
    oled.clearDisplay(); oled.display();
  }

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Restore NVS
  restoreState();

  // Boot Diagnostics
  runDiagnostics();

  // Watchdog
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  lastBeaconTime = millis();
  Serial.println("[NODE1] Boot complete — waiting for BEACON\n");
}

// ─────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────
unsigned long lastOLED = 0;
unsigned long beaconWaitStart = 0;
bool waitingForBeacon = true;

void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  // Listen for LoRa packets (BEACON or ACK)
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String incoming = "";
    while (LoRa.available()) incoming += (char)LoRa.read();
    incoming.trim();

    if (incoming == "BEACON") {
      // Beacon received — schedule TDMA slot
      Serial.printf("[TDMA] Beacon received at %lu ms\n", now);
      lastBeaconTime = now;
      missedBeacons  = 0;
      consecutiveGoodBeacons++;

      // Anti-flap: exit autonomous after 5 good beacons
      if (autonomousMode && consecutiveGoodBeacons >= AUTONOMOUS_EXIT_THRESHOLD) {
        autonomousMode = false;
        Serial.println("[MODE] Exiting autonomous mode -> NORMAL");
        snprintf(statusMsg, sizeof(statusMsg), "Waiting POLL");
      }

      // Wait TDMA slot 0 = 500ms, then transmit
      delay(TDMA_SLOT_MS);

      // Read sensors fresh
      readAllSensors();

      // Transmit
      transmitToMaster();

      // Now wait for ACK (up to 1500ms)
      unsigned long ackStart = millis();
      while (millis() - ackStart < 1500) {
        int aSz = LoRa.parsePacket();
        if (aSz > 0) {
          String ackMsg = "";
          while (LoRa.available()) ackMsg += (char)LoRa.read();
          ackMsg.trim();
          if (ackMsg.startsWith("ACK:")) {
            parseACK(ackMsg);
            break;
          }
        }
        delay(10);
      }

      waitingForBeacon = false;

    } else if (incoming.startsWith("ACK:")) {
      parseACK(incoming);
    }
  }

  // Check for missed beacons
  if (now - lastBeaconTime > BEACON_TIMEOUT && lastBeaconTime > 0) {
    // Reset to detect next miss correctly
    lastBeaconTime = now;
    missedBeacons++;
    consecutiveGoodBeacons = 0;
    Serial.printf("[TDMA] Missed beacon count: %d\n", missedBeacons);

    if (!autonomousMode && missedBeacons >= AUTONOMOUS_ENTER_THRESHOLD) {
      autonomousMode = true;
      Serial.println("[MODE] Entering AUTONOMOUS mode");
      snprintf(statusMsg, sizeof(statusMsg), "Autonomous Mode");
    }
  }

  // Autonomous mode: re-read sensors and decide
  if (autonomousMode) {
    static unsigned long lastAutoRead = 0;
    if (now - lastAutoRead >= 30000) {  // every 30 seconds
      lastAutoRead = now;
      readAllSensors();
      autonomousDecision();
    }
  }

  // OLED update every 2 seconds
  if (now - lastOLED >= 2000) {
    lastOLED = now;
    oledRuntime();
  }

  delay(10);
}
