/*
 * AquaSol Smart Irrigation — NODE 2 FIRMWARE
 * Board: ESP32
 * MAC: 28:05:A5:24:D6:08
 *
 * Differences from Node 1:
 *  - No DHT22 (temperature/humidity come from Node 1 via backend)
 *  - TDMA Slot 1: transmits 2000ms after beacon
 *  - OLED shows VALVE ON/OFF only (no temp/humidity section)
 *
 * Hardware:
 *  LoRa SX1278     : SCK=18, MISO=19, MOSI=23, NSS=5, RST=27, DIO0=26
 *  Soil Moisture   : AO=GPIO34
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
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_task_wdt.h"

// ─── Device Identity ───────────────────────────────────────────────────────
#define NODE_MAC   "28:05:A5:24:D6:08"
#define NODE_ZONE  "ZoneB"

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
#define BATTERY_PIN  32
#define SOLAR_PIN    35

// ─── Actuator / LED Pins ───────────────────────────────────────────────────
#define VALVE_PIN    2
#define LED_POWER    25
#define LED_VALVE    13
#define LED_LORA     33

// ─── Constants ─────────────────────────────────────────────────────────────
#define SOLAR_MAX_V  18.0f
#define BAT_MAX_V    12.6f
#define BAT_MIN_V    8.0f
#define BAT_FAULT_V  9.0f

// TDMA Slot 1: 2000ms after beacon
#define TDMA_SLOT_MS   2000UL
#define BEACON_TIMEOUT 10500UL

// Watchdog
#define WDT_TIMEOUT_S  30

// Autonomous mode thresholds
#define AUTONOMOUS_ENTER_THRESHOLD  5
#define AUTONOMOUS_EXIT_THRESHOLD   5
#define AUTONOMOUS_VALVE_ON_PCT   15.0f
#define AUTONOMOUS_VALVE_OFF_PCT  30.0f

// ─── OLED ──────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─────────────────────────────────────────────────────────────────────────
// Global State
// ─────────────────────────────────────────────────────────────────────────
Preferences prefs;

bool  valveOpen        = false;
float learnedThreshold = 60.0f;

unsigned long lastBeaconTime       = 0;
int           missedBeacons        = 0;
int           consecutiveGoodBeacons = 0;
bool          autonomousMode       = false;

float soilMoisture = 0.0f;
float batteryPct   = 0.0f;
float solarEff     = 0.0f;

char statusMsg[32] = "Waiting sync";

#define DIAG_HOLD_MS 1000

// ─────────────────────────────────────────────────────────────────────────
// Soil Moisture: Piecewise Calibration (same table as Node 1)
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
// Valve control
// ─────────────────────────────────────────────────────────────────────────
void setValve(bool open) {
  valveOpen = open;
  digitalWrite(VALVE_PIN, open ? HIGH : LOW);
  digitalWrite(LED_VALVE,  open ? HIGH : LOW);
  Serial.printf("[VALVE] Node2 valve -> %s\n", open ? "OPEN" : "CLOSED");

  prefs.begin("node2", false);
  prefs.putBool("valve", valveOpen);
  prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────
// OLED — Node 2 simplified layout
// ─────────────────────────────────────────────────────────────────────────
void oledMsg(const char* line1, const char* line2) {
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0); oled.print(line1);
  oled.setCursor(0, 12); oled.print(line2);
  oled.display();
}

void oledRuntime() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Top: Soil moisture (small text)
  oled.setTextSize(1);
  char buf[32];
  snprintf(buf, sizeof(buf), "Soil: %.1f%%", soilMoisture);
  oled.setCursor(0, 0); oled.print(buf);

  snprintf(buf, sizeof(buf), "Bat: %.0f%% Sol: %.0f%%", batteryPct, solarEff);
  oled.setCursor(0, 14); oled.print(buf);

  // Divider
  oled.drawLine(0, 26, 127, 26, SSD1306_WHITE);

  // Central large: Valve ON/OFF
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
  oledMsg("Node 2 v1.0", "TDMA Slot 1");
  delay(DIAG_HOLD_MS);
  Serial.println("\n[DIAG] ===== NODE 2 BOOT DIAGNOSTICS =====");

  // LoRa
  bool loraOK = LoRa.begin(LORA_FREQ);
  oledMsg("LoRa:", loraOK ? " PASS" : " FAILED");
  Serial.printf("[DIAG] LoRa: %s\n", loraOK ? "PASS" : "FAIL");
  delay(DIAG_HOLD_MS);

  // Battery
  float batV = readBatteryVoltage();
  bool  batOK = (batV >= BAT_FAULT_V);
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

  // No DHT22
  oledMsg("DHT22:", " N/A (Node2)");
  Serial.println("[DIAG] DHT22: Not present — uses Node 1 reading");
  delay(DIAG_HOLD_MS);

  bool allOK = loraOK && batOK && soilOK;
  oledMsg(allOK ? "All OK!" : "Some Issues", "Waiting BEACON");
  snprintf(statusMsg, sizeof(statusMsg), allOK ? "Waiting sync" : "Check sensors");
  Serial.printf("[DIAG] ===== %s =====\n\n", allOK ? "ALL PASS" : "SOME FAILURES");
  delay(DIAG_HOLD_MS);
}

// ─────────────────────────────────────────────────────────────────────────
// NVS: Restore
// ─────────────────────────────────────────────────────────────────────────
void restoreState() {
  prefs.begin("node2", true);
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
  Serial.printf("[SENSORS] Soil=%.1f%% Bat=%.1f%% Solar=%.1f%%\n",
                soilMoisture, batteryPct, solarEff);
}

// ─────────────────────────────────────────────────────────────────────────
// Transmit sensor data to Master via LoRa
// Note: temp/humidity sent as 0 — backend will copy from Node 1
// ─────────────────────────────────────────────────────────────────────────
void transmitToMaster() {
  StaticJsonDocument<256> doc;
  doc["mac"]   = NODE_MAC;
  doc["zone"]  = NODE_ZONE;
  doc["soil"]  = soilMoisture;
  doc["temp"]  = 0.0f;   // no DHT22; backend copies from Node 1
  doc["hum"]   = 0.0f;
  doc["bat"]   = batteryPct;
  doc["solar"] = solarEff;
  doc["valve"] = valveOpen ? 1 : 0;

  String payload;
  serializeJson(doc, payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  digitalWrite(LED_LORA, HIGH); delay(80); digitalWrite(LED_LORA, LOW);

  Serial.printf("[LORA] >> Transmitted: %s\n", payload.c_str());
}

// ─────────────────────────────────────────────────────────────────────────
// Parse ACK from master: "ACK:<mac>:<cmd>"
// ─────────────────────────────────────────────────────────────────────────
void parseACK(const String& msg) {
  if (!msg.startsWith("ACK:")) return;
  int lastColon = msg.lastIndexOf(':');
  if (lastColon < 4) return;

  String macPart = msg.substring(4, lastColon);
  int    cmd     = msg.substring(lastColon + 1).toInt();

  if (macPart == NODE_MAC) {
    Serial.printf("[ACK] Received valve command = %d\n", cmd);
    setValve(cmd == 1);
    snprintf(statusMsg, sizeof(statusMsg), cmd ? "Normal:Valve ON" : "Normal");
  }
}

// ─────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[NODE2] AquaSol Node 2 Firmware v1.0 booting...");
  Serial.printf("[NODE2] MAC: %s\n", NODE_MAC);

  // GPIO
  pinMode(LED_POWER, OUTPUT); digitalWrite(LED_POWER, HIGH);
  pinMode(LED_VALVE, OUTPUT); digitalWrite(LED_VALVE, LOW);
  pinMode(LED_LORA,  OUTPUT); digitalWrite(LED_LORA,  LOW);
  pinMode(VALVE_PIN, OUTPUT); digitalWrite(VALVE_PIN, LOW);

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
  Serial.println("[NODE2] Boot complete — waiting for BEACON\n");
}

// ─────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────
unsigned long lastOLED = 0;

void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  // Listen for LoRa (BEACON or ACK)
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String incoming = "";
    while (LoRa.available()) incoming += (char)LoRa.read();
    incoming.trim();

    if (incoming == "BEACON") {
      Serial.printf("[TDMA] Beacon received at %lu ms — waiting slot 2000ms\n", now);
      lastBeaconTime = now;
      missedBeacons  = 0;
      consecutiveGoodBeacons++;

      // Anti-flap: exit autonomous mode
      if (autonomousMode && consecutiveGoodBeacons >= AUTONOMOUS_EXIT_THRESHOLD) {
        autonomousMode = false;
        Serial.println("[MODE] Exiting autonomous mode -> NORMAL");
        snprintf(statusMsg, sizeof(statusMsg), "Waiting sync");
      }

      // TDMA Slot 1: wait 2000ms
      delay(TDMA_SLOT_MS);

      // Read sensors
      readAllSensors();

      // Transmit
      transmitToMaster();

      // Wait for ACK (up to 1500ms)
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

    } else if (incoming.startsWith("ACK:")) {
      parseACK(incoming);
    }
  }

  // Missed beacon watchdog
  if (now - lastBeaconTime > BEACON_TIMEOUT && lastBeaconTime > 0) {
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

  // Autonomous decisions every 30 seconds
  if (autonomousMode) {
    static unsigned long lastAutoRead = 0;
    if (now - lastAutoRead >= 30000) {
      lastAutoRead = now;
      readAllSensors();
      autonomousDecision();
    }
  }

  // OLED every 2 seconds
  if (now - lastOLED >= 2000) {
    lastOLED = now;
    oledRuntime();
  }

  delay(10);
}
