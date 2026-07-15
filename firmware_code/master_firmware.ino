/*
 * AquaSol Smart Irrigation — MASTER FIRMWARE
 * Board: ESP32
 * MAC: 28:05:A5:25:4D:78
 * master_secret: 654321
 *
 * Hardware:
 *  LoRa SX1278  : SCK=18, MISO=19, MOSI=23, NSS=5, RST=27, DIO0=26
 *  LCD I2C 0x3F : SDA=21, SCL=22
 *  Rain Sensor  : DO=13, AO=34
 *  Flow Sensor  : Yellow=GPIO2
 *  Solar Voltage: GPIO35 (voltage divider)
 *  Battery      : GPIO32 (voltage divider)
 *  LED Power    : GPIO4  (Green, always ON)
 *  LED Pump     : GPIO25 (Yellow, ON when any valve active)
 *  LED LoRa     : GPIO33 (Blue, blink on packet receive)
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_task_wdt.h"

// ─── WiFi ──────────────────────────────────────────────────────────────────
#define WIFI_SSID      "GT 3T"
#define WIFI_PASS      "abhishek"

// ─── Backend ───────────────────────────────────────────────────────────────
#define BACKEND_INGEST   "http://127.0.0.1:8000/api/v1/sensors/ingest"
#define BACKEND_COMMANDS "http://127.0.0.1:8000/api/control/commands"
#define BACKEND_ACK      "http://127.0.0.1:8000/api/control/ack"

// ─── Device Identity ───────────────────────────────────────────────────────
#define MASTER_MAC     "28:05:A5:25:4D:78"
#define MASTER_SECRET  "654321"

// ─── LoRa Pins ─────────────────────────────────────────────────────────────
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   27
#define LORA_DIO0  26
#define LORA_FREQ  433E6

// ─── Sensor Pins ───────────────────────────────────────────────────────────
#define RAIN_DIGITAL  13
#define RAIN_ANALOG   34
#define FLOW_PIN      2
#define SOLAR_PIN     35
#define BATTERY_PIN   32

// ─── LED Pins ──────────────────────────────────────────────────────────────
#define LED_POWER  4
#define LED_PUMP   25
#define LED_LORA   33

// ─── Solar / Battery Constants ─────────────────────────────────────────────
#define SOLAR_MAX_VOLTAGE  18.0f
#define BAT_MAX_V          12.6f
#define BAT_MIN_V          8.0f

// ─── TDMA Beacon ───────────────────────────────────────────────────────────
#define BEACON_INTERVAL_MS  10000UL  // 10 seconds

// ─── Aggregation Window ────────────────────────────────────────────────────
#define AGG_WINDOW_MS  30000UL  // 30 seconds

// ─── HTTP Timeout ──────────────────────────────────────────────────────────
#define HTTP_TIMEOUT_MS  5000

// ─── Watchdog ──────────────────────────────────────────────────────────────
#define WDT_TIMEOUT_S  30

// ─────────────────────────────────────────────────────────────────────────
// Global State
// ─────────────────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x3F, 16, 2);
Preferences prefs;

// Flow sensor (interrupt driven)
volatile int pulseCount = 0;
float totalLiters    = 0.0f;
float flowRate       = 0.0f;
unsigned long lastFlowTime = 0;

// Node data received via LoRa
struct NodeData {
  bool     valid;
  char     mac[20];
  float    soilMoisture;
  float    temperature;
  float    humidity;
  float    battery;
  float    solarEff;
  int      valveStatus;
  unsigned long rxTime;
};

NodeData node1 = {false};
NodeData node2 = {false};

// Valve states restored from NVS
bool valve1Active = false;
bool valve2Active = false;

// Aggregation window
bool     aggWindowOpen = false;
unsigned long aggWindowStart = 0;

// Beacon timing
unsigned long lastBeaconTime = 0;

// Backend connectivity flag
bool backendReachable = true;

// ─────────────────────────────────────────────────────────────────────────
// Flow Sensor ISR
// ─────────────────────────────────────────────────────────────────────────
void IRAM_ATTR pulseISR() {
  pulseCount++;
}

// ─────────────────────────────────────────────────────────────────────────
// Utility: Read Solar Efficiency %
// ─────────────────────────────────────────────────────────────────────────
float readSolarEfficiency() {
  int raw = analogRead(SOLAR_PIN);
  float vOut = raw * (3.3f / 4095.0f);
  float vIn  = (6.6f * vOut) + 0.2f;
  float pct  = (vIn / SOLAR_MAX_VOLTAGE) * 100.0f;
  if (pct > 100.0f) pct = 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  return pct;
}

float readSolarVoltage() {
  int raw = analogRead(SOLAR_PIN);
  float vOut = raw * (3.3f / 4095.0f);
  return (6.6f * vOut) + 0.2f;
}

// ─────────────────────────────────────────────────────────────────────────
// Utility: Read Battery %
// ─────────────────────────────────────────────────────────────────────────
float readBatteryPercent() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }
  int raw = sum / 10;
  float vOut = raw * (3.3f / 4095.0f);
  float vIn  = vOut * 4.13f;
  float pct  = ((vIn - BAT_MIN_V) / (BAT_MAX_V - BAT_MIN_V)) * 100.0f;
  if (pct > 100.0f) pct = 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  return pct;
}

// ─────────────────────────────────────────────────────────────────────────
// Utility: Update flow rate every second
// ─────────────────────────────────────────────────────────────────────────
void updateFlow() {
  unsigned long now = millis();
  if (now - lastFlowTime >= 1000) {
    int count = pulseCount;
    pulseCount = 0;
    lastFlowTime = now;
    flowRate = count / 7.5f;
    float litersThisSec = count / 450.0f;
    totalLiters += litersThisSec;
  }
}

// ─────────────────────────────────────────────────────────────────────────
// LCD helpers (backlight OFF during normal operation per spec)
// ─────────────────────────────────────────────────────────────────────────
void lcdShow(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}

// ─────────────────────────────────────────────────────────────────────────
// Boot Diagnostics
// ─────────────────────────────────────────────────────────────────────────
void runDiagnostics() {
  Serial.println("\n[DIAG] ===== MASTER BOOT DIAGNOSTICS =====");
  lcdShow("AquaSol Master", "v1.0 Starting...");
  delay(1500);

  // LoRa
  bool loraOK = LoRa.begin(LORA_FREQ);
  Serial.printf("[DIAG] LoRa: %s\n", loraOK ? "PASS" : "FAIL");
  lcdShow("LoRa:", loraOK ? " PASS" : " FAIL");
  delay(1000);

  // Battery
  float bat = readBatteryPercent();
  bool batOK = (bat > 10.0f);
  Serial.printf("[DIAG] Battery: %.1f%%  %s\n", bat, batOK ? "OK" : "LOW");
  char buf[16]; snprintf(buf, sizeof(buf), " %.1f%%", bat);
  lcdShow("Battery:", buf);
  delay(1000);

  // Rain sensor
  int rainD = digitalRead(RAIN_DIGITAL);
  Serial.printf("[DIAG] Rain sensor digital: %d (%s)\n", rainD, rainD == LOW ? "RAIN" : "DRY");
  lcdShow("Rain Sensor:", rainD == LOW ? " RAIN" : " DRY");
  delay(1000);

  // Flow sensor (just check pin)
  Serial.println("[DIAG] Flow sensor: attached to GPIO2");
  lcdShow("Flow Sensor:", " ATTACHED");
  delay(1000);

  // WiFi
  Serial.printf("[DIAG] WiFi SSID: %s\n", WIFI_SSID);
  lcdShow("WiFi:", " Connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); tries++;
    Serial.print(".");
  }
  bool wifiOK = (WiFi.status() == WL_CONNECTED);
  Serial.printf("\n[DIAG] WiFi: %s  IP: %s\n", wifiOK ? "PASS" : "FAIL",
                wifiOK ? WiFi.localIP().toString().c_str() : "N/A");
  lcdShow("WiFi:", wifiOK ? " CONNECTED" : " NO WIFI");
  delay(1000);

  bool allOK = loraOK && batOK && wifiOK;
  Serial.printf("[DIAG] ===== ALL %s =====\n\n", allOK ? "PASS" : "SOME FAILURES");
  lcdShow(allOK ? "All OK!" : "Some Issues", "Starting...");
  delay(1500);
  lcd.noBacklight();
}

// ─────────────────────────────────────────────────────────────────────────
// NVS: Save / Restore valve state
// ─────────────────────────────────────────────────────────────────────────
void saveValveStates() {
  prefs.begin("irrigation", false);
  prefs.putBool("v1", valve1Active);
  prefs.putBool("v2", valve2Active);
  prefs.end();
}

void restoreValveStates() {
  prefs.begin("irrigation", true);
  valve1Active = prefs.getBool("v1", false);
  valve2Active = prefs.getBool("v2", false);
  prefs.end();
  Serial.printf("[NVS] Restored valve1=%d valve2=%d\n", valve1Active, valve2Active);
}

// ─────────────────────────────────────────────────────────────────────────
// LoRa: Send Beacon
// ─────────────────────────────────────────────────────────────────────────
void sendBeacon() {
  LoRa.beginPacket();
  LoRa.print("BEACON");
  LoRa.endPacket();
  Serial.println("[LORA] Beacon sent");
}

// ─────────────────────────────────────────────────────────────────────────
// LoRa: Send valve ACK to a node
// Format: ACK:<node_mac>:<valve_cmd>
// valve_cmd: 1=open, 0=close
// ─────────────────────────────────────────────────────────────────────────
void sendValveACK(const char* nodeMac, int cmd) {
  char msg[64];
  snprintf(msg, sizeof(msg), "ACK:%s:%d", nodeMac, cmd);
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.printf("[LORA] ACK sent -> %s valve=%d\n", nodeMac, cmd);
}

// ─────────────────────────────────────────────────────────────────────────
// LoRa: Parse incoming JSON packet from nodes
// Expected: {"mac":"...","soil":45.2,"temp":28.1,"hum":62.0,
//            "bat":78.5,"solar":65.3,"valve":0}
// ─────────────────────────────────────────────────────────────────────────
void parseNodePacket(const String& json) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.printf("[LORA] JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* mac = doc["mac"] | "";
  if (strlen(mac) == 0) return;

  NodeData nd;
  nd.valid       = true;
  nd.soilMoisture = doc["soil"]  | 0.0f;
  nd.temperature  = doc["temp"]  | 0.0f;
  nd.humidity     = doc["hum"]   | 0.0f;
  nd.battery      = doc["bat"]   | 0.0f;
  nd.solarEff     = doc["solar"] | 0.0f;
  nd.valveStatus  = doc["valve"] | 0;
  nd.rxTime       = millis();
  strncpy(nd.mac, mac, sizeof(nd.mac)-1);

  // Identify node by MAC
  if (strcmp(mac, "E0:8C:FE:34:1B:18") == 0) {
    node1 = nd;
    Serial.println("[LORA] << Node 1 packet received:");
    Serial.printf("  MAC=%s Soil=%.1f%% Temp=%.1fC Hum=%.1f%% Bat=%.1f%% Solar=%.1f%% Valve=%d\n",
      nd.mac, nd.soilMoisture, nd.temperature, nd.humidity, nd.battery, nd.solarEff, nd.valveStatus);
    sendValveACK(mac, valve1Active ? 1 : 0);
  } else if (strcmp(mac, "28:05:A5:24:D6:08") == 0) {
    node2 = nd;
    // Node 2 has no DHT22 — copy from node1 if available
    if (node1.valid) {
      node2.temperature = node1.temperature;
      node2.humidity    = node1.humidity;
    }
    Serial.println("[LORA] << Node 2 packet received:");
    Serial.printf("  MAC=%s Soil=%.1f%% Bat=%.1f%% Solar=%.1f%% Valve=%d (Temp/Hum from Node1)\n",
      nd.mac, nd.soilMoisture, nd.battery, nd.solarEff, nd.valveStatus);
    sendValveACK(mac, valve2Active ? 1 : 0);
  }

  // Flash LoRa LED
  digitalWrite(LED_LORA, HIGH);
  delay(80);
  digitalWrite(LED_LORA, LOW);

  // Open aggregation window on first packet
  if (!aggWindowOpen) {
    aggWindowOpen  = true;
    aggWindowStart = millis();
    Serial.println("[AGG] Aggregation window opened (30s)");
  }
}

// ─────────────────────────────────────────────────────────────────────────
// LoRa: Receive all waiting packets
// ─────────────────────────────────────────────────────────────────────────
void receiveLoRa() {
  int pktSize = LoRa.parsePacket();
  while (pktSize > 0) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    incoming.trim();
    Serial.printf("[LORA] Raw RX (%d bytes): %s\n", pktSize, incoming.c_str());
    if (incoming.startsWith("{")) {
      parseNodePacket(incoming);
    }
    pktSize = LoRa.parsePacket();
  }
}

// ─────────────────────────────────────────────────────────────────────────
// HTTP: POST telemetry batch to backend
// ─────────────────────────────────────────────────────────────────────────
void postTelemetryBatch() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] WiFi not connected — skipping ingest");
    backendReachable = false;
    return;
  }

  float solarEff = readSolarEfficiency();
  float solarV   = readSolarVoltage();
  float batPct   = readBatteryPercent();
  int   rainD    = digitalRead(RAIN_DIGITAL);

  // Build JSON
  StaticJsonDocument<1024> doc;
  doc["master_mac"]    = MASTER_MAC;
  doc["master_secret"] = MASTER_SECRET;
  doc["battery_pct"]   = batPct;
  doc["solar_pct"]     = solarEff;
  doc["solar_voltage"] = solarV;
  doc["rain_detected"] = (rainD == LOW) ? 1 : 0;
  doc["flow_rate"]     = flowRate;
  doc["total_water"]   = totalLiters;

  JsonArray events = doc.createNestedArray("events");

  if (node1.valid) {
    JsonObject n1 = events.createNestedObject();
    n1["node_mac"]     = node1.mac;
    n1["soil_moisture"]= node1.soilMoisture;
    n1["temperature"]  = node1.temperature;
    n1["humidity"]     = node1.humidity;
    n1["battery_pct"]  = node1.battery;
    n1["solar_pct"]    = node1.solarEff;
    n1["valve_status"] = node1.valveStatus;
  }

  if (node2.valid) {
    JsonObject n2 = events.createNestedObject();
    n2["node_mac"]     = node2.mac;
    n2["soil_moisture"]= node2.soilMoisture;
    n2["temperature"]  = node2.temperature;   // copied from node1
    n2["humidity"]     = node2.humidity;       // copied from node1
    n2["battery_pct"]  = node2.battery;
    n2["solar_pct"]    = node2.solarEff;
    n2["valve_status"] = node2.valveStatus;
  }

  String payload;
  serializeJson(doc, payload);

  Serial.println("\n[HTTP] POST /sensors/ingest");
  Serial.println(payload);

  HTTPClient http;
  http.begin(BACKEND_INGEST);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(HTTP_TIMEOUT_MS);

  int code = http.POST(payload);
  if (code > 0) {
    backendReachable = true;
    String resp = http.getString();
    Serial.printf("[HTTP] Ingest response %d: %s\n", code, resp.c_str());

    // Parse valve commands from response
    StaticJsonDocument<512> rDoc;
    if (!deserializeJson(rDoc, resp)) {
      if (rDoc.containsKey("valve1")) {
        valve1Active = rDoc["valve1"].as<bool>();
        saveValveStates();
      }
      if (rDoc.containsKey("valve2")) {
        valve2Active = rDoc["valve2"].as<bool>();
        saveValveStates();
      }
    }
  } else {
    backendReachable = false;
    Serial.printf("[HTTP] Ingest FAILED: %s\n", http.errorToString(code).c_str());
  }
  http.end();
}

// ─────────────────────────────────────────────────────────────────────────
// HTTP: Poll for pending valve commands
// ─────────────────────────────────────────────────────────────────────────
void pollCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(BACKEND_COMMANDS) + "?mac_address=" + MASTER_MAC;
  HTTPClient http;
  http.begin(url);
  http.setTimeout(HTTP_TIMEOUT_MS);

  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();
    Serial.printf("[HTTP] Commands: %s\n", resp.c_str());

    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, resp)) {
      const char* action = doc["action"] | "";
      const char* cmdId  = doc["command_id"] | "";
      int zone           = doc["zone"] | 0;

      if (strcmp(action, "START_IRRIGATION") == 0) {
        Serial.printf("[CMD] START_IRRIGATION zone=%d\n", zone);
        if (zone == 1) { valve1Active = true; }
        if (zone == 2) { valve2Active = true; }
        saveValveStates();
        sendACK(cmdId, "executed");
      } else if (strcmp(action, "STOP_IRRIGATION") == 0) {
        Serial.printf("[CMD] STOP_IRRIGATION zone=%d\n", zone);
        if (zone == 1) { valve1Active = false; }
        if (zone == 2) { valve2Active = false; }
        if (zone == 0) { valve1Active = false; valve2Active = false; }
        saveValveStates();
        sendACK(cmdId, "executed");
      } else if (strcmp(action, "REBOOT") == 0) {
        sendACK(cmdId, "executed");
        delay(500);
        ESP.restart();
      }
    }
  }
  http.end();
}

// ─────────────────────────────────────────────────────────────────────────
// HTTP: Send ACK for a command
// ─────────────────────────────────────────────────────────────────────────
void sendACK(const char* cmdId, const char* status) {
  if (!cmdId || strlen(cmdId) == 0) return;
  StaticJsonDocument<128> doc;
  doc["command_id"] = cmdId;
  doc["status"]     = status;
  String payload;
  serializeJson(doc, payload);

  HTTPClient http;
  http.begin(BACKEND_ACK);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(HTTP_TIMEOUT_MS);
  int code = http.POST(payload);
  Serial.printf("[HTTP] ACK sent, response: %d\n", code);
  http.end();
}

// ─────────────────────────────────────────────────────────────────────────
// Serial Monitor: Print master sensor dashboard
// ─────────────────────────────────────────────────────────────────────────
unsigned long lastSerialPrint = 0;
void printSerialDashboard() {
  unsigned long now = millis();
  if (now - lastSerialPrint < 5000) return;
  lastSerialPrint = now;

  float solarEff = readSolarEfficiency();
  float solarV   = readSolarVoltage();
  float batPct   = readBatteryPercent();
  int   rainD    = digitalRead(RAIN_DIGITAL);
  int   rainA    = analogRead(RAIN_ANALOG);

  Serial.println("\n═══════════════════════════════════════════════════");
  Serial.println("            AquaSol MASTER — Sensor Dashboard");
  Serial.println("═══════════════════════════════════════════════════");
  Serial.printf("  Battery       : %.1f%%\n", batPct);
  Serial.printf("  Solar Voltage : %.2f V\n", solarV);
  Serial.printf("  Solar Eff     : %.1f%%\n", solarEff);
  Serial.printf("  Rain (Digital): %s  (Analog: %d)\n",
                rainD == LOW ? "RAIN DETECTED 🌧" : "NO RAIN ☀", rainA);
  Serial.printf("  Flow Rate     : %.2f L/min\n", flowRate);
  Serial.printf("  Total Water   : %.3f L\n", totalLiters);
  Serial.printf("  Backend       : %s\n", backendReachable ? "REACHABLE ✓" : "OFFLINE ✗");
  Serial.println("───────────────────────────────────────────────────");
  if (node1.valid) {
    Serial.println("  NODE 1 (E0:8C:FE:34:1B:18):");
    Serial.printf("    Soil=%.1f%%  Temp=%.1f°C  Hum=%.1f%%\n",
                  node1.soilMoisture, node1.temperature, node1.humidity);
    Serial.printf("    Battery=%.1f%%  Solar=%.1f%%  Valve=%s\n",
                  node1.battery, node1.solarEff, node1.valveStatus ? "OPEN" : "CLOSED");
    Serial.printf("    Last RX: %lu ms ago\n", millis() - node1.rxTime);
  } else {
    Serial.println("  NODE 1: No data yet");
  }
  Serial.println("───────────────────────────────────────────────────");
  if (node2.valid) {
    Serial.println("  NODE 2 (28:05:A5:24:D6:08):");
    Serial.printf("    Soil=%.1f%%  Temp=%.1f°C (from N1)  Hum=%.1f%% (from N1)\n",
                  node2.soilMoisture, node2.temperature, node2.humidity);
    Serial.printf("    Battery=%.1f%%  Solar=%.1f%%  Valve=%s\n",
                  node2.battery, node2.solarEff, node2.valveStatus ? "OPEN" : "CLOSED");
    Serial.printf("    Last RX: %lu ms ago\n", millis() - node2.rxTime);
  } else {
    Serial.println("  NODE 2: No data yet");
  }
  Serial.println("═══════════════════════════════════════════════════\n");
}

// ─────────────────────────────────────────────────────────────────────────
// LCD: Update pump status
// ─────────────────────────────────────────────────────────────────────────
void updateLCD() {
  bool anyValveActive = valve1Active || valve2Active;
  char line2[17] = {0};
  if (valve1Active && valve2Active) snprintf(line2, sizeof(line2), "Z1+Z2 OPEN");
  else if (valve1Active)            snprintf(line2, sizeof(line2), "ZONE 1 OPEN");
  else if (valve2Active)            snprintf(line2, sizeof(line2), "ZONE 2 OPEN");
  else                              snprintf(line2, sizeof(line2), "ALL CLOSED");
  lcdShow(anyValveActive ? "PUMP ON" : "PUMP OFF", line2);
  digitalWrite(LED_PUMP, anyValveActive ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n[MASTER] AquaSol Master Firmware v1.0 booting...");
  Serial.printf("[MASTER] MAC: %s\n", MASTER_MAC);

  // GPIO
  pinMode(LED_POWER, OUTPUT); digitalWrite(LED_POWER, HIGH);
  pinMode(LED_PUMP,  OUTPUT); digitalWrite(LED_PUMP,  LOW);
  pinMode(LED_LORA,  OUTPUT); digitalWrite(LED_LORA,  LOW);
  pinMode(RAIN_DIGITAL, INPUT);
  pinMode(FLOW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseISR, RISING);

  // I2C + LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.noBacklight();

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Restore NVS
  restoreValveStates();

  // Boot Diagnostics
  runDiagnostics();

  // Watchdog
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  lastBeaconTime = millis();
  lastFlowTime   = millis();
  Serial.println("[MASTER] Boot complete — entering main loop\n");
}

// ─────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────
void loop() {
  esp_task_wdt_reset();

  unsigned long now = millis();

  // 1. Flow sensor update
  updateFlow();

  // 2. Receive LoRa packets
  receiveLoRa();

  // 3. Send TDMA beacon every 10s
  if (now - lastBeaconTime >= BEACON_INTERVAL_MS) {
    lastBeaconTime = now;
    sendBeacon();
  }

  // 4. Aggregation window: flush after 30s or when both nodes received
  if (aggWindowOpen) {
    bool bothReceived = node1.valid && node2.valid &&
                        (now - node1.rxTime < AGG_WINDOW_MS) &&
                        (now - node2.rxTime < AGG_WINDOW_MS);
    bool windowExpired = (now - aggWindowStart >= AGG_WINDOW_MS);

    if (bothReceived || windowExpired) {
      Serial.println("[AGG] Window closed — posting to backend");
      aggWindowOpen = false;
      postTelemetryBatch();
    }
  }

  // 5. Poll commands every 15 seconds
  static unsigned long lastPoll = 0;
  if (now - lastPoll >= 15000) {
    lastPoll = now;
    pollCommands();
  }

  // 6. Update LCD every 2 seconds
  static unsigned long lastLCD = 0;
  if (now - lastLCD >= 2000) {
    lastLCD = now;
    updateLCD();
  }

  // 7. Serial dashboard every 5 seconds
  printSerialDashboard();

  delay(10);
}
