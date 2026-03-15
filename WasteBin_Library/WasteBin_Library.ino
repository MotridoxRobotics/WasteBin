/* =====================================================================
 * WESTO Smart Waste Bin — Library Edition
 * =====================================================================
 * Firmware v3.0.0
 *
 * Rewritten using StepperDriver (A4988) + ESP32Servo libraries for
 * cleaner, simpler code. Pin mapping from WasteBin_Components_test.ino.
 *
 * Hardware:
 *   - ESP32 DevKit
 *   - 2x HC-SR04 Ultrasonic (top: plate/waste, front: person detection)
 *   - 2x Stepper Motors + A4988 Drivers (commonly wired — single control)
 *   - 1x Servo Motor (lid)
 *
 * Libraries (install via Arduino Library Manager):
 *   - "StepperDriver" by Laurentiu Badea  (provides "A4988.h")
 *   - "ESP32Servo"
 *   - "ArduinoJson" 6.x by Benoit Blanchon
 *   - WiFi, WebServer (built into ESP32 Arduino core)
 *
 * Both stepper motors are commonly wired to the SAME A4988 outputs,
 * so a single driver object controls both simultaneously.
 *
 * REST API (backward-compatible with Westo Flutter app):
 *   GET  /status       → waste level, compressor state, connectivity
 *   POST /compress     → trigger compression cycle
 *   GET  /update?level → manual waste level override (dashboard)
 *   GET  /device/info  → firmware version, MAC, IP
 *   GET  /             → web dashboard
 *
 * Changelog:
 *   v1.1.0 — Irfan: WiFi AP, dashboard, LED blink on compress trigger
 *   v2.0.0 — Jarvis: Full hardware control, state machine, sensors, servo
 *   v2.1.0 — Jarvis: Stall debounce, proximity debounce, timeout fix
 *   v2.2.0 — Jarvis: 1/16 microstepping (shared MS pins)
 *   v3.0.0 — Jarvis: Library rewrite (StepperDriver + ESP32Servo),
 *            new pinout, single common stepper, cleaner code
 * ===================================================================== */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "A4988.h"
#include <ESP32Servo.h>

/* =========================== PIN CONFIG ==============================
 * Pinout from WasteBin_Components_test.ino
 * Both stepper motors share the same STEP/DIR/SLEEP/MS lines.
 * ===================================================================== */

// Stepper Motor (A4988 driver — both motors commonly wired)
#define STEP_PIN     26
#define DIR_PIN      25
#define SLEEP_PIN    27     // LOW = sleep, HIGH = active
#define MS1_PIN      13     // Microstepping select
#define MS2_PIN      12
#define MS3_PIN      14

// Top Ultrasonic Sensor (plate position & waste level)
#define TOP_TRIG     32
#define TOP_ECHO     33

// Front Ultrasonic Sensor (person detection)
#define FRONT_TRIG   18
#define FRONT_ECHO   19

// Lid Servo Motor
#define SERVO_PIN    4

// Built-in LED (status indicator)
#define LED_PIN      2

/* ========================= MOTOR SPECS ===============================
 * MOTOR_STEPS: Physical steps per revolution (200 for 1.8° stepper)
 * RPM:         Motor speed — 150 tested OK in component test
 * MICROSTEPS:  1=full (max torque), 2=half, 4=quarter, 16=smoothest
 *              The A4988 library configures MS pins automatically.
 * ===================================================================== */
#define MOTOR_STEPS  200
#define RPM          150
#define MICROSTEPS   1

/* ========================= CALIBRATION ===============================
 * >>> MEASURE ON YOUR ACTUAL BIN AND UPDATE! <<<
 *
 * PLATE_TOP_CM:    Distance (cm) from top sensor to plate at TOP (resting)
 * PLATE_BOTTOM_CM: Distance (cm) from top sensor to plate at BOTTOM (empty)
 *
 * To calibrate:
 *   1. Move plate to TOP manually → read distance from serial → PLATE_TOP_CM
 *   2. Move plate to BOTTOM       → read distance              → PLATE_BOTTOM_CM
 * ===================================================================== */
#define PLATE_TOP_CM      10.0f
#define PLATE_BOTTOM_CM   50.0f

/* ======================== TUNING CONSTANTS =========================== */

// --- Proximity / Lid ---
#define PROXIMITY_CM         20.0f    // Open lid when person within this distance
#define PROXIMITY_CHECK_MS   200UL    // Front sensor poll interval (ms)
#define PROX_DEBOUNCE        2        // Consecutive reads needed to open lid
#define LID_OPEN_ANGLE       80       // Servo angle: open (degrees)
#define LID_CLOSED_ANGLE     0        // Servo angle: closed
#define LID_HOLD_OPEN_MS     3000UL   // Keep lid open after person leaves (ms)

// --- Compression ---
#define COMPRESS_EVERY_MS    600000UL // Auto-compress interval (10 min)
#define COMPRESS_TIMEOUT_MS  120000UL // Safety: abort after 2 min
#define STALL_CHECK_MS       5000UL   // Check for stall every 5s
#define STALL_THRESHOLD_CM   1.0f     // Plate moved < this = stalled
#define STALL_CONFIRM        2        // Consecutive stalls needed to confirm

// --- Stepper direction ---
// Negative degrees = plate moves DOWN (compression)
// Positive degrees = plate moves UP   (return)
// >>> Swap signs if plate moves the wrong way! <<<
#define ROTATE_DOWN  -360000L
#define ROTATE_UP     360000L

// --- Sensor ---
#define SENSOR_READ_MS   200UL       // Top sensor poll interval during compression
#define US_TIMEOUT_US    30000UL     // Ultrasonic echo timeout (~5m max range)

// --- LED ---
#define BLINK_MS         300UL       // LED blink interval during compression

/* ========================== WIFI CONFIG ============================== */
const char* sta_ssid     = "Motridox";
const char* sta_password = "Bassim@8371";
const char* ap_ssid      = "Westo_ESP32";
const char* ap_password  = "12345678";

/* ========================== OBJECTS ================================== */
A4988 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);
Servo lidServo;
WebServer server(80);

/* ===================== COMPRESSOR STATE MACHINE ====================== */
enum CompState {
  COMP_IDLE,        // Plate at top, waiting
  COMP_DOWN,        // Moving plate down (compressing waste)
  COMP_MEASURE,     // Stalled — measuring waste level
  COMP_UP           // Returning plate to top
};

CompState     compState         = COMP_IDLE;
unsigned long compStartMs       = 0;
unsigned long lastStallCheckMs  = 0;
float         lastStallDist     = 0;
int           stallCount        = 0;
unsigned long lastCompressMs    = 0;
unsigned long lastSensorReadMs  = 0;
float         topDist           = 0;

/* ========================= LID STATE ================================= */
bool          lidOpen           = false;
unsigned long lidLastSeenMs     = 0;
unsigned long lastProxCheckMs   = 0;
int           proxCount         = 0;

/* ========================= SYSTEM DATA =============================== */
int           wasteLevel            = 0;
bool          lastUpdatedFromMobile = false;
unsigned long lastUpdateTime        = 0;
float         frontDist             = 999.0f;

/* ========================= LED STATE ================================= */
bool          ledState    = false;
unsigned long lastBlinkMs = 0;

/* =====================================================================
 * ULTRASONIC: Read distance in cm (returns -1 on timeout/error)
 * ===================================================================== */
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long dur = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;
}

/* =====================================================================
 * STEPPER: Sleep/Wake (library handles stepping, we manage SLEEP pin)
 *
 * A4988 wiring reminder:
 *   RESET pin → jumper wire to SLEEP pin on each A4988 board!
 *   (mandatory — without it RESET floats and driver behaves randomly)
 * ===================================================================== */
void enableStepper() {
  digitalWrite(SLEEP_PIN, HIGH);
  delay(2);  // A4988 needs ~1ms to wake from sleep
}

void disableStepper() {
  digitalWrite(SLEEP_PIN, LOW);
}

/* =====================================================================
 * COMPRESSION: Start a new cycle
 * ===================================================================== */
void startCompression() {
  if (compState != COMP_IDLE) {
    Serial.println("[COMP] Already running — ignored");
    return;
  }
  if (lidOpen) {
    Serial.println("[COMP] Lid is open — delaying compression");
    return;
  }

  Serial.println("[COMP] === Starting compression cycle ===");
  compState   = COMP_DOWN;
  compStartMs = millis();
  stallCount  = 0;

  enableStepper();

  // Seed stall detection with current distance
  float d = readUltrasonic(TOP_TRIG, TOP_ECHO);
  lastStallDist    = (d > 0) ? d : topDist;
  lastStallCheckMs = millis();

  // Start continuous rotation downward
  // (library manages pulse timing; we call nextAction() in the loop)
  stepper.startRotate(ROTATE_DOWN);
}

/* =====================================================================
 * COMPRESSION: End cycle (shared cleanup)
 * ===================================================================== */
void endCompression(const char* reason) {
  Serial.print("[COMP] === Cycle ended: ");
  Serial.print(reason);
  Serial.println(" ===");
  stepper.stop();
  disableStepper();
  compState      = COMP_IDLE;
  stallCount     = 0;
  lastCompressMs = millis();
}

/* =====================================================================
 * COMPRESSION: Non-blocking state machine (called every loop)
 *
 * Uses stepper.startRotate() + stepper.nextAction() for non-blocking
 * step generation. The library handles all pulse timing internally.
 * ===================================================================== */
void handleCompression() {
  if (compState == COMP_IDLE) return;

  unsigned long nowMs = millis();

  // --- Safety timeout ---
  if ((nowMs - compStartMs) >= COMPRESS_TIMEOUT_MS) {
    Serial.println("[COMP] ⚠ SAFETY TIMEOUT — aborting!");
    endCompression("safety timeout");
    return;
  }

  // --- Generate step pulses (non-blocking) ---
  if (compState == COMP_DOWN || compState == COMP_UP) {
    unsigned wait = stepper.nextAction();
    // If the large rotation somehow finishes, restart it
    if (wait == 0) {
      stepper.startRotate(compState == COMP_DOWN ? ROTATE_DOWN : ROTATE_UP);
    }
  }

  // --- Periodically read top sensor ---
  if ((nowMs - lastSensorReadMs) >= SENSOR_READ_MS) {
    lastSensorReadMs = nowMs;
    float d = readUltrasonic(TOP_TRIG, TOP_ECHO);
    if (d > 0) topDist = d;
  }

  // --- State logic ---
  switch (compState) {

    case COMP_DOWN: {
      // Bottom limit reached
      if (topDist >= PLATE_BOTTOM_CM) {
        Serial.println("[COMP] Plate at bottom limit");
        stepper.stop();
        compState  = COMP_MEASURE;
        stallCount = 0;
        break;
      }

      // Stall detection every STALL_CHECK_MS
      if ((nowMs - lastStallCheckMs) >= STALL_CHECK_MS) {
        float moved = topDist - lastStallDist;  // Positive = plate moved further down
        Serial.print("[COMP] Stall check: moved ");
        Serial.print(moved, 1);
        Serial.print("cm  dist=");
        Serial.print(topDist, 1);
        Serial.print("cm  count=");
        Serial.println(stallCount);

        if (moved < STALL_THRESHOLD_CM) {
          stallCount++;
          if (stallCount >= STALL_CONFIRM) {
            Serial.println("[COMP] Stall CONFIRMED — compression done");
            stepper.stop();
            compState = COMP_MEASURE;
          }
        } else {
          stallCount    = 0;
          lastStallDist = topDist;
        }
        lastStallCheckMs = nowMs;
      }
      break;
    }

    case COMP_MEASURE: {
      // Fresh reading for accuracy
      float d = readUltrasonic(TOP_TRIG, TOP_ECHO);
      if (d > 0) topDist = d;

      // Calculate waste level:
      //   Plate close to top (small topDist) → bin is FULL
      //   Plate far from top (large topDist) → bin is EMPTY
      float pct = 100.0f * (1.0f - (topDist - PLATE_TOP_CM)
                                    / (PLATE_BOTTOM_CM - PLATE_TOP_CM));
      wasteLevel = constrain((int)pct, 0, 100);

      Serial.print("[COMP] Waste level = ");
      Serial.print(wasteLevel);
      Serial.print("% (topDist=");
      Serial.print(topDist, 1);
      Serial.println("cm)");

      // Reverse — return plate to top
      Serial.println("[COMP] Returning plate to top...");
      stepper.startRotate(ROTATE_UP);
      compState = COMP_UP;
      break;
    }

    case COMP_UP: {
      // Stop when plate reaches top resting position
      if (topDist <= PLATE_TOP_CM) {
        endCompression("plate at top");
      }
      break;
    }

    default: break;
  }
}

/* =====================================================================
 * LID: Proximity-based open/close with debounce (called every loop)
 * ===================================================================== */
void handleLid() {
  unsigned long nowMs = millis();
  if ((nowMs - lastProxCheckMs) < PROXIMITY_CHECK_MS) return;
  lastProxCheckMs = nowMs;

  // Force lid closed during compression
  if (compState != COMP_IDLE) {
    if (lidOpen) {
      lidServo.write(LID_CLOSED_ANGLE);
      lidOpen = false;
    }
    proxCount = 0;
    return;
  }

  float d = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
  frontDist = (d > 0) ? d : 999.0f;

  if (frontDist <= PROXIMITY_CM) {
    // Person in range
    proxCount++;
    if (!lidOpen && proxCount >= PROX_DEBOUNCE) {
      Serial.print("[LID] Person confirmed at ");
      Serial.print(frontDist, 1);
      Serial.println("cm — opening");
      lidServo.write(LID_OPEN_ANGLE);
      lidOpen = true;
    }
    if (lidOpen) {
      lidLastSeenMs = nowMs;  // Reset hold timer
    }
  } else {
    // No person
    proxCount = 0;
    if (lidOpen && (nowMs - lidLastSeenMs) >= LID_HOLD_OPEN_MS) {
      Serial.println("[LID] No person — closing");
      lidServo.write(LID_CLOSED_ANGLE);
      lidOpen = false;
    }
  }
}

/* =====================================================================
 * AUTO-COMPRESS: Timer-based trigger (called every loop)
 * ===================================================================== */
void handleAutoCompress() {
  if (compState != COMP_IDLE) return;
  if (lidOpen) return;
  if ((millis() - lastCompressMs) >= COMPRESS_EVERY_MS) {
    Serial.println("[AUTO] Timer fired — starting compression");
    startCompression();
  }
}

/* =====================================================================
 * LED: Blink during compression, off when idle
 * ===================================================================== */
void handleLED() {
  if (compState != COMP_IDLE) {
    unsigned long nowMs = millis();
    if ((nowMs - lastBlinkMs) >= BLINK_MS) {
      lastBlinkMs = nowMs;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  } else if (ledState) {
    ledState = false;
    digitalWrite(LED_PIN, LOW);
  }
}

/* =====================================================================
 * WIFI HELPER
 * ===================================================================== */
bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

/* =====================================================================
 * HTML DASHBOARD
 * ===================================================================== */
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Westo Smart Waste Dashboard</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--p:#2563eb;--bg:#f4f6f9;--card:#fff;--ok:#22c55e;--err:#ef4444;--warn:#f59e0b;--m:#6b7280;}
*{box-sizing:border-box;}
body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);}
header{background:var(--p);color:#fff;padding:18px;font-size:20px;font-weight:600;}
.g{padding:16px;display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px;}
.c{background:var(--card);border-radius:14px;padding:16px;box-shadow:0 10px 25px rgba(0,0,0,.08);}
.c h3{margin:0 0 12px;font-size:16px;}
.r{display:flex;justify-content:space-between;margin-bottom:8px;color:var(--m);font-size:14px;}
.b{padding:4px 10px;border-radius:12px;font-size:13px;color:#fff;}
.bon{background:var(--ok);}.boff{background:var(--err);}.bwarn{background:var(--warn);}
.ip{font-family:monospace;font-size:13px;background:#f1f5f9;border-radius:8px;padding:6px 10px;color:#1e40af;font-weight:600;}
.bar{height:14px;background:#e5e7eb;border-radius:10px;overflow:hidden;margin-bottom:8px;}
.bar-in{height:100%;background:linear-gradient(90deg,#22c55e,#16a34a);transition:.4s;}
input[type=range]{width:100%;margin:8px 0;}
button{width:100%;padding:12px;background:var(--p);border:none;color:#fff;border-radius:10px;font-size:15px;margin-top:8px;cursor:pointer;}
button:disabled{opacity:.5;cursor:not-allowed;}
.note{margin-top:8px;padding:8px;border-radius:8px;background:#ecfeff;color:#0369a1;display:none;font-size:13px;}
</style>
</head>
<body>
<header>Westo &bull; Smart Waste Dashboard v3</header>
<div class="g">

  <div class="c">
    <h3>&#128225; Network</h3>
    <div class="r"><span>Wi-Fi</span><span id="wifi" class="b boff">--</span></div>
    <div class="r"><span>Clients</span><span id="cli">0</span></div>
    <div class="r"><span>AP IP</span></div><div class="ip" id="apIp">---</div>
    <div class="r" style="margin-top:8px"><span>STA IP</span></div><div class="ip" id="staIp">---</div>
  </div>

  <div class="c">
    <h3>&#128465; Waste Level</h3>
    <div class="r"><span>Level</span><span id="lvl">--%</span></div>
    <div class="bar"><div class="bar-in" id="bar" style="width:0%"></div></div>
    <div class="r"><span>Top Sensor</span><span id="topD">-- cm</span></div>
    <input type="range" min="0" max="100" id="sl">
    <button onclick="doUpdate()">Update (manual)</button>
    <div class="note" id="note">Updated from mobile</div>
  </div>

  <div class="c">
    <h3>&#9881; Compressor</h3>
    <div class="r"><span>State</span><span id="cst" class="b boff">Idle</span></div>
    <div class="r"><span>Front Sensor</span><span id="frD">-- cm</span></div>
    <div class="r"><span>Lid</span><span id="lid">Closed</span></div>
    <button id="cbtn" onclick="doCompress()">Compress Now</button>
  </div>

</div>
<script>
var STM={"idle":"Idle","down":"Compressing ↓","measure":"Measuring","up":"Returning ↑"};
var SCL={"idle":"boff","down":"bwarn","measure":"bon","up":"bwarn"};
function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('cli').innerText=d.connectedClients;
    document.getElementById('lvl').innerText=d.wasteLevel+"%";
    document.getElementById('bar').style.width=d.wasteLevel+"%";
    document.getElementById('sl').value=d.wasteLevel;
    document.getElementById('apIp').innerText=d.apIp;
    document.getElementById('staIp').innerText=d.staIp||"N/A";
    var w=document.getElementById('wifi');
    if(d.isConnected){w.innerText="Connected";w.className="b bon";}
    else{w.innerText="Disconnected";w.className="b boff";}
    if(d.mobileUpdate){var n=document.getElementById('note');n.style.display="block";setTimeout(()=>{n.style.display="none";},3000);}
    var cs=document.getElementById('cst');
    var sk=d.compressorState||"idle";
    cs.innerText=STM[sk]||sk;
    cs.className="b "+(SCL[sk]||"boff");
    document.getElementById('cbtn').disabled=(sk!=="idle");
    document.getElementById('topD').innerText=(d.topDistance>=0?d.topDistance.toFixed(1):"--")+" cm";
    document.getElementById('frD').innerText=(d.frontDistance>=0&&d.frontDistance<900?d.frontDistance.toFixed(1):"--")+" cm";
    document.getElementById('lid').innerText=d.lidOpen?"Open":"Closed";
  }).catch(()=>{});
}
function doUpdate(){fetch('/update?level='+document.getElementById('sl').value);}
function doCompress(){fetch('/compress',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({trigger:1})});}
setInterval(poll,2000);
poll();
</script>
</body>
</html>
)rawliteral";

/* =====================================================================
 * API: GET /status
 * ===================================================================== */
void handleStatus() {
  StaticJsonDocument<512> doc;
  doc["wasteLevel"]       = wasteLevel;
  doc["isConnected"]      = isWiFiConnected();
  doc["connectedClients"] = WiFi.softAPgetStationNum();
  doc["mobileUpdate"]     = lastUpdatedFromMobile;
  doc["apIp"]             = WiFi.softAPIP().toString();
  doc["staIp"]            = isWiFiConnected() ? WiFi.localIP().toString() : "";

  bool isActive = (compState != COMP_IDLE);
  doc["triggerActive"]      = isActive;
  doc["isCompressorActive"] = isActive;

  const char* stateStr = "idle";
  switch (compState) {
    case COMP_DOWN:    stateStr = "down";    break;
    case COMP_MEASURE: stateStr = "measure"; break;
    case COMP_UP:      stateStr = "up";      break;
    default:           stateStr = "idle";    break;
  }
  doc["compressorState"] = stateStr;

  doc["topDistance"]   = topDist;
  doc["frontDistance"] = frontDist;
  doc["lidOpen"]       = lidOpen;

  lastUpdatedFromMobile = false;

  String res;
  serializeJson(doc, res);
  server.send(200, "application/json", res);
}

/* =====================================================================
 * API: GET /update?level=XX
 * ===================================================================== */
void handleUpdate() {
  if (server.hasArg("level")) {
    int newLevel = constrain(server.arg("level").toInt(), 0, 100);
    Serial.print("[UPDATE] Waste level: ");
    Serial.print(wasteLevel);
    Serial.print("% → ");
    Serial.print(newLevel);
    Serial.println("%");
    wasteLevel            = newLevel;
    lastUpdatedFromMobile = true;
    lastUpdateTime        = millis();
  }
  server.send(200, "text/plain", "OK");
}

/* =====================================================================
 * API: POST /compress
 * ===================================================================== */
void handleCompress() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  StaticJsonDocument<64> req;
  DeserializationError err = deserializeJson(req, server.arg("plain"));

  int triggerValue = 1;
  if (!err && req.containsKey("trigger")) {
    triggerValue = req["trigger"].as<int>();
  }

  StaticJsonDocument<128> res;

  if (triggerValue == 1) {
    if (compState == COMP_IDLE) {
      if (lidOpen) {
        res["status"]  = "lid_open";
        res["message"] = "Cannot compress while lid is open. Wait for lid to close.";
      } else {
        startCompression();
        res["status"]  = "activated";
        res["message"] = "Compression cycle started.";
      }
    } else {
      res["status"]  = "already_active";
      res["message"] = "Compressor already running.";
    }
  } else {
    res["status"]  = "acknowledged";
    res["message"] = "Noted (compression runs its own cycle).";
  }

  String out;
  serializeJson(res, out);
  server.send(200, "application/json", out);
}

/* =====================================================================
 * API: GET /device/info
 * ===================================================================== */
void handleDeviceInfo() {
  StaticJsonDocument<256> doc;
  doc["deviceName"]      = "WESTO Smart Bin";
  doc["firmwareVersion"] = "v3.0.0";
  doc["macAddress"]      = WiFi.softAPmacAddress();
  doc["ipAddress"]       = WiFi.softAPIP().toString();
  doc["mode"]            = "AP + STA";
  String res;
  serializeJson(doc, res);
  server.send(200, "application/json", res);
}

/* =====================================================================
 * API: GET / (dashboard)
 * ===================================================================== */
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

/* =====================================================================
 * SERIAL: Status banner
 * ===================================================================== */
void printBanner() {
  Serial.println();
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║    WESTO SMART BIN v3.0 — LIBRARY ED.     ║");
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.print("║  AP:  http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("/");
  if (isWiFiConnected()) {
    Serial.print("║  STA: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
  } else {
    Serial.println("║  STA: Not connected");
  }
  Serial.println("╠════════════════════════════════════════════╣");
  Serial.print("║  Waste: "); Serial.print(wasteLevel); Serial.println("%");
  Serial.print("║  Top sensor: "); Serial.print(topDist, 1); Serial.println(" cm");
  Serial.print("║  Compressor: ");
  switch (compState) {
    case COMP_IDLE:    Serial.println("Idle");          break;
    case COMP_DOWN:    Serial.println("Compressing ↓"); break;
    case COMP_MEASURE: Serial.println("Measuring");     break;
    case COMP_UP:      Serial.println("Returning ↑");   break;
  }
  Serial.println("╚════════════════════════════════════════════╝");
}

/* =====================================================================
 * SETUP
 * ===================================================================== */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n========== WESTO ESP32 v3.0 BOOTING ==========");

  // --- LED ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- Ultrasonic pins ---
  pinMode(TOP_TRIG, OUTPUT);
  pinMode(TOP_ECHO, INPUT);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  // --- Stepper SLEEP pin (set LOW first to prevent boot-pulse movement) ---
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);

  // --- Initialize stepper library ---
  // The library configures STEP, DIR, and MS pins internally
  stepper.begin(RPM, MICROSTEPS);
  stepper.setSpeedProfile(BasicStepperDriver::CONSTANT_SPEED);
  Serial.print("[STEPPER] Ready — ");
  Serial.print(RPM);
  Serial.print(" RPM, ");
  Serial.print(MICROSTEPS);
  Serial.println("x microstepping, sleeping");

  // --- Servo (ESP32Servo handles LEDC channel allocation) ---
  lidServo.attach(SERVO_PIN);
  lidServo.write(LID_CLOSED_ANGLE);
  Serial.println("[SERVO] Lid closed (init)");

  // --- Initial sensor reading ---
  delay(100);
  float d = readUltrasonic(TOP_TRIG, TOP_ECHO);
  topDist = (d > 0) ? d : PLATE_TOP_CM;
  Serial.print("[SENSOR] Initial plate distance: ");
  Serial.print(topDist, 1);
  Serial.println(" cm");

  // --- WiFi AP+STA ---
  Serial.println("[WiFi] Starting AP+STA...");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("[WiFi] AP: ");
  Serial.print(ap_ssid);
  Serial.print(" → ");
  Serial.println(WiFi.softAPIP());

  Serial.print("[WiFi] Connecting STA: ");
  Serial.println(sta_ssid);
  WiFi.begin(sta_ssid, sta_password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (isWiFiConnected()) {
    Serial.print("[WiFi] STA connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] STA failed — AP-only mode");
  }

  // --- HTTP routes ---
  server.on("/",            handleRoot);
  server.on("/status",      handleStatus);
  server.on("/update",      handleUpdate);
  server.on("/compress",    handleCompress);
  server.on("/device/info", handleDeviceInfo);
  server.begin();
  Serial.println("[Server] HTTP ready on port 80");

  // --- Timers ---
  lastCompressMs = millis();  // Don't auto-compress immediately on boot

  printBanner();
  Serial.println("========== READY ==========\n");
}

/* =====================================================================
 * MAIN LOOP (all non-blocking)
 * ===================================================================== */
unsigned long lastBannerMs = 0;

void loop() {
  server.handleClient();     // HTTP requests
  handleLid();               // Proximity → lid servo
  handleCompression();       // Compressor state machine
  handleAutoCompress();      // Timer-based auto compression
  handleLED();               // Status LED blink

  // Status banner every 30s
  if ((millis() - lastBannerMs) > 30000UL) {
    lastBannerMs = millis();
    printBanner();
  }
}
