#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/* =====================================================================
 * WESTO Smart Waste Bin — ESP32 Firmware v2.1.0
 * =====================================================================
 * Hardware:
 *   - ESP32 DevKit
 *   - 2x HC-SR04 Ultrasonic Sensors (front: person, top: plate/waste)
 *   - 2x Stepper Motors + A4988 Drivers (compression plate)
 *   - 1x Servo Motor (lid)
 *
 * Libraries needed (install via Arduino Library Manager):
 *   - ArduinoJson 6.x (by Benoit Blanchon)
 *   - WiFi, WebServer (built into ESP32 Arduino core)
 *
 * Compatible with: ESP32 Arduino Core 2.x
 *   If using Core 3.x, replace ledcSetup()+ledcAttachPin() with
 *   ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_BITS) in setup().
 *
 * REST API (backward-compatible with Westo Flutter app):
 *   GET  /status       → waste level, compressor state, connectivity
 *   POST /compress     → trigger compression cycle
 *   GET  /update?level → manual waste level override (dashboard)
 *   GET  /device/info  → firmware version, MAC, IP
 *   GET  /             → web dashboard
 *
 * Changelog:
 *   v1.1.0 — Original (Irfan): WiFi AP, dashboard, LED blink on compress
 *   v2.0.0 — Jarvis: Full hardware control, state machine, sensors, servo
 *   v2.1.0 — Jarvis: Stall debounce, proximity debounce, timeout fix
 * ===================================================================== */

/* =========================== PIN CONFIG ==============================
 * Adjust these to match YOUR wiring. All pins must be valid ESP32 GPIOs.
 * Avoid: GPIO 0,1,3 (boot/serial), GPIO 6-11 (flash), GPIO 34-39 (input-only)
 * ===================================================================== */

// Front Ultrasonic Sensor (person detection)
#define FRONT_TRIG_PIN   23
#define FRONT_ECHO_PIN   22

// Top Ultrasonic Sensor (plate position & waste level)
#define TOP_TRIG_PIN     19
#define TOP_ECHO_PIN     18

// Lid Servo Motor
#define SERVO_PIN        4

// Stepper Motor 1 — A4988 Driver
//   STEP → GPIO 25    DIR → GPIO 26    SLEEP → GPIO 27
//   RESET → jumper to SLEEP (physical wire on board!)
//   MS1/MS2/MS3 → leave unconnected (internal pull-down = full-step)
#define STEPPER1_STEP    25
#define STEPPER1_DIR     26
#define STEPPER1_SLP     27

// Stepper Motor 2 — A4988 Driver
//   STEP → GPIO 32    DIR → GPIO 33    SLEEP → GPIO 14
//   RESET → jumper to SLEEP (physical wire on board!)
//   MS1/MS2/MS3 → leave unconnected (internal pull-down = full-step)
#define STEPPER2_STEP    32
#define STEPPER2_DIR     33
#define STEPPER2_SLP     14    // NOTE: GPIO14 may pulse HIGH briefly at boot

// Built-in LED (status indicator)
#define LED_PIN          2

/* ========================= CALIBRATION ===============================
 * >>> MEASURE THESE ON YOUR ACTUAL BIN AND UPDATE! <<<
 *
 * PLATE_TOP_CM:    Distance (cm) from top ultrasonic sensor to the
 *                  compressor plate when plate is at its TOP resting
 *                  position. (Plate fully retracted upward.)
 *
 * PLATE_BOTTOM_CM: Distance (cm) from top ultrasonic sensor to the
 *                  compressor plate when plate is at the very BOTTOM
 *                  of the bin. (Empty bin, plate fully lowered.)
 *
 * To calibrate:
 *   1. Move plate to TOP manually → read distance from serial → set PLATE_TOP_CM
 *   2. Move plate to BOTTOM manually → read distance → set PLATE_BOTTOM_CM
 * ===================================================================== */
#define PLATE_TOP_CM      10.0f    // ~10cm when plate is at top (resting)
#define PLATE_BOTTOM_CM   50.0f    // ~50cm when plate is at bottom (empty bin)

/* ======================== TUNING CONSTANTS ========================== */

// --- Proximity / Lid ---
#define PROXIMITY_CM         20.0f    // Open lid when person is within this distance
#define PROXIMITY_CHECK_MS   200UL    // How often to check front sensor (ms)
#define PROX_DEBOUNCE        2        // Require N consecutive reads to open lid
#define LID_OPEN_ANGLE       80       // Servo angle for open lid (degrees, 75-85 range)
#define LID_CLOSED_ANGLE     0        // Servo angle for closed lid
#define LID_HOLD_OPEN_MS     3000UL   // Keep lid open this long after person leaves

// --- Compression ---
#define COMPRESS_EVERY_MS    600000UL // Auto-compress interval (600000 = 10 min)
#define COMPRESS_TIMEOUT_MS  120000UL // Safety: abort if cycle exceeds 2 minutes
#define STALL_CHECK_MS       5000UL   // Check for stall every 5 seconds
#define STALL_THRESHOLD_CM   1.0f     // Plate moved < this in stall period → stalled
#define STALL_CONFIRM        2        // Require N consecutive stall checks to confirm
#define STEP_INTERVAL_US     3000UL   // Microseconds between step pulses (~100 RPM at 200 spr)

// --- Stepper direction (swap HIGH/LOW if plate moves the wrong way) ---
#define DIR_DOWN             HIGH     // A4988 DIR pin state to move plate DOWN
#define DIR_UP               LOW      // A4988 DIR pin state to move plate UP

// --- If stepper 2 is physically mirrored, set true to invert its direction ---
#define STEPPER2_INVERT      false

// --- Sensor ---
#define SENSOR_READ_MS       200UL    // Read top sensor this often during compression
#define US_TIMEOUT_US        30000UL  // Ultrasonic echo timeout (~5m max range)

// --- Servo PWM (ESP32 LEDC) ---
#define SERVO_CHANNEL        0
#define SERVO_FREQ           50       // 50 Hz standard servo
#define SERVO_BITS           16       // 16-bit resolution

// --- LED ---
#define BLINK_MS             300UL    // LED blink interval during compression

/* ========================== WIFI CONFIG ============================== */
const char* sta_ssid     = "Motridox";
const char* sta_password = "Bassim@8371";
const char* ap_ssid      = "Westo_ESP32";
const char* ap_password  = "12345678";

/* ========================== WEB SERVER ============================== */
WebServer server(80);

/* ===================== COMPRESSOR STATE MACHINE ===================== */
enum CompState {
  COMP_IDLE,        // Plate at top, waiting
  COMP_DOWN,        // Moving plate down (compressing waste)
  COMP_MEASURE,     // Stalled — measuring waste level
  COMP_UP           // Returning plate to top
};

CompState     compState         = COMP_IDLE;
unsigned long compStartMs       = 0;      // When current cycle started
unsigned long lastStepUs        = 0;      // Last stepper pulse (micros)
unsigned long lastStallCheckMs  = 0;      // Last stall-check timestamp
float         lastStallDist     = 0;      // Distance at last stall check
int           stallCount        = 0;      // Consecutive stall detections
unsigned long lastCompressMs    = 0;      // When last cycle completed (for auto timer)
unsigned long lastSensorReadMs  = 0;      // Top sensor read timer
float         topDist           = 0;      // Latest top sensor reading (cm)

/* ========================= LID STATE ================================ */
bool          lidOpen           = false;
unsigned long lidLastSeenMs     = 0;      // Last time person was detected
unsigned long lastProxCheckMs   = 0;
int           proxCount         = 0;      // Consecutive proximity detections

/* ========================= SYSTEM DATA ============================== */
int           wasteLevel            = 0;        // 0-100 %
bool          lastUpdatedFromMobile = false;
unsigned long lastUpdateTime        = 0;
float         frontDist             = 999.0f;   // Latest front sensor (cm)

/* ========================= LED STATE ================================ */
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
 * SERVO: Write angle using ESP32 LEDC (no external library needed)
 * ===================================================================== */
void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  // Map 0°-180° to 500us-2500us pulse width
  uint32_t pulseUs = map(angle, 0, 180, 500, 2500);
  // Convert to 16-bit duty: duty = pulseUs / 20000us * 65536
  uint32_t duty = (pulseUs * 65536UL) / 20000UL;
  ledcWrite(SERVO_CHANNEL, duty);
}

/* =====================================================================
 * STEPPERS: Low-level A4988 control (no library needed)
 * A4988 wiring (per driver):
 *   STEP → ESP32 GPIO (pulse = one step)
 *   DIR  → ESP32 GPIO (direction control)
 *   SLEEP → ESP32 GPIO (LOW = sleep, HIGH = active)
 *   RESET → jumper wire to SLEEP pin on the SAME board!
 *           (RESET is active-LOW with no pull-up — floating = random resets)
 *   MS1/MS2/MS3 → leave unconnected (internal pull-down = full-step mode)
 *   VMOT → motor supply (8-35V)    VDD → 3.3V from ESP32
 *   GND  → common ground (ESP32 + motor supply)
 *   1A/1B/2A/2B → stepper coil wires
 * ===================================================================== */
void enableSteppers() {
  digitalWrite(STEPPER1_SLP, HIGH);
  digitalWrite(STEPPER2_SLP, HIGH);
  delay(2);  // A4988 needs ~1ms to wake from sleep
}

void disableSteppers() {
  digitalWrite(STEPPER1_SLP, LOW);
  digitalWrite(STEPPER2_SLP, LOW);
}

void setStepDir(bool goingDown) {
  uint8_t d1 = goingDown ? DIR_DOWN : DIR_UP;
  uint8_t d2 = STEPPER2_INVERT ? !d1 : d1;
  digitalWrite(STEPPER1_DIR, d1);
  digitalWrite(STEPPER2_DIR, d2);
}

void pulseSteppers() {
  digitalWrite(STEPPER1_STEP, HIGH);
  digitalWrite(STEPPER2_STEP, HIGH);
  delayMicroseconds(50);  // A4988 min pulse width = 1us; 50us is safe
  digitalWrite(STEPPER1_STEP, LOW);
  digitalWrite(STEPPER2_STEP, LOW);
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

  enableSteppers();
  setStepDir(true);  // DOWN

  // Seed stall detection with current distance
  float d = readUltrasonic(TOP_TRIG_PIN, TOP_ECHO_PIN);
  lastStallDist    = (d > 0) ? d : topDist;
  lastStallCheckMs = millis();
  lastStepUs       = micros();
}

/* =====================================================================
 * COMPRESSION: End cycle (shared cleanup for normal + timeout)
 * ===================================================================== */
void endCompression(const char* reason) {
  Serial.print("[COMP] === Cycle ended: ");
  Serial.print(reason);
  Serial.println(" ===");
  disableSteppers();
  compState      = COMP_IDLE;
  stallCount     = 0;
  lastCompressMs = millis();  // Reset auto-compress timer
}

/* =====================================================================
 * COMPRESSION: Non-blocking state machine (called every loop)
 * ===================================================================== */
void handleCompression() {
  if (compState == COMP_IDLE) return;

  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  // --- Safety timeout ---
  if ((nowMs - compStartMs) >= COMPRESS_TIMEOUT_MS) {
    Serial.println("[COMP] ⚠ SAFETY TIMEOUT — aborting!");
    endCompression("safety timeout");
    return;
  }

  // --- Pulse steppers at target speed (non-blocking) ---
  if (compState == COMP_DOWN || compState == COMP_UP) {
    if ((nowUs - lastStepUs) >= STEP_INTERVAL_US) {
      lastStepUs = nowUs;
      pulseSteppers();
    }
  }

  // --- Periodically read top sensor ---
  if ((nowMs - lastSensorReadMs) >= SENSOR_READ_MS) {
    lastSensorReadMs = nowMs;
    float d = readUltrasonic(TOP_TRIG_PIN, TOP_ECHO_PIN);
    if (d > 0) topDist = d;
  }

  // --- State logic ---
  switch (compState) {

    case COMP_DOWN: {
      // Safety: stop if plate reached bottom limit
      if (topDist >= PLATE_BOTTOM_CM) {
        Serial.println("[COMP] Plate at bottom limit");
        compState = COMP_MEASURE;
        stallCount = 0;
        break;
      }

      // Stall detection every STALL_CHECK_MS
      if ((nowMs - lastStallCheckMs) >= STALL_CHECK_MS) {
        float moved = topDist - lastStallDist;  // positive = plate moved further down
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
            compState = COMP_MEASURE;
          }
          // Keep lastStallDist as-is so next check is also against last good position
        } else {
          stallCount       = 0;   // Reset: plate is moving
          lastStallDist    = topDist;
        }
        lastStallCheckMs = nowMs;
      }
      break;
    }

    case COMP_MEASURE: {
      // Take a fresh reading for accuracy
      float d = readUltrasonic(TOP_TRIG_PIN, TOP_ECHO_PIN);
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

      // Reverse direction — return plate to top
      Serial.println("[COMP] Returning plate to top...");
      setStepDir(false);  // UP
      compState  = COMP_UP;
      lastStepUs = micros();
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
      servoWrite(LID_CLOSED_ANGLE);
      lidOpen = false;
    }
    proxCount = 0;
    return;
  }

  float d = readUltrasonic(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  frontDist = (d > 0) ? d : 999.0f;

  if (frontDist <= PROXIMITY_CM) {
    // Person in range
    proxCount++;
    if (!lidOpen && proxCount >= PROX_DEBOUNCE) {
      Serial.print("[LID] Person confirmed at ");
      Serial.print(frontDist, 1);
      Serial.println("cm — opening");
      servoWrite(LID_OPEN_ANGLE);
      lidOpen = true;
    }
    if (lidOpen) {
      lidLastSeenMs = nowMs;  // Reset hold timer while person present
    }
  } else {
    // No person
    proxCount = 0;
    if (lidOpen && (nowMs - lidLastSeenMs) >= LID_HOLD_OPEN_MS) {
      Serial.println("[LID] No person — closing");
      servoWrite(LID_CLOSED_ANGLE);
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
<header>Westo &bull; Smart Waste Dashboard v2</header>
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

  // Compressor status (backward-compatible + new fields)
  bool isActive = (compState != COMP_IDLE);
  doc["triggerActive"]      = isActive;           // Legacy field (v1 dashboard)
  doc["isCompressorActive"] = isActive;           // Flutter app field

  const char* stateStr = "idle";
  switch (compState) {
    case COMP_DOWN:    stateStr = "down";    break;
    case COMP_MEASURE: stateStr = "measure"; break;
    case COMP_UP:      stateStr = "up";      break;
    default:           stateStr = "idle";    break;
  }
  doc["compressorState"] = stateStr;

  // Sensor readings
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
  doc["firmwareVersion"] = "v2.1.0";
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
 * SERIAL BANNER
 * ===================================================================== */
void printBanner() {
  Serial.println();
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║    WESTO SMART BIN v2.1 — DASHBOARD       ║");
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
    case COMP_IDLE:    Serial.println("Idle");         break;
    case COMP_DOWN:    Serial.println("Compressing ↓"); break;
    case COMP_MEASURE: Serial.println("Measuring");    break;
    case COMP_UP:      Serial.println("Returning ↑");  break;
  }
  Serial.println("╚════════════════════════════════════════════╝");
}

/* =====================================================================
 * SETUP
 * ===================================================================== */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n========== WESTO ESP32 v2.1 BOOTING ==========");

  // --- Pin modes ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Ultrasonic pins
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(TOP_TRIG_PIN, OUTPUT);
  pinMode(TOP_ECHO_PIN, INPUT);

  // Stepper pins (set SLEEP LOW first to prevent boot-pulse movement)
  pinMode(STEPPER1_SLP, OUTPUT);
  digitalWrite(STEPPER1_SLP, LOW);
  pinMode(STEPPER2_SLP, OUTPUT);
  digitalWrite(STEPPER2_SLP, LOW);

  pinMode(STEPPER1_STEP, OUTPUT);
  pinMode(STEPPER1_DIR, OUTPUT);
  pinMode(STEPPER2_STEP, OUTPUT);
  pinMode(STEPPER2_DIR, OUTPUT);

  digitalWrite(STEPPER1_STEP, LOW);
  digitalWrite(STEPPER2_STEP, LOW);
  digitalWrite(STEPPER1_DIR, LOW);
  digitalWrite(STEPPER2_DIR, LOW);

  Serial.println("[STEPPER] Drivers in sleep mode (init)");

  // --- Servo (LEDC PWM) ---
  // For ESP32 Arduino Core 3.x, replace these two lines with:
  //   ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_BITS);
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  servoWrite(LID_CLOSED_ANGLE);
  Serial.println("[SERVO] Lid closed (init)");

  // --- Initial sensor reading ---
  delay(100);  // Let sensors settle
  float d = readUltrasonic(TOP_TRIG_PIN, TOP_ECHO_PIN);
  topDist = (d > 0) ? d : PLATE_TOP_CM;
  Serial.print("[SENSOR] Initial plate distance: ");
  Serial.print(topDist, 1);
  Serial.println(" cm");

  // --- WiFi ---
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
  server.handleClient();   // Handle HTTP requests
  handleLid();             // Proximity → lid servo
  handleCompression();     // Compressor state machine
  handleAutoCompress();    // Timer-based auto compression
  handleLED();             // Status LED

  // Print status banner periodically (every 30s)
  if ((millis() - lastBannerMs) > 30000UL) {
    lastBannerMs = millis();
    printBanner();
  }
}
