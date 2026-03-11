#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/* ================= WIFI CONFIG ================= */
const char* sta_ssid     = "STARVISION WIfi";
const char* sta_password = "0plmnko9";

const char* ap_ssid      = "Westo_ESP32";
const char* ap_password  = "12345678";

/* ================= SERVER ====================== */
WebServer server(80);

/* ================= SYSTEM DATA ================= */
int wasteLevel            = 50;
bool lastUpdatedFromMobile = false;
unsigned long lastUpdateTime = 0;

/* ================= COMPRESSOR STATE ============ */
bool isCompressorActive         = false;
unsigned long compressorStartTime = 0;
const unsigned long COMPRESSOR_DURATION_MS = 10000; // 10 seconds

/* ================= HTML DASHBOARD ============== */
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Westo Inventory Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
:root{
  --primary:#2563eb;
  --bg:#f4f6f9;
  --card:#ffffff;
  --success:#22c55e;
  --danger:#ef4444;
  --warning:#f59e0b;
  --muted:#6b7280;
}

body{
  margin:0;
  font-family:system-ui, sans-serif;
  background:var(--bg);
}

header{
  background:var(--primary);
  color:white;
  padding:18px;
  font-size:20px;
  font-weight:600;
}

.container{
  padding:16px;
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(280px,1fr));
  gap:16px;
}

.card{
  background:var(--card);
  border-radius:14px;
  padding:16px;
  box-shadow:0 10px 25px rgba(0,0,0,.08);
}

.card h3{
  margin:0 0 12px;
  font-size:16px;
}

.stat{
  display:flex;
  justify-content:space-between;
  margin-bottom:8px;
  color:var(--muted);
}

.badge{
  padding:4px 10px;
  border-radius:12px;
  font-size:13px;
  color:white;
}

.online   { background:var(--success); }
.offline  { background:var(--danger);  }
.running  { background:var(--warning); animation:pulse 1s infinite; }
.idle     { background:var(--muted);   }

@keyframes pulse{
  0%,100%{ opacity:1; }
  50%    { opacity:.6; }
}

.progress{
  height:14px;
  background:#e5e7eb;
  border-radius:10px;
  overflow:hidden;
}

.progress-bar{
  height:100%;
  background:linear-gradient(90deg,#22c55e,#16a34a);
  width:0%;
  transition:.4s;
}

/* Compressor timer ring */
.timer-wrap{
  display:flex;
  flex-direction:column;
  align-items:center;
  margin:14px 0 6px;
}

.timer-ring{
  position:relative;
  width:90px;
  height:90px;
}

.timer-ring svg{
  transform:rotate(-90deg);
}

.timer-ring circle{
  fill:none;
  stroke-width:8;
}

.ring-bg  { stroke:#e5e7eb; }
.ring-fg  {
  stroke:var(--warning);
  stroke-linecap:round;
  stroke-dasharray:226;
  stroke-dashoffset:226;
  transition:.5s linear;
}

.timer-label{
  position:absolute;
  inset:0;
  display:flex;
  align-items:center;
  justify-content:center;
  font-size:18px;
  font-weight:700;
  color:#374151;
}

input[type=range]{
  width:100%;
  margin-top:12px;
}

button{
  width:100%;
  padding:12px;
  background:var(--primary);
  border:none;
  color:white;
  border-radius:10px;
  font-size:15px;
  margin-top:12px;
  cursor:pointer;
}

button:disabled{
  background:#9ca3af;
  cursor:not-allowed;
}

.btn-compress{
  background:#f59e0b;
}

.btn-compress:hover:not(:disabled){
  background:#d97706;
}

.notice{
  margin-top:10px;
  padding:10px;
  border-radius:10px;
  background:#ecfeff;
  color:#0369a1;
  display:none;
  font-size:14px;
}

.compress-notice{
  margin-top:10px;
  padding:10px;
  border-radius:10px;
  background:#fffbeb;
  color:#92400e;
  font-size:14px;
  display:none;
}
</style>
</head>

<body>

<header>Westo • Smart Waste Dashboard</header>

<div class="container">

  <!-- Network Card -->
  <div class="card">
    <h3>Network Status</h3>
    <div class="stat">
      <span>Wi-Fi</span>
      <span id="wifi" class="badge offline">Checking</span>
    </div>
    <div class="stat">
      <span>Connected Devices</span>
      <span id="clients">0</span>
    </div>
  </div>

  <!-- Waste Level Card -->
  <div class="card">
    <h3>Waste Level</h3>
    <div class="stat">
      <span>Current Level</span>
      <span id="levelText">--%</span>
    </div>

    <div class="progress">
      <div class="progress-bar" id="progress"></div>
    </div>

    <input type="range" min="0" max="100" id="slider">
    <button onclick="updateWaste()">Update Waste Level</button>

    <div class="notice" id="notice">
      🔔 Waste level updated from mobile device
    </div>
  </div>

  <!-- Compressor Card -->
  <div class="card">
    <h3>Compressor Control</h3>

    <div class="stat">
      <span>Status</span>
      <span id="compBadge" class="badge idle">Idle</span>
    </div>

    <!-- Countdown ring -->
    <div class="timer-wrap">
      <div class="timer-ring">
        <svg viewBox="0 0 90 90" width="90" height="90">
          <circle class="ring-bg" cx="45" cy="45" r="36"/>
          <circle class="ring-fg" id="ringFg" cx="45" cy="45" r="36"/>
        </svg>
        <div class="timer-label" id="timerLabel">--</div>
      </div>
    </div>

    <button id="compBtn" class="btn-compress" onclick="triggerCompressor()">
      ⚙️ Start Compressor
    </button>

    <div class="compress-notice" id="compNotice">
      ⚠️ Compressor is running! Auto-stops in <span id="countdownText">10</span>s
    </div>
  </div>

</div>

<script>
const COMPRESS_DURATION = 10; // seconds — must match firmware
let compressorEndTime = null;
let countdownInterval = null;

function loadStatus(){
  fetch('/status')
    .then(r => r.json())
    .then(d => {
      // --- Network ---
      document.getElementById('clients').innerText = d.connectedClients;
      let wifi = document.getElementById('wifi');
      if(d.isConnected){
        wifi.innerText = "Connected";
        wifi.className = "badge online";
      } else {
        wifi.innerText = "Not Connected";
        wifi.className = "badge offline";
      }

      // --- Waste Level ---
      document.getElementById('levelText').innerText = d.wasteLevel + "%";
      document.getElementById('progress').style.width = d.wasteLevel + "%";
      document.getElementById('slider').value = d.wasteLevel;

      if(d.mobileUpdate){
        document.getElementById('notice').style.display = "block";
        setTimeout(() => { document.getElementById('notice').style.display = "none"; }, 3000);
      }

      // --- Compressor ---
      updateCompressorUI(d.isCompressorActive, d.compressorElapsedMs);
    });
}

function updateCompressorUI(active, elapsedMs){
  const badge    = document.getElementById('compBadge');
  const btn      = document.getElementById('compBtn');
  const notice   = document.getElementById('compNotice');
  const ring     = document.getElementById('ringFg');
  const timerLbl = document.getElementById('timerLabel');
  const CIRC     = 226; // 2 * π * 36

  if(active){
    badge.innerText  = "Running";
    badge.className  = "badge running";
    btn.disabled     = true;
    notice.style.display = "block";

    // remaining seconds
    let remainSec = Math.max(0, COMPRESS_DURATION - Math.floor(elapsedMs / 1000));
    timerLbl.innerText = remainSec + "s";
    document.getElementById('countdownText').innerText = remainSec;

    // ring fill: fraction of time elapsed
    let fraction = Math.min(elapsedMs / (COMPRESS_DURATION * 1000), 1);
    ring.style.strokeDashoffset = CIRC - (CIRC * fraction);
  } else {
    badge.innerText  = "Idle";
    badge.className  = "badge idle";
    btn.disabled     = false;
    notice.style.display = "none";
    timerLbl.innerText = "--";
    ring.style.strokeDashoffset = CIRC; // empty ring
  }
}

function triggerCompressor(){
  fetch('/compress', { method:'POST' })
    .then(r => r.text())
    .then(msg => {
      console.log("Compressor response:", msg);
      loadStatus(); // refresh immediately
    });
}

function updateWaste(){
  let val = document.getElementById('slider').value;
  fetch('/update?level=' + val);
}

setInterval(loadStatus, 2000);
loadStatus();
</script>

</body>
</html>
)rawliteral";

/* ================= HELPERS ===================== */
bool isWiFiConnected(){
  return WiFi.status() == WL_CONNECTED;
}

/* ================= API HANDLERS ================= */
void handleRoot(){
  server.send(200, "text/html", htmlPage);
}

void handleStatus(){
  // Auto-stop compressor after COMPRESSOR_DURATION_MS
  unsigned long elapsed = 0;
  if(isCompressorActive){
    elapsed = millis() - compressorStartTime;
    if(elapsed >= COMPRESSOR_DURATION_MS){
      isCompressorActive = false;
      elapsed = 0;
    }
  }

  StaticJsonDocument<256> doc;
  doc["wasteLevel"]          = wasteLevel;
  doc["isConnected"]         = isWiFiConnected();
  doc["isCompressorActive"]  = isCompressorActive;
  doc["compressorElapsedMs"] = isCompressorActive ? elapsed : 0;  // NEW: send elapsed time for ring UI
  doc["connectedClients"]    = WiFi.softAPgetStationNum();
  doc["mobileUpdate"]        = lastUpdatedFromMobile;

  lastUpdatedFromMobile = false; // auto-clear

  String res;
  serializeJson(doc, res);
  server.send(200, "application/json", res);
}

void handleUpdate(){
  if(server.hasArg("level")){
    wasteLevel = constrain(server.arg("level").toInt(), 0, 100);
    lastUpdatedFromMobile = true;
    lastUpdateTime = millis();
  }
  server.send(200, "text/plain", "OK");
}

void handleCompress(){
  if(!isCompressorActive){
    isCompressorActive    = true;
    compressorStartTime   = millis();
    server.send(200, "text/plain", "Compressor started");
  } else {
    server.send(200, "text/plain", "Compressor already running");
  }
}

/* ================= SETUP ======================= */
void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("       Westo ESP32 Booting...");
  Serial.println("========================================");

  WiFi.mode(WIFI_AP_STA);

  // --- Start Access Point ---
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("\n[AP] Hotspot Started!");
  Serial.print("[AP] SSID     : "); Serial.println(ap_ssid);
  Serial.print("[AP] Password : "); Serial.println(ap_password);
  Serial.print("[AP] IP Addr  : "); Serial.println(WiFi.softAPIP());

  // --- Connect to Station WiFi ---
  Serial.println("\n[WiFi] Connecting to: " + String(sta_ssid));
  WiFi.begin(sta_ssid, sta_password);

  int attempts = 0;
  while(WiFi.status() != WL_CONNECTED && attempts < 20){
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\n[WiFi] Connected Successfully!");
    Serial.print("[WiFi] IP Address : "); Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Signal (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  } else {
    Serial.println("\n[WiFi] Connection FAILED — running in AP-only mode.");
  }

  // --- Start Web Server ---
  server.on("/",         handleRoot);
  server.on("/status",   handleStatus);
  server.on("/update",   handleUpdate);
  server.on("/compress", HTTP_POST, handleCompress);

  server.begin();

  Serial.println("\n========================================");
  Serial.println("[Server] Web Server Started!");
  Serial.print("[Dashboard] AP URL  : http://"); Serial.print(WiFi.softAPIP()); Serial.println("/");
  if(WiFi.status() == WL_CONNECTED){
    Serial.print("[Dashboard] LAN URL : http://"); Serial.print(WiFi.localIP()); Serial.println("/");
  }
  Serial.println("========================================\n");
}

/* ================= LOOP ======================== */
void loop(){
  server.handleClient();
}