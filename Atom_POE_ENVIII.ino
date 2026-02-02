/*
 * ATOM POE ENV III Sensor Reader
 *
 * Hardware:
 *   - M5Stack ATOM Lite
 *   - AtomPOE (W5500 Ethernet)
 *   - ENV III Unit (SHT30 + QMP6988)
 *
 * Features:
 *   - Temperature, Humidity, Pressure readings via REST API
 *   - Web interface for monitoring
 *   - Two-Stage OTA (LittleFS-based, avoids W5500 SPI conflicts)
 *
 * Sensor Libraries:
 *   - M5Unit-ENV Library: https://github.com/m5stack/M5Unit-ENV
 *   - SHT30 Temperature/Humidity sensor (I2C address 0x44)
 *   - QMP6988 Barometric Pressure sensor (I2C address 0x70)
 *   - QMP6988 Datasheet: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/unit/enviii/QMP6988%20Datasheet.pdf
 */

#include <M5Atom.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <Update.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_freertos_hooks.h>  // For CPU load measurement via idle hooks
// M5Unit-ENV Library for ENV III sensors
// GitHub: https://github.com/m5stack/M5Unit-ENV
// Install: arduino-cli lib install "M5Unit-ENV"
#include <M5UnitENV.h>

// ENV III sensor objects
// SHT3X: Temperature & Humidity (I2C 0x44)
// QMP6988: Barometric Pressure (I2C 0x70) - uses fixed-point calibration
SHT3X sht30;
QMP6988 qmp6988;

// ============== CONFIGURATION ==============
// Default network settings (used if no saved config)
byte mac[6];
IPAddress ip(10, 200, 45, 63);
IPAddress gateway(10, 200, 45, 1);
IPAddress subnet(255, 255, 252, 0);  // /22

// Preferences for persistent storage
Preferences prefs;
bool configLoaded = false;

// W5500 SPI pins for AtomPOE
#define SCK   22
#define MISO  23
#define MOSI  33
#define CS    19

// I2C pins for ENV III (Grove port)
#define I2C_SDA  26
#define I2C_SCL  32

// Firmware version
const char* FW_VERSION = "1.1.0";

// OTA file path
const char* OTA_FILE = "/update.bin";

// Device info defaults
const char* DEVICE_NAME = "ATOM POE ENV III Sensor";
const char* DEF_CTRL_TYPE = "M5 ATOM Lite";
const char* DEF_CTRL_LOC = "Data Center";
const char* DEF_CTRL_PWR = "Ethernet POE";
const char* DEF_CTRL_MOD = "ENV III (Temperature, Humidity, Pressure)";

// OTA Security - change this password!
const char* OTA_PASSWORD = "ponywall2024";

// Boot log buffer for remote debugging via /api/bootlog
#define BOOTLOG_SIZE 4096
char bootLog[BOOTLOG_SIZE];
size_t bootLogPos = 0;
bool bootLogEnabled = true;
int64_t bootStartTime = 0;
int64_t bootEndTime = 0;

// Log to both Serial and boot buffer
void logPrint(const char* msg) {
  Serial.print(msg);
  if (bootLogEnabled && bootLogPos < BOOTLOG_SIZE - 1) {
    size_t len = strlen(msg);
    size_t space = BOOTLOG_SIZE - 1 - bootLogPos;
    size_t toCopy = (len < space) ? len : space;
    memcpy(&bootLog[bootLogPos], msg, toCopy);
    bootLogPos += toCopy;
    bootLog[bootLogPos] = '\0';
  }
}

void logPrintln(const char* msg) {
  logPrint(msg);
  logPrint("\n");
}

void logPrintf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logPrint(buf);
}

// ============== LED COLORS ==============
#define COLOR_RED     0xFF0000
#define COLOR_GREEN   0x00FF00
#define COLOR_BLUE    0x0000FF
#define COLOR_YELLOW  0xFFFF00
#define COLOR_OFF     0x000000

// ============== GLOBAL OBJECTS ==============
EthernetServer server(80);

// OTA state
File otaFile;
bool otaInProgress = false;
bool shouldRestart = false;
int otaContentLength = 0;

// Status flags
bool envConnected = false;

// ENV III sensor readings
float temperature = 0.0;    // Celsius
float humidity = 0.0;       // %RH
float pressure = 0.0;       // hPa
unsigned long lastSensorRead = 0;

// CPU load monitoring using FreeRTOS idle hooks
// See: https://esp32.com/viewtopic.php?t=9820
volatile uint32_t idleCount0 = 0;  // Core 0 idle counter
volatile uint32_t idleCount1 = 0;  // Core 1 idle counter
uint32_t lastIdleCount0 = 0;
uint32_t lastIdleCount1 = 0;
unsigned long lastCpuCheck = 0;
float cpuPercent = 0.0;            // Combined CPU load percentage (3 decimal precision)
uint8_t cpuLevel = 0;              // For adaptive sensor interval
unsigned long sensorInterval = 500;

// Calibrated idle counts per second (measured at boot with minimal load)
uint32_t idleBaseline0 = 0;
uint32_t idleBaseline1 = 0;
bool cpuCalibrated = false;

// Idle hook for Core 0 - called by FreeRTOS when CPU is idle
bool idleHook0(void) {
  idleCount0++;
  return false;  // Don't skip default idle behavior
}

// Idle hook for Core 1
bool idleHook1(void) {
  idleCount1++;
  return false;
}

// Sensor read speed profiles (intervals in ms for idle/light/moderate/busy states)
// Adaptive profiles (0-2): interval changes based on CPU load
// Fixed profiles (3-5): constant interval regardless of CPU load
// Profile 0: Adaptive Normal  - 200, 500, 1000, 2000ms (max 5/sec)
// Profile 1: Adaptive Fast    - 100, 250, 500, 1000ms  (max 10/sec)
// Profile 2: Adaptive Fastest - 50, 100, 200, 500ms    (max 20/sec)
// Profile 3: Fixed 200ms (5/sec)
// Profile 4: Fixed 100ms (10/sec)
// Profile 5: Fixed 50ms (20/sec)
const unsigned long SPEED_PROFILES[6][4] = {
  {200, 500, 1000, 2000},   // 0: Adaptive Normal
  {100, 250, 500, 1000},    // 1: Adaptive Fast
  {50, 100, 200, 500},      // 2: Adaptive Fastest
  {200, 200, 200, 200},     // 3: Fixed 200ms
  {100, 100, 100, 100},     // 4: Fixed 100ms
  {50, 50, 50, 50}          // 5: Fixed 50ms
};
const char* SPEED_NAMES[] = {"Adaptive Normal", "Adaptive Fast", "Adaptive Fastest", "Fixed 200ms", "Fixed 100ms", "Fixed 50ms"};
uint8_t sensorSpeedProfile = 0;  // Default to Adaptive Normal

// ============== HTML WEBPAGE ==============
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ENV III Sensor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1a1a2e;color:#eee;padding:20px}
h1{color:#0f0;font-size:1.4em;margin-bottom:20px;text-align:center}
.card{background:#16213e;border-radius:12px;padding:20px;margin-bottom:15px}
.card h2{font-size:1.1em;margin-bottom:15px;color:#4cc9f0}
.sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:20px}
.sensor-box{background:#0f3460;border-radius:10px;padding:20px;text-align:center}
.sensor-label{color:#888;font-size:0.85em;margin-bottom:8px}
.sensor-value{font-size:2em;font-weight:bold;color:#4cc9f0}
.sensor-unit{font-size:0.5em;color:#888}
.temp .sensor-value{color:#ff6b6b}
.hum .sensor-value{color:#4ecdc4}
.pres .sensor-value{color:#a29bfe}
.status{text-align:center;padding:10px;font-size:0.9em;color:#888}
a{color:#4cc9f0;text-decoration:none}
.info-icon{display:inline-flex;align-items:center;justify-content:center;width:16px;height:16px;border-radius:50%;background:#4cc9f0;color:#000;font-size:10px;font-weight:bold;cursor:pointer;margin-left:5px;position:relative}
.info-icon:hover .tooltip{display:block}
.tooltip{display:none;position:absolute;bottom:24px;left:50%;transform:translateX(-50%);background:#0f3460;color:#fff;padding:8px 12px;border-radius:6px;font-size:11px;white-space:nowrap;z-index:10;border:1px solid #4cc9f0}
.tooltip::after{content:'';position:absolute;top:100%;left:50%;transform:translateX(-50%);border:6px solid transparent;border-top-color:#4cc9f0}
</style>
</head>
<body>
<h1 id="title">ENV III Sensor</h1>
<div style="text-align:center;font-size:0.8em;color:#888;margin-bottom:10px">Firmware: <span id="fwver">--</span></div>
<div class="card">
<h2>Environment Sensors</h2>
<div class="sensor-grid">
<div class="sensor-box temp">
<div class="sensor-label">Temperature</div>
<div class="sensor-value"><span id="temp">--</span><span class="sensor-unit"> F</span></div>
</div>
<div class="sensor-box hum">
<div class="sensor-label">Humidity</div>
<div class="sensor-value"><span id="hum">--</span><span class="sensor-unit"> %</span></div>
</div>
<div class="sensor-box pres">
<div class="sensor-label">Pressure (hPa)</div>
<div class="sensor-value"><span id="pres">--</span><span class="sensor-unit"> hPa</span></div>
</div>
<div class="sensor-box pres">
<div class="sensor-label">Pressure (inWC)<span class="info-icon">i<span class="tooltip">inWC = hPa × 0.401463</span></span></div>
<div class="sensor-value"><span id="pres_inwc">--</span><span class="sensor-unit"> inWC</span></div>
</div>
</div>
</div>
<div class="card">
<h2>Device Info</h2>
<div style="font-family:monospace;font-size:0.85em;line-height:1.8">
<div>IP: <span id="devip" style="color:#4cc9f0">--</span></div>
<div>MAC: <span id="devmac" style="color:#4cc9f0">--</span></div>
<div>Uptime: <span id="uptime" style="color:#4cc9f0">--</span></div>
<div>CPU: <span id="cpu" style="color:#4cc9f0">--</span></div>
</div>
</div>
<div class="status" id="st">Connecting...</div>
<div style="text-align:center;margin-top:15px">
<a href="/config">Settings</a> | <a href="/api">API Docs</a> | <a href="/update">Firmware Update</a>
</div>
<script>
function init(){
var x=new XMLHttpRequest();
x.open('GET','/api/status',true);
x.onload=function(){
if(x.status==200){
var j=JSON.parse(x.responseText);
var s=j.sensors[0];
document.getElementById('title').textContent=j.device.name;
document.getElementById('fwver').textContent=j.device.firmware||'--';
document.getElementById('devip').textContent=j.device.ip;
document.getElementById('devmac').textContent=j.device.mac;
if(s.connected){
document.getElementById('temp').textContent=s.temperature.f.toFixed(1);
document.getElementById('hum').textContent=s.humidity.percent.toFixed(1);
document.getElementById('pres').textContent=s.pressure.hpa.toFixed(3);
document.getElementById('pres_inwc').textContent=s.pressure.inwc.toFixed(3);
document.getElementById('st').textContent='Sensors OK - Updated ' + new Date().toLocaleTimeString();
}else{
document.getElementById('st').textContent='ENV III sensor not detected';
document.getElementById('st').style.color='#e74c3c';
}
document.getElementById('uptime').textContent=j.device.uptime;
document.getElementById('cpu').textContent=j.system.cpu_percent.toFixed(3) + '% (' + j.system.cpu_state + ')';
}else{
document.getElementById('st').textContent='Connection error';
document.getElementById('st').style.color='#e74c3c';
}
};
x.onerror=function(){
document.getElementById('st').textContent='Connection failed';
document.getElementById('st').style.color='#e74c3c';
};
x.send();
}
init();
setInterval(init,2000);
</script>
</body>
</html>
)rawliteral";

// ============== OTA UPDATE PAGE ==============
const char update_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Firmware Update</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1a1a2e;color:#eee;padding:20px}
h1{color:#4cc9f0;font-size:1.4em;margin-bottom:20px;text-align:center}
.card{background:#16213e;border-radius:12px;padding:20px;margin-bottom:15px;max-width:400px;margin:0 auto}
.card h2{font-size:1.1em;margin-bottom:15px;color:#4cc9f0}
input[type=file],input[type=password]{width:100%;padding:10px;margin-bottom:15px;background:#0f3460;border:none;border-radius:6px;color:#fff}
.btn{background:#4cc9f0;color:#000;border:none;padding:12px 24px;border-radius:8px;font-size:1em;cursor:pointer;font-weight:bold;width:100%}
.btn:hover{background:#7dd8f7}
.btn:disabled{background:#555;cursor:not-allowed}
#progress{width:100%;height:20px;background:#0f3460;border-radius:10px;margin:15px 0;display:none}
#bar{width:0%;height:100%;background:#0f0;border-radius:10px;transition:width 0.3s}
#status{text-align:center;padding:10px;font-size:0.9em;color:#888}
a{color:#4cc9f0;text-decoration:none}
</style>
</head>
<body>
<h1>Firmware Update</h1>
<div style="text-align:center;margin-bottom:15px;color:#888">Current: <span id="fwver" style="color:#4cc9f0;font-weight:bold">--</span></div>
<div class="card">
<h2>Upload New Firmware</h2>
<form id="form" method="POST" enctype="multipart/form-data">
<input type="password" name="password" id="pwd" placeholder="Enter OTA Password">
<input type="file" name="firmware" id="file" accept=".bin">
<div id="progress"><div id="bar"></div></div>
<button type="submit" class="btn" id="btn">Upload & Update</button>
</form>
<div id="status">Select a .bin file to upload</div>
<p style="margin-top:15px;text-align:center"><a href="/">Back to Dashboard</a></p>
</div>
<script>
document.getElementById('form').addEventListener('submit',function(e){
e.preventDefault();
var file=document.getElementById('file').files[0];
var pwd=document.getElementById('pwd').value;
if(!pwd){alert('Enter password');return;}
if(!file){alert('Select a file first');return;}
var formData=new FormData();
formData.append('password',pwd);
formData.append('firmware',file);
var xhr=new XMLHttpRequest();
xhr.open('POST','/update',true);
xhr.upload.onprogress=function(e){
if(e.lengthComputable){
var pct=Math.round((e.loaded/e.total)*100);
document.getElementById('progress').style.display='block';
document.getElementById('bar').style.width=pct+'%';
document.getElementById('status').textContent='Uploading: '+pct+'%';
}
};
xhr.onload=function(){
if(xhr.status==200){
document.getElementById('status').textContent='Update successful! Rebooting...';
document.getElementById('bar').style.background='#0f0';
setTimeout(function(){location.href='/';},5000);
}else{
document.getElementById('status').textContent='Update failed: '+xhr.responseText;
document.getElementById('bar').style.background='#f00';
}
};
xhr.onerror=function(){document.getElementById('status').textContent='Connection error';};
document.getElementById('btn').disabled=true;
document.getElementById('status').textContent='Starting upload...';
xhr.send(formData);
});
var v=new XMLHttpRequest();
v.open('GET','/api/status',true);
v.onload=function(){if(v.status==200){var j=JSON.parse(v.responseText);document.getElementById('fwver').textContent=j.firmware||'--';}};
v.send();
</script>
</body>
</html>
)rawliteral";

// ============== CONFIG PAGE ==============
const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Device Configuration</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1a1a2e;color:#eee;padding:20px}
h1{color:#4cc9f0;font-size:1.4em;margin-bottom:20px;text-align:center}
.card{background:#16213e;border-radius:12px;padding:20px;margin-bottom:15px;max-width:500px;margin:0 auto 15px auto}
.card h2{font-size:1.1em;margin-bottom:15px;color:#4cc9f0}
label{display:block;margin-bottom:5px;color:#aaa;font-size:0.9em}
input[type=text],select{width:100%;padding:8px;margin-bottom:12px;background:#0f3460;border:none;border-radius:6px;color:#fff;font-family:monospace}
select{cursor:pointer}
.speed-info{font-size:0.75em;color:#888;margin-top:-8px;margin-bottom:12px}
.btn{background:#4cc9f0;color:#000;border:none;padding:12px 24px;border-radius:8px;font-size:1em;cursor:pointer;font-weight:bold;width:100%;margin-bottom:10px}
.btn:hover{background:#7dd8f7}
.btn-red{background:#e74c3c}
.btn-red:hover{background:#c0392b}
.info{background:#0f3460;padding:10px;border-radius:6px;margin-bottom:15px;font-family:monospace;font-size:0.85em;line-height:1.6}
.info span{color:#0f0}
a{color:#4cc9f0;text-decoration:none}
#status{text-align:center;padding:10px;font-size:0.9em;color:#888}
</style>
</head>
<body>
<h1>Device Configuration</h1>
<div class="card">
<h2>Current Settings</h2>
<div class="info">
Device: <span id="devname">--</span><br>
Firmware: <span id="fwver">--</span><br>
MAC: <span id="mac">--</span><br>
IP: <span id="curip">--</span><br>
Gateway: <span id="curgw">--</span><br>
Subnet: <span id="cursn">--</span>
</div>
</div>
<form id="form">
<div class="card">
<h2>Device Settings</h2>
<label>Device Name</label>
<input type="text" name="name" id="name" maxlength="32">
<label>Device Location</label>
<input type="text" name="devloc" id="devloc" maxlength="64" placeholder="e.g., Data Center Room 1">
</div>
<div class="card">
<h2>Network Settings</h2>
<label>IP Address</label>
<input type="text" name="ip" id="ip" placeholder="10.200.45.63">
<label>Gateway</label>
<input type="text" name="gw" id="gw" placeholder="10.200.45.1">
<label>Subnet Mask</label>
<input type="text" name="sn" id="sn" placeholder="255.255.252.0">
</div>
<div class="card">
<h2>Sensor Settings</h2>
<label>Sensor Name</label>
<input type="text" name="senname" id="senname" maxlength="32" placeholder="e.g., ENV III">
<label>Sensor Location</label>
<input type="text" name="senloc" id="senloc" maxlength="64" placeholder="e.g., Server Rack A">
<label>Sensor Height</label>
<input type="text" name="senht" id="senht" maxlength="32" placeholder="e.g., 6 ft or 1.8m">
<label>Sensor Read Speed</label>
<select name="speed" id="speed">
<optgroup label="Adaptive (adjusts with CPU load)">
<option value="0">Normal - 5/sec when idle, slows when busy</option>
<option value="1">Fast - 10/sec when idle, slows when busy</option>
<option value="2">Fastest - 20/sec when idle, slows when busy</option>
</optgroup>
<optgroup label="Fixed (constant rate)">
<option value="3">Fixed 200ms (5 reads/sec)</option>
<option value="4">Fixed 100ms (10 reads/sec)</option>
<option value="5">Fixed 50ms (20 reads/sec)</option>
</optgroup>
</select>
<div class="speed-info">Adaptive modes back off when CPU is busy. Fixed modes maintain constant rate.</div>
</div>
<div class="card">
<button type="submit" class="btn">Save & Reboot</button>
<div id="status"></div>
</div>
</form>
<div class="card">
<h2>Maintenance</h2>
<button class="btn" id="rebootbtn">Reboot Device</button>
<button class="btn" onclick="location.href='/update'">Firmware Update</button>
<button class="btn btn-red" id="resetbtn">Factory Reset</button>
</div>
<div style="text-align:center;margin-top:15px"><a href="/">Dashboard</a> | <a href="/api">API Docs</a></div>
<script>
function load(){
var x=new XMLHttpRequest();
x.open('GET','/api/config',true);
x.onload=function(){
if(x.status==200){
var j=JSON.parse(x.responseText);
document.getElementById('devname').textContent=j.name;
document.getElementById('fwver').textContent=j.firmware||'--';
document.getElementById('mac').textContent=j.mac;
document.getElementById('curip').textContent=j.ip;
document.getElementById('curgw').textContent=j.gateway;
document.getElementById('cursn').textContent=j.subnet;
document.getElementById('name').value=j.name;
document.getElementById('devloc').value=j.device_location||'';
document.getElementById('ip').value=j.ip;
document.getElementById('gw').value=j.gateway;
document.getElementById('sn').value=j.subnet;
document.getElementById('senname').value=j.sensor_name||'ENV III';
document.getElementById('senloc').value=j.sensor_location||'';
document.getElementById('senht').value=j.sensor_height||'';
document.getElementById('speed').value=j.speed||0;
}
};
x.send();
}
document.getElementById('form').addEventListener('submit',function(e){
e.preventDefault();
var pwd=prompt('Enter password to save settings:');
if(!pwd)return;
var data='password='+encodeURIComponent(pwd);
data+='&name='+encodeURIComponent(document.getElementById('name').value);
data+='&devloc='+encodeURIComponent(document.getElementById('devloc').value);
data+='&ip='+document.getElementById('ip').value;
data+='&gw='+document.getElementById('gw').value;
data+='&sn='+document.getElementById('sn').value;
data+='&senname='+encodeURIComponent(document.getElementById('senname').value);
data+='&senloc='+encodeURIComponent(document.getElementById('senloc').value);
data+='&senht='+encodeURIComponent(document.getElementById('senht').value);
data+='&speed='+document.getElementById('speed').value;
fetch('/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data})
.then(function(r){
if(r.ok){document.getElementById('status').textContent='Saved! Rebooting...';setTimeout(function(){location.href='http://'+document.getElementById('ip').value+'/';},3000);}
else if(r.status==401){alert('Invalid password');}
else{r.text().then(function(t){alert('Error: '+t);});}
})
.catch(function(e){alert('Connection error: '+e);});
});
document.getElementById('rebootbtn').addEventListener('click',function(){
var pwd=prompt('Enter password to reboot:');
if(!pwd)return;
var x=new XMLHttpRequest();
x.open('POST','/reboot',true);
x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
x.onload=function(){
if(x.status==200){alert('Device rebooting...');setTimeout(function(){location.reload();},5000);}
else if(x.status==401){alert('Invalid password');}
else{alert('Error: '+x.responseText);}
};
x.send('password='+encodeURIComponent(pwd));
});
document.getElementById('resetbtn').addEventListener('click',function(){
var pwd=prompt('Enter password to factory reset:');
if(!pwd)return;
if(!confirm('This will erase all settings. Continue?'))return;
var x=new XMLHttpRequest();
x.open('POST','/reset',true);
x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
x.onload=function(){
if(x.status==200){alert('Reset complete. Device rebooting...');location.href='/';}
else if(x.status==401){alert('Invalid password');}
else{alert('Error: '+x.responseText);}
};
x.send('password='+encodeURIComponent(pwd));
});
load();
</script>
</body>
</html>
)rawliteral";

// ============== API DOCUMENTATION PAGE ==============
const char api_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>API Documentation</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1a1a2e;color:#eee;padding:20px}
h1{color:#4cc9f0;font-size:1.4em;margin-bottom:20px;text-align:center}
.card{background:#16213e;border-radius:12px;padding:20px;margin-bottom:15px;max-width:700px;margin:0 auto 15px auto}
.card h2{font-size:1.1em;margin-bottom:15px;color:#4cc9f0}
.card h3{font-size:0.95em;margin:15px 0 10px 0;color:#a29bfe}
.endpoint{background:#0f3460;padding:12px;border-radius:6px;margin-bottom:8px;font-family:monospace;font-size:0.85em}
.method{color:#0f0;font-weight:bold}
.url{color:#4cc9f0}
.format{color:#ff6b6b;font-size:0.8em;margin-left:8px}
.desc{color:#aaa;margin-top:4px;font-family:system-ui;font-size:0.8em}
a{color:#4cc9f0;text-decoration:none}
pre{background:#0f3460;padding:12px;border-radius:6px;font-size:0.75em;overflow-x:auto;line-height:1.4}
.tab{display:inline-block;padding:6px 12px;cursor:pointer;border-radius:6px 6px 0 0;background:#0f3460;margin-right:4px}
.tab.active{background:#4cc9f0;color:#000}
.tab-content{display:none}
.tab-content.active{display:block}
</style>
</head>
<body>
<h1>API Documentation</h1>

<div class="card">
<h2>JSON Endpoints</h2>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/status" class="url">/api/status</a> <span class="format">JSON</span>
<div class="desc">Full device status with sensors and system info (default)</div>
</div>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/sensors" class="url">/api/sensors</a> <span class="format">JSON</span>
<div class="desc">Sensor data only (default)</div>
</div>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/json/status" class="url">/api/json/status</a> <span class="format">JSON</span>
<div class="desc">Full device status (explicit JSON)</div>
</div>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/json/sensors" class="url">/api/json/sensors</a> <span class="format">JSON</span>
<div class="desc">Sensor data only (explicit JSON)</div>
</div>
</div>

<div class="card">
<h2>XML Endpoints</h2>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/xml/status" class="url">/api/xml/status</a> <span class="format">XML</span>
<div class="desc">Full device status in XML format</div>
</div>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/xml/sensors" class="url">/api/xml/sensors</a> <span class="format">XML</span>
<div class="desc">Sensor data only in XML format</div>
</div>
</div>

<div class="card">
<h2>Configuration &amp; Debug</h2>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/config" class="url">/api/config</a> <span class="format">JSON</span>
<div class="desc">Device configuration</div>
</div>
<div class="endpoint">
<span class="method">GET</span> <a href="/api/bootlog" class="url">/api/bootlog</a> <span class="format">TEXT</span>
<div class="desc">Boot log for debugging</div>
</div>
</div>

<div class="card">
<h2>Response Examples</h2>
<div style="margin-bottom:10px">
<span class="tab active" onclick="showTab('json')">JSON</span>
<span class="tab" onclick="showTab('xml')">XML</span>
</div>
<div id="json" class="tab-content active">
<pre>{
  "device": {
    "name": "ATOM POE ENV III Sensor",
    "firmware": "1.1.0",
    "ip": "10.200.45.63",
    "mac": "AC:0B:FB:AB:96:D0",
    "uptime": "0d 0h 5m 30s"
  },
  "sensors": [
    {
      "id": "env3",
      "type": "SHT30+QMP6988",
      "connected": true,
      "temperature": {"c": 23.5, "f": 74.3},
      "humidity": {"percent": 45.2},
      "pressure": {"hpa": 871.26, "inwc": 349.78}
    }
  ],
  "system": {
    "cpu_percent": 0.500,
    "cpu_state": "idle"
  }
}</pre>
</div>
<div id="xml" class="tab-content">
<pre>&lt;?xml version="1.0" encoding="UTF-8"?&gt;
&lt;status&gt;
  &lt;device&gt;
    &lt;name&gt;ATOM POE ENV III Sensor&lt;/name&gt;
    &lt;firmware&gt;1.1.0&lt;/firmware&gt;
    &lt;ip&gt;10.200.45.63&lt;/ip&gt;
    &lt;mac&gt;AC:0B:FB:AB:96:D0&lt;/mac&gt;
    &lt;uptime&gt;0d 0h 5m 30s&lt;/uptime&gt;
  &lt;/device&gt;
  &lt;sensors&gt;
    &lt;sensor id="env3" type="SHT30+QMP6988"&gt;
      &lt;connected&gt;true&lt;/connected&gt;
      &lt;temperature&gt;
        &lt;c&gt;23.5&lt;/c&gt;&lt;f&gt;74.3&lt;/f&gt;
      &lt;/temperature&gt;
      &lt;humidity&gt;&lt;percent&gt;45.2&lt;/percent&gt;&lt;/humidity&gt;
      &lt;pressure&gt;
        &lt;hpa&gt;871.26&lt;/hpa&gt;&lt;inwc&gt;349.78&lt;/inwc&gt;
      &lt;/pressure&gt;
    &lt;/sensor&gt;
  &lt;/sensors&gt;
  &lt;system&gt;
    &lt;cpu_percent&gt;0.500&lt;/cpu_percent&gt;
    &lt;cpu_state&gt;idle&lt;/cpu_state&gt;
  &lt;/system&gt;
&lt;/status&gt;</pre>
</div>
</div>

<div class="card">
<h2>Data Structure</h2>
<pre style="color:#4cc9f0">
device
├── name, firmware, ip, mac, uptime

sensors[] (array for multiple sensors)
└── [0]
    ├── id: "env3"
    ├── type: "SHT30+QMP6988"
    ├── connected: true/false
    ├── temperature: {c, f}
    ├── humidity: {percent}
    └── pressure: {hpa, inwc}

system
├── cpu_percent: 0.000-100.000
└── cpu_state: idle|light|moderate|busy
</pre>
</div>

<div style="text-align:center;margin-top:15px"><a href="/">Dashboard</a> | <a href="/config">Settings</a></div>
<script>
function showTab(t){
document.querySelectorAll('.tab').forEach(e=>e.classList.remove('active'));
document.querySelectorAll('.tab-content').forEach(e=>e.classList.remove('active'));
document.getElementById(t).classList.add('active');
event.target.classList.add('active');
}
</script>
</body>
</html>
)rawliteral";

// ============== CONFIG FUNCTIONS ==============
void loadConfig() {
  prefs.begin("enviii", true);  // Read-only

  if (prefs.isKey("configured")) {
    ip[0] = prefs.getUChar("ip0", 10);
    ip[1] = prefs.getUChar("ip1", 200);
    ip[2] = prefs.getUChar("ip2", 45);
    ip[3] = prefs.getUChar("ip3", 63);

    gateway[0] = prefs.getUChar("gw0", 10);
    gateway[1] = prefs.getUChar("gw1", 200);
    gateway[2] = prefs.getUChar("gw2", 45);
    gateway[3] = prefs.getUChar("gw3", 1);

    subnet[0] = prefs.getUChar("sn0", 255);
    subnet[1] = prefs.getUChar("sn1", 255);
    subnet[2] = prefs.getUChar("sn2", 252);
    subnet[3] = prefs.getUChar("sn3", 0);

    sensorSpeedProfile = prefs.getUChar("speed", 0);
    if (sensorSpeedProfile > 5) sensorSpeedProfile = 0;  // Validate

    configLoaded = true;
    logPrintln("Config loaded from NVS");
  } else {
    logPrintln("Using default config");
  }

  // Always load sensor speed (even if not configured)
  sensorSpeedProfile = prefs.getUChar("speed", 0);
  if (sensorSpeedProfile > 5) sensorSpeedProfile = 0;

  prefs.end();
}

void saveConfig(IPAddress newIP, IPAddress newGW, IPAddress newSN, const char* newName,
                 const char* newDevLoc, const char* newSenName, const char* newSenLoc,
                 const char* newSenHeight, uint8_t newSpeed) {
  prefs.begin("enviii", false);  // Read-write

  prefs.putUChar("ip0", newIP[0]);
  prefs.putUChar("ip1", newIP[1]);
  prefs.putUChar("ip2", newIP[2]);
  prefs.putUChar("ip3", newIP[3]);

  prefs.putUChar("gw0", newGW[0]);
  prefs.putUChar("gw1", newGW[1]);
  prefs.putUChar("gw2", newGW[2]);
  prefs.putUChar("gw3", newGW[3]);

  prefs.putUChar("sn0", newSN[0]);
  prefs.putUChar("sn1", newSN[1]);
  prefs.putUChar("sn2", newSN[2]);
  prefs.putUChar("sn3", newSN[3]);

  if (newName && strlen(newName) > 0) {
    prefs.putString("devname", newName);
  }

  // Device location
  prefs.putString("devloc", newDevLoc ? newDevLoc : "");

  // Sensor settings
  prefs.putString("senname", newSenName ? newSenName : "ENV III");
  prefs.putString("senloc", newSenLoc ? newSenLoc : "");
  prefs.putString("senht", newSenHeight ? newSenHeight : "");

  if (newSpeed <= 5) {
    prefs.putUChar("speed", newSpeed);
  }

  prefs.putBool("configured", true);
  prefs.end();

  logPrintln("Config saved to NVS");
}

void resetConfig() {
  prefs.begin("enviii", false);
  prefs.clear();
  prefs.end();
  logPrintln("Config reset to defaults");
}

String getDeviceName() {
  prefs.begin("enviii", true);
  String name = prefs.getString("devname", DEVICE_NAME);
  prefs.end();
  return name;
}

String getDeviceLocation() {
  prefs.begin("enviii", true);
  String loc = prefs.getString("devloc", "");
  prefs.end();
  return loc;
}

String getSensorName() {
  prefs.begin("enviii", true);
  String name = prefs.getString("senname", "ENV III");
  prefs.end();
  return name;
}

String getSensorLocation() {
  prefs.begin("enviii", true);
  String loc = prefs.getString("senloc", "");
  prefs.end();
  return loc;
}

String getSensorHeight() {
  prefs.begin("enviii", true);
  String height = prefs.getString("senht", "");
  prefs.end();
  return height;
}

String getMacString() {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

String getIPString(IPAddress addr) {
  return String(addr[0]) + "." + String(addr[1]) + "." + String(addr[2]) + "." + String(addr[3]);
}

bool parseIP(const String& str, IPAddress& addr) {
  int parts[4];
  int count = sscanf(str.c_str(), "%d.%d.%d.%d", &parts[0], &parts[1], &parts[2], &parts[3]);
  if (count != 4) return false;
  for (int i = 0; i < 4; i++) {
    if (parts[i] < 0 || parts[i] > 255) return false;
    addr[i] = parts[i];
  }
  return true;
}

String getUptimeString() {
  int64_t uptimeUs = esp_timer_get_time();
  int64_t secs = uptimeUs / 1000000;
  int64_t mins = secs / 60;
  int64_t hrs = mins / 60;
  int64_t days = hrs / 24;

  char buf[32];
  snprintf(buf, sizeof(buf), "%lldd %lldh %lldm %llds", days, hrs % 24, mins % 60, secs % 60);
  return String(buf);
}

// ============== TWO-STAGE OTA FUNCTIONS ==============
void checkAndApplyOTA() {
  if (!LittleFS.begin(false)) {
    logPrintln("LittleFS mount failed, formatting...");
    LittleFS.format();
    if (!LittleFS.begin(false)) {
      logPrintln("LittleFS format failed, skipping OTA check");
      return;
    }
    logPrintln("LittleFS formatted successfully");
  }

  if (!LittleFS.exists(OTA_FILE)) {
    logPrintln("No pending OTA update");
    return;
  }

  File file = LittleFS.open(OTA_FILE, "r");
  if (!file) {
    logPrintln("Failed to open OTA file");
    return;
  }

  size_t fileSize = file.size();
  logPrintln("*** PENDING OTA UPDATE FOUND ***");
  logPrintf("Update file size: %u bytes\n", fileSize);

  if (fileSize < 100000) {
    logPrintln("File too small, deleting");
    file.close();
    LittleFS.remove(OTA_FILE);
    return;
  }

  if (!Update.begin(fileSize)) {
    logPrintf("Update.begin failed: %s\n", Update.errorString());
    file.close();
    return;
  }

  logPrintln("Starting firmware update...");

  uint8_t buf[4096];
  size_t written = 0;
  int lastProgress = -1;

  while (file.available()) {
    size_t toRead = min((size_t)file.available(), sizeof(buf));
    size_t bytesRead = file.read(buf, toRead);

    if (bytesRead > 0) {
      size_t writeResult = Update.write(buf, bytesRead);
      if (writeResult != bytesRead) {
        logPrintf("Write error at %u bytes\n", written);
        break;
      }
      written += bytesRead;

      int progress = (written * 100) / fileSize;
      if (progress / 10 > lastProgress) {
        lastProgress = progress / 10;
        logPrintf("Progress: %d%%\n", progress);
      }
    }
  }

  file.close();
  logPrintf("Written %u bytes to flash\n", written);

  if (Update.end(true)) {
    logPrintln("Update successful!");
    LittleFS.remove(OTA_FILE);
    logPrintln("Rebooting with new firmware...");
    delay(1000);
    ESP.restart();
  } else {
    logPrintf("Update.end failed: %s\n", Update.errorString());
    LittleFS.remove(OTA_FILE);
  }
}

// ============== ENV III SENSOR FUNCTIONS ==============
void readENVSensors() {
  if (!envConnected) return;

  // Read SHT30 (temperature and humidity) using M5Stack library
  if (sht30.update()) {
    temperature = sht30.cTemp;
    humidity = sht30.humidity;
  }

  // Read QMP6988 (pressure) using M5Stack library
  if (qmp6988.update()) {
    pressure = qmp6988.pressure / 100.0f;  // Library returns Pa, convert to hPa
  }

  lastSensorRead = millis();
}

// ============== SETUP ==============
void setup() {
  bootStartTime = esp_timer_get_time();
  bootLog[0] = '\0';

  // Initialize M5Atom
  M5.begin(true, false, true);
  M5.dis.setBrightness(100);
  M5.dis.drawpix(0, COLOR_YELLOW);

  // Initialize Serial
  Serial.begin(115200);
  delay(500);
  logPrintln("\n================================");
  logPrintln(DEVICE_NAME);
  logPrintf("Firmware Version: %s\n", FW_VERSION);
  logPrintln("================================");

  // Check for pending OTA update FIRST
  checkAndApplyOTA();

  // Get unique MAC using Espressif OUI + chip ID
  uint64_t chipid = ESP.getEfuseMac();
  mac[0] = 0xAC;  // Espressif OUI
  mac[1] = 0x0B;
  mac[2] = 0xFB;
  mac[3] = (chipid >> 24) & 0xFF;
  mac[4] = (chipid >> 32) & 0xFF;
  mac[5] = (chipid >> 40) & 0xFF;

  logPrintf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Load saved configuration
  loadConfig();

  // Initialize I2C for ENV III
  Wire.begin(I2C_SDA, I2C_SCL);

  // Check for ENV III module (SHT30 + QMP6988) using M5Stack library
  logPrint("Checking for ENV III... ");

  // Initialize SHT30 using M5Stack library
  if (sht30.begin(&Wire, SHT3X_I2C_ADDR, I2C_SDA, I2C_SCL, 100000U)) {
    envConnected = true;
    logPrintln("OK (SHT30 detected)");

    // Initialize QMP6988 using M5Stack library
    if (qmp6988.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, I2C_SDA, I2C_SCL, 100000U)) {
      logPrintln("  QMP6988 pressure sensor initialized (official M5Stack library)");
    } else {
      logPrintln("  QMP6988 init failed!");
    }

    // Do initial sensor read
    readENVSensors();
  } else {
    logPrintln("Not found!");
    M5.dis.drawpix(0, COLOR_RED);
  }

  // Initialize SPI for W5500
  SPI.begin(SCK, MISO, MOSI, CS);

  // Initialize Ethernet with W5500
  logPrint("Initializing W5500 Ethernet... ");
  Ethernet.init(CS);
  Ethernet.begin(mac, ip, gateway, gateway, subnet);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    logPrintln("W5500 not found!");
    M5.dis.drawpix(0, COLOR_RED);
    while (true) delay(1000);
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    logPrintln("Cable not connected");
    M5.dis.drawpix(0, COLOR_YELLOW);
  } else {
    logPrintln("OK");
  }

  // Print network configuration
  logPrintln("\n--- Network Configuration ---");
  logPrintf("IP Address:  %s\n", getIPString(Ethernet.localIP()).c_str());
  logPrintf("Gateway:     %s\n", getIPString(gateway).c_str());
  logPrintf("Subnet Mask: %s\n", getIPString(subnet).c_str());
  logPrintf("MAC Address: %s\n", getMacString().c_str());
  logPrintln("-----------------------------\n");

  // Start server
  server.begin();
  logPrintln("HTTP server started on port 80");
  logPrintf("Dashboard: http://%s/\n", getIPString(Ethernet.localIP()).c_str());
  logPrintf("API: http://%s/api/sensors\n", getIPString(Ethernet.localIP()).c_str());
  logPrintln("================================\n");

  // Record boot end time
  bootEndTime = esp_timer_get_time();
  int64_t bootMs = (bootEndTime - bootStartTime) / 1000;
  logPrintf("Boot completed in %lld ms\n", bootMs);
  bootLogEnabled = false;

  // Register FreeRTOS idle hooks for CPU load measurement
  esp_register_freertos_idle_hook_for_cpu(idleHook0, 0);
  esp_register_freertos_idle_hook_for_cpu(idleHook1, 1);
  logPrintln("CPU idle hooks registered");

  // Calibrate CPU baseline (measure idle counts for 1 second with minimal load)
  logPrint("Calibrating CPU baseline... ");
  idleCount0 = 0;
  idleCount1 = 0;
  delay(1000);
  idleBaseline0 = idleCount0;
  idleBaseline1 = idleCount1;
  cpuCalibrated = true;
  logPrintf("Core0: %lu, Core1: %lu idle/sec\n", idleBaseline0, idleBaseline1);

  // Ready - green LED if sensors connected
  if (envConnected) {
    M5.dis.drawpix(0, COLOR_GREEN);
  }
}

// ============== MAIN LOOP ==============
void loop() {
  // Maintain Ethernet
  Ethernet.maintain();

  // Handle HTTP clients
  for (int i = 0; i < 4; i++) {
    if (!handleClient()) break;
  }

  M5.update();

  // Calculate CPU load every second using FreeRTOS idle hook counts
  if (millis() - lastCpuCheck >= 1000) {
    lastCpuCheck = millis();

    if (cpuCalibrated && idleBaseline0 > 0 && idleBaseline1 > 0) {
      // Get idle counts since last check
      uint32_t idle0 = idleCount0 - lastIdleCount0;
      uint32_t idle1 = idleCount1 - lastIdleCount1;
      lastIdleCount0 = idleCount0;
      lastIdleCount1 = idleCount1;

      // Dynamically update baseline if we see higher idle counts (self-calibrating)
      // This corrects for baseline captured during boot activity
      if (idle0 > idleBaseline0) idleBaseline0 = idle0;
      if (idle1 > idleBaseline1) idleBaseline1 = idle1;

      // Calculate load for each core: load% = 100 - (current_idle / baseline_idle * 100)
      float load0 = 100.0f - ((float)idle0 * 100.0f / (float)idleBaseline0);
      float load1 = 100.0f - ((float)idle1 * 100.0f / (float)idleBaseline1);

      // Clamp values
      if (load0 < 0.0f) load0 = 0.0f; if (load0 > 100.0f) load0 = 100.0f;
      if (load1 < 0.0f) load1 = 0.0f; if (load1 > 100.0f) load1 = 100.0f;

      // Average both cores for combined CPU load
      cpuPercent = (load0 + load1) / 2.0f;

      // Set level for adaptive sensor interval
      if (cpuPercent < 20) cpuLevel = 0;       // idle
      else if (cpuPercent < 50) cpuLevel = 1;  // light
      else if (cpuPercent < 80) cpuLevel = 2;  // moderate
      else cpuLevel = 3;                        // busy

      sensorInterval = SPEED_PROFILES[sensorSpeedProfile][cpuLevel];
    }
  }

  // Read ENV III sensors with adaptive interval
  if (envConnected && (millis() - lastSensorRead > sensorInterval)) {
    readENVSensors();
  }

  // Button press to force sensor read
  if (M5.Btn.wasPressed()) {
    if (envConnected) {
      readENVSensors();
      Serial.printf("Temp: %.1f C (%.1f F), Humidity: %.1f%%, Pressure: %.2f hPa\n",
        temperature, (temperature * 9.0 / 5.0) + 32.0, humidity, pressure);
      M5.dis.drawpix(0, COLOR_BLUE);
      delay(100);
      M5.dis.drawpix(0, COLOR_GREEN);
    }
  }
}

// ============== HTTP SERVER ==============
bool handleClient() {
  EthernetClient client = server.available();
  if (!client) return false;

  Serial.println("New client");
  String requestLine = "";
  bool firstLine = true;
  bool headersDone = false;
  bool isPostUpdate = false;
  otaContentLength = 0;
  unsigned long timeout = millis();

  while (client.connected() && !headersDone && (millis() - timeout < 500)) {
    if (client.available()) {
      char c = client.read();

      if (firstLine && c != '\r' && c != '\n') {
        requestLine += c;
      }

      if (firstLine && c == '\n') {
        firstLine = false;
        Serial.println("Request: " + requestLine);
        isPostUpdate = requestLine.startsWith("POST /update");
      }

      if (!firstLine && isPostUpdate) {
        static String headerLine = "";
        if (c == '\n') {
          headerLine.trim();
          if (headerLine.startsWith("Content-Length:")) {
            otaContentLength = headerLine.substring(15).toInt();
          }
          headerLine = "";
        } else if (c != '\r') {
          headerLine += c;
        }
      }

      static String line = "";
      if (c == '\n') {
        if (line.length() == 0) {
          headersDone = true;
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  // Route requests
  if (requestLine.startsWith("GET / HTTP") || requestLine.startsWith("GET /index")) {
    sendHtml(client);
  }
  // JSON API routes (explicit)
  else if (requestLine.startsWith("GET /api/json/status")) {
    sendStatusJSON(client);
  }
  else if (requestLine.startsWith("GET /api/json/sensors")) {
    sendSensorsJSON(client);
  }
  // XML API routes
  else if (requestLine.startsWith("GET /api/xml/status")) {
    sendStatusXML(client);
  }
  else if (requestLine.startsWith("GET /api/xml/sensors")) {
    sendSensorsXML(client);
  }
  // Default API routes (JSON)
  else if (requestLine.startsWith("GET /api/status")) {
    sendStatusJSON(client);
  }
  else if (requestLine.startsWith("GET /api/sensors")) {
    sendSensorsJSON(client);
  }
  else if (requestLine.startsWith("GET /api/bootlog")) {
    sendBootlog(client);
  }
  else if (requestLine.startsWith("GET /api/config")) {
    sendConfigJSON(client);
  }
  else if (requestLine.startsWith("GET /api HTTP") || requestLine.startsWith("GET /api ")) {
    sendApiPage(client);
  }
  else if (requestLine.startsWith("GET /config")) {
    sendConfigPage(client);
  }
  else if (requestLine.startsWith("POST /config")) {
    handleConfigSave(client);
    return true;
  }
  else if (requestLine.startsWith("POST /reboot")) {
    handleReboot(client);
    return true;
  }
  else if (requestLine.startsWith("POST /reset")) {
    handleFactoryReset(client);
    return true;
  }
  else if (requestLine.startsWith("GET /reset")) {
    // Reject GET requests - must use POST with password
    client.println("HTTP/1.1 405 Method Not Allowed");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Use POST with password");
    return true;
  }
  else if (requestLine.startsWith("GET /update")) {
    sendUpdatePage(client);
  }
  else if (requestLine.startsWith("POST /update")) {
    handleOTAUpload(client);
    return true;
  }
  else {
    send404(client);
  }

  client.flush();
  while (client.available()) {
    client.read();
  }
  delay(1);
  client.stop();
  Serial.println("Client disconnected");
  return true;
}

void sendHtml(EthernetClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  const char* ptr = index_html;
  size_t len = strlen_P(index_html);
  char buf[256];
  size_t i = 0;
  while (i < len) {
    size_t chunk = min((size_t)256, len - i);
    memcpy_P(buf, &ptr[i], chunk);
    client.write(buf, chunk);
    i += chunk;
  }
}

void sendStatusJSON(EthernetClient& client) {
  char json[1024];
  const char* cpuStates[] = {"idle", "light", "moderate", "busy"};
  float temp_f = (temperature * 9.0 / 5.0) + 32.0;
  float pressure_inwc = pressure * 0.401463;

  int len = snprintf(json, sizeof(json),
    "{"
    "\"device\":{"
      "\"name\":\"%s\","
      "\"location\":\"%s\","
      "\"firmware\":\"%s\","
      "\"ip\":\"%s\","
      "\"mac\":\"%s\","
      "\"uptime\":\"%s\""
    "},"
    "\"sensors\":["
      "{"
        "\"id\":\"env3\","
        "\"name\":\"%s\","
        "\"location\":\"%s\","
        "\"height\":\"%s\","
        "\"type\":\"SHT30+QMP6988\","
        "\"connected\":%s,"
        "\"temperature\":{\"c\":%.2f,\"f\":%.2f},"
        "\"humidity\":{\"percent\":%.2f},"
        "\"pressure\":{\"hpa\":%.3f,\"inwc\":%.3f}"
      "}"
    "],"
    "\"system\":{"
      "\"cpu_percent\":%.3f,"
      "\"cpu_state\":\"%s\""
    "}"
    "}",
    getDeviceName().c_str(), getDeviceLocation().c_str(), FW_VERSION,
    getIPString(Ethernet.localIP()).c_str(), getMacString().c_str(),
    getUptimeString().c_str(),
    getSensorName().c_str(), getSensorLocation().c_str(), getSensorHeight().c_str(),
    envConnected ? "true" : "false",
    temperature, temp_f, humidity, pressure, pressure_inwc,
    cpuPercent, cpuStates[cpuLevel]);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.write(json, len);
}

void sendSensorsJSON(EthernetClient& client) {
  char json[640];
  float temp_f = (temperature * 9.0 / 5.0) + 32.0;
  float pressure_inwc = pressure * 0.401463;

  int len = snprintf(json, sizeof(json),
    "{\"sensors\":["
      "{"
        "\"id\":\"env3\","
        "\"name\":\"%s\","
        "\"location\":\"%s\","
        "\"height\":\"%s\","
        "\"type\":\"SHT30+QMP6988\","
        "\"connected\":%s,"
        "\"temperature\":{\"c\":%.2f,\"f\":%.2f},"
        "\"humidity\":{\"percent\":%.2f},"
        "\"pressure\":{\"hpa\":%.3f,\"inwc\":%.3f}"
      "}"
    "]}",
    getSensorName().c_str(), getSensorLocation().c_str(), getSensorHeight().c_str(),
    envConnected ? "true" : "false",
    temperature, temp_f, humidity, pressure, pressure_inwc);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.write(json, len);
}

void sendStatusXML(EthernetClient& client) {
  const char* cpuStates[] = {"idle", "light", "moderate", "busy"};
  float temp_f = (temperature * 9.0 / 5.0) + 32.0;
  float pressure_inwc = pressure * 0.401463;

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/xml");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  client.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  client.println("<status>");
  client.println("  <device>");
  client.printf("    <name>%s</name>\n", getDeviceName().c_str());
  client.printf("    <location>%s</location>\n", getDeviceLocation().c_str());
  client.printf("    <firmware>%s</firmware>\n", FW_VERSION);
  client.printf("    <ip>%s</ip>\n", getIPString(Ethernet.localIP()).c_str());
  client.printf("    <mac>%s</mac>\n", getMacString().c_str());
  client.printf("    <uptime>%s</uptime>\n", getUptimeString().c_str());
  client.println("  </device>");
  client.println("  <sensors>");
  client.println("    <sensor id=\"env3\" type=\"SHT30+QMP6988\">");
  client.printf("      <name>%s</name>\n", getSensorName().c_str());
  client.printf("      <location>%s</location>\n", getSensorLocation().c_str());
  client.printf("      <height>%s</height>\n", getSensorHeight().c_str());
  client.printf("      <connected>%s</connected>\n", envConnected ? "true" : "false");
  client.println("      <temperature>");
  client.printf("        <c>%.2f</c>\n", temperature);
  client.printf("        <f>%.2f</f>\n", temp_f);
  client.println("      </temperature>");
  client.println("      <humidity>");
  client.printf("        <percent>%.2f</percent>\n", humidity);
  client.println("      </humidity>");
  client.println("      <pressure>");
  client.printf("        <hpa>%.3f</hpa>\n", pressure);
  client.printf("        <inwc>%.3f</inwc>\n", pressure_inwc);
  client.println("      </pressure>");
  client.println("    </sensor>");
  client.println("  </sensors>");
  client.println("  <system>");
  client.printf("    <cpu_percent>%.3f</cpu_percent>\n", cpuPercent);
  client.printf("    <cpu_state>%s</cpu_state>\n", cpuStates[cpuLevel]);
  client.println("  </system>");
  client.println("</status>");
}

void sendSensorsXML(EthernetClient& client) {
  float temp_f = (temperature * 9.0 / 5.0) + 32.0;
  float pressure_inwc = pressure * 0.401463;

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/xml");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  client.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  client.println("<sensors>");
  client.println("  <sensor id=\"env3\" type=\"SHT30+QMP6988\">");
  client.printf("    <name>%s</name>\n", getSensorName().c_str());
  client.printf("    <location>%s</location>\n", getSensorLocation().c_str());
  client.printf("    <height>%s</height>\n", getSensorHeight().c_str());
  client.printf("    <connected>%s</connected>\n", envConnected ? "true" : "false");
  client.println("    <temperature>");
  client.printf("      <c>%.2f</c>\n", temperature);
  client.printf("      <f>%.2f</f>\n", temp_f);
  client.println("    </temperature>");
  client.println("    <humidity>");
  client.printf("      <percent>%.2f</percent>\n", humidity);
  client.println("    </humidity>");
  client.println("    <pressure>");
  client.printf("      <hpa>%.3f</hpa>\n", pressure);
  client.printf("      <inwc>%.3f</inwc>\n", pressure_inwc);
  client.println("    </pressure>");
  client.println("  </sensor>");
  client.println("</sensors>");
}

void sendBootlog(EthernetClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.print(bootLog);

  client.printf("\n--- Current Status ---\n");
  client.printf("Uptime: %s\n", getUptimeString().c_str());
  client.printf("Boot time: %lld ms\n", (bootEndTime - bootStartTime) / 1000);
  client.printf("CPU Load: %.3f%% (%s)\n", cpuPercent,
    cpuLevel == 0 ? "idle" : cpuLevel == 1 ? "light" : cpuLevel == 2 ? "moderate" : "busy");

  if (envConnected) {
    client.printf("\n--- Sensor Readings ---\n");
    client.printf("Temperature: %.2f C (%.2f F)\n", temperature, (temperature * 9.0 / 5.0) + 32.0);
    client.printf("Humidity: %.2f %%\n", humidity);
    client.printf("Pressure: %.2f hPa\n", pressure);
  }
}

void sendConfigPage(EthernetClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  const char* ptr = config_html;
  size_t len = strlen_P(config_html);
  char buf[256];
  size_t i = 0;
  while (i < len) {
    size_t chunk = min((size_t)256, len - i);
    memcpy_P(buf, &ptr[i], chunk);
    client.write(buf, chunk);
    i += chunk;
  }
}

void sendConfigJSON(EthernetClient& client) {
  char json[600];

  int len = snprintf(json, sizeof(json),
    "{\"name\":\"%s\",\"device_location\":\"%s\",\"firmware\":\"%s\","
    "\"mac\":\"%s\",\"ip\":\"%s\",\"gateway\":\"%s\",\"subnet\":\"%s\","
    "\"sensor_name\":\"%s\",\"sensor_location\":\"%s\",\"sensor_height\":\"%s\","
    "\"speed\":%d,\"env\":%s}",
    getDeviceName().c_str(), getDeviceLocation().c_str(), FW_VERSION,
    getMacString().c_str(),
    getIPString(Ethernet.localIP()).c_str(),
    getIPString(gateway).c_str(),
    getIPString(subnet).c_str(),
    getSensorName().c_str(), getSensorLocation().c_str(), getSensorHeight().c_str(),
    sensorSpeedProfile,
    envConnected ? "true" : "false");

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.write(json, len);
}

void sendApiPage(EthernetClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  const char* ptr = api_html;
  size_t len = strlen_P(api_html);
  char buf[256];
  size_t i = 0;
  while (i < len) {
    size_t chunk = min((size_t)256, len - i);
    memcpy_P(buf, &ptr[i], chunk);
    client.write(buf, chunk);
    i += chunk;
  }
}

void sendUpdatePage(EthernetClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  const char* ptr = update_html;
  size_t len = strlen_P(update_html);
  char buf[256];
  size_t i = 0;
  while (i < len) {
    size_t chunk = min((size_t)256, len - i);
    memcpy_P(buf, &ptr[i], chunk);
    client.write(buf, chunk);
    i += chunk;
  }
}

void send404(EthernetClient& client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("404 Not Found");
}

// Helper to URL decode a string
String urlDecode(String input) {
  input.replace('+', ' ');
  input.replace("%20", " ");
  input.replace("%21", "!");
  input.replace("%27", "'");
  input.replace("%28", "(");
  input.replace("%29", ")");
  input.replace("%2C", ",");
  input.replace("%2F", "/");
  input.replace("%3A", ":");
  return input;
}

// Helper to extract form field value
String getFormValue(const String& body, const char* field) {
  String search = String(field) + "=";
  int idx = body.indexOf(search);
  if (idx < 0) return "";
  int start = idx + search.length();
  int end = body.indexOf('&', start);
  if (end < 0) end = body.length();
  return urlDecode(body.substring(start, end));
}

void handleConfigSave(EthernetClient& client) {
  String body = "";
  unsigned long timeout = millis();

  while (client.connected() && (millis() - timeout < 2000)) {
    if (client.available()) {
      body += (char)client.read();
      timeout = millis();
    }
  }

  // Verify password first
  String pwd = getFormValue(body, "password");
  if (pwd != OTA_PASSWORD) {
    client.println("HTTP/1.1 401 Unauthorized");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Invalid password");
    return;
  }

  // Parse form data
  IPAddress newIP, newGW, newSN;
  bool valid = true;

  String newName = getFormValue(body, "name");
  String newDevLoc = getFormValue(body, "devloc");
  String newSenName = getFormValue(body, "senname");
  String newSenLoc = getFormValue(body, "senloc");
  String newSenHeight = getFormValue(body, "senht");

  int idx = body.indexOf("ip=");
  if (idx >= 0) {
    int end = body.indexOf('&', idx);
    if (end < 0) end = body.length();
    if (!parseIP(body.substring(idx + 3, end), newIP)) valid = false;
  } else valid = false;

  idx = body.indexOf("gw=");
  if (idx >= 0) {
    int end = body.indexOf('&', idx);
    if (end < 0) end = body.length();
    if (!parseIP(body.substring(idx + 3, end), newGW)) valid = false;
  } else valid = false;

  idx = body.indexOf("sn=");
  if (idx >= 0) {
    int end = body.indexOf('&', idx);
    if (end < 0) end = body.length();
    if (!parseIP(body.substring(idx + 3, end), newSN)) valid = false;
  } else valid = false;

  // Parse speed setting
  uint8_t newSpeed = 0;
  idx = body.indexOf("speed=");
  if (idx >= 0) {
    int end = body.indexOf('&', idx);
    if (end < 0) end = body.length();
    newSpeed = body.substring(idx + 6, end).toInt();
    if (newSpeed > 5) newSpeed = 0;
  }

  if (valid) {
    saveConfig(newIP, newGW, newSN, newName.c_str(), newDevLoc.c_str(),
               newSenName.c_str(), newSenLoc.c_str(), newSenHeight.c_str(), newSpeed);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Saved. Rebooting...");
    client.flush();
    client.stop();

    delay(500);
    ESP.restart();
  } else {
    client.println("HTTP/1.1 400 Bad Request");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Invalid configuration");
  }
}

void handleReboot(EthernetClient& client) {
  // Read POST body
  String body = "";
  unsigned long timeout = millis();
  while (client.connected() && (millis() - timeout < 2000)) {
    if (client.available()) {
      body += (char)client.read();
      timeout = millis();
    }
  }

  // Verify password
  String pwd = getFormValue(body, "password");
  if (pwd != OTA_PASSWORD) {
    client.println("HTTP/1.1 401 Unauthorized");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Invalid password");
    return;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("Rebooting...");
  client.flush();
  client.stop();

  delay(500);
  ESP.restart();
}

void handleFactoryReset(EthernetClient& client) {
  // Read POST body
  String body = "";
  unsigned long timeout = millis();
  while (client.connected() && (millis() - timeout < 2000)) {
    if (client.available()) {
      body += (char)client.read();
      timeout = millis();
    }
  }

  // Verify password
  String pwd = getFormValue(body, "password");
  if (pwd != OTA_PASSWORD) {
    client.println("HTTP/1.1 401 Unauthorized");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Invalid password");
    return;
  }

  resetConfig();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("Factory reset complete. Rebooting...");
  client.flush();
  client.stop();

  delay(1000);
  ESP.restart();
}

void handleOTAUpload(EthernetClient& client) {
  // Read multipart form data
  String boundary = "";
  String body = "";
  bool foundPassword = false;
  bool passwordCorrect = false;

  // Find boundary from Content-Type header (already parsed, look in body for boundary marker)
  unsigned long timeout = millis();

  // Read the entire body up to boundary markers
  while (client.connected() && (millis() - timeout < 30000)) {
    if (client.available()) {
      char c = client.read();
      body += c;
      timeout = millis();

      // Look for password field
      if (!foundPassword && body.indexOf("name=\"password\"") > 0) {
        int pwdStart = body.indexOf("\r\n\r\n", body.indexOf("name=\"password\""));
        if (pwdStart > 0) {
          pwdStart += 4;
          int pwdEnd = body.indexOf("\r\n", pwdStart);
          if (pwdEnd > pwdStart) {
            String pwd = body.substring(pwdStart, pwdEnd);
            pwd.trim();
            foundPassword = true;
            passwordCorrect = (pwd == OTA_PASSWORD);

            if (!passwordCorrect) {
              client.println("HTTP/1.1 401 Unauthorized");
              client.println("Content-Type: text/plain");
              client.println("Connection: close");
              client.println();
              client.println("Invalid password");
              return;
            }
          }
        }
      }

      // Look for start of firmware data
      if (passwordCorrect && body.indexOf("name=\"firmware\"") > 0) {
        int fileStart = body.indexOf("\r\n\r\n", body.indexOf("name=\"firmware\""));
        if (fileStart > 0) {
          fileStart += 4;

          // Open LittleFS file for writing
          if (!LittleFS.begin(true)) {
            client.println("HTTP/1.1 500 Internal Server Error");
            client.println("Content-Type: text/plain");
            client.println("Connection: close");
            client.println();
            client.println("LittleFS init failed");
            return;
          }

          File file = LittleFS.open(OTA_FILE, "w");
          if (!file) {
            client.println("HTTP/1.1 500 Internal Server Error");
            client.println("Content-Type: text/plain");
            client.println("Connection: close");
            client.println();
            client.println("Failed to create update file");
            return;
          }

          // Write any remaining data in body buffer
          String remaining = body.substring(fileStart);
          size_t written = 0;

          // Find and remove the trailing boundary
          int boundaryPos = remaining.lastIndexOf("\r\n--");
          if (boundaryPos > 0) {
            remaining = remaining.substring(0, boundaryPos);
          }

          file.write((uint8_t*)remaining.c_str(), remaining.length());
          written += remaining.length();

          // Continue reading remaining data
          uint8_t buf[512];
          String endMarker = "";

          while (client.connected() && (millis() - timeout < 60000)) {
            if (client.available()) {
              size_t bytesRead = client.readBytes(buf, sizeof(buf));
              if (bytesRead > 0) {
                // Check for boundary at end
                endMarker += String((char*)buf, bytesRead);
                if (endMarker.length() > 100) {
                  endMarker = endMarker.substring(endMarker.length() - 100);
                }

                int boundaryPos = endMarker.lastIndexOf("\r\n--");
                if (boundaryPos > 0) {
                  // Found end boundary, write remaining data without it
                  size_t toWrite = bytesRead - (100 - boundaryPos);
                  if (toWrite > 0 && toWrite <= bytesRead) {
                    file.write(buf, toWrite);
                    written += toWrite;
                  }
                  break;
                }

                file.write(buf, bytesRead);
                written += bytesRead;
                timeout = millis();
              }
            }
          }

          file.close();
          Serial.printf("OTA file written: %u bytes\n", written);

          if (written < 100000) {
            LittleFS.remove(OTA_FILE);
            client.println("HTTP/1.1 400 Bad Request");
            client.println("Content-Type: text/plain");
            client.println("Connection: close");
            client.println();
            client.println("File too small");
            return;
          }

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.println();
          client.println("Update received. Rebooting...");
          client.flush();
          client.stop();

          delay(1000);
          ESP.restart();
          return;
        }
      }

      // Limit body size to prevent memory issues
      if (body.length() > 2000) {
        body = body.substring(body.length() - 1000);
      }
    }
  }

  // Timeout or other error
  client.println("HTTP/1.1 400 Bad Request");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("Upload failed");
}
