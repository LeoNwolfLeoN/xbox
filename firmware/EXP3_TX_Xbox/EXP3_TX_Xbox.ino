/*
  EXPERIMENT 3 - TX (Ground side)  v1.2.0
  Wireless Brushless Motor + Control Surfaces via Xbox Controller + Live Web Dashboard
  Board : ESP32 DevKit V1  (Tools > Board > esp32_bluepad32 > ESP32 Dev Module)
          Tools > Partition Scheme > Huge APP (3MB No OTA/1MB SPIFFS)
  Author: Mohammed Isam Yousif

  Xbox pad (Bluetooth) -> ESP32 TX -> Wi-Fi/UDP -> ESP32-S3 RX -> ESC + 2 servos
  RX sends telemetry back (RSSI + echo timestamp -> round-trip latency)
  Dashboard: join Wi-Fi ESP_MOTOR_CTRL (motor123) and open http://192.168.4.1

  Controls:
    RT (right trigger) : throttle 1000..1200 us (lab limit)
    Left stick X       : aileron  servo (roll)
    Left stick Y       : elevator servo (pitch)
    Hold A 1.5 s       : ARM  (only if RT released)   (also ARM button on dashboard)
    B                  : KILL / DISARM                 (also KILL button on dashboard)
    X                  : PRE-FLIGHT CHECK (servo sweep, only when SAFE)
  Haptics: pad rumble grows with throttle while ARMED

  OLED SH1106 1.3" : VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22
*/

#include <Bluepad32.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ---------- Link settings (same as lab sheet) ----------
const char* SSID = "ESP_MOTOR_CTRL";
const char* PASS = "motor123";
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_MASK(255, 255, 255, 0);
IPAddress DHCP_START(192, 168, 4, 10);   // phones get .10+, RX keeps static .2
IPAddress RX_IP(192, 168, 4, 2);
const uint16_t CMD_PORT = 4210;   // TX -> RX commands
const uint16_t TEL_PORT = 4211;   // RX -> TX telemetry

// ---------- Limits ----------
const int THR_MIN = 1000;
const int THR_MAX = 1200;
const int SRV_MIN = 1100;
const int SRV_MID = 1500;
const int SRV_MAX = 1900;
const int DEADZONE = 40;
const unsigned long SEND_MS = 20;  // 50 Hz
const unsigned long LINK_TIMEOUT_MS = 500;

WiFiUDP udp;
WebServer server(80);
Adafruit_SH1106G oled(128, 64, &Wire, -1);
ControllerPtr pad = nullptr;

bool armed = false;
int thr = THR_MIN, ail = SRV_MID, ele = SRV_MID;
uint32_t seq = 0;
unsigned long armHoldStart = 0, lastSend = 0, lastOled = 0, lastTel = 0, lastLog = 0;
long rttMs = -1;
int rxRssi = 0;
bool linkOk = false, prevLinkOk = false;

// Pre-flight check: aileron left/right, elevator up/down, centre
const int PF_STEPS[][2] = {
  {1100, 1500}, {1900, 1500}, {1500, 1500},
  {1500, 1100}, {1500, 1900}, {1500, 1500}
};
const int PF_COUNT = 6;
const unsigned long PF_STEP_MS = 700;
bool preflight = false, prevX = false;
int pfStep = 0;
unsigned long pfStepStart = 0;

// Haptics
unsigned long rumbleBusyUntil = 0, lastThrRumble = 0;

// ---------- Dashboard page ----------
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>EXP3 Ground Station</title><style>
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3}
.w{max-width:480px;margin:auto;padding:12px}
h1{font-size:17px;margin:4px 0 10px;font-weight:600}
.chips{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px}
.c{padding:4px 10px;border-radius:20px;font-size:13px;background:#21262d}
.ok{background:#12391f;color:#56d364}.pf{background:#3d2c00;color:#e3b341}.bad{background:#3d1114;color:#ff7b72}.arm{background:#0c2d6b;color:#79c0ff}
.card{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:10px;margin-bottom:10px}
.sky{height:250px;perspective:600px;display:flex;align-items:center;justify-content:center;
background:linear-gradient(#1b2b44,#0d1117);border-radius:10px;overflow:hidden}
#pl{width:230px;height:230px;transition:transform .08s linear;transform-style:preserve-3d}
.row{display:flex;gap:8px}.row>div{flex:1;text-align:center}
.lbl{font-size:12px;color:#8b949e}.val{font-size:20px;font-weight:600}
.dial{width:70px;height:40px;margin:auto}
.bar{height:120px;width:28px;margin:6px auto;border:1px solid #30363d;border-radius:6px;position:relative;overflow:hidden}
#tb{position:absolute;bottom:0;width:100%;background:#f0883e;height:0}
canvas{width:100%;height:70px}
.btns{display:flex;gap:8px}button{flex:1;padding:14px;font-size:17px;font-weight:600;border:0;border-radius:10px;color:#fff}
#ba{background:#1f6feb}#bk{background:#da3633}
</style></head><body><div class="w">
<h1>Experiment 3 - Ground Station</h1>
<div class="chips"><span id="cp" class="c">PAD</span><span id="cl" class="c">LINK</span><span id="ca" class="c">SAFE</span></div>
<div class="card sky"><svg id="pl" viewBox="0 0 200 200">
<rect x="92" y="30" width="16" height="140" rx="8" fill="#c9d1d9"/>
<rect x="10" y="70" width="180" height="26" rx="5" fill="#8b949e"/>
<rect id="aL" x="12" y="96" width="50" height="9" fill="#f0883e"/>
<rect id="aR" x="138" y="96" width="50" height="9" fill="#f0883e"/>
<rect x="60" y="150" width="80" height="14" rx="3" fill="#8b949e"/>
<rect id="el" x="62" y="164" width="76" height="8" fill="#a371f7"/>
<g id="pr"><rect x="70" y="24" width="60" height="5" rx="2" fill="#e6edf3"/></g>
<circle cx="100" cy="27" r="5" fill="#58a6ff"/></svg></div>
<div class="card row">
<div><div class="lbl">Aileron</div><svg class="dial" viewBox="0 0 70 40"><path d="M5 38A30 30 0 0 1 65 38" fill="none" stroke="#30363d" stroke-width="4"/><line id="na" x1="35" y1="38" x2="35" y2="10" stroke="#f0883e" stroke-width="3"/></svg><div id="va" class="val">1500</div></div>
<div><div class="lbl">Throttle</div><div class="bar"><div id="tb"></div></div><div id="vt" class="val">1000</div></div>
<div><div class="lbl">Elevator</div><svg class="dial" viewBox="0 0 70 40"><path d="M5 38A30 30 0 0 1 65 38" fill="none" stroke="#30363d" stroke-width="4"/><line id="ne" x1="35" y1="38" x2="35" y2="10" stroke="#a371f7" stroke-width="3"/></svg><div id="ve" class="val">1500</div></div>
</div>
<div class="card row"><div><div class="lbl">RTT</div><div id="vr" class="val">--</div></div>
<div><div class="lbl">RSSI</div><div id="vs" class="val">--</div></div></div>
<div class="card"><div class="lbl">Round-trip latency (ms)</div><canvas id="cv" width="440" height="70"></canvas></div>
<div class="btns"><button id="ba" onclick="cmd('arm')">ARM</button><button id="bk" onclick="cmd('kill')">KILL</button></div>
</div><script>
const $=i=>document.getElementById(i);let ang=0,spd=0,hist=[];
function chip(e,on,t,cls){e.textContent=t;e.className='c '+(on?cls:'bad')}
function cmd(c){fetch('/'+c,{method:'POST'})}
function upd(s){
chip($('cp'),s.pad,s.pad?'PAD OK':'NO PAD','ok');chip($('cl'),s.link,s.link?'LINK OK':'LINK LOST','ok');
$('ca').textContent=s.pf?'PRE-FLIGHT CHECK':(s.arm?'ARMED':'SAFE');$('ca').className='c '+(s.pf?'pf':(s.arm?'arm':''));
const da=(s.ail-1500)/400,de=(s.ele-1500)/400,tp=(s.thr-1000)/200;
$('pl').style.transform='rotateX('+(de*25)+'deg) rotateY('+(da*35)+'deg)';
$('aL').setAttribute('transform','translate(0 '+(da*6)+')');$('aR').setAttribute('transform','translate(0 '+(-da*6)+')');
$('el').setAttribute('transform','translate(0 '+(de*6)+')');
$('na').setAttribute('transform','rotate('+(da*60)+' 35 38)');$('ne').setAttribute('transform','rotate('+(de*60)+' 35 38)');
$('va').textContent=s.ail;$('ve').textContent=s.ele;$('vt').textContent=s.thr;$('tb').style.height=(tp*100)+'%';
spd=s.arm?tp*45:0;
$('vr').textContent=s.link?s.rtt+' ms':'--';$('vs').textContent=s.link?s.rssi+' dBm':'--';
hist.push(s.link?s.rtt:0);if(hist.length>80)hist.shift();draw()}
function draw(){const c=$('cv'),x=c.getContext('2d'),m=Math.max(20,...hist);x.clearRect(0,0,440,70);
x.strokeStyle='#58a6ff';x.lineWidth=2;x.beginPath();hist.forEach((v,i)=>{const px=i*440/79,py=66-v/m*60;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
x.fillStyle='#8b949e';x.font='11px sans-serif';x.fillText('max '+m+' ms',4,12)}
function spin(){ang=(ang+spd)%360;$('pr').setAttribute('transform','rotate('+ang+' 100 27)');requestAnimationFrame(spin)}
async function poll(){try{const r=await fetch('/s');upd(await r.json())}catch(e){chip($('cl'),0,'NO TX','ok')}setTimeout(poll,80)}
spin();poll();
</script></body></html>)HTML";

// ---------- Pad callbacks ----------
void rumble(int ms, uint8_t power = 0xC0) {
  if (pad && pad->isConnected()) {
    pad->playDualRumble(0, ms, power, power);
    rumbleBusyUntil = millis() + ms;
  }
}

void onConnected(ControllerPtr c) {
  if (pad == nullptr) {
    pad = c;
    Serial.printf("Pad connected: %s\n", c->getModelName().c_str());
    rumble(200, 0x80);
  }
}

void onDisconnected(ControllerPtr c) {
  if (pad == c) {
    pad = nullptr;
    armed = false;
    Serial.println("Pad lost -> DISARMED");
  }
}

// ---------- Helpers ----------
int stickToUs(int v, bool invert) {
  if (abs(v) < DEADZONE) return SRV_MID;
  if (invert) v = -v;
  return constrain(map(v, -512, 511, SRV_MIN, SRV_MAX), SRV_MIN, SRV_MAX);
}

void readPad() {
  if (pad == nullptr || !pad->isConnected()) {
    armed = false;
    preflight = false;
    thr = THR_MIN;
    ail = ele = SRV_MID;
    return;
  }

  int rt = pad->throttle();                 // 0..1023
  thr = map(rt, 0, 1023, THR_MIN, THR_MAX);
  ail = stickToUs(pad->axisX(), false);
  ele = stickToUs(pad->axisY(), true);

  // PRE-FLIGHT CHECK on X (rising edge, only when SAFE)
  bool x = pad->x();
  if (x && !prevX && !armed && !preflight) {
    preflight = true;
    pfStep = 0;
    pfStepStart = millis();
    rumble(120, 0x60);
    Serial.println("PRE-FLIGHT CHECK started");
  }
  prevX = x;
  if (preflight) {
    if (pad->b()) {
      preflight = false;
      Serial.println("PRE-FLIGHT CHECK aborted");
    } else {
      if (millis() - pfStepStart >= PF_STEP_MS) {
        pfStep++;
        pfStepStart = millis();
      }
      if (pfStep >= PF_COUNT) {
        preflight = false;
        rumble(300, 0x90);
        Serial.println("PRE-FLIGHT CHECK done");
      } else {
        ail = PF_STEPS[pfStep][0];
        ele = PF_STEPS[pfStep][1];
        thr = THR_MIN;
        return;                      // no arming during the check
      }
    }
  }

  if (!armed) {
    if (pad->a() && rt < 30) {
      if (armHoldStart == 0) armHoldStart = millis();
      else if (millis() - armHoldStart > 1500) {
        armed = true;
        armHoldStart = 0;
        rumble(400);
        Serial.println("ARMED");
      }
    } else {
      armHoldStart = 0;
    }
  }

  if (armed && pad->b()) {
    armed = false;
    rumble(150);
    Serial.println("KILL -> DISARMED");
  }

  if (!armed) thr = THR_MIN;
}

void sendCommand() {
  char msg[64];
  int n = snprintf(msg, sizeof(msg), "C,%lu,%lu,%d,%d,%d,%d",
                   (unsigned long)seq++, millis(), thr, ail, ele, armed ? 1 : 0);
  udp.beginPacket(RX_IP, CMD_PORT);
  udp.write((const uint8_t*)msg, n);
  udp.endPacket();
}

void readTelemetry() {
  int size = udp.parsePacket();
  if (size <= 0) return;
  char buf[64];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = '\0';
  unsigned long ts; int rssi;
  if (sscanf(buf, "T,%lu,%d", &ts, &rssi) == 2) {
    rttMs = (long)(millis() - ts);
    rxRssi = rssi;
    lastTel = millis();
  }
}

void drawOled() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("PAD:");
  oled.print(pad ? "OK " : "-- ");
  oled.print(" LINK:");
  oled.print(linkOk ? "OK" : "LOST");

  oled.setTextSize(2);
  oled.setCursor(0, 12);
  oled.print(preflight ? "CHECK" : (armed ? "ARMED" : "SAFE"));

  oled.setTextSize(1);
  oled.setCursor(0, 31);
  oled.printf("AIL %d  ELE %d", ail, ele);
  oled.setCursor(0, 41);
  if (linkOk) oled.printf("RTT %ldms RSSI %d", rttMs, rxRssi);
  else oled.print("RTT --  RSSI --");

  oled.setCursor(0, 54);
  oled.printf("%d", thr);
  int w = map(thr, THR_MIN, THR_MAX, 0, 98);
  oled.drawRect(28, 54, 100, 9, SH110X_WHITE);
  oled.fillRect(29, 55, w, 7, SH110X_WHITE);
  oled.display();
}

// ---------- Web handlers ----------
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleState() {
  char j[160];
  snprintf(j, sizeof(j),
           "{\"pad\":%d,\"link\":%d,\"arm\":%d,\"thr\":%d,\"ail\":%d,\"ele\":%d,\"rtt\":%ld,\"rssi\":%d,\"pf\":%d}",
           pad ? 1 : 0, linkOk ? 1 : 0, armed ? 1 : 0, thr, ail, ele, rttMs, rxRssi, preflight ? 1 : 0);
  server.send(200, "application/json", j);
}

void handleArm() {
  bool ok = false;
  if (!armed && !preflight && pad && pad->isConnected() && pad->throttle() < 30) {
    armed = true;
    rumble(400);
    ok = true;
    Serial.println("ARMED (dashboard)");
  }
  server.send(200, "text/plain", ok ? "OK" : "NO");
}

void handleKill() {
  armed = false;
  rumble(150);
  Serial.println("KILL (dashboard)");
  server.send(200, "text/plain", "OK");
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  oled.begin(0x3C, true);
  oled.clearDisplay();
  oled.setTextColor(SH110X_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("EXP3 TX v1.2.0");
  oled.println("Starting AP...");
  oled.display();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASS, 1, false, 4);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK, DHCP_START);
  udp.begin(TEL_PORT);
  Serial.printf("AP %s ready, IP %s\n", SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", handleRoot);
  server.on("/s", handleState);
  server.on("/arm", HTTP_POST, handleArm);
  server.on("/kill", HTTP_POST, handleKill);
  server.begin();
  Serial.println("Dashboard: http://192.168.4.1");

  BP32.setup(&onConnected, &onDisconnected);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  Serial.println("Press the Xbox pair button now...");
}

void loop() {
  BP32.update();
  readPad();
  readTelemetry();

  linkOk = (lastTel != 0) && (millis() - lastTel < LINK_TIMEOUT_MS);
  if (prevLinkOk && !linkOk) {
    Serial.println("LINK LOST -> RX failsafe");
    rumble(600, 0xFF);
  }
  prevLinkOk = linkOk;

  // Throttle haptics: rumble strength follows throttle while ARMED
  if (armed && pad && thr > THR_MIN + 5 && millis() > rumbleBusyUntil &&
      millis() - lastThrRumble >= 100) {
    lastThrRumble = millis();
    uint8_t power = map(thr, THR_MIN, THR_MAX, 25, 255);
    pad->playDualRumble(0, 150, power, power / 2);
  }

  if (millis() - lastSend >= SEND_MS) {
    lastSend = millis();
    sendCommand();
  }
  server.handleClient();
  if (millis() - lastOled >= 100) {
    lastOled = millis();
    drawOled();
  }
  if (millis() - lastLog >= 500) {
    lastLog = millis();
    Serial.printf("ARM=%d THR=%d AIL=%d ELE=%d LINK=%d RTT=%ld RSSI=%d\n",
                  armed, thr, ail, ele, linkOk, rttMs, rxRssi);
  }
  delay(1);
}
