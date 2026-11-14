/*
  AD8232_ECG_Web - live ECG in the browser, ESP32-C3 Super Mini + AD8232.
  No TFT, no extra libraries (WiFi + WebServer ship with the ESP32 core).

  Arduino IDE: board "ESP32C3 Dev Module", USB CDC On Boot = Enabled.
  Keep this file in a folder with the same name (AD8232_ECG_Web).

  Wiring (AD8232 -> C3 Super Mini)
    3.3V   -> 3V3
    GND    -> GND
    OUTPUT -> GPIO3   (ADC1 pin, safe with Wi-Fi)
    LO+    -> GPIO6
    LO-    -> GPIO7
    SDN    -> 3V3

  Use it: leave WIFI_SSID empty, join Wi-Fi "ECG-Monitor" (pass ecg12345),
  open http://192.168.4.1 . Or fill in your home Wi-Fi and open the IP
  printed on Serial (115200).

  Hobby prototype, NOT a medical device. For electrode safety run it from a
  battery / power bank, not from a laptop that is plugged into the wall.
*/
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ---------------- settings ----------------
const char* WIFI_SSID = "DreamBeast";            // empty = Depends on the network
const char* WIFI_PASS = "66996699";
const char* AP_SSID   = "arcreactor";
const char* AP_PASS   = "arc12345";    // minimum 8 characters

constexpr int   PIN_OUT = 3, PIN_LOP = 6, PIN_LON = 7;
constexpr float FS       = 250.0f;     // samples/s (task period below = 4 ms)
constexpr float MAINS_HZ = 50.0f;      // India = 50

// ---------------- shared data ----------------
constexpr int BUF = 1024;
int16_t buf[BUF];                      // filtered ECG, 0.1 mV-at-pin units
volatile uint32_t head = 0;            // total samples written
volatile int  bpm = 0;
volatile bool leadsOff = true;
WebServer server(80);

// ---------------- 50 Hz notch ----------------
struct Notch {
  float b0, b1, b2, a1, a2, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  void init(float f, float q) {
    float w = 2 * PI * f / FS, c = cosf(w), al = sinf(w) / (2 * q), a0 = 1 + al;
    b0 = 1 / a0; b1 = -2 * c / a0; b2 = 1 / a0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
  }
  void prime(float x) { x1 = x2 = y1 = y2 = x; }   // start at steady state, no transient
  float run(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
} notch;

// ---------------- sampling + heart-rate task ----------------
void sampler(void*) {
  TickType_t t = xTaskGetTickCount();
  bool primed = false, above = false;
  float dc = 0, sm = 0, prev = 0, ie = 0, pk = 0;
  uint32_t n = 0, lastBeat = 0, rr[5] = {0}, rn = 0, ri = 0;

  for (;;) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(4));          // 4 ms = 250 Hz

    if (digitalRead(PIN_LOP) || digitalRead(PIN_LON)) {   // electrode off
      leadsOff = true; bpm = 0; primed = false; above = false;
      pk = ie = 0; lastBeat = 0; rn = ri = 0;
      buf[head % BUF] = 0; head = head + 1;
      continue;
    }
    leadsOff = false;

    uint32_t s = 0;
    for (int i = 0; i < 4; i++) s += analogReadMilliVolts(PIN_OUT);  // 4x average
    float x = s / 4.0f;                                              // mV at the pin
    if (!primed) { notch.prime(x); dc = x; sm = prev = ie = 0; primed = true; }

    float f  = notch.run(x);                 // remove mains hum
    dc += (f - dc) * 0.005f;                 // slow baseline tracker
    float hp = f - dc;                       // baseline-free ECG

    int v = (int)(hp * 10.0f);
    buf[head % BUF] = (int16_t)constrain(v, -32000, 32000);
    head = head + 1;

    // Heart rate: QRS is the steepest part of the ECG, T waves are slow.
    sm += (hp - sm) * 0.35f;                 // light smoothing
    float d = sm - prev; prev = sm;
    ie += (d * d - ie) * 0.1f;               // slope energy
    ++n;
    pk = fmaxf(ie, pk * 0.998f);
    float th = fmaxf(0.4f * pk, 6.0f);
    if (ie < 0.5f * th) above = false;
    if (!above && ie > th && n - lastBeat > 75) {       // 0.3 s refractory
      above = true;
      if (lastBeat) {
        uint32_t dd = n - lastBeat;
        if (dd <= 375) {                                // 40 bpm or faster
          rr[ri] = dd; ri = (ri + 1) % 5; if (rn < 5) rn++;
          float sum = 0; for (uint32_t i = 0; i < rn; i++) sum += rr[i];
          bpm = (int)(60.0f * FS * rn / sum + 0.5f);
        } else { rn = ri = 0; }
      }
      lastBeat = n;
    }
    if (lastBeat && n - lastBeat > 3 * (uint32_t)FS) { bpm = 0; rn = ri = 0; }
  }
}

// ---------------- web page ----------------
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARC Reactor MK-I Live</title><style>
body{margin:0;background:#0b1118;color:#e6edf3;font-family:system-ui,sans-serif}
header{display:flex;justify-content:space-between;align-items:center;padding:12px 16px}
#bpm{font-size:52px;font-weight:700;color:#ff4d5e}
small{color:#8b9aa5}#st{font-weight:700}
canvas{width:100%;height:62vh;display:block}
</style></head><body>
<header><div><small>HEART RATE</small><div><span id="bpm">--</span> <small>BPM</small></div></div>
<div id="st">CONNECTING</div></header>
<canvas id="c"></canvas>
<script>
const cv=document.getElementById('c'),g=cv.getContext('2d'),N=1250;
let d=[],seq=0,lo=-100,hi=100;
function fit(){const r=devicePixelRatio||1;cv.width=cv.clientWidth*r;cv.height=cv.clientHeight*r}
addEventListener('resize',fit);fit();
function draw(){
  const w=cv.width,h=cv.height;g.clearRect(0,0,w,h);
  if(d.length<2)return;
  lo+=(Math.min(...d)-lo)*.1;hi+=(Math.max(...d)-hi)*.1;
  const span=Math.max(hi-lo,200),mid=(hi+lo)/2;
  g.strokeStyle='#00ff69';g.lineWidth=2*(devicePixelRatio||1);g.beginPath();
  d.forEach((v,i)=>{const x=i*w/N,y=h/2-(v-mid)/span*h*.9;i?g.lineTo(x,y):g.moveTo(x,y)});
  g.stroke();
}
function status(t,c){const s=document.getElementById('st');s.textContent=t;s.style.color=c}
async function tick(){
  try{
    const r=await fetch('/data?seq='+seq,{cache:'no-store'}),j=await r.json();
    seq=j.seq;d.push(...j.y);if(d.length>N)d.splice(0,d.length-N);
    document.getElementById('bpm').textContent=j.bpm>0?j.bpm:'--';
    j.off?status('LEADS OFF','#ffcc33'):status('LIVE','#00ff69');
    draw();
  }catch(e){status('OFFLINE','#ff4d5e')}
  setTimeout(tick,100);
}
tick();
</script></body></html>)HTML";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", PAGE);
}

// Browser asks for everything after the last sample number it already has.
void handleData() {
  uint32_t h = head, from = (uint32_t)server.arg("seq").toInt();
  if (from > h || h - from > BUF - 16) from = h > 250 ? h - 250 : 0;   // resync
  uint32_t cnt = h - from;
  if (cnt > 125) cnt = 125;

  String j;
  j.reserve(1100);
  j += "{\"seq\":";  j += (unsigned long)(from + cnt);
  j += ",\"bpm\":";  j += (int)bpm;
  j += ",\"off\":";  j += leadsOff ? 1 : 0;
  j += ",\"y\":[";
  for (uint32_t i = 0; i < cnt; i++) {
    if (i) j += ',';
    j += (int)buf[(from + i) % BUF];
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// ---------------- Wi-Fi ----------------
void startWifi() {
  WiFi.persistent(false);
  if (WIFI_SSID[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);   // many Super Mini clones cannot join without this
    Serial.printf("Joining '%s' ", WIFI_SSID);
    for (int i = 0; i < 80 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print('.'); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Open http://%s\n", WiFi.localIP().toString().c_str());
      return;
    }
    // status 1 = network not found (5 GHz / too far), 4 or 6 = usually wrong password
    Serial.printf("Wi-Fi join failed, status=%d. Networks the C3 can see:\n", (int)WiFi.status());
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++)
      Serial.printf("  %s  (%d dBm, ch %d)\n", WiFi.SSID(i).c_str(), (int)WiFi.RSSI(i), (int)WiFi.channel(i));
    Serial.println("Starting hotspot instead");
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.printf("Join '%s' (pass %s), open http://%s\n", AP_SSID, AP_PASS,
                WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LOP, INPUT);
  pinMode(PIN_LON, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  notch.init(MAINS_HZ, 10.0f);
  startWifi();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  xTaskCreate(sampler, "ecg", 4096, nullptr, 3, nullptr);   // above loop() priority
}

void loop() {
  server.handleClient();
  delay(2);
}
