/*
  ARC_Reactor_MK1 - ESP32-C3 Super Mini telemetry dashboard.
  AD8232 ECG + BPM, MPU6050 steps + body orientation, touch on/off, blue LED.
  No extra libraries.

  Arduino IDE: board "ESP32C3 Dev Module", USB CDC On Boot = Enabled.
  Keep this file in a folder with the same name (ARC_Reactor_MK1).
  Home Wi-Fi: copy secrets.example.h to secrets.h and fill in your Wi-Fi name + password.

  Wiring
    AD8232 : 3.3V->3V3  GND->GND  OUTPUT->GPIO3  LO+->GPIO6  LO-->GPIO7  SDN->3V3
    MPU6050: VCC->3V3   GND->GND  SDA->GPIO4     SCL->GPIO5  AD0->GND
    TTP223 touch module: VCC->3V3  GND->GND  I/O->GPIO1
    Blue pixel LED (WS2812 / NeoPixel): VCC->5V, GND->GND, DIN->330 ohm->GPIO10

  Touch pad: double tap = system ON (blue LED lights).
             double tap again = LED off + deep sleep. Double tap to wake.
  Power-up / reset starts the system ON (START_ON_POWER_UP below).

  Hobby prototype, NOT a medical device. Run from a battery / power bank
  while electrodes are on you.
*/
#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_sleep.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ---------------- settings ----------------
// Home Wi-Fi lives in secrets.h (copy secrets.example.h, fill it in; it is git-ignored).
// Without that file the board simply makes its own hotspot.
#if __has_include("secrets.h")
  #include "secrets.h"                 // defines WIFI_SSID and WIFI_PASS
#else
  const char* WIFI_SSID = "";          // empty = make our own hotspot
  const char* WIFI_PASS = "";
#endif


constexpr int   PIN_OUT = 3, PIN_LOP = 6, PIN_LON = 7;   // AD8232
constexpr int   PIN_SDA = 4, PIN_SCL = 5;                // MPU6050
constexpr int   PIN_TOUCH = 1;         // must be GPIO0..5 to wake from deep sleep
constexpr int   PIN_LED = 10;          // DIN of the blue pixel LED (WS2812 / NeoPixel)
constexpr uint8_t LED_BLUE = 40;       // pixel brightness 0..255 (255 is blinding)
// Core 3.x calls it rgbLedWrite, core 2.x calls it neopixelWrite.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LED_SET(r, g, b) rgbLedWrite(PIN_LED, r, g, b)
#else
  #define LED_SET(r, g, b) neopixelWrite(PIN_LED, r, g, b)
#endif
constexpr int   MPU_ADDR = 0x68;       // 0x69 if AD0 is wired to 3V3
constexpr float FS       = 250.0f;     // ECG samples/s (task period = 4 ms)
constexpr float MAINS_HZ = 50.0f;      // India = 50

constexpr bool     START_ON_POWER_UP = true;   // false: after power-up wait for a double tap
constexpr uint32_t DOUBLE_TAP_MS  = 700;       // max time between the two taps
constexpr uint32_t WAKE_WINDOW_MS = 1500;      // after wake, time allowed for the 2nd tap

constexpr float STEP_THRESH_G = 0.06f; // step sensitivity: lower = more sensitive
constexpr float STEP_REARM_G  = 0.025f;
constexpr float POSE_AXIS_MIN = 0.70f; // how close to an axis counts as that pose (~45 deg)

// ---------------- shared data ----------------
constexpr int BUF = 1024;
int16_t buf[BUF];                      // filtered ECG, 0.1 mV-at-pin units
volatile uint32_t head = 0;            // total ECG samples written
volatile int  bpm = 0;
volatile bool leadsOff = true;

enum Pose : uint8_t { P_UNKNOWN, P_UPRIGHT, P_BACK, P_STOMACH, P_UPSIDE, P_SIDE, P_TILT };
volatile bool     imuOk = false;
volatile uint32_t steps = 0;
volatile uint8_t  pose = P_UNKNOWN;
volatile bool     walking = false;
volatile float    gx = 0, gy = 1, gz = 0;   // gravity direction (unit vector, sensor axes)
volatile int      pitchDeg = 0, rollDeg = 0;
volatile bool     sleeping = false;

RTC_DATA_ATTR uint32_t stepsRtc = 0;        // survives deep sleep
bool imuStarted = false;
uint32_t activatedAt = 0;
httpd_handle_t web = nullptr;
Preferences prefs;

// ---------------- small vector helpers ----------------
struct V3 { float x, y, z; };
static inline float v_dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 v_cross(V3 a, V3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static inline V3 v_sub(V3 a, V3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline V3 v_add(V3 a, V3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline V3 v_scale(V3 a, float k) { return { a.x * k, a.y * k, a.z * k }; }
static V3 v_unit(V3 a) {
  float n = sqrtf(v_dot(a, a));
  return n < 1e-6f ? V3{0, 1, 0} : v_scale(a, 1.0f / n);
}

// ---------------- orientation calibration ----------------
// U = body "up" (head direction), F = body "front" (out of the chest), both in sensor axes.
struct Cal { V3 U, F; uint32_t flags; };           // flags: bit0 = UP set, bit1 = FRONT set
Cal cal = { {0, 1, 0}, {0, 0, 1}, 0 };             // defaults: Y toward head, Z out of chest
V3 bU = {0, 1, 0}, bF = {0, 0, 1}, bS = {1, 0, 0}; // orthonormal body basis

void rebuildBasis() {
  V3 u = v_unit(cal.U);
  V3 f = v_sub(cal.F, v_scale(u, v_dot(cal.F, u)));
  if (v_dot(f, f) < 0.09f) {                       // FRONT ~ parallel to UP: pick any perpendicular
    V3 alt = fabsf(u.x) < 0.9f ? V3{1, 0, 0} : V3{0, 0, 1};
    f = v_sub(alt, v_scale(u, v_dot(alt, u)));
  }
  bU = u; bF = v_unit(f); bS = v_cross(bU, bF);
}
void loadCal() {
  prefs.begin("imu", false);
  if (prefs.getBytesLength("cal") == sizeof(Cal)) prefs.getBytes("cal", &cal, sizeof(Cal));
  prefs.end();
  rebuildBasis();
}
void saveCal() {
  prefs.begin("imu", false);
  prefs.putBytes("cal", &cal, sizeof(Cal));
  prefs.end();
}
uint8_t classify(V3 g) {
  float u = v_dot(g, bU), f = v_dot(g, bF), s = v_dot(g, bS);
  if (u >  POSE_AXIS_MIN) return P_UPRIGHT;
  if (u < -POSE_AXIS_MIN) return P_UPSIDE;
  if (f >  POSE_AXIS_MIN) return P_BACK;
  if (f < -POSE_AXIS_MIN) return P_STOMACH;
  if (fabsf(s) > POSE_AXIS_MIN) return P_SIDE;
  return P_TILT;
}

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

// ---------------- ECG sampling + heart-rate task ----------------
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

// ---------------- MPU6050 ----------------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpuReadBytes(uint8_t reg, uint8_t* out, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (int)n, 1) != n) return false;
  for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}
bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  uint8_t who = 0;
  if (!mpuReadBytes(0x75, &who, 1)) return false;
  Serial.printf("MPU WHO_AM_I = 0x%02X (0x68 on a genuine MPU6050)\n", who);
  bool ok = mpuWrite(0x6B, 0x00)      // wake up
         && mpuWrite(0x1A, 0x03)      // low-pass ~44 Hz
         && mpuWrite(0x1C, 0x08);     // +-4 g  (8192 LSB per g)
  delay(100);
  return ok;
}

// 50 Hz: gravity direction -> posture, acceleration bumps -> steps.
void imuTask(void*) {
  TickType_t t = xTaskGetTickCount();
  V3 gl = {0, 1, 0};
  float mavg = 1.0f, ds = 0;
  bool armed = true, first = true;
  uint32_t lastStep = 0, seq = 0, candN = 0;
  uint8_t cand = P_UNKNOWN;

  for (;;) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));            // 20 ms = 50 Hz
    if (sleeping) continue;                            // going to deep sleep: leave I2C alone
    uint8_t b[6];
    if (!mpuReadBytes(0x3B, b, 6)) { imuOk = false; continue; }
    imuOk = true;
    V3 a = { (int16_t)((b[0] << 8) | b[1]) / 8192.0f,
             (int16_t)((b[2] << 8) | b[3]) / 8192.0f,
             (int16_t)((b[4] << 8) | b[5]) / 8192.0f };      // in g
    if (first) { gl = a; first = false; }
    gl = v_add(gl, v_scale(v_sub(a, gl), 0.1f));             // slow average = gravity
    V3 gu = v_unit(gl);
    gx = gu.x; gy = gu.y; gz = gu.z;

    float u = v_dot(gu, bU), f = v_dot(gu, bF), s = v_dot(gu, bS);
    pitchDeg = (int)lroundf(atan2f(f, u) * 57.2958f);        // + = leaning back
    rollDeg  = (int)lroundf(atan2f(s, u) * 57.2958f);

    // Posture: must stay the same for ~0.8 s before the display changes.
    uint8_t p = classify(gu);
    if (p == cand) { if (candN < 1000) candN++; } else { cand = p; candN = 1; }
    if (candN >= 40) pose = cand;

    // Steps: bumps in total acceleration. Needs 5 steps in a row to start
    // counting, so fidgeting and arm waves do not add steps.
    float m = sqrtf(v_dot(a, a));
    mavg += (m - mavg) * 0.02f;
    ds += ((m - mavg) - ds) * 0.35f;
    uint32_t now = millis();
    if (ds < STEP_REARM_G) armed = true;
    if (armed && ds > STEP_THRESH_G && now - lastStep > 250) {
      armed = false;
      if (now - lastStep > 1500) seq = 0;
      seq++; lastStep = now;
      if (seq == 5) steps = steps + 5; else if (seq > 5) steps = steps + 1;
    }
    walking = (seq >= 5) && (now - lastStep < 2000);
  }
}

// ---------------- web page ----------------
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARC REACTOR MK-I</title><style>
:root{--line:#103a5a;--cy:#4cc9f0;--tx:#d9ecff;--mut:#6f8ea8;--red:#ff5a6e;--grn:#2de08a;--amb:#ffc247}
*{box-sizing:border-box}
body{margin:0;min-height:100vh;color:var(--tx);font-family:system-ui,Segoe UI,Roboto,sans-serif;background:radial-gradient(1200px 600px at 50% -10%,#0b2238 0,#030810 62%) #030810}
.mono,.chip,h2,.lab,summary,.sys,button{font-family:ui-monospace,SFMono-Regular,Consolas,Menlo,monospace}
header{display:flex;flex-wrap:wrap;align-items:center;gap:14px;padding:14px 18px;border-bottom:1px solid var(--line)}
h1{margin:0;font-size:clamp(16px,3.4vw,24px);letter-spacing:.06em}h1 span{color:var(--cy);font-weight:400}
.sys{display:flex;align-items:center;gap:8px;font-size:12px;letter-spacing:.12em;color:var(--grn)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--grn);box-shadow:0 0 10px var(--grn)}
.sys.off{color:var(--red)}.sys.off .dot{background:var(--red);box-shadow:0 0 10px var(--red)}
.chips{margin-left:auto;display:flex;gap:8px;flex-wrap:wrap}
.chip{border:1px solid var(--line);background:#06131f;border-radius:6px;padding:7px 10px;font-size:12px;color:var(--cy);letter-spacing:.08em}
.chip.warn{color:var(--amb);border-color:#6b5320}
main{padding:14px 18px 28px;display:grid;gap:14px}
.panel{background:rgba(7,18,31,.88);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
h2{margin:0 0 10px;font-size:12px;letter-spacing:.16em;color:var(--cy)}
.top{display:flex;justify-content:space-between;align-items:flex-start;gap:12px;flex-wrap:wrap}
.hr{display:flex;align-items:baseline;gap:8px}.lab{font-size:11px;letter-spacing:.14em;color:var(--mut)}
#bpm{font-size:clamp(40px,8vw,64px);font-weight:700;color:var(--red);line-height:1}
canvas{width:100%;height:clamp(260px,46vh,520px);display:block;border-radius:8px;background:#040d17}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:14px}
.big{font-size:clamp(26px,5vw,38px);font-weight:700;line-height:1.1;color:var(--cy)}
.sub{color:var(--mut);font-size:13px;min-height:18px;margin-top:4px}
.row2{display:flex;align-items:center;justify-content:space-between;gap:10px}
.pr{display:flex;gap:26px;margin-bottom:10px}.pr b{display:block;font-size:26px}
.ax{display:flex;align-items:center;gap:8px;margin:7px 0;font-size:12px}
.ax span{width:12px;color:var(--mut)}.ax b{width:44px;text-align:right;font-weight:600}
.trk{position:relative;flex:1;height:8px;background:#0a1a2a;border-radius:4px}
.trk:after{content:"";position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:#244a68}
.trk i{position:absolute;top:0;bottom:0;background:var(--cy);border-radius:4px}
.kv{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #0d2a42;font-size:13px}
.kv span{color:var(--mut)}.kv:last-child{border:0}
.bars{display:inline-flex;gap:3px;align-items:flex-end;height:16px;margin-left:8px}
.bars i{width:4px;background:var(--grn);opacity:.2}.bars i:nth-child(1){height:4px}.bars i:nth-child(2){height:7px}.bars i:nth-child(3){height:10px}.bars i:nth-child(4){height:13px}.bars i:nth-child(5){height:16px}
button{background:#0a2236;color:var(--cy);border:1px solid var(--line);border-radius:6px;padding:8px 12px;margin:6px 6px 0 0;font-size:12px;letter-spacing:.08em}
details summary{cursor:pointer;color:var(--cy);font-size:12px;letter-spacing:.16em}
#hint{margin-top:10px;color:var(--amb);font-size:13px}
</style></head><body>
<header><h1>ARC REACTOR MK-I <span>// TELEMETRY SYSTEM</span></h1>
<div class="sys off" id="sys"><i class="dot"></i><span id="systxt">CONNECTING</span></div>
<div class="chips"><div class="chip" id="cLink">LINK --</div><div class="chip" id="cLeads">LEADS --</div><div class="chip" id="cImu">IMU --</div></div></header>
<main>
<section class="panel"><div class="top"><h2>ECG LIVE</h2>
<div class="hr"><span class="lab">HEART RATE</span><span id="bpm">--</span><span class="lab">BPM</span></div></div>
<canvas id="c"></canvas></section>
<div class="grid">
<section class="panel"><h2>STEPS</h2><div class="big" id="steps">--</div><div class="sub" id="walk"></div>
<button onclick="cmd('resetsteps')">RESET STEPS</button></section>
<section class="panel"><h2>POSTURE</h2><div class="row2"><div><div class="big" id="pose">--</div><div class="sub" id="psub"></div></div>
<svg viewBox="0 0 100 100" width="84" height="84"><g id="fig" style="transform-origin:50px 50px;transition:transform .4s" fill="#7fa6c4">
<circle cx="50" cy="20" r="9"/><rect x="41" y="31" width="18" height="38" rx="8"/><rect x="42" y="68" width="7" height="24" rx="3"/><rect x="51" y="68" width="7" height="24" rx="3"/>
<circle id="chest" cx="50" cy="44" r="4" fill="#ff5a6e"/></g></svg></div></section>
<section class="panel"><h2>IMU ORIENTATION</h2>
<div class="pr"><div><span class="lab">PITCH</span><b id="pitch">--</b></div><div><span class="lab">ROLL</span><b id="roll">--</b></div></div>
<div class="ax mono"><span>X</span><div class="trk"><i id="bx"></i></div><b id="vx">0.00</b></div>
<div class="ax mono"><span>Y</span><div class="trk"><i id="by"></i></div><b id="vy">0.00</b></div>
<div class="ax mono"><span>Z</span><div class="trk"><i id="bz"></i></div><b id="vz">0.00</b></div>
<div class="sub" id="calst"></div></section>
<section class="panel"><h2>DATA STREAM</h2>
<div class="kv"><span>PACKETS RECEIVED</span><b id="pk">0</b></div>
<div class="kv"><span>ERRORS</span><b id="er">0</b></div>
<div class="kv"><span>SIGNAL</span><b><span id="sig">--</span><span class="bars" id="bars"><i></i><i></i><i></i><i></i><i></i></span></b></div>
<div class="kv"><span>DISPLAY DELAY</span><b id="delay">--</b></div></section>
</div>
<details class="panel"><summary>ORIENTATION SETUP (ONE TIME)</summary>
<button onclick="cal('up','Stand straight and still')">1. STAND STRAIGHT</button>
<button onclick="cal('front','Lie flat on your back')">2. LIE ON BACK</button>
<button onclick="cmd('resetcal')">RESET</button>
<div id="hint">Press a button, then take that pose within 5 seconds and hold still. Double tap the touch pad to sleep the system.</div></details>
</main>
<script>
const $=id=>document.getElementById(id),cv=$('c'),g=cv.getContext('2d');
const FSR=250,N=1250,TARGET=125;
let q=[],disp=[],seq=0,playing=false,acc=0,lastT=0,lo=-100,hi=100,ok=0,err=0,lastOk=0,dpr=1;
function fit(){dpr=devicePixelRatio||1;cv.width=Math.round(cv.clientWidth*dpr);cv.height=Math.round(cv.clientHeight*dpr)}
addEventListener('resize',fit);fit();
function draw(){
  const w=cv.width,h=cv.height;g.clearRect(0,0,w,h);
  g.lineWidth=1;g.strokeStyle='#0d2a42';g.beginPath();
  const sx=w*50/N;for(let x=0;x<w;x+=sx){g.moveTo(x,0);g.lineTo(x,h)}
  for(let i=1;i<8;i++){g.moveTo(0,i*h/8);g.lineTo(w,i*h/8)}
  g.stroke();
  if(disp.length<2)return;
  let mn=1e9,mx=-1e9;for(const v of disp){if(v<mn)mn=v;if(v>mx)mx=v}
  lo+=(mn-lo)*.05;hi+=(mx-hi)*.05;
  const span=Math.max(hi-lo,200),mid=(hi+lo)/2,k=h*.82/span,x0=w-disp.length*w/N,p=new Path2D();
  for(let i=0;i<disp.length;i++){const x=x0+i*w/N,y=h/2-(disp[i]-mid)*k;i?p.lineTo(x,y):p.moveTo(x,y)}
  g.lineJoin='round';g.strokeStyle='rgba(255,90,110,.22)';g.lineWidth=7*dpr;g.stroke(p);
  g.strokeStyle='#ff5a6e';g.lineWidth=2*dpr;g.stroke(p);
}
// Smooth playback: samples arrive in bursts, but are shown at a steady 250/s from a small buffer.
function frame(t){
  requestAnimationFrame(frame);
  const dt=Math.min(100,t-(lastT||t));lastT=t;
  if(!playing&&q.length>=TARGET)playing=true;
  if(playing){
    if(q.length>TARGET*6)q.splice(0,q.length-TARGET);
    const e=(q.length-TARGET)/TARGET;
    acc+=dt*FSR/1000*(1+Math.max(-.2,Math.min(.5,e*.25)));
    const n=Math.floor(acc);acc-=n;
    if(n>0&&q.length){disp.push(...q.splice(0,n));if(disp.length>N)disp.splice(0,disp.length-N)}
  }
  draw();
}
requestAnimationFrame(frame);
const POSE=['DETECTING','UPRIGHT','LYING ON BACK','LYING ON STOMACH','UPSIDE DOWN','LYING ON SIDE','TILTED'];
const ANG=[0,0,90,90,180,90,0];
function axis(id,v){const a=Math.min(1,Math.abs(v)),i=$('b'+id);i.style.left=(v>=0?50:50-a*50)+'%';i.style.width=(a*50)+'%';$('v'+id).textContent=v.toFixed(2)}
function ui(j){
  $('bpm').textContent=j.bpm>0?j.bpm:'--';
  $('cLeads').textContent=j.off?'LEADS OFF':'LEADS OK';$('cLeads').className='chip'+(j.off?' warn':'');
  $('cLink').textContent=j.rssi?'WI-FI '+j.rssi+' dBm':'HOTSPOT';
  $('sig').textContent=j.rssi?j.rssi+' dBm':'HOTSPOT';
  const nb=!j.rssi?5:j.rssi>-55?5:j.rssi>-65?4:j.rssi>-72?3:j.rssi>-80?2:1;
  [...$('bars').children].forEach((b,i)=>b.style.opacity=i<nb?1:.2);
  $('cImu').textContent=j.imu?'IMU OK':'IMU OFFLINE';$('cImu').className='chip'+(j.imu?'':' warn');
  if(!j.imu){$('steps').textContent='--';$('pose').textContent='IMU OFFLINE';$('psub').textContent='check MPU6050 wiring';return}
  $('steps').textContent=j.steps.toLocaleString();
  const p=j.pose;let label=POSE[p],sub=p==1?'standing or sitting':'';
  if(p==1&&j.walk){label='WALKING';sub='upright, steps detected'}
  $('walk').textContent=j.walk?'WALKING':'';
  $('pose').textContent=label;$('psub').textContent=sub;
  $('fig').style.transform='rotate('+ANG[p]+'deg)';
  $('chest').style.opacity=p==3?0:(p==5?.5:1);
  $('pitch').textContent=j.pr[0]+'\u00b0';$('roll').textContent=j.pr[1]+'\u00b0';
  axis('x',j.g[0]);axis('y',j.g[1]);axis('z',j.g[2]);
  $('calst').textContent=(j.cal&3)==3?'axes calibrated':'default axes - calibrate below';
}
async function poll(){
  const ctl=new AbortController(),to=setTimeout(()=>ctl.abort(),1500);
  try{
    const r=await fetch('/data?seq='+seq,{cache:'no-store',signal:ctl.signal}),j=await r.json();
    clearTimeout(to);seq=j.seq;q.push(...j.y);ok++;lastOk=performance.now();ui(j);
  }catch(e){clearTimeout(to);err++}
  setTimeout(poll,110);
}
poll();
setInterval(()=>{
  const on=performance.now()-lastOk<2000;
  $('sys').className='sys'+(on?'':' off');
  $('systxt').textContent=on?'SYSTEM ONLINE':(ok?'SYSTEM OFFLINE':'CONNECTING');
  $('pk').textContent=ok;$('er').textContent=err;$('delay').textContent=Math.round(q.length*1000/FSR)+' ms';
},300);
function cmd(c){return fetch('/cmd?c='+c)}
function cal(c,msg){let n=5;const iv=setInterval(()=>{
  if(n>0){$('hint').textContent=msg+' - capturing in '+n+'s';n--}
  else{clearInterval(iv);cmd(c).then(()=>$('hint').textContent='Saved.').catch(()=>$('hint').textContent='Failed, try again')}
},1000)}
</script></body></html>)HTML";

// ---------------- web server (ESP-IDF httpd: own task, keep-alive, never blocks on idle sockets) ----------------
static bool qval(httpd_req_t* req, const char* key, char* out, size_t n) {
  char q[64];
  if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK) return false;
  return httpd_query_key_value(q, key, out, n) == ESP_OK;
}

static esp_err_t hRoot(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

// Browser asks for everything after the last ECG sample number it already has.
static esp_err_t hData(httpd_req_t* req) {
  char v[16] = {0};
  uint32_t h = head;
  uint32_t from = qval(req, "seq", v, sizeof v) ? (uint32_t)strtoul(v, nullptr, 10) : 0;
  if (from > h || h - from > BUF - 16) from = h > 250 ? h - 250 : 0;   // resync
  uint32_t cnt = h - from;
  if (cnt > 125) cnt = 125;

  String j;
  j.reserve(1500);
  j += "{\"seq\":";   j += (unsigned long)(from + cnt);
  j += ",\"bpm\":";   j += (int)bpm;
  j += ",\"off\":";   j += leadsOff ? 1 : 0;
  j += ",\"imu\":";   j += imuOk ? 1 : 0;
  j += ",\"steps\":"; j += (unsigned long)steps;
  j += ",\"pose\":";  j += (int)pose;
  j += ",\"walk\":";  j += walking ? 1 : 0;
  j += ",\"cal\":";   j += (unsigned long)cal.flags;
  j += ",\"rssi\":";  j += (WiFi.status() == WL_CONNECTED) ? (int)WiFi.RSSI() : 0;
  j += ",\"pr\":[";   j += (int)pitchDeg; j += ','; j += (int)rollDeg;
  j += "],\"g\":[";   j += String((float)gx, 2); j += ',';
  j += String((float)gy, 2); j += ','; j += String((float)gz, 2);
  j += "],\"y\":[";
  for (uint32_t i = 0; i < cnt; i++) {
    if (i) j += ',';
    j += (int)buf[(from + i) % BUF];
  }
  j += "]}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, j.c_str(), j.length());
}

// Buttons on the page: set orientation reference poses, reset steps.
static esp_err_t hCmd(httpd_req_t* req) {
  char c[16] = {0};
  if (!qval(req, "c", c, sizeof c)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing c");
    return ESP_FAIL;
  }
  V3 g = { gx, gy, gz };
  bool calChanged = true;
  if      (!strcmp(c, "up"))         { cal.U = g; cal.flags |= 1; }
  else if (!strcmp(c, "front"))      { cal.F = g; cal.flags |= 2; }
  else if (!strcmp(c, "resetcal"))   { cal = Cal{ {0, 1, 0}, {0, 0, 1}, 0 }; }
  else if (!strcmp(c, "resetsteps")) { steps = 0; stepsRtc = 0; calChanged = false; }
  else { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad command"); return ESP_FAIL; }
  if (calChanged) { rebuildBasis(); saveCal(); }
  return httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
}

void startWeb() {
  httpd_config_t c = HTTPD_DEFAULT_CONFIG();
  c.stack_size = 8192;
  c.task_priority = 2;               // below the ECG sampler (3)
  c.max_open_sockets = 5;
  c.lru_purge_enable = true;         // drop stale browser sockets instead of refusing new ones
  c.recv_wait_timeout = 3;
  c.send_wait_timeout = 3;
  if (httpd_start(&web, &c) != ESP_OK) { Serial.println("Web server failed to start"); return; }
  const httpd_uri_t routes[] = {
    { "/",     HTTP_GET, hRoot, nullptr },
    { "/data", HTTP_GET, hData, nullptr },
    { "/cmd",  HTTP_GET, hCmd,  nullptr },
  };
  for (const auto& r : routes) httpd_register_uri_handler(web, &r);
}

// ---------------- Wi-Fi ----------------
void startWifi() {
  WiFi.persistent(false);
  if (WIFI_SSID[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);   // many Super Mini clones cannot join without this
    WiFi.setSleep(false);                 // no modem power-save: removes 100+ ms lag spikes
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
  WiFi.setSleep(false);
  Serial.printf("Join '%s' (pass %s), open http://%s\n", AP_SSID, AP_PASS,
                WiFi.softAPIP().toString().c_str());
}

// ---------------- touch pad, blue LED, deep sleep ----------------
bool touchNow() { return digitalRead(PIN_TOUCH) == HIGH; }

void goToSleep() {
  LED_SET(0, 0, 0);                              // a pixel keeps its last colour, so send "off"
  delay(5);
  sleeping = true;
  stepsRtc = steps;                              // keep the step count through sleep
  if (web) httpd_stop(web);
  WiFi.mode(WIFI_OFF);
  if (imuStarted) { delay(100); mpuWrite(0x6B, 0x40); }   // MPU6050 into its own sleep mode
  uint32_t t0 = millis();
  while (touchNow() && millis() - t0 < 3000) delay(10);   // finger must be off the pad first
  delay(50);
  Serial.println("Deep sleep. Double tap the touch pad to wake.");
  Serial.flush();
  esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_TOUCH, ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

// Woken by tap #1. Needs tap #2 in time, otherwise it was an accidental touch.
void waitForSecondTap() {
  uint32_t t0 = millis();
  bool wasDown = touchNow();                     // finger may still be on the pad
  while (millis() - t0 < WAKE_WINDOW_MS) {
    bool down = touchNow();
    if (down && !wasDown) return;                // second tap: stay awake
    wasDown = down;
    delay(5);
  }
  goToSleep();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TOUCH, INPUT);
  LED_SET(0, 0, 0);                              // pixel off

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) waitForSecondTap();
  else if (!START_ON_POWER_UP) goToSleep();
  LED_SET(0, 0, LED_BLUE);                       // system is active: blue
  steps = stepsRtc;

  pinMode(PIN_LOP, INPUT);
  pinMode(PIN_LON, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  notch.init(MAINS_HZ, 10.0f);
  loadCal();
  imuStarted = imuInit();
  startWifi();
  startWeb();
  xTaskCreate(sampler, "ecg", 4096, nullptr, 3, nullptr);        // ECG: highest priority
  if (imuStarted) xTaskCreate(imuTask, "imu", 4096, nullptr, 2, nullptr);
  else Serial.println("MPU6050 not found - check SDA/SCL (GPIO4/GPIO5) wiring. ECG still works.");
  activatedAt = millis();
}

// Double tap on the touch pad while running = sleep.
void loop() {
  static bool wasDown = touchNow();
  static uint32_t lastTap = 0;
  bool down = touchNow();
  uint32_t now = millis();
  if (down && !wasDown && now - activatedAt > 1500) {
    uint32_t gap = now - lastTap;
    if (lastTap && gap <= DOUBLE_TAP_MS) { if (gap > 80) goToSleep(); }
    else lastTap = now;
  }
  wasDown = down;
  delay(5);
}
