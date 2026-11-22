// MediBot 4WD | Team MedNova | Classic ESP32 DevKit V1 (ESP32-WROOM).
// Arduino IDE: "ESP32 Dev Module", esp32 by Espressif Systems 3.x.
// Upload over USB with the motor battery switched off. Open Serial Monitor at
// 115200 baud; press ? to see the Wi-Fi credentials and control-page address.
// Driver L: front = A, rear = B. Driver R: front = A, rear = B. One motor/channel.
// Common ground. Use a 1k/2k divider on ECHO, 3.3 V digital IR outputs,
// a buffered active-HIGH buzzer, and a separate regulated 5 V servo supply.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_arduino_version.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#if !defined(CONFIG_IDF_TARGET_ESP32) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Select ESP32 Dev Module (classic ESP32) and install esp32 by Espressif Systems 3.x."
#endif

// Wheel order: front left, rear left, front right, rear right.
const uint8_t IN1[4] = {25, 27, 32, 18};
const uint8_t IN2[4] = {26, 14, 33, 19};
const int8_t DIR_SIGN[4] = {1, 1, 1, 1};
const uint8_t MOTOR_PWM = 22, STBY = 23;
const uint8_t IR_L = 34, IR_C = 35, IR_R = 39;
const uint8_t TRIG = 5, ECHO = 21;
const uint8_t SERVO_PIN = 13, BUZZER = 4, GREEN = 16, RED = 17;
const uint8_t BLACK_LEVEL = LOW;  // Check raw L/C/R on tape and white paper.
const uint8_t DRIVE_DUTY = 125;   // 0..255, one shared speed for all four wheels.
const float STOP_CM = 18.0f;
const uint32_t MANUAL_MS = 700, HEARTBEAT_MS = 2000, PING_MS = 80;
const uint16_t CLOSED_US = 1000, OPEN_US = 1500;  // Fit horn after checking travel.
const char SSID[] = "MediBot-4WD";

enum RunMode { IDLE, MANUAL, LINE, DELIVERED, FAULT };
RunMode mode = IDLE;
const char* fault = "";
const char* modeNames[] = {"IDLE", "MANUAL", "LINE", "DELIVERED", "FAULT"};
uint32_t commandTime = 0, beatTime = 0, lastPing = 0, sampledAt = 0, buzzTime = 0;
float distanceCm = -1.0f;
bool motorPwmOK = false, servoPwmOK = false, gateOpen = false, buzzing = false;
bool wifiReady = false, ready = false, sampled = false;
char motion = 'S';
int lastLeft = 0, lastRight = 0;
String apPassword;
WebServer server(80);
SemaphoreHandle_t stateLock = nullptr;

static const char PAGE[] PROGMEM = R"MEDIBOT(
<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MediBot 4WD</title><style>
*{box-sizing:border-box}body{max-width:37rem;margin:2rem auto;padding:0 1rem;color:#123a31;background:#f5f7f1;font:1rem/1.5 system-ui,sans-serif}
h1{margin:0}h2{font-size:1.05rem;margin:1.2rem 0 .4rem}p{margin:.35rem 0 .7rem}
button{padding:.7rem;border:1px solid #93aaa0;border-radius:.45rem;background:white;color:#173b30;font:inherit}
button:focus-visible{outline:3px solid #bb7518}button:active{background:#dcebdc}
nav{display:grid;grid-template-columns:repeat(2,1fr);gap:.45rem}
#stop{width:100%;font-weight:700;background:#812c25;color:white;margin:.6rem 0}
output{display:block;min-height:1.6em;color:#31554b}#status{background:white;border-left:4px solid #21614a;padding:.7rem}
</style><h1>MediBot 4WD</h1><p>Team MedNova</p><p id="status">Reading robot status…</p>
<button id="stop" data-cmd="STOP">Stop motors</button><h2>Test motion</h2>
<p>Each tap drives for up to 0.7 seconds. Raise the wheels for the first test.</p>
<nav><button data-cmd="F">Forward</button><button data-cmd="B">Reverse</button>
<button data-cmd="L">Turn left</button><button data-cmd="R">Turn right</button></nav>
<h2>Track and delivery</h2><nav><button data-cmd="LINE">Follow line</button><button data-cmd="RESET">Reset fault</button>
<button data-cmd="DELIVER">Open tray</button><button data-cmd="CLOSE">Close tray</button></nav>
<p>Keep this page visible during line following. A person confirms collection.</p>
<output id="reply" aria-live="polite"></output><script>
let lineLease=false, requestNumber=0;
const endpoint=cmd=>'/command?cmd='+encodeURIComponent(cmd);
function release(){if(lineLease){lineLease=false;navigator.sendBeacon(endpoint('STOP'));}}
document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});
addEventListener('pagehide',release);
document.querySelectorAll('[data-cmd]').forEach(b=>b.onclick=async()=>{
  const cmd=b.dataset.cmd, ticket=++requestNumber;lineLease=false;
  try{const res=await fetch(endpoint(cmd),{method:'POST'});
    const answer=await res.text();if(ticket!==requestNumber)return;
    lineLease=res.ok && cmd==='LINE';if(document.hidden)release();
    document.querySelector('#reply').textContent=answer;
  }catch{document.querySelector('#reply').textContent='Connection lost. Check the robot Wi-Fi.'}
});
async function refresh(){
  const heldLease=lineLease, ticket=requestNumber;
  try{if(lineLease && !document.hidden)await fetch('/heartbeat',{method:'POST'});
    const res=await fetch('/status',{cache:'no-store'});const s=await res.json();
    if(heldLease && ticket===requestNumber && s.mode!=='LINE')lineLease=false;
    document.querySelector('#status').textContent=s.mode+'  |  '+(s.distance_cm<0?'No valid echo':s.distance_cm+' cm')+
      '  |  IR raw L/C/R: '+s.ir_raw.join('/')+'  |  Tray '+(s.gate_open?'open':'closed / at rest')+(s.fault?'  |  '+s.fault:'');
  }catch{lineLease=false;document.querySelector('#status').textContent='Waiting for MediBot Wi-Fi…'}
  setTimeout(refresh,450);
}refresh();
</script></html>
)MEDIBOT";

void motorsOff() {
  digitalWrite(STBY, LOW);
  if (motorPwmOK) ledcWrite(MOTOR_PWM, 0);
  for (int i = 0; i < 4; ++i) {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);
  }
  lastLeft = lastRight = 0;
}

void latchFault(const char* reason) {
  motorsOff();
  mode = FAULT;
  fault = reason;
}

void drive(int left, int right) {
  if (left == lastLeft && right == lastRight) return;
  digitalWrite(STBY, LOW);
  for (int i = 0; i < 4; ++i) {
    int d = (i < 2 ? left : right) * DIR_SIGN[i];
    digitalWrite(IN1[i], d > 0 ? HIGH : LOW);
    digitalWrite(IN2[i], d < 0 ? HIGH : LOW);
  }
  if (!ledcWrite(MOTOR_PWM, DRIVE_DUTY)) {
    latchFault("motor_pwm_failed");
    return;
  }
  digitalWrite(STBY, HIGH);
  lastLeft = left;
  lastRight = right;
}

float pingCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long pulse = pulseIn(ECHO, HIGH, 26000UL);
  float cm = pulse * 0.0343f / 2.0f;
  return cm >= 2 && cm <= 350 ? cm : -1.0f;
}

bool rangeBlocked(uint32_t now) {
  return !sampled || now - sampledAt > 250 || distanceCm < STOP_CM;
}

// Called with stateLock held. Neither the HTTP request parser nor a network
// write holds this lock, so the physical controller can still stop on timeout.
void controlStep(uint32_t now, bool clientConnected) {
  const bool blocked = rangeBlocked(now);
  if (buzzing && now - buzzTime >= 250) {
    digitalWrite(BUZZER, LOW);
    buzzing = false;
  }
  if (mode == MANUAL || mode == LINE) {
    if (blocked) latchFault("obstacle_or_range_invalid");
    else if (!clientConnected || now - beatTime >= HEARTBEAT_MS) latchFault("wifi_heartbeat_timeout");
    else if (mode == MANUAL) {
      if (now - commandTime >= MANUAL_MS) {
        motorsOff();
        mode = IDLE;
      } else if (motion == 'F') drive(1, 1);
      else if (motion == 'B') drive(-1, -1);
      else if (motion == 'L') drive(-1, 1);
      else drive(1, -1);
    } else {
      const bool l = digitalRead(IR_L) == BLACK_LEVEL;
      const bool c = digitalRead(IR_C) == BLACK_LEVEL;
      const bool r = digitalRead(IR_R) == BLACK_LEVEL;
      if (c && !l && !r) drive(1, 1);
      else if (l && !r) drive(-1, 1);
      else if (r && !l) drive(1, -1);
      else latchFault("junction_or_lost_line");
    }
  }
  digitalWrite(RED, (blocked || mode == FAULT) ? HIGH : LOW);
}

bool moveGate(bool open) {
  const uint32_t pulse = open ? OPEN_US : CLOSED_US;
  // 50 Hz means 20,000 us per period. Servo uses a separate timer group.
  const uint32_t duty = (pulse * 16384UL + 10000UL) / 20000UL;
  if (!servoPwmOK || !ledcWrite(SERVO_PIN, duty)) {
    latchFault("servo_pwm_failed");
    return false;
  }
  gateOpen = open;
  digitalWrite(GREEN, open ? HIGH : LOW);
  return true;
}

int applyCommand(const String& cmd, const char*& reply) {
  const uint32_t now = millis();
  if (cmd == "STOP" || cmd == "RESET") {
    motorsOff();
    mode = ready ? IDLE : FAULT;
    fault = ready ? "" : "initialization_failed";
    reply = ready ? "Motors stopped." : "PWM initialization failed. Motors disabled.";
    return ready ? 200 : 503;
  }
  if (!ready) { reply = "Controller initialization failed."; return 503; }
  if (cmd == "CLOSE") {
    motorsOff();
    if (!moveGate(false)) { reply = fault; return 503; }
    if (mode != FAULT) mode = IDLE;
    reply = "Tray closed."; return 200;
  }
  if (mode == FAULT) { reply = "Inspect the robot, then press Reset fault."; return 409; }
  bool manual = cmd == "F" || cmd == "B" || cmd == "L" || cmd == "R";
  if (!manual && cmd != "LINE" && cmd != "DELIVER") { reply = "Unknown command."; return 400; }
  if (rangeBlocked(now)) { latchFault("obstacle_or_range_invalid"); reply = fault; return 409; }
  if (cmd == "DELIVER") {
    motorsOff();
    if (!moveGate(true)) { reply = fault; return 503; }
    mode = DELIVERED;
    buzzTime = now;
    buzzing = true;
    digitalWrite(BUZZER, HIGH);
    reply = "Tray open. Confirm collection in person."; return 200;
  }
  if (gateOpen) { reply = "Close the tray before moving."; return 409; }
  mode = manual ? MANUAL : LINE;
  motion = manual ? cmd[0] : 'S';
  commandTime = beatTime = now;
  reply = manual ? "Manual pulse." : "Following line.";
  return 200;
}

void commandHandler() {
  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  const char* reply;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  int code = applyCommand(cmd, reply);
  xSemaphoreGive(stateLock);
  server.send(code, "text/plain", reply);
}

void statusHandler() {
  char json[260];
  const bool connected = WiFi.softAPgetStationNum() > 0;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  snprintf(json, sizeof(json),
    "{\"mode\":\"%s\",\"distance_cm\":%.1f,\"blocked\":%s,\"fault\":\"%s\","
    "\"wifi\":%s,\"gate_open\":%s,\"ir_raw\":[%d,%d,%d]}",
    modeNames[mode], distanceCm, rangeBlocked(millis()) ? "true" : "false", fault,
    connected ? "true" : "false", gateOpen ? "true" : "false",
    digitalRead(IR_L), digitalRead(IR_C), digitalRead(IR_R));
  xSemaphoreGive(stateLock);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void webTask(void*) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5) + 1);
  }
}

String loadPassword() {
  Preferences prefs;
  bool stored = prefs.begin("medibot", false);
  String key = stored ? prefs.getString("ap-pass", "") : String("");
  if (key.length() < 8 || key.length() > 63) {
    char fresh[17];
    snprintf(fresh, sizeof(fresh), "%08lx%08lx", (unsigned long)esp_random(), (unsigned long)esp_random());
    key = fresh;
    if (stored) prefs.putString("ap-pass", key);
  }
  if (stored) prefs.end();
  return key;
}

void showAccess() {
  if (!wifiReady) { Serial.println("Wi-Fi access point unavailable. Motors disabled."); return; }
  Serial.print("Wi-Fi: "); Serial.println(SSID);
  Serial.print("Password: "); Serial.println(apPassword);
  Serial.print("Controls: http://"); Serial.println(WiFi.softAPIP());
}

void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  for (int i = 0; i < 4; ++i) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
  }
  motorsOff();
  Serial.begin(115200);
  pinMode(IR_L, INPUT); pinMode(IR_C, INPUT); pinMode(IR_R, INPUT);
  pinMode(TRIG, OUTPUT); digitalWrite(TRIG, LOW); pinMode(ECHO, INPUT);
  pinMode(BUZZER, OUTPUT); digitalWrite(BUZZER, LOW);
  pinMode(GREEN, OUTPUT); digitalWrite(GREEN, LOW);
  pinMode(RED, OUTPUT); digitalWrite(RED, HIGH);

  // The original ESP32 has two separate LEDC groups: channels 0 and 8.
  motorPwmOK = ledcAttachChannel(MOTOR_PWM, 5000, 8, 0);
  servoPwmOK = ledcAttachChannel(SERVO_PIN, 50, 14, 8);
  motorsOff();
  if (servoPwmOK) servoPwmOK = ledcWrite(SERVO_PIN, 0); // No servo movement at boot.
  stateLock = xSemaphoreCreateMutex();
  if (!stateLock) { Serial.println("State lock unavailable. Motors disabled."); return; }

  WiFi.mode(WIFI_AP);
  apPassword = loadPassword();
  // Only one controlling device (phone or laptop) on the private access point.
  wifiReady = WiFi.softAP(SSID, apPassword.c_str(), 1, false, 1);
  ready = wifiReady && motorPwmOK && servoPwmOK;
  if (!ready) latchFault("initialization_failed");
  if (wifiReady) {
    server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html; charset=utf-8", PAGE); });
    server.on("/status", HTTP_GET, statusHandler);
    server.on("/command", HTTP_POST, commandHandler);
    server.on("/heartbeat", HTTP_POST, [](){
      xSemaphoreTake(stateLock, portMAX_DELAY);
      beatTime = millis();
      xSemaphoreGive(stateLock);
      server.send(200, "text/plain", "ok");
    });
    server.begin();
    if (xTaskCreate(webTask, "MediBotWeb", 8192, nullptr, 1, nullptr) != pdPASS) {
      ready = false;
      latchFault("web_task_failed");
    }
  }
  Serial.println(ready ? "MEDIBOT READY - motors stopped." : "Controller fault. Motors disabled.");
  showAccess();
}

void loop() {
  if (!stateLock) { delay(25); return; }
  const uint32_t now = millis();
  const bool sampleDue = !sampled || now - lastPing >= PING_MS;
  float cm = -1.0f;
  if (sampleDue) {
    lastPing = now;
    cm = pingCm();
  }
  const bool connected = wifiReady && WiFi.softAPgetStationNum() > 0;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  if (sampleDue) {
    distanceCm = cm;
    sampledAt = millis();
    sampled = true;
  }
  controlStep(millis(), connected);
  xSemaphoreGive(stateLock);
  if (Serial.available() && Serial.read() == '?') showAccess();
  delay(2);
}
