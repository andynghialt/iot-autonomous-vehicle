/* =====================================================================
 * PROJECT: Maze Solver Master-Gateway + 2-Tab Web UI
 * FILE: ESP32_Master_2026-11-20.ino
 * DATE: 2026-11-20
 * NOTE: Cảm biến IR 5 mắt: Nền đen = Bật/1, Nền trắng = Tắt (0).
 *       S4 (MG996R) là loại 360 độ (điều khiển chiều/tốc độ, 90 = dừng).
 * ===================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU6050 mpu;

const char* ssid = "Maze_Solver_AP";
const char* password = "12345678";

WebServer server(80);

const int IR_1 = 32; 
const int IR_2 = 33; 
const int IR_3 = 25; 
const int IR_4 = 26; 
const int IR_5 = 27; 

const int US_TRIG = 13; 
const int US_ECHO = 14;

// Ngưỡng lọc nhiễu gyro (deg/s) khi đứng yên để chống tự tăng Yaw
const float GYRO_DEADBAND = 1.8;

String currentCommand = "STOP";
float yawAngle = 0;
float targetYaw = 0;
float lockedHeading = 0; 
int lastCorrectionVal = 0;
unsigned long lastTime = 0;
float gz_offset = 0;
String latestMapLog = "Sẵn sàng giải mê cung...";
String workerEncoderData = "Enc: X:0 Y:0";

enum RobotState { IDLE, STRAIGHT, TURNING, ALIGN_BEFORE_STOP, CELL_FORWARD };
RobotState robotState = IDLE;

const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Maze Solver & Arm</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 8px; background: #f0f2f5; user-select: none; -webkit-user-select: none; }
    h1 { color: #333; font-size: 16px; margin: 4px 0 8px 0; }
    
    .tabs { display: flex; max-width: 350px; margin: 0 auto 8px auto; border-radius: 8px; overflow: hidden; border: 1px solid #1565c0; }
    .tab-btn { flex: 1; padding: 10px; font-size: 13px; font-weight: bold; border: none; cursor: pointer; background: #fff; color: #1565c0; }
    .tab-btn.active { background: #1565c0; color: #fff; }
    .page { display: none; max-width: 350px; margin: 0 auto; }
    .page.active { display: block; }

    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
    button { padding: 9px; font-size: 12px; cursor: pointer; border-radius: 6px; border: none; color: white; font-weight: bold; }
    .btn-forward { background-color: #2e7d32; grid-column: span 2; }
    .btn-cell { background-color: #00796b; grid-column: span 2; }
    .btn-left { background-color: #1565c0; }
    .btn-right { background-color: #ef6c00; }
    .btn-left180 { background-color: #0d47a1; grid-column: span 2; }
    .btn-reset { background-color: #546e7a; grid-column: span 2; }
    .btn-stop { background-color: #c2185b; grid-column: span 2; }

    .panel { background: #fff; padding: 8px; border-radius: 6px; margin-top: 8px; text-align: left; font-size: 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
    .panel input[type="number"] { width: 65px; padding: 3px; }

    .servo-card { background: #fff; padding: 8px; border-radius: 6px; margin-bottom: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
    .servo-header { display: flex; justify-content: space-between; font-size: 13px; font-weight: bold; margin-bottom: 6px; color: #222; }
    .servo-btns { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
    .step-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px; margin-top: 6px; }
    .btn-step-l { background: #42a5f5; padding: 7px 4px; font-size: 11px; }
    .btn-step-r { background: #ff9800; padding: 7px 4px; font-size: 11px; }
    .btn-twin-fwd { background: #2e7d32; padding: 10px 6px; font-size: 12px; }
    .btn-twin-back { background: #6a1b9a; padding: 10px 6px; font-size: 12px; }

    .btn-jog-l { background: #1976d2; padding: 10px; font-size: 13px; }
    .btn-jog-r { background: #e65100; padding: 10px; font-size: 13px; }
    .btn-jog-l:active, .btn-jog-r:active, .btn-step-l:active, .btn-step-r:active, .btn-twin-fwd:active, .btn-twin-back:active { opacity: 0.75; transform: scale(0.98); }
    .btn-release { background: #607d8b; width: 100%; margin-top: 4px; padding: 9px; font-size: 12px; }

    .telemetry { background: #e3f2fd; color: #0d47a1; padding: 6px; border-radius: 6px; margin-top: 8px; font-size: 11px; font-weight: bold; max-width: 350px; margin-left: auto; margin-right: auto; line-height: 1.3; text-align: left; }
    .log-box { margin-top: 6px; background: #000; color: #0f0; padding: 6px; border-radius: 6px; text-align: left; font-family: monospace; font-size: 10px; height: 48px; overflow-y: auto; max-width: 350px; margin-left: auto; margin-right: auto; }
  </style>
</head>
<body>
  <h1>MAZE SOLVER & ARM CONTROL</h1>

  <div class="tabs">
    <button id="tabCarBtn" class="tab-btn active" onclick="switchTab('car')">1. ĐIỀU KHIỂN XE</button>
    <button id="tabArmBtn" class="tab-btn" onclick="switchTab('arm')">2. CÁNH TAY 4 BẬC</button>
  </div>

  <!-- TRANG 1: ĐIỀU KHIỂN XE -->
  <div id="pageCar" class="page active">
    <div class="grid">
      <button class="btn-cell" onclick="sendCmd('CELL_FWD')">TIẾN ĐÚNG 1 Ô (30cm)</button>
      <button class="btn-forward" onclick="sendCmd('FORWARD')">ĐI THẲNG LIÊN TỤC</button>
      <button class="btn-left" onclick="sendCmd('TURN_LEFT_90')">QUAY TRÁI 90°</button>
      <button class="btn-right" onclick="sendCmd('TURN_RIGHT_90')">QUAY PHẢI 90°</button>
      <button class="btn-left180" onclick="sendCmd('TURN_LEFT_180')">QUAY ĐẦU 180° (NGÕ CỤT)</button>
      <button class="btn-reset" onclick="sendCmd('RESET_YAW')">RESET YAW & LOCKED = 0°</button>
      <button class="btn-stop" onclick="sendCmd('STOP')">STOP & CĂN CHỈNH GỐC</button>
    </div>

    <div class="panel">
      Scale Trái: <input type="number" id="sX" step="0.05" value="1.0"> 
      Phải: <input type="number" id="sY" step="0.05" value="1.0">
      <button onclick="updateScale()" style="padding:4px 8px; background:#3f51b5; color:#fff; border:none; border-radius:4px;">Lưu</button>
    </div>
  </div>

  <!-- TRANG 2: ĐIỀU KHIỂN CÁNH TAY -->
  <div id="pageArm" class="page">
    <!-- S1: Tay gắp 2 chế độ 90 độ và 0 độ -->
    <div class="servo-card">
      <div class="servo-header"><span>S1 - Tay gắp (2 chế độ 90° / 0°)</span><span id="ang1">90°</span></div>
      <div class="servo-btns">
        <button class="btn-jog-l" onclick="setGripAngle(90)">ĐÓNG (90°)</button>
        <button class="btn-jog-r" onclick="setGripAngle(0)">MỞ (0°)</button>
      </div>
    </div>

    <!-- Đồng bộ 2 servo hông (ngược chiều nhau để cùng đưa tới / thu về) -->
    <div class="servo-card">
      <div class="servo-header"><span>S2 + S3 Đồng bộ (Ngược dấu nhau)</span><span id="twinState">S2:90° | S3:90°</span></div>
      <div class="servo-btns">
        <button class="btn-twin-fwd" onpointerdown="startTwinJog(1)" onpointerup="stopTwinJog()" onpointerleave="stopTwinJog()" onpointercancel="stopTwinJog()">▲ CÙNG ĐƯA TỚI</button>
        <button class="btn-twin-back" onpointerdown="startTwinJog(-1)" onpointerup="stopTwinJog()" onpointerleave="stopTwinJog()" onpointercancel="stopTwinJog()">▼ CÙNG THU VỀ</button>
      </div>
    </div>

    <!-- S2: Khớp Hông 1 -->
    <div class="servo-card">
      <div class="servo-header"><span>S2 - Khớp Hông 1 (SG90)</span><span id="ang2">90°</span></div>
      <div class="servo-btns">
        <button class="btn-jog-l" onpointerdown="startJog(2, -1)" onpointerup="stopJog()" onpointerleave="stopJog()" onpointercancel="stopJog()">◀ GIỮ TRÁI (-)</button>
        <button class="btn-jog-r" onpointerdown="startJog(2, 1)" onpointerup="stopJog()" onpointerleave="stopJog()" onpointercancel="stopJog()">GIỮ PHẢI (+) ▶</button>
      </div>
      <div class="step-grid">
        <button class="btn-step-l" onclick="stepByAngle(2, -15)">-15°</button>
        <button class="btn-step-l" onclick="stepByAngle(2, -30)">-30°</button>
        <button class="btn-step-l" onclick="stepByAngle(2, -60)">-60°</button>
        <button class="btn-step-l" onclick="stepByAngle(2, -90)">-90°</button>
        <button class="btn-step-r" onclick="stepByAngle(2, 15)">+15°</button>
        <button class="btn-step-r" onclick="stepByAngle(2, 30)">+30°</button>
        <button class="btn-step-r" onclick="stepByAngle(2, 60)">+60°</button>
        <button class="btn-step-r" onclick="stepByAngle(2, 90)">+90°</button>
      </div>
    </div>

    <!-- S3: Khớp Hông 2 -->
    <div class="servo-card">
      <div class="servo-header"><span>S3 - Khớp Hông 2 (SG90)</span><span id="ang3">90°</span></div>
      <div class="servo-btns">
        <button class="btn-jog-l" onpointerdown="startJog(3, -1)" onpointerup="stopJog()" onpointerleave="stopJog()" onpointercancel="stopJog()">◀ GIỮ TRÁI (-)</button>
        <button class="btn-jog-r" onpointerdown="startJog(3, 1)" onpointerup="stopJog()" onpointerleave="stopJog()" onpointercancel="stopJog()">GIỮ PHẢI (+) ▶</button>
      </div>
      <div class="step-grid">
        <button class="btn-step-l" onclick="stepByAngle(3, -15)">-15°</button>
        <button class="btn-step-l" onclick="stepByAngle(3, -30)">-30°</button>
        <button class="btn-step-l" onclick="stepByAngle(3, -60)">-60°</button>
        <button class="btn-step-l" onclick="stepByAngle(3, -90)">-90°</button>
        <button class="btn-step-r" onclick="stepByAngle(3, 15)">+15°</button>
        <button class="btn-step-r" onclick="stepByAngle(3, 30)">+30°</button>
        <button class="btn-step-r" onclick="stepByAngle(3, 60)">+60°</button>
        <button class="btn-step-r" onclick="stepByAngle(3, 90)">+90°</button>
      </div>
    </div>

    <!-- S4: Servo 360 độ -> Nhấn giữ quay chậm, nhả ra dừng (90) -->
    <div class="servo-card">
      <div class="servo-header"><span>S4 - Đế xoay 360° (Nhả = Dừng)</span><span id="ang4">DỪNG</span></div>
      <div class="servo-btns">
        <button class="btn-jog-l" onpointerdown="spinBase(-1)" onpointerup="stopBase()" onpointerleave="stopBase()" onpointercancel="stopBase()">◀ QUAY CHẬM A</button>
        <button class="btn-jog-r" onpointerdown="spinBase(1)" onpointerup="stopBase()" onpointerleave="stopBase()" onpointercancel="stopBase()">QUAY CHẬM B ▶</button>
      </div>
    </div>

    <button class="btn-release" onclick="releaseArm()">NGẮT XUNG SERVO (THẢ ĐỨNG YÊN)</button>
  </div>

  <div class="telemetry" id="telemetryBox">Đang chờ lệnh...</div>
  <div class="log-box" id="logBox">Đang kết nối...</div>

  <script>
    function switchTab(tab) {
      document.getElementById('pageCar').classList.toggle('active', tab === 'car');
      document.getElementById('pageArm').classList.toggle('active', tab === 'arm');
      document.getElementById('tabCarBtn').classList.toggle('active', tab === 'car');
      document.getElementById('tabArmBtn').classList.toggle('active', tab === 'arm');
    }

    function sendCmd(cmd) { fetch('/cmd?action=' + cmd); }

    const servoAngles = { 1: 90, 2: 90, 3: 90 };
    const servoLimits = {
      1: { min: 0, max: 90 },
      2: { min: 0, max: 180 },
      3: { min: 0, max: 180 }
    };

    let jogTimer = null;
    let holdDelayTimer = null;
    let twinTimer = null;
    let isSending = false;

    function updateTwinLabel() {
      document.getElementById('twinState').innerText = 'S2:' + servoAngles[2] + '° | S3:' + servoAngles[3] + '°';
    }

    function pushServo(id, angle) {
      if (id !== 4) document.getElementById('ang' + id).innerText = angle + '°';
      updateTwinLabel();
      if (isSending) return;
      isSending = true;
      fetch('/arm_single?id=' + id + '&ang=' + angle)
        .finally(() => { isSending = false; });
    }

    function pushBothHips(a2, a3) {
      document.getElementById('ang2').innerText = a2 + '°';
      document.getElementById('ang3').innerText = a3 + '°';
      updateTwinLabel();
      fetch('/arm_twin?s2=' + a2 + '&s3=' + a3);
    }

    function setGripAngle(angle) {
      stopJog();
      stopTwinJog();
      servoAngles[1] = angle;
      pushServo(1, angle);
    }

    function stepByAngle(id, delta) {
      stopJog();
      stopTwinJog();
      let next = servoAngles[id] + delta;
      next = Math.max(servoLimits[id].min, Math.min(servoLimits[id].max, next));
      servoAngles[id] = next;
      pushServo(id, next);
    }

    function stepServo(id, delta) {
      let next = servoAngles[id] + delta;
      next = Math.max(servoLimits[id].min, Math.min(servoLimits[id].max, next));
      if (next !== servoAngles[id]) {
        servoAngles[id] = next;
        pushServo(id, next);
      }
    }

    function startJog(id, dir) {
      stopJog();
      stopTwinJog();
      stepServo(id, dir * 3);
      holdDelayTimer = setTimeout(() => {
        jogTimer = setInterval(() => {
          stepServo(id, dir * 2);
        }, 70);
      }, 220);
    }

    function stopJog() {
      if (holdDelayTimer) { clearTimeout(holdDelayTimer); holdDelayTimer = null; }
      if (jogTimer) { clearInterval(jogTimer); jogTimer = null; }
    }

    // 2 servo hông đối diện nhau: đi cùng hướng cơ khí (đưa tới / thu về) bằng cách đổi dấu ngược nhau
    function stepTwin(dir) {
      let n2 = Math.max(0, Math.min(180, servoAngles[2] + dir * 2));
      let n3 = Math.max(0, Math.min(180, servoAngles[3] - dir * 2));
      servoAngles[2] = n2;
      servoAngles[3] = n3;
      pushBothHips(n2, n3);
    }

    function startTwinJog(dir) {
      stopJog();
      stopTwinJog();
      stepTwin(dir);
      twinTimer = setInterval(() => {
        stepTwin(dir);
      }, 75);
    }

    function stopTwinJog() {
      if (twinTimer) { clearInterval(twinTimer); twinTimer = null; }
    }

    function spinBase(dir) {
      let pwmCmd = dir < 0 ? 76 : 104;
      document.getElementById('ang4').innerText = dir < 0 ? 'QUAY A' : 'QUAY B';
      fetch('/arm_single?id=4&ang=' + pwmCmd);
    }

    function stopBase() {
      document.getElementById('ang4').innerText = 'DỪNG';
      fetch('/arm_single?id=4&ang=90');
    }

    function releaseArm() {
      stopJog();
      stopTwinJog();
      stopBase();
      fetch('/arm_release');
    }

    function updateScale() {
      let sx = document.getElementById("sX").value, sy = document.getElementById("sY").value;
      fetch('/scale?sx=' + sx + '&sy=' + sy).then(() => alert("Đã lưu Scale!"));
    }

    setInterval(function() {
      fetch('/getdata').then(res => res.json()).then(data => {
        document.getElementById("telemetryBox").innerHTML = 
          "Yaw: " + data.yaw + "° | Lck: " + data.locked + "°<br>" + data.sensor;
        if(data.log) document.getElementById("logBox").innerText = data.log;
      });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html; charset=utf-8", MAIN_page); }

void handleCmd() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    currentCommand = action;
    if (action == "CELL_FWD") { Serial2.println("CMD:C"); robotState = CELL_FORWARD; latestMapLog = "Tiến đúng 1 ô (30cm)..."; }
    else if (action == "FORWARD") { Serial2.println("CMD:F"); robotState = STRAIGHT; latestMapLog = "Đi thẳng liên tục..."; }
    else if (action == "TURN_LEFT_90") { targetYaw = lockedHeading + 90.0; robotState = TURNING; latestMapLog = "Quay trái 90 độ..."; }
    else if (action == "TURN_RIGHT_90") { targetYaw = lockedHeading - 90.0; robotState = TURNING; latestMapLog = "Quay phải 90 độ..."; }
    else if (action == "TURN_LEFT_180") { targetYaw = lockedHeading + 180.0; robotState = TURNING; latestMapLog = "Quay đầu 180 độ (Ngõ cụt)..."; }
    else if (action == "RESET_YAW") {
      yawAngle = 0.0;
      targetYaw = 0.0;
      lockedHeading = 0.0;
      lastCorrectionVal = 0;
      robotState = IDLE;
      currentCommand = "STOP";
      Serial2.println("CMD:S");
      latestMapLog = "Đã reset Yaw và Locked về 0.0°";
    }
    else if (action == "STOP") { robotState = ALIGN_BEFORE_STOP; latestMapLog = "Đang căn chỉnh về gốc..."; }
    server.send(200, "text/plain", currentCommand);
  }
}

void handleArmSingle() {
  if (server.hasArg("id") && server.hasArg("ang")) {
    int id = constrain(server.arg("id").toInt(), 1, 4);
    int ang = constrain(server.arg("ang").toInt(), 0, 180);
    Serial2.println("ARMS:" + String(id) + "," + String(ang));
    latestMapLog = "Servo S" + String(id) + " -> " + String(ang);
    server.send(200, "text/plain", "OK");
  }
}

void handleArmTwin() {
  if (server.hasArg("s2") && server.hasArg("s3")) {
    int a2 = constrain(server.arg("s2").toInt(), 0, 180);
    int a3 = constrain(server.arg("s3").toInt(), 0, 180);
    Serial2.println("ARMT:" + String(a2) + "," + String(a3));
    latestMapLog = "Twin S2:" + String(a2) + "° S3:" + String(a3) + "°";
    server.send(200, "text/plain", "OK");
  }
}

void handleArmRelease() {
  Serial2.println("ARM_OFF");
  latestMapLog = "Đã ngắt xung giữ 4 Servo.";
  server.send(200, "text/plain", "OK");
}

void handleScale() {
  if (server.hasArg("sx") && server.hasArg("sy")) {
    Serial2.println("SCALE:" + server.arg("sx") + "," + server.arg("sy"));
    server.send(200, "text/plain", "OK");
  }
}

long readMasterUltrasonic() {
  digitalWrite(US_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  long duration = pulseIn(US_ECHO, HIGH, 20000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void handleGetData() {
  String irStr = String(digitalRead(IR_1)) + String(digitalRead(IR_2)) + String(digitalRead(IR_3)) + String(digitalRead(IR_4)) + String(digitalRead(IR_5));
  long cargoDist = readMasterUltrasonic();
  String sensorInfo = "IR:" + irStr + " | US:" + cargoDist + "cm<br>" + workerEncoderData;

  String json = "{";
  json += "\"yaw\":" + String(yawAngle, 1) + ",";
  json += "\"locked\":" + String(lockedHeading, 1) + ",";
  json += "\"sensor\":\"" + sensorInfo + "\",";
  json += "\"log\":\"" + latestMapLog + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Wire.begin(21, 22);
  mpu.initialize();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

  pinMode(IR_1, INPUT_PULLUP); pinMode(IR_2, INPUT_PULLUP); 
  pinMode(IR_3, INPUT_PULLUP); pinMode(IR_4, INPUT_PULLUP); pinMode(IR_5, INPUT_PULLUP);
  pinMode(US_TRIG, OUTPUT); pinMode(US_ECHO, INPUT);

  delay(500);
  double sum = 0.0;
  const int calibrationSamples = 1000;
  for(int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gz_offset = sum / calibrationSamples;
  yawAngle = 0.0;
  lockedHeading = 0.0;

  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/arm_single", handleArmSingle);
  server.on("/arm_twin", handleArmTwin);
  server.on("/arm_release", handleArmRelease);
  server.on("/scale", handleScale);
  server.on("/getdata", handleGetData);
  server.begin();
  lastTime = millis();
}

void loop() {
  server.handleClient();
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gz_deg = (gz - gz_offset) / 131.0;
  if (abs(gz_deg) > GYRO_DEADBAND) {
    yawAngle += gz_deg * dt;
  }

  if (robotState == STRAIGHT || robotState == CELL_FORWARD) {
    float headingError = yawAngle - lockedHeading;
    lastCorrectionVal = (int)(headingError * 2);
    Serial2.println("CORRECTION:" + String(lastCorrectionVal));
  } else if (robotState == TURNING) {
    float error = targetYaw - yawAngle;
    if (abs(error) <= 0.8) {
      Serial2.println("CMD:S"); robotState = IDLE; currentCommand = "STOP";
      lockedHeading = targetYaw; latestMapLog = "Quay xong, Locked=" + String(lockedHeading, 1) + "°";
    } else {
      int turnPower = constrain(abs(error) * 2.5, 50, 90);
      Serial2.println(error > 0 ? "TURNC:L," + String(turnPower) : "TURNC:R," + String(turnPower));
    }
  } else if (robotState == ALIGN_BEFORE_STOP) {
    float error = lockedHeading - yawAngle;
    if (abs(error) <= 0.8) {
      Serial2.println("CMD:S"); robotState = IDLE; currentCommand = "STOP";
      latestMapLog = "Đã căn chuẩn hướng gốc!";
    } else {
      Serial2.println(error > 0 ? "TURNC:L,55" : "TURNC:R,55");
    }
  }

  while (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n'); incoming.trim();
    if (incoming.startsWith("LOG:")) latestMapLog = incoming.substring(4);
    else if (incoming.startsWith("ENC:")) workerEncoderData = incoming.substring(4);
  }

  display.clearDisplay(); display.setCursor(0, 0);
  display.print("Yaw:"); display.print(yawAngle, 1); display.print(" Lck:"); display.println(lockedHeading, 1);
  display.print("US:"); display.print(readMasterUltrasonic()); display.print("cm "); display.println(currentCommand);
  display.display();
  delay(15);
}