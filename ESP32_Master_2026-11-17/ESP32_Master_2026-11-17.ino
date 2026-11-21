/* =====================================================================
 * PROJECT: Dual-ESP32 Autonomous Maze Solver + 3x US + 2x IR + Canvas Map
 * FILE: ESP32_Master_2026-11-21.ino
 * DATE: 2026-11-21
 * NOTE: IR: Nền đen = 1, Nền trắng = 0.
 *       3x US (Front, Left, Right) kích tuần tự tránh nhiễu chéo.
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

// 3 cảm biến siêu âm HC-SR05 (Trái - Trước - Phải)
const int US_FRONT_TRIG = 13;
const int US_FRONT_ECHO = 14;
const int US_LEFT_TRIG  = 32;
const int US_LEFT_ECHO  = 33;
const int US_RIGHT_TRIG = 25;
const int US_RIGHT_ECHO = 26;

// 2 cảm biến IR đơn: 1 cái chĩa trước (nhận diện line trên vật thể/hàng), 1 cái chĩa xuống sàn (vạch xuất phát)
const int IR_FRONT = 27;
const int IR_FLOOR = 12;

// Ngưỡng khoảng cách (cm) & ngưỡng nhận diện hàng
const int WALL_DIST_CM = 18;
const int CARGO_DIST_CM = 20;

// Hạ deadband để không mất góc khi xe quay chậm gần tới góc đích
const float GYRO_DEADBAND = 0.4;

String currentCommand = "STOP";
float yawAngle = 0;
float targetYaw = 0;
float lockedHeading = 0;
int lastCorrectionVal = 0;
unsigned long lastTime = 0;
float gz_offset = 0;
String latestMapLog = "Sẵn sàng (Chế độ 1: Dò map | Chế độ 2: Chuyển hàng)";
String workerEncoderData = "Enc: X:0 Y:0";

enum RobotState { IDLE, STRAIGHT, TURNING, ALIGN_BEFORE_STOP, CELL_FORWARD };
RobotState robotState = IDLE;

// 0 = Thủ công / Chờ, 1 = Chế độ 1 (Dò mê cung & lưu map), 2 = Chế độ 2 (Chuyển hàng thử nghiệm)
int autoMode = 0;
bool mapSaved = false;
int cellsMovedSinceStart = 0;
unsigned long lastStartLineTrigger = 0;

// Bản đồ lưới 11x11 (tâm tại 5,5 = điểm xuất phát)
// bit 0 (1): tường Bắc, bit 1 (2): tường Đông, bit 2 (4): tường Nam, bit 3 (8): tường Tây, bit 4 (16): có cờ hàng
const int GRID_SIZE = 11;
uint8_t mapGrid[GRID_SIZE][GRID_SIZE];
int robotGX = 5, robotGY = 5;
int robotDir = 0; // 0: Bắc (lên), 1: Đông (phải), 2: Nam (xuống), 3: Tây (trái)

enum AutoStep {
  AUTO_IDLE,
  AUTO_SCAN_AND_DECIDE,
  AUTO_WAIT_REVERSE,
  AUTO_WAIT_TURN,
  AUTO_WAIT_CELL_FWD,
  AUTO_CARGO_DEMO
};
AutoStep autoStep = AUTO_IDLE;
int pendingTurnDelta = 0; // +90 trái, -90 phải, +180 quay đầu
bool pendingReverseBeforeTurn = false;
unsigned long stepTimeoutAt = 0;

long distFront = 999, distLeft = 999, distRight = 999;
int irFrontVal = 0, irFloorVal = 0;

long readUltrasonicPin(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 14000);
  if (duration == 0) return 999;
  long d = duration * 0.034 / 2;
  if (d <= 0 || d > 300) return 999;
  return d;
}

void scanSensorsSequential() {
  // Kích từng cảm biến siêu âm cách nhau ~35ms để không dội sóng chéo
  distFront = readUltrasonicPin(US_FRONT_TRIG, US_FRONT_ECHO);
  delay(35);
  distLeft = readUltrasonicPin(US_LEFT_TRIG, US_LEFT_ECHO);
  delay(35);
  distRight = readUltrasonicPin(US_RIGHT_TRIG, US_RIGHT_ECHO);
  delay(35);

  irFrontVal = digitalRead(IR_FRONT); // 1 = nền đen (line trên vật thể/hàng), 0 = nền trắng
  irFloorVal = digitalRead(IR_FLOOR); // 1 = nền đen (vạch line ngang xuất phát), 0 = nền trắng
}

void markCurrentCellWalls() {
  // Cập nhật tường theo hướng hiện tại của xe
  // Trái = (robotDir + 3) % 4, Phải = (robotDir + 1) % 4, Sau = (robotDir + 2) % 4
  int dLeft = (robotDir + 3) % 4;
  int dRight = (robotDir + 1) % 4;
  int dFront = robotDir;

  if (distLeft > 0 && distLeft < WALL_DIST_CM) mapGrid[robotGY][robotGX] |= (1 << dLeft);
  if (distRight > 0 && distRight < WALL_DIST_CM) mapGrid[robotGY][robotGX] |= (1 << dRight);

  if (distFront > 0 && distFront < WALL_DIST_CM) {
    mapGrid[robotGY][robotGX] |= (1 << dFront);
    // Nếu trước mặt có vật cản/tường kèm line đen trên vật thể -> đánh cờ có hàng
    if (irFrontVal == 1 && distFront <= CARGO_DIST_CM) {
      mapGrid[robotGY][robotGX] |= 16;
      latestMapLog = "Phát hiện HÀNG (Tường + Line vật thể) tại (" + String(robotGX - 5) + "," + String(5 - robotGY) + ")!";
    }
  }
}

void advanceGridPosition() {
  if (robotDir == 0 && robotGY > 0) robotGY--;
  else if (robotDir == 1 && robotGX < GRID_SIZE - 1) robotGX++;
  else if (robotDir == 2 && robotGY < GRID_SIZE - 1) robotGY++;
  else if (robotDir == 3 && robotGX > 0) robotGX--;
  cellsMovedSinceStart++;
}

void updateRobotDirFromTurn(int deltaDeg) {
  int steps = (int)round(deltaDeg / 90.0);
  robotDir = (robotDir - steps) % 4;
  if (robotDir < 0) robotDir += 4;
}

void startTurnToTarget(float newTargetYaw, const String& logMsg) {
  targetYaw = newTargetYaw;
  robotState = TURNING;
  currentCommand = "TURNING";
  latestMapLog = logMsg;
}

void startOneCellForward() {
  Serial2.println("CMD:C");
  robotState = CELL_FORWARD;
  currentCommand = "CELL_FWD";
  stepTimeoutAt = millis() + 5000;
}

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
    
    .tabs { display: flex; max-width: 360px; margin: 0 auto 8px auto; border-radius: 8px; overflow: hidden; border: 1px solid #1565c0; }
    .tab-btn { flex: 1; padding: 10px; font-size: 13px; font-weight: bold; border: none; cursor: pointer; background: #fff; color: #1565c0; }
    .tab-btn.active { background: #1565c0; color: #fff; }
    .page { display: none; max-width: 360px; margin: 0 auto; }
    .page.active { display: block; }

    .mode-bar { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 8px; }
    .btn-mode1 { background: #1b5e20; padding: 10px; font-size: 12px; }
    .btn-mode2 { background: #e65100; padding: 10px; font-size: 12px; }
    .btn-mode-stop { background: #37474f; grid-column: span 2; padding: 8px; font-size: 12px; }

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

    .map-card { background: #fff; padding: 8px; border-radius: 6px; margin-top: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
    .map-title { font-size: 12px; font-weight: bold; margin-bottom: 4px; color: #222; display: flex; justify-content: space-between; }
    canvas { border: 1px solid #bbb; border-radius: 4px; background: #fafafa; display: block; margin: 0 auto; }

    .servo-card { background: #fff; padding: 8px; border-radius: 6px; margin-bottom: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
    .servo-header { display: flex; justify-content: space-between; font-size: 13px; font-weight: bold; margin-bottom: 6px; color: #222; }
    .servo-btns { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
    .step-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px; margin-top: 6px; }
    .btn-step-l { background: #42a5f5; padding: 7px 4px; font-size: 11px; }
    .btn-step-r { background: #ff9800; padding: 7px 4px; font-size: 11px; }
    .btn-twin-fwd { background: #2e7d32; padding: 10px 6px; font-size: 12px; }
    .btn-twin-back { background: #6a1b9a; padding: 10px 6px; font-size: 12px; }
    .btn-special { background: #ad1457; width: 100%; padding: 10px; font-size: 12px; margin-bottom: 8px; }

    .btn-jog-l { background: #1976d2; padding: 10px; font-size: 13px; }
    .btn-jog-r { background: #e65100; padding: 10px; font-size: 13px; }
    .btn-jog-l:active, .btn-jog-r:active, .btn-step-l:active, .btn-step-r:active, .btn-twin-fwd:active, .btn-twin-back:active, .btn-special:active { opacity: 0.75; transform: scale(0.98); }
    .btn-release { background: #607d8b; width: 100%; margin-top: 4px; padding: 9px; font-size: 12px; }

    .telemetry { background: #e3f2fd; color: #0d47a1; padding: 6px; border-radius: 6px; margin-top: 8px; font-size: 11px; font-weight: bold; max-width: 360px; margin-left: auto; margin-right: auto; line-height: 1.3; text-align: left; }
    .log-box { margin-top: 6px; background: #000; color: #0f0; padding: 6px; border-radius: 6px; text-align: left; font-family: monospace; font-size: 10px; height: 52px; overflow-y: auto; max-width: 360px; margin-left: auto; margin-right: auto; }
  </style>
</head>
<body>
  <h1>MAZE SOLVER & ARM CONTROL</h1>

  <div class="tabs">
    <button id="tabCarBtn" class="tab-btn active" onclick="switchTab('car')">1. XE & BẢN ĐỒ</button>
    <button id="tabArmBtn" class="tab-btn" onclick="switchTab('arm')">2. CÁNH TAY 4 BẬC</button>
  </div>

  <!-- TRANG 1: XE & BẢN ĐỒ -->
  <div id="pageCar" class="page active">
    <div class="mode-bar">
      <button class="btn-mode1" onclick="setMode(1)">NÚT 1: DÒ MAP & LƯU</button>
      <button class="btn-mode2" onclick="setMode(2)">NÚT 2: CHUYỂN HÀNG</button>
      <button class="btn-mode-stop" onclick="setMode(0)">DỪNG CHẾ ĐỘ TỰ ĐỘNG</button>
    </div>

    <div class="grid">
      <button class="btn-cell" onclick="sendCmd('CELL_FWD')">TIẾN ĐÚNG 1 Ô (30cm)</button>
      <button class="btn-forward" onclick="sendCmd('FORWARD')">ĐI THẲNG LIÊN TỤC</button>
      <button class="btn-left" onclick="sendCmd('TURN_LEFT_90')">QUAY TRÁI 90°</button>
      <button class="btn-right" onclick="sendCmd('TURN_RIGHT_90')">QUAY PHẢI 90°</button>
      <button class="btn-left180" onclick="sendCmd('TURN_LEFT_180')">QUAY ĐẦU 180° (NGÕ CỤT)</button>
      <button class="btn-reset" onclick="sendCmd('RESET_YAW')">RESET YAW & LOCKED = 0°</button>
      <button class="btn-stop" onclick="sendCmd('STOP')">STOP & CĂN CHỈNH GỐC</button>
    </div>

    <div class="map-card">
      <div class="map-title">
        <span>Bản đồ trực quan (1 ô = 30cm)</span>
        <span id="mapStatus">Chưa lưu</span>
      </div>
      <canvas id="mapCanvas" width="286" height="286"></canvas>
    </div>

    <div class="panel">
      Scale Trái: <input type="number" id="sX" step="0.05" value="1.0"> 
      Phải: <input type="number" id="sY" step="0.05" value="1.0">
      <button onclick="updateScale()" style="padding:4px 8px; background:#3f51b5; color:#fff; border:none; border-radius:4px;">Lưu</button>
    </div>
  </div>

  <!-- TRANG 2: ĐIỀU KHIỂN CÁNH TAY -->
  <div id="pageArm" class="page">
    <button class="btn-special" onclick="runSpecialMode()">CHẾ ĐỘ ĐẶC BIỆT (S2: 180° [0.5s] -> 90° x3 LẦN)</button>

    <div class="servo-card">
      <div class="servo-header"><span>S1 - Tay gắp (Chạy từ từ 90° / 0°)</span><span id="ang1">90°</span></div>
      <div class="servo-btns">
        <button class="btn-jog-l" onclick="sweepGripTo(90)">ĐÓNG TỪ TỪ (90°)</button>
        <button class="btn-jog-r" onclick="sweepGripTo(0)">MỞ TỪ TỪ (0°)</button>
      </div>
    </div>

    <div class="servo-card">
      <div class="servo-header"><span>S2 + S3 Đồng bộ (Ngược dấu nhau)</span><span id="twinState">S2:90° | S3:90°</span></div>
      <div class="servo-btns">
        <button class="btn-twin-fwd" onpointerdown="startTwinJog(1)" onpointerup="stopTwinJog()" onpointerleave="stopTwinJog()" onpointercancel="stopTwinJog()">▲ CÙNG ĐƯA TỚI</button>
        <button class="btn-twin-back" onpointerdown="startTwinJog(-1)" onpointerup="stopTwinJog()" onpointerleave="stopTwinJog()" onpointercancel="stopTwinJog()">▼ CÙNG THU VỀ</button>
      </div>
    </div>

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

    function setMode(m) { fetch('/mode?m=' + m); }
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

    function sweepGripTo(targetAngle) {
      stopJog();
      stopTwinJog();
      servoAngles[1] = targetAngle;
      document.getElementById('ang1').innerText = targetAngle + '°';
      fetch('/arm_grip_sweep?target=' + targetAngle);
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

    function runSpecialMode() {
      stopJog();
      stopTwinJog();
      servoAngles[2] = 90;
      document.getElementById('ang2').innerText = '90°';
      updateTwinLabel();
      fetch('/arm_special');
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

    function drawMap(gridArr, gx, gy, dir) {
      const canvas = document.getElementById('mapCanvas');
      const ctx = canvas.getContext('2d');
      const N = 11;
      const cellPx = 26;
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      for (let y = 0; y < N; y++) {
        for (let x = 0; x < N; x++) {
          const px = x * cellPx;
          const py = y * cellPx;
          const val = gridArr[y * N + x] || 0;

          // Ô xuất phát (5,5)
          if (x === 5 && y === 5) {
            ctx.fillStyle = '#e8f5e9';
            ctx.fillRect(px, py, cellPx, cellPx);
          }

          // Cờ có hàng (bit 4 = 16)
          if ((val & 16) !== 0) {
            ctx.fillStyle = '#ffe082';
            ctx.fillRect(px + 3, py + 3, cellPx - 6, cellPx - 6);
            ctx.fillStyle = '#bf360c';
            ctx.font = 'bold 9px sans-serif';
            ctx.fillText('HÀNG', px + 3, py + 16);
          }

          // Lưới mờ
          ctx.strokeStyle = '#e0e0e0';
          ctx.lineWidth = 1;
          ctx.strokeRect(px, py, cellPx, cellPx);

          // Tường đã đo (0: Bắc, 1: Đông, 2: Nam, 3: Tây)
          ctx.strokeStyle = '#212121';
          ctx.lineWidth = 2.5;
          if (val & 1) { ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px + cellPx, py); ctx.stroke(); }
          if (val & 2) { ctx.beginPath(); ctx.moveTo(px + cellPx, py); ctx.lineTo(px + cellPx, py + cellPx); ctx.stroke(); }
          if (val & 4) { ctx.beginPath(); ctx.moveTo(px, py + cellPx); ctx.lineTo(px + cellPx, py + cellPx); ctx.stroke(); }
          if (val & 8) { ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py + cellPx); ctx.stroke(); }
        }
      }

      // Vẽ xe + hướng mũi tên
      const cx = gx * cellPx + cellPx / 2;
      const cy = gy * cellPx + cellPx / 2;
      ctx.fillStyle = '#d32f2f';
      ctx.beginPath();
      ctx.arc(cx, cy, 7, 0, Math.PI * 2);
      ctx.fill();

      let dx = 0, dy = 0;
      if (dir === 0) dy = -10;
      else if (dir === 1) dx = 10;
      else if (dir === 2) dy = 10;
      else if (dir === 3) dx = -10;
      ctx.strokeStyle = '#ffeb3b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + dx, cy + dy);
      ctx.stroke();
    }

    setInterval(function() {
      fetch('/getdata').then(res => res.json()).then(data => {
        document.getElementById("telemetryBox").innerHTML = 
          "Mode: " + data.mode + " | Yaw: " + data.yaw + "° | Lck: " + data.locked + "°<br>" + data.sensor;
        document.getElementById("mapStatus").innerText = data.saved ? "Đã lưu bản đồ" : "Đang cập nhật";
        if (data.log) document.getElementById("logBox").innerText = data.log;
        if (data.map) drawMap(data.map, data.gx, data.gy, data.dir);
      });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html; charset=utf-8", MAIN_page); }

void handleMode() {
  if (server.hasArg("m")) {
    int m = server.arg("m").toInt();
    autoMode = constrain(m, 0, 2);
    if (autoMode == 0) {
      autoStep = AUTO_IDLE;
      robotState = IDLE;
      currentCommand = "STOP";
      Serial2.println("CMD:S");
      latestMapLog = "Đã dừng chế độ tự động.";
    } else if (autoMode == 1) {
      // Khởi động Chế độ 1: reset bản đồ & bắt đầu dò từ ô xuất phát (5,5)
      memset(mapGrid, 0, sizeof(mapGrid));
      robotGX = 5; robotGY = 5; robotDir = 0;
      cellsMovedSinceStart = 0;
      mapSaved = false;
      autoStep = AUTO_SCAN_AND_DECIDE;
      latestMapLog = "Bắt đầu Chế độ 1: Dò mê cung (1 ô = 30cm)...";
    } else if (autoMode == 2) {
      // Nút 2: Chế độ chuyển hàng (thử nghiệm)
      autoStep = AUTO_CARGO_DEMO;
      robotState = IDLE;
      currentCommand = "CARGO_TEST";
      latestMapLog = mapSaved ? "Chế độ 2 (Thử nghiệm): Đã tải bản đồ lưu, sẵn sàng đi tới ô gắn cờ HÀNG."
                              : "Chế độ 2 (Thử nghiệm): Chưa có bản đồ lưu, chạy thử nghiệm trên lưới hiện tại.";
    }
    server.send(200, "text/plain", "OK");
  }
}

void handleCmd() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    autoMode = 0;
    autoStep = AUTO_IDLE;
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
    latestMapLog = "Servo S" + String(id) + " -> " + String(ang) + " (0.5s ngắt)";
    server.send(200, "text/plain", "OK");
  }
}

void handleArmTwin() {
  if (server.hasArg("s2") && server.hasArg("s3")) {
    int a2 = constrain(server.arg("s2").toInt(), 0, 180);
    int a3 = constrain(server.arg("s3").toInt(), 0, 180);
    Serial2.println("ARMT:" + String(a2) + "," + String(a3));
    latestMapLog = "Twin S2:" + String(a2) + "° S3:" + String(a3) + "° (0.5s ngắt)";
    server.send(200, "text/plain", "OK");
  }
}

void handleArmGripSweep() {
  if (server.hasArg("target")) {
    int target = constrain(server.arg("target").toInt(), 0, 90);
    Serial2.println("GRIPS:" + String(target));
    latestMapLog = "Tay gắp chạy từ từ -> " + String(target) + "°";
    server.send(200, "text/plain", "OK");
  }
}

void handleArmSpecial() {
  Serial2.println("ARMSP");
  latestMapLog = "Chế độ đặc biệt: S2 180° -> 90° x3 (mỗi bước 0.5s & ngắt xung)";
  server.send(200, "text/plain", "OK");
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

void handleGetData() {
  String sensorInfo = "US L:" + String(distLeft) + " F:" + String(distFront) + " R:" + String(distRight) +
                      " | IRF:" + String(irFrontVal) + " IRS:" + String(irFloorVal) + "<br>" + workerEncoderData;

  String mapArrJson = "[";
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      mapArrJson += String(mapGrid[y][x]);
      if (!(y == GRID_SIZE - 1 && x == GRID_SIZE - 1)) mapArrJson += ",";
    }
  }
  mapArrJson += "]";

  String json = "{";
  json += "\"mode\":" + String(autoMode) + ",";
  json += "\"saved\":" + String(mapSaved ? "true" : "false") + ",";
  json += "\"yaw\":" + String(yawAngle, 1) + ",";
  json += "\"locked\":" + String(lockedHeading, 1) + ",";
  json += "\"gx\":" + String(robotGX) + ",";
  json += "\"gy\":" + String(robotGY) + ",";
  json += "\"dir\":" + String(robotDir) + ",";
  json += "\"sensor\":\"" + sensorInfo + "\",";
  json += "\"log\":\"" + latestMapLog + "\",";
  json += "\"map\":" + mapArrJson;
  json += "}";
  server.send(200, "application/json", json);
}

void runAutonomousMode1Step() {
  unsigned long now = millis();

  if (autoStep == AUTO_SCAN_AND_DECIDE) {
    scanSensorsSequential();
    markCurrentCellWalls();

    // Kiểm tra quay về vạch xuất phát (vạch line ngang dưới sàn = IR_FLOOR 1 sau khi đã đi ít nhất 2 ô)
    if (cellsMovedSinceStart >= 2 && irFloorVal == 1 && (now - lastStartLineTrigger > 2500)) {
      lastStartLineTrigger = now;
      Serial2.println("CMD:S");
      robotState = IDLE;
      currentCommand = "STOP";
      autoMode = 0;
      autoStep = AUTO_IDLE;
      mapSaved = true;
      latestMapLog = "Đã về vạch xuất phát (line ngang) & LƯU BẢN ĐỒ! Sẵn sàng Nút 2.";
      return;
    }

    bool leftClear = (distLeft >= WALL_DIST_CM);
    bool frontClear = (distFront >= WALL_DIST_CM);
    bool rightClear = (distRight >= WALL_DIST_CM);

    // Thuật toán dò mê cung ưu tiên: Trái -> Thẳng -> Phải -> Lùi nhẹ rồi Quay đầu 180°
    if (leftClear) {
      pendingTurnDelta = 90;
      pendingReverseBeforeTurn = false;
      startTurnToTarget(lockedHeading + 90.0, "Dò: Trái trống -> Quay trái 90°");
      autoStep = AUTO_WAIT_TURN;
    } else if (frontClear) {
      startOneCellForward();
      autoStep = AUTO_WAIT_CELL_FWD;
    } else if (rightClear) {
      pendingTurnDelta = -90;
      pendingReverseBeforeTurn = false;
      startTurnToTarget(lockedHeading - 90.0, "Dò: Phải trống -> Quay phải 90°");
      autoStep = AUTO_WAIT_TURN;
    } else {
      // Có vật cản/ngõ cụt phía trước: lùi nhẹ 1 tẹo rồi mới quay xe 180°
      pendingTurnDelta = 180;
      pendingReverseBeforeTurn = true;
      Serial2.println("CMD:R");
      currentCommand = "REV_微";
      stepTimeoutAt = now + 260;
      autoStep = AUTO_WAIT_REVERSE;
      latestMapLog = "Thấy vật cản/ngõ cụt -> Lùi nhẹ rồi quay đầu 180°...";
    }
  }
  else if (autoStep == AUTO_WAIT_REVERSE) {
    if (now >= stepTimeoutAt) {
      Serial2.println("CMD:S");
      startTurnToTarget(lockedHeading + pendingTurnDelta, "Đã lùi nhẹ -> Đang quay 180°...");
      autoStep = AUTO_WAIT_TURN;
    }
  }
  else if (autoStep == AUTO_WAIT_TURN) {
    if (robotState == IDLE) {
      updateRobotDirFromTurn(pendingTurnDelta);
      startOneCellForward();
      autoStep = AUTO_WAIT_CELL_FWD;
    }
  }
  else if (autoStep == AUTO_WAIT_CELL_FWD) {
    if (robotState == IDLE) {
      advanceGridPosition();
      autoStep = AUTO_SCAN_AND_DECIDE;
    } else if (now > stepTimeoutAt) {
      Serial2.println("CMD:S");
      robotState = IDLE;
      advanceGridPosition();
      autoStep = AUTO_SCAN_AND_DECIDE;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Wire.begin(21, 22);
  mpu.initialize();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

  pinMode(US_FRONT_TRIG, OUTPUT); pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_LEFT_TRIG, OUTPUT);  pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_RIGHT_TRIG, OUTPUT); pinMode(US_RIGHT_ECHO, INPUT);

  pinMode(IR_FRONT, INPUT_PULLUP);
  pinMode(IR_FLOOR, INPUT_PULLUP);

  memset(mapGrid, 0, sizeof(mapGrid));

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
  server.on("/mode", handleMode);
  server.on("/cmd", handleCmd);
  server.on("/arm_single", handleArmSingle);
  server.on("/arm_twin", handleArmTwin);
  server.on("/arm_grip_sweep", handleArmGripSweep);
  server.on("/arm_special", handleArmSpecial);
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

  if (autoMode == 1) {
    runAutonomousMode1Step();
  }

  if (robotState == STRAIGHT || robotState == CELL_FORWARD) {
    float headingError = yawAngle - lockedHeading;
    lastCorrectionVal = (int)(headingError * 2);
    Serial2.println("CORRECTION:" + String(lastCorrectionVal));
  } else if (robotState == TURNING) {
    float error = targetYaw - yawAngle;
    if (abs(error) <= 1.2) {
      Serial2.println("CMD:S");
      robotState = IDLE;
      currentCommand = "STOP";
      lockedHeading = targetYaw;
      latestMapLog = "Quay xong, Locked=" + String(lockedHeading, 1) + "°";
    } else {
      int turnPower = constrain(abs(error) * 2.5, 65, 130);
      Serial2.println(error > 0 ? "TURNC:L," + String(turnPower) : "TURNC:R," + String(turnPower));
    }
  } else if (robotState == ALIGN_BEFORE_STOP) {
    float error = lockedHeading - yawAngle;
    if (abs(error) <= 1.2) {
      Serial2.println("CMD:S");
      robotState = IDLE;
      currentCommand = "STOP";
      latestMapLog = "Đã căn chuẩn hướng gốc!";
    } else {
      Serial2.println(error > 0 ? "TURNC:L,70" : "TURNC:R,70");
    }
  }

  while (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n'); incoming.trim();
    if (incoming.startsWith("LOG:")) latestMapLog = incoming.substring(4);
    else if (incoming.startsWith("ENC:")) workerEncoderData = incoming.substring(4);
    else if (incoming == "CELL_DONE") {
      if (robotState == CELL_FORWARD) {
        robotState = IDLE;
        currentCommand = "STOP";
      }
    }
  }

  display.clearDisplay(); display.setCursor(0, 0);
  display.print("M:"); display.print(autoMode);
  display.print(" Y:"); display.print(yawAngle, 0);
  display.print(" L:"); display.println(lockedHeading, 0);
  display.print("F:"); display.print(distFront);
  display.print(" IRF:"); display.print(irFrontVal);
  display.print(" IRS:"); display.println(irFloorVal);
  display.display();
  delay(15);
}