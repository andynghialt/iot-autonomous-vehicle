/* =====================================================================
 * PROJECT: Maze Solver Master-Gateway
 * FILE: ESP32_Master_2026-11-17.ino
 * DATE: 2026-11-17
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

String currentCommand = "STOP";
float yawAngle = 0;
float targetYaw = 0;
float lockedHeading = 0; 
int lastCorrectionVal = 0;
unsigned long lastTime = 0;
int16_t gz_offset = 0;
String latestMapLog = "Sẵn sàng giải mê cung...";
String workerEncoderData = "Enc: X:0 Y:0";

enum RobotState { IDLE, STRAIGHT, TURNING, ALIGN_BEFORE_STOP, CELL_FORWARD };
RobotState robotState = IDLE;

const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Maze Solver Master</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 10px; background: #f0f2f5; }
    h1 { color: #333; font-size: 16px; margin-bottom: 6px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; max-width: 320px; margin: 0 auto; }
    button { padding: 10px; font-size: 13px; cursor: pointer; border-radius: 6px; border: none; color: white; font-weight: bold; }
    .btn-forward { background-color: #2e7d32; grid-column: span 2; }
    .btn-cell { background-color: #00796b; grid-column: span 2; }
    .btn-left { background-color: #1565c0; }
    .btn-right { background-color: #ef6c00; }
    .btn-left180 { background-color: #0d47a1; grid-column: span 2; }
    .btn-stop { background-color: #c2185b; grid-column: span 2; }
    
    .panel { background: #fff; padding: 8px; border-radius: 6px; max-width: 320px; margin: 8px auto; text-align: left; font-size: 12px; }
    .panel input { width: 65px; padding: 3px; }
    .telemetry { background: #e3f2fd; color: #0d47a1; padding: 6px; border-radius: 6px; margin-top: 6px; font-size: 11px; font-weight: bold; max-width: 320px; margin-left: auto; margin-right: auto; line-height: 1.3; text-align: left; }
    .log-box { margin-top: 6px; background: #000; color: #0f0; padding: 6px; border-radius: 6px; text-align: left; font-family: monospace; font-size: 10px; height: 50px; overflow-y: auto; max-width: 320px; margin-left: auto; margin-right: auto; }
  </style>
</head>
<body>
  <h1>MAZE SOLVER MASTER</h1>
  
  <div class="grid">
    <button class="btn-cell" onclick="sendCmd('CELL_FWD')">TIẾN ĐÚNG 1 Ô (30cm)</button>
    <button class="btn-forward" onclick="sendCmd('FORWARD')">ĐI THẲNG LIÊN TỤC</button>
    <button class="btn-left" onclick="sendCmd('TURN_LEFT_90')">QUAY TRÁI 90°</button>
    <button class="btn-right" onclick="sendCmd('TURN_RIGHT_90')">QUAY PHẢI 90°</button>
    <button class="btn-left180" onclick="sendCmd('TURN_LEFT_180')">QUAY ĐẦU 180° (NGÕ CỤT)</button>
    <button class="btn-stop" onclick="sendCmd('STOP')">STOP & CĂN CHỈNH GỐC</button>
  </div>

  <div class="panel">
    Scale Trái: <input type="number" id="sX" step="0.05" value="1.0"> 
    Phải: <input type="number" id="sY" step="0.05" value="1.0">
    <button onclick="updateScale()" style="padding:4px 8px; background:#3f51b5; color:#fff; border:none; border-radius:4px;">Lưu</button>
  </div>

  <div class="telemetry" id="telemetryBox">Đang chờ lệnh...</div>
  <div class="log-box" id="logBox">Đang kết nối...</div>

  <script>
    function sendCmd(cmd) { fetch('/cmd?action=' + cmd); }
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
    }, 400);
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
    else if (action == "STOP") { robotState = ALIGN_BEFORE_STOP; latestMapLog = "Đang căn chỉnh về gốc..."; }
    server.send(200, "text/plain", currentCommand);
  }
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

  long sum = 0;
  for(int i = 0; i < 500; i++) { int16_t ax, ay, az, gx, gy, gz; mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); sum += gz; delay(2); }
  gz_offset = sum / 500;
  yawAngle = 0.0; lockedHeading = 0.0;

  WiFi.softAP(ssid, password);
  server.on("/", handleRoot); server.on("/cmd", handleCmd); server.on("/scale", handleScale); server.on("/getdata", handleGetData);
  server.begin();
  lastTime = millis();
}

void loop() {
  server.handleClient();
  int16_t ax, ay, az, gx, gy, gz; mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long now = millis(); float dt = (now - lastTime) / 1000.0; lastTime = now;
  float gz_deg = (gz - gz_offset) / 131.0;
  if (abs(gz_deg) > 0.8) yawAngle += gz_deg * dt;

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
      int turnPower = constrain(abs(error) * 3.5, 55, 110);
      Serial2.println(error > 0 ? "TURNC:L," + String(turnPower) : "TURNC:R," + String(turnPower));
    }
  } else if (robotState == ALIGN_BEFORE_STOP) {
    float error = lockedHeading - yawAngle;
    if (abs(error) <= 0.8) {
      Serial2.println("CMD:S"); robotState = IDLE; currentCommand = "STOP";
      latestMapLog = "Đã căn chuẩn hướng gốc!";
    } else {
      Serial2.println(error > 0 ? "TURNC:L,60" : "TURNC:R,60");
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