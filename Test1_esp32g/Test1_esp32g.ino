/* =====================================================================
 * PROJECT: Maze & Warehouse Differential-Drive Robot (ESP32 Master-Gateway)
 * FILE: ESP32_Master_Gateway.ino
 * VERSION: v1.2.8
 * DATE: 2026-11-14
 * 
 * COMPATIBILITY MATRIX:
 * - Master v1.2.8 <--> Bắt buộc đi kèm Worker v1.2.7
 * 
 * CHANGES IN v1.2.8 (SO VỚI v1.2.7):
 * - Sửa lại logic cập nhật Locked Heading: Chỉ cập nhật khi xe thực hiện xong 
 *   các lệnh quay góc (Trái/Phải 90°/180°), cộng/trừ dồn chuẩn xác theo tọa độ mê cung.
 * - Khi Tiến/Lùi, xe giữ nguyên Locked Heading hiện tại làm mốc bám thẳng tuyệt đối.
 * - Bổ sung truyền tải thông số sai số bù hướng (`Correction`) để hiển thị lên OLED và Web.
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

const char* ssid = "Robot_Warehouse_AP";
const char* password = "12345678";

WebServer server(80);

String currentCommand = "STOP";
float yawAngle = 0;
float targetYaw = 0;
float lockedHeading = 0; // Góc gốc chuẩn của mê cung (0, 90, 180, 270...)
int lastCorrectionVal = 0; // Lưu sai số bù hướng để đẩy lên Web/OLED
unsigned long lastTime = 0;
int16_t gz_offset = 0;
String latestMapLog = "Hệ thống sẵn sàng...";
String workerEncoderData = "X: 0 | Y: 0";

enum RobotState { IDLE, STRAIGHT, TURNING, ALIGN_BEFORE_STOP };
RobotState robotState = IDLE;

const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Maze Robot - v1.2.8 Telemetry</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 10px; background: #f0f2f5; }
    h1 { color: #333; font-size: 17px; margin-bottom: 8px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; max-width: 320px; margin: 0 auto; }
    button { padding: 12px; font-size: 14px; cursor: pointer; border-radius: 8px; border: none; color: white; font-weight: bold; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .btn-forward { background-color: #2e7d32; grid-column: span 2; }
    .btn-left { background-color: #1565c0; }
    .btn-right { background-color: #ef6c00; }
    .btn-left180 { background-color: #0d47a1; }
    .btn-right180 { background-color: #e65100; }
    .btn-backward { background-color: #d32f2f; grid-column: span 2; }
    .btn-go { background-color: #00897b; }
    .btn-stop { background-color: #c2185b; }
    .full-w { grid-column: span 2; }
    button:active { transform: scale(0.96); }
    
    .panel { background: #fff; padding: 10px; border-radius: 8px; max-width: 320px; margin: 10px auto; box-shadow: 0 2px 4px rgba(0,0,0,0.05); text-align: left; font-size: 13px; }
    .panel label { font-weight: bold; color: #444; display: block; margin-bottom: 4px; }
    .panel input { width: 70px; padding: 4px; font-size: 13px; }
    .panel button { padding: 6px 12px; font-size: 12px; background: #3f51b5; margin-top: 5px; }
    
    .status { margin-top: 8px; font-size: 14px; color: #111; font-weight: bold; }
    .log-box { margin-top: 8px; background: #000; color: #0f0; padding: 8px; border-radius: 6px; text-align: left; font-family: monospace; font-size: 11px; height: 55px; overflow-y: auto; max-width: 320px; margin-left: auto; margin-right: auto; }
    .telemetry { background: #e3f2fd; color: #0d47a1; padding: 6px; border-radius: 6px; margin-top: 8px; font-size: 11px; font-weight: bold; max-width: 320px; margin-left: auto; margin-right: auto; line-height: 1.4; }
  </style>
</head>
<body>
  <h1>MAZE ROBOT v1.2.8</h1>
  
  <div class="grid">
    <button class="btn-forward" onclick="sendCmd('FORWARD')">TIẾN</button>
    <button class="btn-left" onclick="sendCmd('TURN_LEFT_90')">QUAY TRÁI 90°</button>
    <button class="btn-right" onclick="sendCmd('TURN_RIGHT_90')">QUAY PHẢI 90°</button>
    <button class="btn-left180" onclick="sendCmd('TURN_LEFT_180')">QUAY TRÁI 180°</button>
    <button class="btn-right180" onclick="sendCmd('TURN_RIGHT_180')">QUAY PHẢI 180°</button>
    <button class="btn-backward" onclick="sendCmd('BACKWARD')">LÙI</button>
    <button class="btn-go full-w" onclick="sendCmd('GO')">ĐI (GO)</button>
    <button class="btn-stop full-w" onclick="sendCmd('STOP')">STOP (DỪNG & CĂN CHỈNH)</button>
  </div>

  <div class="panel">
    <label>Cài đặt Scale Encoder:</label>
    Trái (X): <input type="number" id="sX" step="0.05" value="1.0"> 
    Phải (Y): <input type="number" id="sY" step="0.05" value="1.0">
    <br>
    <button onclick="updateScale()">Cập nhật Scale</button>
  </div>

  <div class="telemetry" id="telemetryBox">Yaw: 0.0° | Locked: 0.0°<br>Sai số bù (Corr): 0 | X:0 Y:0</div>
  <div class="status" id="st">Trạng thái: ĐANG DỪNG</div>
  <div class="log-box" id="logBox">Đang kết nối...</div>

  <script>
    function sendCmd(cmd) {
      fetch('/cmd?action=' + cmd)
        .then(res => res.text())
        .then(val => { document.getElementById("st").innerText = "Lệnh: " + val; });
    }

    function updateScale() {
      let sx = document.getElementById("sX").value;
      let sy = document.getElementById("sY").value;
      fetch('/scale?sx=' + sx + '&sy=' + sy)
        .then(res => res.text())
        .then(val => { alert("Đã cập nhật Scale thành công!"); });
    }

    setInterval(function() {
      fetch('/getdata')
        .then(res => res.json())
        .then(data => {
          document.getElementById("telemetryBox").innerHTML = 
            "Yaw: " + data.yaw + "° | Locked: " + data.locked + "°<br>Sai số bù (Corr): " + data.corr + " | " + data.enc;
          if(data.log) document.getElementById("logBox").innerText = data.log;
        });
    }, 400);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", MAIN_page);
}

void handleCmd() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    currentCommand = action;

    if (action == "FORWARD") {
      Serial2.println("CMD:F");
      robotState = STRAIGHT;
    } else if (action == "BACKWARD") {
      Serial2.println("CMD:B");
      robotState = STRAIGHT;
    } else if (action == "TURN_LEFT_90") {
      targetYaw = lockedHeading + 90.0; // Cộng dồn chuẩn xác vào góc gốc hiện tại
      robotState = TURNING;
    } else if (action == "TURN_RIGHT_90") {
      targetYaw = lockedHeading - 90.0; // Trừ dồn chuẩn xác vào góc gốc hiện tại
      robotState = TURNING;
    } else if (action == "TURN_LEFT_180") {
      targetYaw = lockedHeading + 180.0;
      robotState = TURNING;
    } else if (action == "TURN_RIGHT_180") {
      targetYaw = lockedHeading - 180.0; // Lưu ý: Quay phải 180 cũng chuẩn hóa trừ dồn
      robotState = TURNING;
    } else if (action == "STOP") {
      robotState = ALIGN_BEFORE_STOP;
      latestMapLog = "Đang hiệu chỉnh góc về hướng gốc trước khi dừng...";
    } else if (action == "GO") {
      robotState = STRAIGHT;
    }

    server.send(200, "text/plain; charset=utf-8", currentCommand);
  } else {
    server.send(400, "text/plain; charset=utf-8", "Bad Request");
  }
}

void handleScale() {
  if (server.hasArg("sx") && server.hasArg("sy")) {
    String sx = server.arg("sx");
    String sy = server.arg("sy");
    Serial2.println("SCALE:" + sx + "," + sy);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Scale");
  }
}

void handleGetData() {
  String json = "{";
  json += "\"yaw\":" + String(yawAngle, 1) + ",";
  json += "\"locked\":" + String(lockedHeading, 1) + ",";
  json += "\"corr\":" + String(lastCorrectionVal) + ",";
  json += "\"enc\":\"" + workerEncoderData + "\",";
  json += "\"log\":\"" + latestMapLog + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);
  mpu.initialize();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  long sum = 0;
  for(int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gz_offset = sum / 500;
  yawAngle = 0.0;
  lockedHeading = 0.0; // Khởi tạo góc gốc ban đầu là 0.0 độ

  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
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
  if (abs(gz_deg) > 0.8) {
    yawAngle += gz_deg * dt;
  }

  // 1. ĐI THẲNG: Tính sai số hướng so với góc gốc Locked Heading
  if (robotState == STRAIGHT) {
    float headingError = yawAngle - lockedHeading; 
    int headingKp = 2; // Giảm hệ số Kp xuống 2 để xe nắn hướng từ từ, tránh giật / quay vòng tròn
    lastCorrectionVal = (int)(headingError * headingKp);
    Serial2.println("CORRECTION:" + String(lastCorrectionVal));
  }
  // 2. QUAY GÓC 2 CHIỀU
  else if (robotState == TURNING) {
    float error = targetYaw - yawAngle; 
    if (abs(error) <= 0.8) {
      Serial2.println("CMD:S");
      robotState = IDLE;
      currentCommand = "STOP";
      lockedHeading = targetYaw; // CẬP NHẬT GÓC GỐC MỚI SAU KHI QUAY XONG XUÔI!
      latestMapLog = "Đã quay xong, cập nhật Locked=" + String(lockedHeading, 1) + "°";
    } 
    else {
      int turnPower = constrain(abs(error) * 3.5, 55, 110); // Giảm tốc độ quay để chống trượt quán tính
      if (error > 0) {
        Serial2.println("TURNC:L," + String(turnPower));
      } else {
        Serial2.println("TURNC:R," + String(turnPower));
      }
    }
  }
  // 3. CĂN CHỈNH TRƯỚC KHI DỪNG
  else if (robotState == ALIGN_BEFORE_STOP) {
    float error = lockedHeading - yawAngle; 
    if (abs(error) <= 0.8) {
      Serial2.println("CMD:S");
      robotState = IDLE;
      currentCommand = "STOP";
      lastCorrectionVal = 0;
      latestMapLog = "Đã căn chuẩn hướng gốc, dừng hẳn!";
    } 
    else {
      int alignPower = 60;
      if (error > 0) {
        Serial2.println("TURNC:L," + String(alignPower));
      } else {
        Serial2.println("TURNC:R," + String(alignPower));
      }
    }
  }

  while (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim();
    if (incoming.startsWith("LOG:")) {
      latestMapLog = incoming.substring(4);
    } else if (incoming.startsWith("ENC:")) {
      workerEncoderData = incoming.substring(4);
    }
  }

  // Hiển thị trực quan thông số Yaw, Locked Heading và Sai số bù (Corr) lên OLED 0.96"
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Yaw:"); display.print(yawAngle, 1); display.print(" Lck:"); display.println(lockedHeading, 1);
  display.print("Corr:"); display.print(lastCorrectionVal); display.print(" "); display.println(currentCommand);
  display.print(workerEncoderData);
  display.display();
  
  delay(15);
}