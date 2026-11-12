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
unsigned long lastTime = 0;
int16_t gz_offset = 0;
String latestMapLog = "Hệ thống sẵn sàng...";

// Biến lưu thông số nhận từ Worker để đẩy lên Web real-time
String workerEncoderData = "X: 0 | Y: 0";

enum RobotState { IDLE, TURNING };
RobotState robotState = IDLE;

const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Điều Khiển Robot Kho Hàng & Real-time Dashboard</title>
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
    .log-box { margin-top: 8px; background: #000; color: #0f0; padding: 8px; border-radius: 6px; text-align: left; font-family: monospace; font-size: 11px; height: 65px; overflow-y: auto; max-width: 320px; margin-left: auto; margin-right: auto; }
    .telemetry { background: #e3f2fd; color: #0d47a1; padding: 6px; border-radius: 6px; margin-top: 8px; font-size: 12px; font-weight: bold; max-width: 320px; margin-left: auto; margin-right: auto; }
  </style>
</head>
<body>
  <h1>ĐIỀU KHIỂN & TELEMETRY ROBOT</h1>
  
  <div class="grid">
    <button class="btn-forward" onclick="sendCmd('FORWARD')">TIẾN</button>
    <button class="btn-left" onclick="sendCmd('TURN_LEFT_90')">QUAY TRÁI 90°</button>
    <button class="btn-right" onclick="sendCmd('TURN_RIGHT_90')">QUAY PHẢI 90°</button>
    <button class="btn-left180" onclick="sendCmd('TURN_LEFT_180')">QUAY TRÁI 180°</button>
    <button class="btn-right180" onclick="sendCmd('TURN_RIGHT_180')">QUAY PHẢI 180°</button>
    <button class="btn-backward" onclick="sendCmd('BACKWARD')">LÙI</button>
    <button class="btn-go full-w" onclick="sendCmd('GO')">ĐI (GO)</button>
    <button class="btn-stop full-w" onclick="sendCmd('STOP')">STOP (DỪNG)</button>
  </div>

  <div class="panel">
    <label>Cài đặt Scale Encoder:</label>
    Trái (X): <input type="number" id="sX" step="0.05" value="1.0"> 
    Phải (Y): <input type="number" id="sY" step="0.05" value="1.0">
    <br>
    <button onclick="updateScale()">Cập nhật Scale</button>
  </div>

  <div class="telemetry" id="telemetryBox">IMU Yaw: 0.0° | Encoder: X:0 Y:0</div>
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
          document.getElementById("telemetryBox").innerText = "Yaw: " + data.yaw + "° | " + data.enc;
          if(data.log) document.getElementById("logBox").innerText = data.log;
        });
    }, 500);
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
      robotState = IDLE;
    } else if (action == "BACKWARD") {
      Serial2.println("CMD:B");
      robotState = IDLE;
    } else if (action == "TURN_LEFT_90") {
      targetYaw = yawAngle + 90.0;
      Serial2.println("CMD:L");
      robotState = TURNING;
    } else if (action == "TURN_RIGHT_90") {
      targetYaw = yawAngle - 90.0;
      Serial2.println("CMD:R");
      robotState = TURNING;
    } else if (action == "TURN_LEFT_180") {
      targetYaw = yawAngle + 180.0;
      Serial2.println("CMD:L");
      robotState = TURNING;
    } else if (action == "TURN_RIGHT_180") {
      targetYaw = yawAngle - 180.0;
      Serial2.println("CMD:R");
      robotState = TURNING;
    } else if (action == "STOP") {
      Serial2.println("CMD:S");
      robotState = IDLE;
    } else if (action == "GO") {
      Serial2.println("CMD:G");
      robotState = IDLE;
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

  // Lấy 500 mẫu để khử triệt để sai số trôi tĩnh (drift) của Gyro ban đầu
  long sum = 0;
  for(int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gz_offset = sum / 500;
  yawAngle = 0.0;

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
  if (abs(gz_deg) > 0.8) { // Tăng ngưỡng lọc lên 0.8 độ/s để chống trôi khi xe đứng yên
    yawAngle += gz_deg * dt;
  }

  // Vòng lặp giám sát góc quay vòng kín (Closed-Loop Turning) chuẩn IMU
  if (robotState == TURNING) {
    if (currentCommand.indexOf("LEFT") != -1) {
      if (yawAngle >= targetYaw) {
        Serial2.println("CMD:S");
        robotState = IDLE;
        currentCommand = "STOP";
        latestMapLog = "Đã quay xong đúng góc!";
      }
    } else if (currentCommand.indexOf("RIGHT") != -1) {
      if (yawAngle <= targetYaw) {
        Serial2.println("CMD:S");
        robotState = IDLE;
        currentCommand = "STOP";
        latestMapLog = "Đã quay xong đúng góc!";
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

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Cmd: "); display.println(currentCommand);
  display.print("Yaw: "); display.print(yawAngle, 1); display.println(" deg");
  display.print(workerEncoderData);
  display.display();
  
  delay(15);
}