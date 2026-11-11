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

// Trạng thái điều khiển góc tự động
enum RobotState { IDLE, TURNING };
RobotState robotState = IDLE;

// Giao diện Web có thêm nút Quay 180 độ
const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Điều Khiển Robot Kho Hàng</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 10px; background: #f0f2f5; }
    h1 { color: #333; font-size: 18px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; max-width: 320px; margin: 0 auto; }
    button { padding: 14px; font-size: 14px; cursor: pointer; border-radius: 8px; border: none; color: white; font-weight: bold; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
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
    .status { margin-top: 12px; font-size: 15px; color: #111; font-weight: bold; }
    .log-box { margin-top: 10px; background: #000; color: #0f0; padding: 8px; border-radius: 6px; text-align: left; font-family: monospace; font-size: 11px; height: 75px; overflow-y: auto; max-width: 320px; margin-left: auto; margin-right: auto; }
  </style>
</head>
<body>
  <h1>ĐIỀU KHIỂN ROBOT CHUẨN IMU</h1>
  
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

  <div class="status" id="st">Trạng thái: ĐANG DỪNG</div>
  <div class="log-box" id="logBox">Đang kết nối...</div>

  <script>
    function sendCmd(cmd) {
      fetch('/cmd?action=' + cmd)
        .then(res => res.text())
        .then(val => { document.getElementById("st").innerText = "Lệnh: " + val; });
    }
    setInterval(function() {
      fetch('/getlog')
        .then(res => res.text())
        .then(data => { if(data.trim() !== "") document.getElementById("logBox").innerText = data; });
    }, 1000);
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
      Serial2.write('F');
      robotState = IDLE;
    } else if (action == "BACKWARD") {
      Serial2.write('B');
      robotState = IDLE;
    } else if (action == "TURN_LEFT_90") {
      targetYaw = yawAngle + 90.0; // Mốc góc mục tiêu quay trái
      Serial2.write('L');
      robotState = TURNING;
    } else if (action == "TURN_RIGHT_90") {
      targetYaw = yawAngle - 90.0; // Mốc góc mục tiêu quay phải
      Serial2.write('R');
      robotState = TURNING;
    } else if (action == "TURN_LEFT_180") {
      targetYaw = yawAngle + 180.0;
      Serial2.write('L');
      robotState = TURNING;
    } else if (action == "TURN_RIGHT_180") {
      targetYaw = yawAngle - 180.0;
      Serial2.write('R');
      robotState = TURNING;
    } else if (action == "STOP") {
      Serial2.write('S');
      robotState = IDLE;
    } else if (action == "GO") {
      Serial2.write('G');
      robotState = IDLE;
    }

    server.send(200, "text/plain; charset=utf-8", currentCommand);
  } else {
    server.send(400, "text/plain; charset=utf-8", "Bad Request");
  }
}

void handleGetLog() {
  server.send(200, "text/plain; charset=utf-8", latestMapLog);
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

  // Hiệu chuẩn IMU ban đầu lấy điểm tham chiếu (Yaw = 0)
  long sum = 0;
  for(int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gz_offset = sum / 200;
  yawAngle = 0.0; // Thiết lập mốc tham chiếu ban đầu

  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/getlog", handleGetLog);
  server.begin();

  lastTime = millis();
}

void loop() {
  server.handleClient();

  // Đọc dữ liệu IMU liên tục
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gz_deg = (gz - gz_offset) / 131.0;
  if (abs(gz_deg) > 0.5) { // Lọc nhiễu tĩnh
    yawAngle += gz_deg * dt;
  }

  // Vòng lặp giám sát góc quay vòng kín (Closed-Loop Turning)
  if (robotState == TURNING) {
    if (currentCommand.indexOf("LEFT") != -1) {
      if (yawAngle >= targetYaw) {
        Serial2.write('S'); // Đạt góc mục tiêu -> Gửi lệnh dừng ngay lập tức
        robotState = IDLE;
        currentCommand = "STOP";
        latestMapLog = "Đã quay xong đúng góc!";
      }
    } else if (currentCommand.indexOf("RIGHT") != -1) {
      if (yawAngle <= targetYaw) {
        Serial2.write('S'); // Đạt góc mục tiêu -> Gửi lệnh dừng ngay lập tức
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
    }
  }

  // Hiển thị đầy đủ thông tin IMU lên LCD 0.98 inch để kiểm tra trực tiếp
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Cmd: "); display.println(currentCommand);
  display.print("Yaw: "); display.print(yawAngle, 1); display.println(" deg");
  display.print("Target: "); display.print(targetYaw, 1); display.println(" deg");
  display.display();
  
  delay(15);
}