#include <Arduino.h>

const int PWMA = 4;
const int AIN1 = 13;
const int AIN2 = 14; // Đã đổi chân AIN2 thành chân 14 theo yêu cầu
const int STBY = 27;
const int BIN1 = 26;
const int BIN2 = 19;
const int PWMB = 23;

const int ENC_X_A = 25;
const int ENC_X_B = 33;
const int ENC_Y_A = 32;
const int ENC_Y_B = 35;

volatile long encoderXCount = 0; // Động cơ Trái (X)
volatile long encoderYCount = 0; // Động cơ Phải (Y)

// Biến Scale nhận từ Web (Có thể tinh chỉnh trực tiếp trên giao diện)
float scaleX = 1.0; 
float scaleY = 1.0; 

String currentAction = "STOP";
String lastActiveAction = "FORWARD";
unsigned long lastSendTime = 0;

void IRAM_ATTR isrEncoderX() {
  if (digitalRead(ENC_X_B) == HIGH) encoderXCount++;
  else encoderXCount--;
}

void IRAM_ATTR isrEncoderY() {
  if (digitalRead(ENC_Y_B) == HIGH) encoderYCount++;
  else encoderYCount--;
}

void stopRobot() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

// Hàm điều khiển động cơ Trái (X) tách bạch để tránh liệt chân 14 (AIN2)
void setMotorLeft(int dir, int speed) {
  if (dir == 1) { // Tiến
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (dir == -1) { // Lùi
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else { // Dừng
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, speed);
}

// Hàm điều khiển động cơ Phải (Y)
void setMotorRight(int dir, int speed) {
  if (dir == 1) { // Tiến
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (dir == -1) { // Lùi
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else { // Dừng
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, speed);
}

void moveStraight(int baseSpeed) {
  long normalizedX = encoderXCount * scaleX;
  long normalizedY = encoderYCount * scaleY;

  long error = normalizedY - normalizedX; 
  int kp = 3; 
  int correction = error * kp;

  // ĐÃ BỎ HỆ SỐ 1.15, DÙNG TRỰC TIẾP baseSpeed VÀ DÙNG SCALE ĐỂ CÂN BẰNG
  int baseSpeedX = baseSpeed; 
  int baseSpeedY = baseSpeed;       

  int speedX = baseSpeedX + correction; 
  int speedY = baseSpeedY - correction; 

  speedX = constrain(speedX, 100, 240);
  speedY = constrain(speedY, 100, 240);

  // Cả hai cùng tiến (dir = 1)
  setMotorLeft(1, speedX);
  setMotorRight(1, speedY);
}

void moveBackward(int baseSpeed) {
  long normalizedX = encoderXCount * scaleX;
  long normalizedY = encoderYCount * scaleY;

  long error = normalizedY - normalizedX;
  int kp = 3;
  int correction = error * kp;

  int baseSpeedX = baseSpeed;
  int baseSpeedY = baseSpeed;

  int speedX = baseSpeedX + correction;
  int speedY = baseSpeedY - correction;

  speedX = constrain(speedX, 100, 240);
  speedY = constrain(speedY, 100, 240);

  // Cả hai cùng lùi (dir = -1)
  setMotorLeft(-1, speedX);
  setMotorRight(-1, speedY);
}

void turnLeft(int speed) {
  // Quay trái: Bánh trái lùi (-1), Bánh phải tiến (1)
  setMotorLeft(-1, speed);
  setMotorRight(1, speed);
}

void turnRight(int speed) {
  // Quay phải: Bánh trái tiến (1), Bánh phải lùi (-1)
  setMotorLeft(1, speed);
  setMotorRight(-1, speed);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_X_A, INPUT_PULLUP); pinMode(ENC_X_B, INPUT_PULLUP);
  pinMode(ENC_Y_A, INPUT_PULLUP); pinMode(ENC_Y_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_X_A), isrEncoderX, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Y_A), isrEncoderY, RISING);

  stopRobot();
}

void loop() {
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n');
    incoming.trim();

    if (incoming.startsWith("CMD:")) {
      char cmd = incoming.charAt(4);
      if (cmd == 'F') {
        encoderXCount = 0; encoderYCount = 0;
        currentAction = "FORWARD";
        lastActiveAction = "FORWARD";
        Serial2.println("LOG:Đang đi TIẾN");
      } 
      else if (cmd == 'B') {
        encoderXCount = 0; encoderYCount = 0;
        currentAction = "BACKWARD";
        lastActiveAction = "BACKWARD";
        Serial2.println("LOG:Đang LÙI");
      } 
      else if (cmd == 'L') {
        currentAction = "TURN_LEFT";
        Serial2.println("LOG:Đang quay TRÁI chuẩn IMU");
      } 
      else if (cmd == 'R') {
        currentAction = "TURN_RIGHT";
        Serial2.println("LOG:Đang quay PHẢI chuẩn IMU");
      } 
      else if (cmd == 'S') {
        currentAction = "STOP";
        Serial2.println("LOG:Đã STOP");
      } 
      else if (cmd == 'G') {
        currentAction = lastActiveAction;
        Serial2.println("LOG:Tiếp tục GO");
      }
    } 
    else if (incoming.startsWith("SCALE:")) {
      int commaIndex = incoming.indexOf(',');
      if (commaIndex != -1) {
        scaleX = incoming.substring(6, commaIndex).toFloat();
        scaleY = incoming.substring(commaIndex + 1).toFloat();
        Serial2.println("LOG:Đã cập nhật Scale X=" + String(scaleX) + " Y=" + String(scaleY));
      }
    }
  }

  if (currentAction == "FORWARD") {
    moveStraight(170);
  } 
  else if (currentAction == "BACKWARD") {
    moveBackward(170);
  } 
  else if (currentAction == "TURN_LEFT") {
    turnLeft(150);
  } 
  else if (currentAction == "TURN_RIGHT") {
    turnRight(150);
  } 
  else if (currentAction == "STOP") {
    stopRobot();
  }

  // Gửi telemetry xung Encoder về Master mỗi 300ms
  if (millis() - lastSendTime > 300) {
    lastSendTime = millis();
    Serial2.print("ENC:X:");
    Serial2.print(encoderXCount);
    Serial2.print(" Y:");
    Serial2.println(encoderYCount);
  }

  delay(20);
}