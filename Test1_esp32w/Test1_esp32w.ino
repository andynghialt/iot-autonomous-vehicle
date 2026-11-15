/* =====================================================================
 * PROJECT: Maze Robot Worker Motor & Sensor Controller
 * FILE: ESP32_Worker_Motor.ino
 * VERSION: v1.2.7
 * DATE: 2026-11-14
 * 
 * COMPATIBILITY MATRIX:
 * - Worker v1.2.7 <--> Bắt buộc đi kèm Master v1.2.8
 * 
 * CHANGES IN v1.2.7 (SO VỚI v1.2.6):
 * - Sửa lại công thức bám hướng Tiến/Lùi: Đảo chiều phép toán bù trừ (`liveHeadingCorrection`)
 *   để triệt tiêu hoàn toàn hiện tượng quay vòng tròn, giúp xe nắn thẳng hướng nhẹ nhàng.
 * ===================================================================== */

#include <Arduino.h>

const int PWMA = 4;
const int AIN1 = 13;
const int AIN2 = 27; 
const int STBY = 26; 
const int BIN1 = 33; 
const int BIN2 = 19;
const int PWMB = 23;

const int ENC_X_A = 25;
const int ENC_X_B = 32; 
const int ENC_Y_A = 34;
const int ENC_Y_B = 35;

volatile long encoderXCount = 0; 
volatile long encoderYCount = 0; 

float scaleX = 1.0; 
float scaleY = 1.0; 
int liveHeadingCorrection = 0; 

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

void setMotorLeft(int dir, int speed) {
  if (dir == 1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (dir == -1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, speed);
}

void setMotorRight(int dir, int speed) {
  if (dir == 1) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (dir == -1) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, speed);
}

// ĐÃ SỬA CHIỀU BÙ HƯỚNG ĐỂ CHỐNG QUAY VÒNG TRÒN KHI TIẾN/LÙI
void moveStraight(int baseSpeed) {
  // Nếu Yaw > Locked Heading (lệch phải), ta cần giảm tốc bánh Phải và tăng tốc bánh Trái để kéo xe về trái
  int speedX = baseSpeed + liveHeadingCorrection; // Bánh Trái (X)
  int speedY = baseSpeed - liveHeadingCorrection; // Bánh Phải (Y)

  speedX = constrain(speedX, 90, 240);
  speedY = constrain(speedY, 90, 240);

  setMotorLeft(1, speedX);
  setMotorRight(1, speedY);
}

void moveBackward(int baseSpeed) {
  int speedX = baseSpeed - liveHeadingCorrection;
  int speedY = baseSpeed + liveHeadingCorrection;

  speedX = constrain(speedX, 90, 240);
  speedY = constrain(speedY, 90, 240);

  setMotorLeft(-1, speedX);
  setMotorRight(-1, speedY);
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
      } 
      else if (cmd == 'B') {
        encoderXCount = 0; encoderYCount = 0;
        currentAction = "BACKWARD";
        lastActiveAction = "BACKWARD";
      } 
      else if (cmd == 'S') {
        currentAction = "STOP";
        liveHeadingCorrection = 0;
        stopRobot();
      } 
      else if (cmd == 'G') {
        currentAction = lastActiveAction;
      }
    } 
    else if (incoming.startsWith("SCALE:")) {
      int commaIndex = incoming.indexOf(',');
      if (commaIndex != -1) {
        scaleX = incoming.substring(6, commaIndex).toFloat();
        scaleY = incoming.substring(commaIndex + 1).toFloat();
      }
    }
    else if (incoming.startsWith("CORRECTION:")) {
      liveHeadingCorrection = incoming.substring(11).toInt();
    }
    else if (incoming.startsWith("TURNC:")) {
      currentAction = "DYNAMIC_TURN";
      char dir = incoming.charAt(6);
      int spd = incoming.substring(8).toInt();
      if (dir == 'L') {
        setMotorLeft(-1, spd);
        setMotorRight(1, spd);
      } else {
        setMotorLeft(1, spd);
        setMotorRight(-1, spd);
      }
    }
  }

  if (currentAction == "FORWARD") {
    moveStraight(170);
  } 
  else if (currentAction == "BACKWARD") {
    moveBackward(170);
  } 
  else if (currentAction == "DYNAMIC_TURN") {
    // Đang thực thi quay/căn chỉnh góc
  } 
  else if (currentAction == "STOP") {
    stopRobot();
  }

  if (millis() - lastSendTime > 300) {
    lastSendTime = millis();
    Serial2.print("ENC:X:");
    Serial2.print(encoderXCount);
    Serial2.print(" Y:");
    Serial2.println(encoderYCount);
  }

  delay(20);
}