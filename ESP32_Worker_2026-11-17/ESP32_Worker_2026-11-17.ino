/* =====================================================================
 * PROJECT: Maze Solver Worker Motor Controller + 4-DOF Arm Servo Driver
 * FILE: ESP32_Worker_2026-11-20.ino
 * DATE: 2026-11-20
 * NOTE: Khi mới cấp nguồn, không attach/xuất xung servo để cánh tay đứng yên hoàn toàn.
 * ===================================================================== */

#include <Arduino.h>
#include <ESP32Servo.h>

const int PWMA = 4;
const int AIN1 = 13;
const int AIN2 = 27; 
const int STBY = 26; 
const int BIN1 = 33; 
const int BIN2 = 19;
const int PWMB = 23;

const int ENC_X_A = 25; const int ENC_X_B = 32; 
const int ENC_Y_A = 34; const int ENC_Y_B = 35;

// Chân điều khiển 4 servo cánh tay
const int SERVO_GRIP_PIN = 22; // S1: SG90 tay gắp (90° / 0°)
const int SERVO_ARM1_PIN = 18; // S2: SG90 khớp hông 1
const int SERVO_ARM2_PIN = 21; // S3: SG90 khớp hông 2
const int SERVO_BASE_PIN = 5;  // S4: MG996R 360° đế xoay

Servo servoGrip;
Servo servoArm1;
Servo servoArm2;
Servo servoBase;

bool gripAttached = false;
bool arm1Attached = false;
bool arm2Attached = false;
bool baseAttached = false;

volatile long encoderXCount = 0, encoderYCount = 0;
float scaleX = 1.0, scaleY = 1.0;
int liveHeadingCorrection = 0;
String currentAction = "STOP", lastActiveAction = "FORWARD";
unsigned long lastSendTime = 0;

const long TICKS_PER_CELL = 350; 

void IRAM_ATTR isrEncoderX() { if (digitalRead(ENC_X_B) == HIGH) encoderXCount++; else encoderXCount--; }
void IRAM_ATTR isrEncoderY() { if (digitalRead(ENC_Y_B) == HIGH) encoderYCount++; else encoderYCount--; }

void stopRobot() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void setMotorLeft(int dir, int speed) {
  digitalWrite(AIN1, dir == 1 ? HIGH : LOW);
  digitalWrite(AIN2, dir == -1 ? HIGH : LOW);
  analogWrite(PWMA, speed);
}

void setMotorRight(int dir, int speed) {
  digitalWrite(BIN1, dir == 1 ? HIGH : LOW);
  digitalWrite(BIN2, dir == -1 ? HIGH : LOW);
  analogWrite(PWMB, speed);
}

void moveStraight(int baseSpeed) {
  int speedX = baseSpeed + liveHeadingCorrection;
  int speedY = baseSpeed - liveHeadingCorrection;
  setMotorLeft(1, constrain(speedX, 90, 240));
  setMotorRight(1, constrain(speedY, 90, 240));
}

void moveSingleServo(int id, int angle) {
  int safeAngle = constrain(angle, 0, 180);
  if (id == 1) {
    if (!gripAttached) { servoGrip.attach(SERVO_GRIP_PIN, 500, 2500); gripAttached = true; }
    servoGrip.write(safeAngle);
  } else if (id == 2) {
    if (!arm1Attached) { servoArm1.attach(SERVO_ARM1_PIN, 500, 2500); arm1Attached = true; }
    servoArm1.write(safeAngle);
  } else if (id == 3) {
    if (!arm2Attached) { servoArm2.attach(SERVO_ARM2_PIN, 500, 2500); arm2Attached = true; }
    servoArm2.write(safeAngle);
  } else if (id == 4) {
    if (!baseAttached) { servoBase.attach(SERVO_BASE_PIN, 500, 2500); baseAttached = true; }
    servoBase.write(safeAngle);
  }
}

void moveTwinHips(int a2, int a3) {
  int safe2 = constrain(a2, 0, 180);
  int safe3 = constrain(a3, 0, 180);
  if (!arm1Attached) { servoArm1.attach(SERVO_ARM1_PIN, 500, 2500); arm1Attached = true; }
  if (!arm2Attached) { servoArm2.attach(SERVO_ARM2_PIN, 500, 2500); arm2Attached = true; }
  servoArm1.write(safe2);
  servoArm2.write(safe3);
}

void detachAllServos() {
  if (gripAttached) { servoGrip.detach(); gripAttached = false; }
  if (arm1Attached) { servoArm1.detach(); arm1Attached = false; }
  if (arm2Attached) { servoArm2.detach(); arm2Attached = false; }
  if (baseAttached) { servoBase.detach(); baseAttached = false; }
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

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  stopRobot();
}

void loop() {
  if (Serial2.available()) {
    String incoming = Serial2.readStringUntil('\n'); incoming.trim();
    if (incoming.startsWith("CMD:")) {
      char cmd = incoming.charAt(4);
      if (cmd == 'C') { encoderXCount = 0; encoderYCount = 0; currentAction = "CELL_FWD"; }
      else if (cmd == 'F') { encoderXCount = 0; encoderYCount = 0; currentAction = "FORWARD"; lastActiveAction = "FORWARD"; }
      else if (cmd == 'S') { currentAction = "STOP"; liveHeadingCorrection = 0; stopRobot(); }
    } else if (incoming.startsWith("ARMS:")) {
      int comma = incoming.indexOf(',', 5);
      if (comma != -1) {
        int id = incoming.substring(5, comma).toInt();
        int ang = incoming.substring(comma + 1).toInt();
        moveSingleServo(id, ang);
      }
    } else if (incoming.startsWith("ARMT:")) {
      int comma = incoming.indexOf(',', 5);
      if (comma != -1) {
        int a2 = incoming.substring(5, comma).toInt();
        int a3 = incoming.substring(comma + 1).toInt();
        moveTwinHips(a2, a3);
      }
    } else if (incoming == "ARM_OFF") {
      detachAllServos();
    } else if (incoming.startsWith("SCALE:")) {
      int comma = incoming.indexOf(',');
      if (comma != -1) { scaleX = incoming.substring(6, comma).toFloat(); scaleY = incoming.substring(comma + 1).toFloat(); }
    } else if (incoming.startsWith("CORRECTION:")) {
      liveHeadingCorrection = incoming.substring(11).toInt();
    } else if (incoming.startsWith("TURNC:")) {
      currentAction = "DYNAMIC_TURN";
      char dir = incoming.charAt(6); int spd = incoming.substring(8).toInt();
      if (dir == 'L') { setMotorLeft(-1, spd); setMotorRight(1, spd); }
      else { setMotorLeft(1, spd); setMotorRight(-1, spd); }
    }
  }

  if (currentAction == "CELL_FWD") {
    long avgEnc = (abs(encoderXCount * scaleX) + abs(encoderYCount * scaleY)) / 2;

    if (avgEnc >= TICKS_PER_CELL) {
      setMotorLeft(-1, 90); 
      setMotorRight(-1, 90); 
      delay(120); 
      
      stopRobot(); 
      currentAction = "STOP";
      Serial2.println("LOG:Đã đi xong 1 ô 30cm và lùi chỉnh về tâm chuẩn!");
    } 
    else {
      int speedX = 140 + liveHeadingCorrection;
      int speedY = 140 - liveHeadingCorrection;
      setMotorLeft(1, constrain(speedX, 80, 200));
      setMotorRight(1, constrain(speedY, 80, 200));
    }
  }
  else if (currentAction == "FORWARD") {
    int speedX = 160 + liveHeadingCorrection;
    int speedY = 160 - liveHeadingCorrection;
    setMotorLeft(1, constrain(speedX, 90, 220));
    setMotorRight(1, constrain(speedY, 90, 220));
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