/* =====================================================================
 * PROJECT: Maze Solver Worker Motor Controller + 4-DOF Arm Servo Driver
 * FILE: ESP32_Worker_2026-11-20.ino
 * DATE: 2026-11-20
 * NOTE: Tự động ngắt xung sau 0.5s (500ms) cho mọi thao tác tay máy để bảo vệ kẹt trục.
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
const int SERVO_GRIP_PIN = 22; // S1: SG90 tay gắp
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

// Quản lý tự động ngắt xung sau 0.5s (500ms) cho thao tác thông thường
unsigned long autoDetachAt = 0;
bool autoDetachArmed = false;

// Quản lý sweep từ từ cho tay gắp S1
bool gripSweeping = false;
int currentGripAngle = 90;
int targetGripAngle = 90;
unsigned long lastGripStepTime = 0;

// Quản lý chế độ đặc biệt cho S2:
// Step 1: S2 -> 180°, giữ 0.5s rồi ngắt xung
// Step 2..4: gọi 90° 3 lần liên tiếp, mỗi lần 0.5s rồi ngắt xung (đảm bảo trước khi gọi mỗi lần đều có ngắt xung)
enum SpecialStage { SPEC_NONE, SPEC_180_HOLD, SPEC_90_A, SPEC_90_B, SPEC_90_C };
SpecialStage specialStage = SPEC_NONE;
unsigned long specialTimer = 0;

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

void attachServoIfNeeded(int id) {
  if (id == 1 && !gripAttached) { servoGrip.attach(SERVO_GRIP_PIN, 500, 2500); gripAttached = true; }
  else if (id == 2 && !arm1Attached) { servoArm1.attach(SERVO_ARM1_PIN, 500, 2500); arm1Attached = true; }
  else if (id == 3 && !arm2Attached) { servoArm2.attach(SERVO_ARM2_PIN, 500, 2500); arm2Attached = true; }
  else if (id == 4 && !baseAttached) { servoBase.attach(SERVO_BASE_PIN, 500, 2500); baseAttached = true; }
}

void detachAllServos() {
  if (gripAttached) { servoGrip.detach(); gripAttached = false; }
  if (arm1Attached) { servoArm1.detach(); arm1Attached = false; }
  if (arm2Attached) { servoArm2.detach(); arm2Attached = false; }
  if (baseAttached) { servoBase.detach(); baseAttached = false; }
  autoDetachArmed = false;
}

void armAutoDetachAfter500ms() {
  autoDetachAt = millis() + 500;
  autoDetachArmed = true;
}

void moveSingleServo(int id, int angle) {
  int safeAngle = constrain(angle, 0, 180);
  attachServoIfNeeded(id);
  if (id == 1) { servoGrip.write(safeAngle); currentGripAngle = safeAngle; }
  else if (id == 2) servoArm1.write(safeAngle);
  else if (id == 3) servoArm2.write(safeAngle);
  else if (id == 4) servoBase.write(safeAngle);
  armAutoDetachAfter500ms();
}

void moveTwinHips(int a2, int a3) {
  attachServoIfNeeded(2);
  attachServoIfNeeded(3);
  servoArm1.write(constrain(a2, 0, 180));
  servoArm2.write(constrain(a3, 0, 180));
  armAutoDetachAfter500ms();
}

void startGripSweep(int target) {
  targetGripAngle = constrain(target, 0, 90);
  attachServoIfNeeded(1);
  servoGrip.write(currentGripAngle);
  gripSweeping = true;
  autoDetachArmed = false;
  lastGripStepTime = millis();
}

void startSpecialSequence() {
  // Đảm bảo trước khi bắt đầu chuỗi đều đã ngắt xung sạch
  detachAllServos();
  gripSweeping = false;
  attachServoIfNeeded(2);
  servoArm1.write(180);
  specialStage = SPEC_180_HOLD;
  specialTimer = millis();
  Serial2.println("LOG:Spec B1 -> S2:180° (chờ 0.5s ngắt xung)");
}

void updateArmTasks() {
  unsigned long now = millis();

  // 1. Chạy từ từ cho tay gắp S1 để có lực đóng/mở đều
  if (gripSweeping && (now - lastGripStepTime >= 18)) {
    lastGripStepTime = now;
    if (currentGripAngle < targetGripAngle) {
      currentGripAngle++;
      servoGrip.write(currentGripAngle);
    } else if (currentGripAngle > targetGripAngle) {
      currentGripAngle--;
      servoGrip.write(currentGripAngle);
    } else {
      gripSweeping = false;
      armAutoDetachAfter500ms();
    }
  }

  // 2. Chế độ đặc biệt: S2 180° (0.5s -> ngắt) rồi gọi 90° 3 lần (mỗi lần có ngắt xung trước khi gọi & sau 0.5s)
  if (specialStage != SPEC_NONE) {
    if (specialStage == SPEC_180_HOLD && (now - specialTimer >= 500)) {
      if (arm1Attached) { servoArm1.detach(); arm1Attached = false; } // ngắt xung sau 0.5s
      attachServoIfNeeded(2);
      servoArm1.write(90);
      specialStage = SPEC_90_A;
      specialTimer = now;
      Serial2.println("LOG:Spec B2 (Lần 1/3) -> S2:90°");
    } else if (specialStage == SPEC_90_A && (now - specialTimer >= 500)) {
      if (arm1Attached) { servoArm1.detach(); arm1Attached = false; } // ngắt xung trước lần gọi tiếp theo
      attachServoIfNeeded(2);
      servoArm1.write(90);
      specialStage = SPEC_90_B;
      specialTimer = now;
      Serial2.println("LOG:Spec B2 (Lần 2/3) -> S2:90°");
    } else if (specialStage == SPEC_90_B && (now - specialTimer >= 500)) {
      if (arm1Attached) { servoArm1.detach(); arm1Attached = false; } // ngắt xung trước lần gọi thứ 3
      attachServoIfNeeded(2);
      servoArm1.write(90);
      specialStage = SPEC_90_C;
      specialTimer = now;
      Serial2.println("LOG:Spec B2 (Lần 3/3) -> S2:90°");
    } else if (specialStage == SPEC_90_C && (now - specialTimer >= 500)) {
      detachAllServos();
      specialStage = SPEC_NONE;
      Serial2.println("LOG:Hoàn tất chế độ đặc biệt & đã ngắt xung!");
    }
  }

  // 3. Tự động ngắt xung sau 0.5s cho thao tác thường
  if (autoDetachArmed && !gripSweeping && specialStage == SPEC_NONE && (now >= autoDetachAt)) {
    detachAllServos();
  }
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
    } else if (incoming.startsWith("GRIPS:")) {
      int tgt = incoming.substring(6).toInt();
      startGripSweep(tgt);
    } else if (incoming == "ARMSP") {
      startSpecialSequence();
    } else if (incoming == "ARM_OFF") {
      specialStage = SPEC_NONE;
      gripSweeping = false;
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

  updateArmTasks();

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