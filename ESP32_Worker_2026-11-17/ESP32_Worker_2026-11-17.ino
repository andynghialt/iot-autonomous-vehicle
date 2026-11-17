/* =====================================================================
 * PROJECT: Maze Solver Worker Motor Controller
 * FILE: ESP32_Worker_2026-11-17.ino
 * DATE: 2026-11-17
 * ===================================================================== */

#include <Arduino.h>

const int PWMA = 4;
const int AIN1 = 13;
const int AIN2 = 27; 
const int STBY = 26; 
const int BIN1 = 33; 
const int BIN2 = 19;
const int PWMB = 23;

const int ENC_X_A = 25; const int ENC_X_B = 32; 
const int ENC_Y_A = 34; const int ENC_Y_B = 35;

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
    String incoming = Serial2.readStringUntil('\n'); incoming.trim();
    if (incoming.startsWith("CMD:")) {
      char cmd = incoming.charAt(4);
      if (cmd == 'C') { encoderXCount = 0; encoderYCount = 0; currentAction = "CELL_FWD"; }
      else if (cmd == 'F') { encoderXCount = 0; encoderYCount = 0; currentAction = "FORWARD"; lastActiveAction = "FORWARD"; }
      else if (cmd == 'S') { currentAction = "STOP"; liveHeadingCorrection = 0; stopRobot(); }
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