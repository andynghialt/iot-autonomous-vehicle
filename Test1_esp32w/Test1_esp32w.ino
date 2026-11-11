#include <Arduino.h>

const int PWMA = 4;
const int AIN1 = 13;
const int AIN2 = 14;
const int STBY = 27;
const int BIN1 = 26;
const int BIN2 = 19;
const int PWMB = 23;

const int ENC_X_A = 25;
const int ENC_X_B = 33;
const int ENC_Y_A = 32;
const int ENC_Y_B = 35;

volatile long encoderXCount = 0;
volatile long encoderYCount = 0;

String currentAction = "STOP";
String lastActiveAction = "FORWARD";

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

// Hàm điều khiển động cơ vi sai tích hợp bù sai số Encoder khi đi thẳng
void moveStraight(int baseSpeed) {
  // So sánh xung encoder 2 bánh để hiệu chỉnh đi thẳng tự động ngay lập tức
  long diff = encoderXCount - encoderYCount; 
  int correction = diff * 2; // Hệ số P điều chỉnh lệch encoder

  int speedX = baseSpeed - correction;
  int speedY = baseSpeed + correction;

  // Giới hạn tốc độ PWM trong khoảng an toàn (0 - 255)
  speedX = constrain(speedX, 100, 240);
  speedY = constrain(speedY, 100, 240);

  // Cả 2 bánh cùng tiến
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speedX);
  analogWrite(PWMB, speedY);
}

void moveBackward(int baseSpeed) {
  long diff = encoderXCount - encoderYCount;
  int correction = diff * 2;

  int speedX = baseSpeed - correction;
  int speedY = baseSpeed + correction;

  speedX = constrain(speedX, 100, 240);
  speedY = constrain(speedY, 100, 240);

  // Cả 2 bánh cùng lùi
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speedX);
  analogWrite(PWMB, speedY);
}

void turnLeft(int speed) {
  // Bánh trái lùi, bánh phải tiến (Quay tại chỗ sang trái)
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void turnRight(int speed) {
  // Bánh trái tiến, bánh phải lùi (Quay tại chỗ sang phải)
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
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
    char cmd = Serial2.read();
    
    if (cmd == 'F') {
      encoderXCount = 0; encoderYCount = 0; // Reset encoder khi bắt đầu đi thẳng
      currentAction = "FORWARD";
      lastActiveAction = "FORWARD";
      Serial2.println("LOG:Đang đi TIẾN (Có bù lệch Encoder)");
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
      Serial2.println("LOG:Đã STOP (Đạt góc/Dừng)");
    } 
    else if (cmd == 'G') {
      currentAction = lastActiveAction;
      Serial2.println("LOG:Tiếp tục GO");
    }
  }

  // Thực thi hành vi
  if (currentAction == "FORWARD") {
    moveStraight(170); // Tốc độ cơ sở đi thẳng
  } 
  else if (currentAction == "BACKWARD") {
    moveBackward(170);
  } 
  else if (currentAction == "TURN_LEFT") {
    turnLeft(150); // Tốc độ quay
  } 
  else if (currentAction == "TURN_RIGHT") {
    turnRight(150);
  } 
  else if (currentAction == "STOP") {
    stopRobot();
  }

  delay(20);
}