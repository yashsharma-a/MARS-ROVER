#include <SoftwareSerial.h>
#include <Servo.h>

// ===== Motor Pins =====
int motorA_PWM = 3;
int motorA_DIR = 8;
int motorB_PWM = 6;
int motorB_DIR = 11;
int motorC_PWM = 4;
int motorC_DIR = 9;
int motorD_PWM = 12;
int motorD_DIR = 7;

// ===== Servo Pins =====
#define CAMERA_SERVO_PIN A0
#define DRILLER_SERVO_PIN A1
#define GRIPPER_SERVO_PIN A2
#define ARM1_SERVO_PIN A3

// ===== Servo Objects & Positions =====
Servo cameraServo;
Servo drillerServo;
Servo gripperServo;
Servo arm1Servo;

int currentPositionCamera = 90;
int currentPositionDriller = 90;
int currentPositionGripper = 90;
int currentPositionArm1 = 90;

// ===== Ultrasonic Sensor =====
#define TRIG_PIN A5
#define ECHO_PIN A4
long duration;
int distance = 0;
int OAV = 0; // Obstacle Avoidance Value
bool motorStopped = false;

// ===== Timers =====
unsigned long lastUltrasonicReadTime = 0;
const unsigned long ultrasonicInterval = 200;

// ===== Bluetooth =====
SoftwareSerial BTSerial(13, 10); // RX, TX
String receivedData = "";

// ===== Setup =====
void setup() {
  // Motor Pins
  pinMode(motorA_PWM, OUTPUT);
  pinMode(motorA_DIR, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  pinMode(motorB_DIR, OUTPUT);
  pinMode(motorC_PWM, OUTPUT);
  pinMode(motorC_DIR, OUTPUT);
  pinMode(motorD_PWM, OUTPUT);
  pinMode(motorD_DIR, OUTPUT);

  // Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Start Serial
  BTSerial.begin(9600);
  Serial.begin(9600);

  Serial.println("4WD + Ultrasonic + Bluetooth + 4 Servo Control Ready!");
}

// ===== Loop =====
void loop() {
  unsigned long currentMillis = millis();

  // ---- Ultrasonic Reading ----
  if (currentMillis - lastUltrasonicReadTime >= ultrasonicInterval) {
    distance = getUltrasonicDistance();
    lastUltrasonicReadTime = currentMillis;
  }

  // ---- Obstacle Detection ----
  if (distance <= OAV && !motorStopped && OAV > 0) {
    stopMotors();
    motorStopped = true;
    Serial.println("Obstacle detected! Motors stopped.");
  }

  if (distance > OAV && motorStopped) {
    motorStopped = false;
    Serial.println("Obstacle cleared.");
  }

  // ---- Bluetooth Data ----
  while (BTSerial.available()) {
    char received = BTSerial.read();
    receivedData += received;
    delay(2);
  }

  if (receivedData.length() > 0) {
    receivedData.trim();
    Serial.print("Received: ");
    Serial.println(receivedData);

    // ---- OAV Command ----
    if (receivedData.startsWith("OAV")) {
      OAV = constrain(receivedData.substring(3).toInt(), 0, 200);
      Serial.print("OAV set to: ");
      Serial.println(OAV);
    }

    // ---- Stop Command ----
    else if (receivedData.indexOf('S') != -1 || receivedData.indexOf('0') != -1) {
      stopMotors();
      Serial.println("Stop command received.");
    }

    // ---- Movement Commands ----
    else if (receivedData.startsWith("F") || receivedData.startsWith("B") ||
             receivedData.startsWith("L") || receivedData.startsWith("R")) {
      if (!motorStopped) {
        handleJoystickCommands(receivedData);
      } else {
        if (receivedData.startsWith("B")) handleJoystickCommands(receivedData);
        else Serial.println("Ignored (Obstacle detected)");
      }
    }

    // ---- Servo Commands ----
    handleServoCommands(receivedData);

    // Clear buffer after handling
    receivedData = "";
  }
}

// ===== Servo Handling Function =====
void handleServoCommands(String data) {
  int value;
  if (data.charAt(0) == 'C') { 
    // Camera servo data has 'C' prefix
    value = data.substring(1).toInt();
    
    if (value >= 1 && value <= 30) {
      int target = map(value, 1, 30, 0, 180);
      moveServoSmoothly(cameraServo, target, currentPositionCamera, CAMERA_SERVO_PIN);
      Serial.print("Camera Servo → ");
      Serial.println(target);
    } else {
      Serial.println("Camera value out of range!");
    }
  } else {
    // Other servos: numeric data only
    value = data.toInt();

    if (value >= 31 && value <= 61) { // Driller Servo
      int target = map(value, 31, 61, 0, 180);
      moveServoSmoothly(drillerServo, target, currentPositionDriller, DRILLER_SERVO_PIN);
      Serial.print("Driller Servo → ");
      Serial.println(target);
    }
    else if (value >= 62 && value <= 92) { // Gripper Servo
      int target = map(value, 62, 92, 0, 180);
      moveServoSmoothly(gripperServo, target, currentPositionGripper, GRIPPER_SERVO_PIN);
      Serial.print("Gripper Servo → ");
      Serial.println(target);
    }
    else if (value >= 93 && value <= 123) { // Arm Servo
      int target = map(value, 93, 123, 0, 180);
      moveServoSmoothly(arm1Servo, target, currentPositionArm1, ARM1_SERVO_PIN);
      Serial.print("Arm1 Servo → ");
      Serial.println(target);
    }
    else {
      Serial.println("Servo value out of range!");
    }
  }
}

// ===== Smooth Servo Control =====
void moveServoSmoothly(Servo &servo, int target, int &current, int pin) {
  servo.attach(pin);
  int step = (target > current) ? 1 : -1;
  while (current != target) {
    current += step;
    if ((step > 0 && current > target) || (step < 0 && current < target)) {
      current = target;
    }
    servo.write(current);
    delay(10);
  }
  delay(50);
  servo.detach();
}

// ===== Ultrasonic Function =====
int getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 25000);
  int dist = duration * 0.034 / 2;
  return dist;
}

// ===== Joystick Command Handling =====
void handleJoystickCommands(String command) {
  int speed = map(command.charAt(1) - '0', 1, 7, 100, 240);

  if (command.startsWith("F") && command.length() == 2) moveForward(speed);
  else if (command.startsWith("B") && command.length() == 2) moveBackward(speed);
  else if (command.startsWith("L") && command.length() == 2) turnLeft(speed);
  else if (command.startsWith("R") && command.length() == 2) turnRight(speed);

  // ---- Diagonals ----
  else if (command.startsWith("F") && command.indexOf("R") != -1) moveForwardRight(speed);
  else if (command.startsWith("F") && command.indexOf("L") != -1) moveForwardLeft(speed);
  else if (command.startsWith("B") && command.indexOf("R") != -1) moveBackwardRight(speed);
  else if (command.startsWith("B") && command.indexOf("L") != -1) moveBackwardLeft(speed);
}

// ===== Movement Functions =====
void moveForward(int speed) {
  digitalWrite(motorA_DIR, LOW);
  digitalWrite(motorB_DIR, LOW);
  digitalWrite(motorC_DIR, LOW);
  digitalWrite(motorD_DIR, LOW);
  analogWrite(motorA_PWM, speed);
  analogWrite(motorB_PWM, speed);
  analogWrite(motorC_PWM, speed);
  analogWrite(motorD_PWM, speed);
  Serial.print("Forward — Speed: ");
  Serial.println(speed);
}

void moveBackward(int speed) {
  digitalWrite(motorA_DIR, HIGH);
  digitalWrite(motorB_DIR, HIGH);
  digitalWrite(motorC_DIR, HIGH); 
  digitalWrite(motorD_DIR, HIGH);
  analogWrite(motorA_PWM, 255 - speed);
  analogWrite(motorB_PWM, 255 - speed);
  analogWrite(motorC_PWM, 255 - speed);
  analogWrite(motorD_PWM, 255 - speed);
  Serial.print("Backward — Speed: ");
  Serial.println(speed);
}

void turnRight(int speed) {
  digitalWrite(motorA_DIR, HIGH);
  digitalWrite(motorC_DIR, HIGH);
  digitalWrite(motorB_DIR, LOW);
  digitalWrite(motorD_DIR, LOW);
  analogWrite(motorA_PWM, speed);
  analogWrite(motorB_PWM, speed);
  analogWrite(motorC_PWM, speed);
  analogWrite(motorD_PWM, speed);
  Serial.print("Left Turn — Speed: ");
  Serial.println(speed);
}

void turnLeft(int speed) {
  digitalWrite(motorA_DIR, LOW);
  digitalWrite(motorC_DIR, LOW);
  digitalWrite(motorB_DIR, HIGH);
  digitalWrite(motorD_DIR, HIGH);
  analogWrite(motorA_PWM, speed);
  analogWrite(motorB_PWM, speed);
  analogWrite(motorC_PWM, speed);
  analogWrite(motorD_PWM, speed);
  Serial.print("Right Turn — Speed: ");
  Serial.println(speed);
}

// ===== Diagonal Movements =====
void moveForwardLeft(int speed) {
  digitalWrite(motorA_DIR, LOW);
  digitalWrite(motorC_DIR, LOW);
  digitalWrite(motorB_DIR, LOW);
  digitalWrite(motorD_DIR, LOW);
  analogWrite(motorA_PWM, speed);
  analogWrite(motorC_PWM, speed);
  analogWrite(motorB_PWM, speed / 2);
  analogWrite(motorD_PWM, speed / 2);
  Serial.print("Forward Right — Speed: ");
  Serial.println(speed);
}

void moveForwardRight(int speed) {
  digitalWrite(motorA_DIR, LOW);
  digitalWrite(motorC_DIR, LOW);
  digitalWrite(motorB_DIR, LOW);
  digitalWrite(motorD_DIR, LOW);
  analogWrite(motorA_PWM, speed / 2);
  analogWrite(motorC_PWM, speed / 2);
  analogWrite(motorB_PWM, speed);
  analogWrite(motorD_PWM, speed);
  Serial.print("Forward Left — Speed: ");
  Serial.println(speed);
}
void moveBackwardLeft(int speed) {
  digitalWrite(motorA_DIR, HIGH);
  digitalWrite(motorC_DIR, HIGH);
  digitalWrite(motorB_DIR, HIGH);
  digitalWrite(motorD_DIR, HIGH);
  analogWrite(motorA_PWM, (255 - speed));
  analogWrite(motorC_PWM, (255 - speed));
  analogWrite(motorB_PWM, (255 - speed / 2));
  analogWrite(motorD_PWM, (255 - speed / 2));
  Serial.print("Backward Right — Speed: ");
  Serial.println(speed);
}


void moveBackwardRight(int speed) {
  digitalWrite(motorA_DIR, HIGH);
  digitalWrite(motorC_DIR, HIGH);
  digitalWrite(motorB_DIR, HIGH);
  digitalWrite(motorD_DIR, HIGH);
  analogWrite(motorA_PWM, (255 - speed / 2));
  analogWrite(motorC_PWM, (255 - speed / 2));
  analogWrite(motorB_PWM, (255 - speed));
  analogWrite(motorD_PWM, (255 - speed));
  Serial.print("Backward Left — Speed: ");
  Serial.println(speed);
}

// ===== Stop Motors =====
void stopMotors() {
  analogWrite(motorA_PWM, 0);
  analogWrite(motorB_PWM, 0);
  analogWrite(motorC_PWM, 0);
  analogWrite(motorD_PWM, 0);
  digitalWrite(motorA_DIR, LOW);
  digitalWrite(motorC_DIR, LOW);
  digitalWrite(motorB_DIR, LOW);
  digitalWrite(motorD_DIR, LOW);
  Serial.println("Motors stopped.");
}