#include <Servo.h>
#include <SoftwareSerial.h> // For Bluetooth communication

// Pin definitions
#define TRIG_PIN 8       // Trig pin for HC-SR04
#define ECHO_PIN 9       // Echo pin for HC-SR04
#define ENA 3            // Enable/speed motors Right (PWM pin)
#define ENB 11           // Enable/speed motors Left (PWM pin)
#define IN1 7            // L298N in1 motors Right
#define IN2 6            // L298N in2 motors Right
#define IN3 5            // L298N in3 motors Left
#define IN4 4            // L298N in4 motors Left
#define SERVO_PIN 12     // Servo control pin
#define SPEED 100        // Motor speed for autonomous mode
#define MIDPOINT 90      // Servo midpoint for forward-facing

// Variables
Servo servo;
int distance = 0;
int speedCar = 125;      // Default RC speed
bool isRCMode = false;   // Flag to check RC mode
SoftwareSerial SerialBT(0, 1); // RX, TX for HC-05/HC-06 Bluetooth module

void setup() {
  // Serial monitor for debugging
  Serial.begin(9600); // Debug via USB
  SerialBT.begin(9600); // Bluetooth serial
  
  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo setup
  servo.attach(SERVO_PIN);
  servo.write(MIDPOINT); // Start with servo at midpoint

  Serial.println("Bluetooth RC + Obstacle Avoidance Ready.");
}

void loop() {
  if (SerialBT.available()) {
    isRCMode = true; // Switch to RC mode if Bluetooth input is detected
    handleRCMode();
  } else {
    isRCMode = false; // If no Bluetooth commands, enter autonomous mode
    handleAutonomousMode();
  }
}

// Autonomous obstacle avoidance mode
void handleAutonomousMode() {
  distance = measureDistance();

  if (distance <= 25) {
    stopMotors();
    delay(200); // Ensure proper stopping

    // Check left and right distances
    int leftDistance = checkLeft();
    int rightDistance = checkRight();

    // Reset servo to initial midpoint position
    servo.write(MIDPOINT);
    delay(500);

    // Decide direction based on the distances
    if (leftDistance > rightDistance) {
      turnLeft();
    } else {
      turnRight();
    }
  } else {
    moveForward();
  }
}

// Measure distance using the HC-SR04 sensor
int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distanceCm = duration * 0.034 / 2; // Convert to cm
  return distanceCm;
}

// Servo function to check left distance
int checkLeft() {
  servo.write(150); // Turn servo to the left
  delay(500);       // Allow servo to stabilize
  int leftDist = measureDistance();
  return leftDist;
}

// Servo function to check right distance
int checkRight() {
  servo.write(30); // Turn servo to the right
  delay(500);      // Allow servo to stabilize
  int rightDist = measureDistance();
  return rightDist;
}

// Motor control functions for autonomous mode
void moveForward() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(500); // Adjust turn duration
  stopMotors();
}

void turnRight() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(500); // Adjust turn duration
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// RC mode (remote control)
void handleRCMode() {
  if (SerialBT.available()) {
    char receivedChar = SerialBT.read(); // Read a single character from Bluetooth

    switch (receivedChar) {
      case 'F':
        goAhead();
        break;
      case 'B':
        goBack();
        break;
      case 'L':
        goLeft();
        break;
      case 'R':
        goRight();
        break;
      case 'S':
        stopRobot();
        break;
      default:
        if (receivedChar >= '0' && receivedChar <= '9') {
          speedCar = map(receivedChar - '0', 0, 9, 0, 255); // Map speed value
        }
        break;
    }
  }
}

// RC mode motor controls
void goAhead() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedCar);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedCar);
}

void goBack() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedCar);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedCar);
}

void goLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speedCar);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedCar);
}

void goRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedCar);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speedCar);
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
