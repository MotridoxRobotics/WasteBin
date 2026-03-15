#include <Arduino.h>
#include "A4988.h"
#include <ESP32Servo.h>

// ─────────────────────────────────────────
// STEPPER MOTOR PINS
// ─────────────────────────────────────────
int Step = 26;
int Dire = 25;
int Sleep = 27;
int MS1 = 13;
int MS2 = 12;
int MS3 = 14;

// ─────────────────────────────────────────
// ULTRASONIC SENSOR 1 (Stepper Control)
// ─────────────────────────────────────────
const int TRIG1 = 32;
const int ECHO1 = 33;

// ─────────────────────────────────────────
// ULTRASONIC SENSOR 2 (Servo Control)
// ─────────────────────────────────────────
const int TRIG2 = 18;
const int ECHO2 = 19;

// ─────────────────────────────────────────
// SERVO PIN
// ─────────────────────────────────────────
const int SERVO_PIN = 4;

// ─────────────────────────────────────────
// MOTOR SPECS
// ─────────────────────────────────────────
const int spr = 200;
int RPM = 150;
int Microsteps = 0;

// ─────────────────────────────────────────
// THRESHOLDS
// ─────────────────────────────────────────
const int STEPPER_CLOSE = 15;
const int STEPPER_FAR   = 16;
const int SERVO_TRIGGER = 10;

// ─────────────────────────────────────────
// TIMING (Non-blocking)
// ─────────────────────────────────────────
unsigned long servoStartTime   = 0;
bool servoActive               = false;
bool servoReturning            = false;

A4988 stepper(spr, Dire, Step, MS1, MS2, MS3);
Servo myServo;

// ─────────────────────────────────────────
// Get distance from a given ultrasonic sensor
// ─────────────────────────────────────────
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;

  return (duration * 0.0343) / 2;
}

// ─────────────────────────────────────────
// FUNCTION 1: Stepper Motor Control
// ─────────────────────────────────────────
void handleStepper() {
  float distance = getDistance(TRIG1, ECHO1);

  Serial.print("[Stepper Sensor] Distance: ");
  if (distance == -1) {
    Serial.println("Out of range — Motor sleeping");
    digitalWrite(Sleep, LOW);
    return;
  }
  Serial.print(distance);
  Serial.println(" cm");

  digitalWrite(Sleep, HIGH);

  if (distance < STEPPER_CLOSE) {
    Serial.println(">> Close! Rotating FORWARD");
    stepper.setRPM(RPM);
    stepper.rotate(-360);
  } else if (distance > STEPPER_FAR) {
    Serial.println(">> Far! Rotating BACKWARD");
    stepper.setRPM(RPM);
    stepper.rotate(360);
  }
  // Dead zone between 15–16cm: motor holds position
}

// ─────────────────────────────────────────
// FUNCTION 2: Servo Control (Non-blocking)
// ─────────────────────────────────────────
void handleServo() {
  float distance = getDistance(TRIG2, ECHO2);

  Serial.print("[Servo Sensor] Distance: ");
  if (distance == -1) {
    Serial.println("Out of range");
    return;
  }
  Serial.print(distance);
  Serial.println(" cm");

  // Trigger servo cycle only if not already running
  if (distance < SERVO_TRIGGER && !servoActive && !servoReturning) {
    Serial.println(">> Object detected! Starting servo sweep.");
    myServo.write(80);             // Rotate to 80 degrees
    servoStartTime = millis();     // Record start time
    servoActive = true;
    servoReturning = false;
  }

  // After 5 seconds at 80°, return to 0°
  if (servoActive && !servoReturning) {
    if (millis() - servoStartTime >= 5000) {
      Serial.println(">> 5s done. Returning servo to 0°.");
      myServo.write(0);
      servoReturning = true;
      servoStartTime = millis();   // Reuse timer for return travel time
    }
  }

  // Give servo ~1 second to physically return, then reset flags
  if (servoReturning && (millis() - servoStartTime >= 1000)) {
    Serial.println(">> Servo cycle complete. Ready for next trigger.");
    servoActive = false;
    servoReturning = false;
  }
}

// ─────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Stepper pins
  pinMode(Step, OUTPUT);
  pinMode(Dire, OUTPUT);
  pinMode(Sleep, OUTPUT);
  digitalWrite(Step, LOW);
  digitalWrite(Dire, LOW);

  // Ultrasonic 1 pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  // Ultrasonic 2 pins
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Start at 0 degrees

  stepper.begin(RPM, Microsteps);
  Serial.println("System Ready.");
}

// ─────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────
void loop() {
  handleStepper();   // Stepper + Ultrasonic 1
  handleServo();     // Servo   + Ultrasonic 2
}