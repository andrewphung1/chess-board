#include <Arduino.h>
#include <ESP32Encoder.h>

// --------- MOTOR PINS ---------
#define MOTOR_IN1 26   // same as your BIN_1
#define MOTOR_IN2 25   // same as your BIN_2

// --------- ENCODER SETUP ---------
ESP32Encoder encoder;

// --------- MOTOR TEST PARAMETERS ---------
const int MOTOR_SPEED = 255;      // full speed
const int DRIVE_TIME_MS = 3000;   // motor runs for 3 sec
const int REST_TIME_MS  = 2000;   // pause for 2 sec

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MOTOR + ENCODER COMBINED TEST ===");

  // ----- MOTOR PINS -----
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  // ----- ENCODER SETUP -----
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(27, 33);   // your working pins
  encoder.setCount(0);

  Serial.println("Encoder initialized.");
  Serial.println("Starting test in 2 seconds...");
  delay(2000);
}

void loop() {

  // ---- PHASE 1: Motor forward ----
  Serial.println("PHASE: DRIVE (motor forward)");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);

  unsigned long start = millis();
  while (millis() - start < DRIVE_TIME_MS) {
    Serial.print("Encoder count = ");
    Serial.println((int32_t)encoder.getCount());
    delay(100);
  }

  // ---- PHASE 2: Motor stop ----
  Serial.println("PHASE: STOP (motor off)");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  start = millis();
  while (millis() - start < REST_TIME_MS) {
    Serial.print("Encoder count = ");
    Serial.println((int32_t)encoder.getCount());
    delay(100);
  }

  // Loop repeats automatically
}