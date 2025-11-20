#include <Arduino.h>
#include <ESP32Encoder.h>

// --------- MOTOR PINS ---------
#define MOTOR_IN1 26
#define MOTOR_IN2 25

// --------- ENCODER SETUP ---------
ESP32Encoder encoder;

// --------- MOTOR TEST PARAMETERS ---------
const int MOTOR_SPEED = 255;
const int DRIVE_TIME_MS = 3000;
const int REST_TIME_MS  = 2000;

// --------- SAMPLE SPEED ---------
const int SAMPLE_DELAY_MS = 5;   // <- was 100 ms, now MUCH faster

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MOTOR + ENCODER HIGH-RES TEST ===");

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(27, 33);
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
    delay(SAMPLE_DELAY_MS);
  }

  // ---- PHASE 2: Motor stop ----
  Serial.println("PHASE: STOP (motor off)");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  start = millis();
  while (millis() - start < REST_TIME_MS) {
    Serial.print("Encoder count = ");
    Serial.println((int32_t)encoder.getCount());
    delay(SAMPLE_DELAY_MS);
  }
}