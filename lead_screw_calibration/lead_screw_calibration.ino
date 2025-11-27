// calibration seems like 51000 from coupler end ~51000

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ===================== PINS =====================
#define BIN_1 26     
#define BIN_2 25     
#define LED_PIN 13   
#define ENC_A 27  // CHECK THESE WIRES!
#define ENC_B 33  // CHECK THESE WIRES!

// ===================== SETTINGS =====================
const int SPEED_FAST = 255;   
const int SPEED_SLOW = 150;   
const int TOLERANCE = 10;      

bool invertDirection = false; 

// ===================== OBJECTS =====================
ESP32Encoder encoder;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  // ENC Setup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(ENC_A, ENC_B); 
  encoder.setCount(0);

  // Motor Setup
  pinMode(LED_PIN, OUTPUT);
  ledcAttach(BIN_1, 5000, 8);
  ledcAttach(BIN_2, 5000, 8);
  
  // Stop initially
  ledcWrite(BIN_1, LOW); ledcWrite(BIN_2, LOW);

  Serial.println("==============================================");
  Serial.println("      Y-AXIS ENCODER DEBUGGER");
  Serial.println("==============================================");
  Serial.println("  'f' = Forward");
  Serial.println("  'r' = Reverse");
  Serial.println("  's' = STOP (Will print final count)");
  Serial.println("  'z' = ZERO");
  Serial.println("==============================================");
}

// ===================== MOTOR HELPERS =====================
void setMotor(int pwm, bool forward) {
  if (invertDirection) forward = !forward;
  if (forward) {
    ledcWrite(BIN_1, LOW); ledcWrite(BIN_2, pwm);
  } else {
    ledcWrite(BIN_1, pwm); ledcWrite(BIN_2, LOW);
  }
}

void motorStop() {
  ledcWrite(BIN_1, LOW); ledcWrite(BIN_2, LOW);
}

// ===================== MAIN LOOP =====================
void loop() {
  
  // --- LIVE FEEDBACK (FORCE PRINT) ---
  // This prints every 200ms regardless of whether it changed.
  // If you see "En: 0" repeating while moving, your encoder wires are wrong.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    // Only print if we are actually moving (PWM is not 0)
    // Actually, let's just print always so you can see if hand-turning works.
    // Serial.print("En: "); Serial.println((long)encoder.getCount()); 
    lastPrint = millis();
  }

  // --- COMMANDS ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

    if (cmd == 'f') { 
      Serial.println(">>> Jogging FORWARD..."); 
      setMotor(SPEED_FAST, true); 
    }
    else if (cmd == 'r') { 
      Serial.println(">>> Jogging REVERSE..."); 
      setMotor(SPEED_FAST, false); 
    }
    else if (cmd == 's') { 
      motorStop(); 
      Serial.print(">>> STOPPED. Final Count: "); 
      Serial.println((long)encoder.getCount()); 
    }
    else if (cmd == 'z') { 
      encoder.setCount(0); 
      Serial.println(">>> ZEROED."); 
    }
    else if (cmd == 'i') {
      invertDirection = !invertDirection;
      Serial.println(">>> Direction Inverted.");
    }
  }

  // --- CONTINUOUS MONITORING ---
  // If the count changes, print it immediately
  static long lastPos = 0;
  long currentPos = encoder.getCount();
  if (currentPos != lastPos) {
    Serial.print("En: "); Serial.println(currentPos);
    lastPos = currentPos;
  }
}
