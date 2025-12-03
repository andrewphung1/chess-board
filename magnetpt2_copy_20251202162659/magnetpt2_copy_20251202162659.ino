#include <Arduino.h>
#include <ESP32Encoder.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =================================================================================
//                             1. HARDWARE CONFIGURATION
// =================================================================================

// --- X-AXIS (Belt) ---
#define X_PWM_1 26     
#define X_PWM_2 25     
#define X_ENC_A 27
#define X_ENC_B 33 

// --- Y-AXIS (Lead Screw) ---
#define Y_PWM_1 14     
#define Y_PWM_2 12     
#define Y_ENC_A 15 
#define Y_ENC_B 32 

// --- Z-AXIS (Magnet) ---
#define Z_PWM_1 4
#define Z_PWM_2 13

#define LED_PIN 2   

// =================================================================================
//                             2. PHYSICS & CALIBRATION
// =================================================================================

// --- X-AXIS (Belt) ---
const float X_COUNTS_PER_INCH = 720.12;  
const float X_DIST_TO_SQ1     = 0.965;   
const float X_PITCH           = 1.500;   

// --- Y-AXIS (Lead Screw) ---
const float Y_COUNTS_PER_INCH = 4500.0; 
const float Y_WALL_TO_CENTER  = 0.750;
const float Y_MAGNET_OFFSET   = 0.95;
const float Y_PITCH           = 1.500; 

// =================================================================================
//                             3. MOTOR TUNING
// =================================================================================

// --- X-AXIS ---
const int MAX_SPEED_X   = 255;
const int MIN_SPEED_X   = 150;
const int SLOW_ZONE_X   = 400;   
const int TIMEOUT_X_MS  = 5000;

// --- Y-AXIS ---
const int MAX_SPEED_Y   = 255;   
const int MIN_SPEED_Y   = 170;   
const int PRE_STOP_Y    = 2500;  
const int TIMEOUT_Y_MS  = 10000;

// --- Z-AXIS ---
int zSpeed              = 170;   
const int TIMEOUT_Z_MS  = 2000;

const int TOLERANCE     = 15;    

// =================================================================================
//                             4. GLOBALS & BLE
// =================================================================================

#define SERVICE_UUID   "b5b10001-6f3b-4c90-8e2b-7d1a2fa9c001"
#define CMD_CHAR_UUID  "b5b10002-6f3b-4c90-8e2b-7d1a2fa9c002"
#define ACK_CHAR_UUID  "b5b10003-6f3b-4c90-8e2b-7d1a2fa9c003"

const unsigned long HEARTBEAT_INTERVAL_MS = 5000;
unsigned long lastHeartbeat = 0;

bool invertX = true;  
bool invertY = false; 
bool isSystemFaulted = false; 

long lastFailedTargetX = 0;
long lastFailedTargetY = 0;

ESP32Encoder encX;
ESP32Encoder encY;

BLEServer* g_server          = nullptr;
BLECharacteristic* g_cmdChar = nullptr;  
BLECharacteristic* g_ackChar = nullptr;  
bool g_clientConnected       = false;

struct MoveCmd {
  String id;
  String type;     
  String fromSq;
  String toSq;
  String piece;    
  bool   valid = false;
};

// =================================================================================
//                             5. HARDWARE HELPERS
// =================================================================================

long calculateTargetX(int fileIndex) {
  if (fileIndex < 1) fileIndex = 1; if (fileIndex > 8) fileIndex = 8;
  return (long)((X_DIST_TO_SQ1 + ((fileIndex - 1) * X_PITCH)) * X_COUNTS_PER_INCH);
}

long calculateTargetY(int rankIndex) {
  if (rankIndex < 1) rankIndex = 1; if (rankIndex > 8) rankIndex = 8;
  float totalInches = (Y_WALL_TO_CENTER + Y_MAGNET_OFFSET) + ((rankIndex - 1) * Y_PITCH);
  return (long)(totalInches * Y_COUNTS_PER_INCH);
}

void setX(int pwm, bool forward) {
  bool dir = invertX ? !forward : forward;
  if (dir) { ledcWrite(X_PWM_1, 0); ledcWrite(X_PWM_2, pwm); }
  else     { ledcWrite(X_PWM_1, pwm); ledcWrite(X_PWM_2, 0); }
}

void setY(int pwm, bool forward) {
  bool dir = invertY ? !forward : forward;
  if (dir) { ledcWrite(Y_PWM_1, 0); ledcWrite(Y_PWM_2, pwm); }
  else     { ledcWrite(Y_PWM_1, pwm); ledcWrite(Y_PWM_2, 0); }
}

void setZ(int pwm, bool forward) {
  if (forward) { ledcWrite(Z_PWM_1, pwm); ledcWrite(Z_PWM_2, 0); }
  else         { ledcWrite(Z_PWM_1, 0); ledcWrite(Z_PWM_2, pwm); }
}

void brakeY() {
  digitalWrite(Y_PWM_1, HIGH); 
  digitalWrite(Y_PWM_2, HIGH);
  delay(150); 
  ledcWrite(Y_PWM_1, 0); ledcWrite(Y_PWM_2, 0);
}

void stopAll() {
  ledcWrite(X_PWM_1, 0); ledcWrite(X_PWM_2, 0);
  ledcWrite(Y_PWM_1, 0); ledcWrite(Y_PWM_2, 0);
  ledcWrite(Z_PWM_1, 0); ledcWrite(Z_PWM_2, 0);
  Serial.println(">>> EMERGENCY STOP <<<");
}

// --- SAFE Z MOVE ---
void moveZ(bool up) {
  Serial.print(">>> Z-Axis: "); Serial.println(up ? "LIFTING" : "DROPPING");
  setZ(zSpeed, up);
  
  unsigned long start = millis();
  while(millis() - start < TIMEOUT_Z_MS) {
    if (Serial.available()) {
      if (Serial.read() == 's') { stopAll(); return; }
    }
  }
  setZ(0, true);
  Serial.println(">>> Z-Axis: Done.");
}

// --- SMART MOVE LOGIC (X & Y) ---
bool moveToTarget(ESP32Encoder& activeEnc, long target, int axisID) {
  digitalWrite(LED_PIN, HIGH);
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0; 
  
  String label = (axisID == 0) ? "X" : "Y";
  if (axisID == 0) lastFailedTargetX = target; else lastFailedTargetY = target;
  unsigned long currentTimeout = (axisID == 0) ? TIMEOUT_X_MS : TIMEOUT_Y_MS;

  Serial.println(">>> " + label + ": Target " + String(target));

  long startPos = activeEnc.getCount();
  bool forward = (target > startPos);
  long cutoffPoint = target;
  
  // Y-Axis Cutoff Logic
  if (axisID == 1 && abs(target - startPos) > PRE_STOP_Y) {
      if (forward) cutoffPoint = target - PRE_STOP_Y; 
      else         cutoffPoint = target + PRE_STOP_Y; 
  }

  while (true) {
    if (Serial.available() && Serial.peek() == 's') {
      stopAll(); Serial.read(); return false; 
    }

    long current = activeEnc.getCount();
    long error = target - current; 
    long dist = abs(error);

    if (millis() - lastPrintTime > 200) {
      lastPrintTime = millis();
      Serial.print("   " + label + ": " + String(current));
      Serial.println(" | Err: " + String(error));
    }

    bool stopNow = false;
    if (axisID == 0 && dist <= TOLERANCE) stopNow = true;
    if (axisID == 1) {
      if (dist <= TOLERANCE) stopNow = true; 
      if (forward && current >= cutoffPoint) stopNow = true;
      if (!forward && current <= cutoffPoint) stopNow = true;
    }

    if (stopNow) {
      if (axisID == 1) brakeY(); 
      else stopAll();            
      Serial.println(">>> " + label + " ARRIVED! Final: " + String(activeEnc.getCount()));
      return true;
    }

    if (millis() - startTime > currentTimeout) {
      stopAll(); 
      Serial.println(">>> ERROR: " + label + " TIMEOUT!"); 
      return false; 
    }

    int speed = 0;
    if (axisID == 0) {
      speed = (dist > SLOW_ZONE_X) ? MAX_SPEED_X : MIN_SPEED_X;
    } else {
      if (abs(target - current) > 5000) speed = MAX_SPEED_Y;
      else speed = MIN_SPEED_Y; 
    }

    bool direction = (error > 0); 
    if (axisID == 0) setX(speed, direction);
    else             setY(speed, direction);
    
    delay(1); 
  }
}

// =================================================================================
//                             6. BLE STUB
// =================================================================================
void bleNotify(const String& msg) {} 
void printStatus(const String& s) {} 

// =================================================================================
//                             7. SETUP & LOOP
// =================================================================================

void setup() {
  Serial.begin(115200);
  
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encX.attachHalfQuad(X_ENC_A, X_ENC_B); encX.setCount(0);
  encY.attachHalfQuad(Y_ENC_A, Y_ENC_B); encY.setCount(0);

  ledcAttach(X_PWM_1, 5000, 8); ledcAttach(X_PWM_2, 5000, 8);
  ledcAttach(Y_PWM_1, 5000, 8); ledcAttach(Y_PWM_2, 5000, 8);
  ledcAttach(Z_PWM_1, 5000, 8); ledcAttach(Z_PWM_2, 5000, 8);
  
  pinMode(LED_PIN, OUTPUT);
  stopAll();

  Serial.println("\n=== VIBECHESS MASTER (MANUAL) ===");
  Serial.println("X: 'x1'-'x8', 'x[Num]', 'f'/'b'");
  Serial.println("Y: 'y1'-'y8', 'y[Num]', 'l'/'r'");
  Serial.println("Z: 'u'/'d'");
  Serial.println("STOP: 's'");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char c = input.charAt(0);
    
    if (c == 's') { stopAll(); return; }

    // --- X COMMANDS ---
    if (c == 'x') {
      // Is it x1...x8?
      if (input.length() == 2 && isDigit(input.charAt(1)) && input.charAt(1) != '0') {
         int sq = input.substring(1).toInt();
         if(sq >= 1 && sq <= 8) {
            Serial.print(">>> X Square: "); Serial.println(sq);
            moveToTarget(encX, calculateTargetX(sq), 0);
         }
      } 
      // Or is it x5000?
      else if (input.length() > 1) {
         long target = input.substring(1).toInt();
         Serial.print(">>> X Raw Target: "); Serial.println(target);
         moveToTarget(encX, target, 0);
      }
    }

    // --- Y COMMANDS ---
    else if (c == 'y') {
      // Is it y1...y8?
      if (input.length() == 2 && isDigit(input.charAt(1)) && input.charAt(1) != '0') {
         int sq = input.substring(1).toInt();
         if(sq >= 1 && sq <= 8) {
            Serial.print(">>> Y Square: "); Serial.println(sq);
            moveToTarget(encY, calculateTargetY(sq), 1);
         }
      } 
      // Or is it y5000?
      else if (input.length() > 1) {
         long target = input.substring(1).toInt();
         Serial.print(">>> Y Raw Target: "); Serial.println(target);
         moveToTarget(encY, target, 1);
      }
    }

    // --- MANUAL JOGGING ---
    else if (c == 'f') setX(MAX_SPEED_X, true);
    else if (c == 'b') setX(MAX_SPEED_X, false);
    else if (c == 'l') setY(MAX_SPEED_Y, true);
    else if (c == 'r') setY(MAX_SPEED_Y, false);
    
    // --- Z AXIS ---
    else if (c == 'u') moveZ(true);
    else if (c == 'd') moveZ(false);
    
    // --- ZERO ---
    else if (c == 'z') { 
      encX.setCount(0); encY.setCount(0); 
      Serial.println(">>> ZEROED ALL."); 
    }
  }
}