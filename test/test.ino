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

#define LED_PIN 13   

// =================================================================================
//                             2. CALIBRATION & PHYSICS
// =================================================================================

// --- X-AXIS (Belt) ---
const float X_COUNTS_PER_INCH = 720.12;  
const float X_DIST_TO_SQ1     = 0.965;   
const float X_PITCH           = 1.500;   

// --- Y-AXIS (Lead Screw) ---
const float Y_COUNTS_PER_INCH = 4500.0; 

// Geometry Config
const float Y_HOME_TO_CENTER  = 0.750; // Distance from Wall to Center of Square 1
const float Y_MAGNET_OFFSET   = 0.500; // Extra distance for magnet offset
const float Y_PITCH           = 1.500; // Distance between squares

// --- TUNING ---
const int MAX_SPEED     = 255;

// X-Axis (Belt)
const int SLOW_ZONE_X   = 400;   
const int MIN_SPEED_X   = 150;   

// Y-Axis (Lead Screw - Freight Train Logic)
const int MIN_SPEED_Y   = 170;   // High enough to NEVER stall
const int PRE_STOP_Y    = 2500;  // Cut power this many counts EARLY

const int TOLERANCE     = 15;    
const int TIMEOUT_MS    = 10000;  

// --- UUIDS ---
#define SERVICE_UUID   "b5b10001-6f3b-4c90-8e2b-7d1a2fa9c001"
#define CMD_CHAR_UUID  "b5b10002-6f3b-4c90-8e2b-7d1a2fa9c002"
#define ACK_CHAR_UUID  "b5b10003-6f3b-4c90-8e2b-7d1a2fa9c003"

// =================================================================================
//                             3. GLOBALS
// =================================================================================

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
//                             4. HARDWARE HELPERS
// =================================================================================

long calculateTargetX(int fileIndex) {
  if (fileIndex < 1) fileIndex = 1; if (fileIndex > 8) fileIndex = 8;
  return (long)((X_DIST_TO_SQ1 + ((fileIndex - 1) * X_PITCH)) * X_COUNTS_PER_INCH);
}

long calculateTargetY(int rankIndex) {
  if (rankIndex < 1) rankIndex = 1; if (rankIndex > 8) rankIndex = 8;
  
  // Formula: (0.75 + 0.50) + (Index-1 * 1.5)
  float totalInches = (Y_HOME_TO_CENTER + Y_MAGNET_OFFSET) + ((rankIndex - 1) * Y_PITCH);
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

// HARD BRAKE Y
void brakeY() {
  digitalWrite(Y_PWM_1, HIGH); 
  digitalWrite(Y_PWM_2, HIGH);
  delay(150); 
  ledcWrite(Y_PWM_1, 0); ledcWrite(Y_PWM_2, 0);
}

void stopAll() {
  ledcWrite(X_PWM_1, 0); ledcWrite(X_PWM_2, 0);
  ledcWrite(Y_PWM_1, 0); ledcWrite(Y_PWM_2, 0);
}

// --- MOTION LOGIC ---
// axisID: 0=X, 1=Y
bool moveToTarget(ESP32Encoder& activeEnc, long target, int axisID) {
  digitalWrite(LED_PIN, HIGH);
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0; 
  
  String label = (axisID == 0) ? "X" : "Y";
  if (axisID == 0) lastFailedTargetX = target; else lastFailedTargetY = target;

  Serial.println(">>> " + label + ": Target " + String(target));

  // Determine Cutoff for Y
  long startPos = activeEnc.getCount();
  bool forward = (target > startPos);
  
  long cutoffPoint = target;
  
  if (axisID == 1) { 
    // If move is large enough, apply the offset
    if (abs(target - startPos) > PRE_STOP_Y) {
        if (forward) cutoffPoint = target - PRE_STOP_Y; 
        else         cutoffPoint = target + PRE_STOP_Y; 
    }
  }

  while (true) {
    if (Serial.available() && Serial.peek() == 's') {
      stopAll(); Serial.read(); return false; 
    }

    long current = activeEnc.getCount();
    long error = target - current; 
    long dist = abs(error);

    // Live Debug
    if (millis() - lastPrintTime > 200) {
      lastPrintTime = millis();
      Serial.print(label + ": " + String(current));
      Serial.println(" | Err: " + String(error));
    }

    // --- SUCCESS CHECK ---
    bool stopNow = false;
    
    // X Check: Standard Tolerance
    if (axisID == 0 && dist <= TOLERANCE) stopNow = true;

    // Y Check: Reached Cutoff Point?
    if (axisID == 1) {
      if (dist <= TOLERANCE) stopNow = true; // Safety
      if (forward && current >= cutoffPoint) stopNow = true;
      if (!forward && current <= cutoffPoint) stopNow = true;
    }

    if (stopNow) {
      if (axisID == 1) brakeY(); // Slam brakes for Y
      else stopAll();            // Coast for X
      
      Serial.println(">>> STOP TRIGGERED at " + String(current));
      delay(500); // Wait for settle
      Serial.println(">>> FINAL RESTING: " + String(activeEnc.getCount()));
      return true;
    }

    if (millis() - startTime > TIMEOUT_MS) {
      stopAll(); Serial.println("TIMEOUT"); return false; 
    }

    // --- SPEED ---
    int speed = 0;
    
    if (axisID == 0) {
      // X: Bang-Bang
      speed = (dist > SLOW_ZONE_X) ? MAX_SPEED : MIN_SPEED_X;
    } else {
      // Y: Freight Train (Always Fast, rely on cutoff)
      // Only go slow if very close to prevent stalling
      if (abs(target - current) > 5000) speed = MAX_SPEED;
      else speed = MIN_SPEED_Y; 
    }

    bool direction = (error > 0); 
    if (axisID == 0) setX(speed, direction);
    else             setY(speed, direction);
    
    delay(1); 
  }
}

// =================================================================================
//                             5. BLE & SYSTEM
// =================================================================================

void bleNotify(const String& msg) {
  if (!g_clientConnected || !g_ackChar) return;
  g_ackChar->setValue(msg.c_str());
  g_ackChar->notify();
}

void printStatus(const String& s) { bleNotify("status:" + s); }
void printAckAccepted(const String& id) { bleNotify("ack:accepted " + id); }
void printAckDone(const String& id)     { bleNotify("ack:done " + id); }
void printAckError(const String& id, const String& reason) {
  Serial.println("ERROR: " + reason);
  bleNotify("ack:error " + id + " " + reason);
}

bool parseStandardCommand(const String& line, MoveCmd& out) {
  if (!line.startsWith("CMD ")) return false;
  MoveCmd m;
  int pos = 4; 
  while (pos < line.length()) {
    int space = line.indexOf(' ', pos);
    String token = (space == -1) ? line.substring(pos) : line.substring(pos, space);
    token.trim();
    if (token.length() > 0) {
      int eq = token.indexOf('=');
      if (eq > 0) {
        String key = token.substring(0, eq);
        String val = token.substring(eq + 1);
        key.trim(); val.trim(); key.toLowerCase();
        if      (key == "id")        m.id        = val;
        else if (key == "type")      m.type      = val;
        else if (key == "to")        m.toSq      = val;
        else if (key == "piece")     m.piece     = val;
      }
    }
    if (space == -1) break;
    pos = space + 1;
  }
  if (m.id.isEmpty()) return false;
  m.valid = true;
  out = m;
  return true;
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { g_clientConnected = true; printStatus("connected"); }
  void onDisconnect(BLEServer* pServer) { g_clientConnected = false; pServer->getAdvertising()->start(); }
};

class CmdWriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String line = String(ch->getValue().c_str());
    line.trim();
    if (line.length() == 0) return;

    MoveCmd cmd;
    if (!parseStandardCommand(line, cmd)) { printAckError("unknown", "bad_format"); return; }

    if (cmd.type == "home") {
      printAckAccepted(cmd.id);
      encX.setCount(0); encY.setCount(0);
      Serial.println(">>> SYSTEM HOMED");
      printAckDone(cmd.id);
      return;
    }
    
    if (cmd.type == "move") {
      printAckAccepted(cmd.id);
      char file = cmd.toSq.charAt(0); 
      char rank = cmd.toSq.charAt(1);
      int fileIdx = (file >= 'a') ? (file - 'a' + 1) : (file - 'A' + 1);
      int rankIdx = rank - '0';

      long targetX = calculateTargetX(fileIdx);
      if (!moveToTarget(encX, targetX, 0)) {
        isSystemFaulted = true; printAckError(cmd.id, "FAULT_HOMING_REQUIRED"); return;
      }

      long targetY = calculateTargetY(rankIdx);
      if (!moveToTarget(encY, targetY, 1)) {
        isSystemFaulted = true; printAckError(cmd.id, "FAULT_HOMING_REQUIRED"); return;
      }

      printAckDone(cmd.id);
      Serial.println(">>> MOVE COMPLETE.");
    }
  }
};

// =================================================================================
//                             6. SETUP & LOOP
// =================================================================================

void setup() {
  Serial.begin(115200);
  
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encX.attachHalfQuad(X_ENC_A, X_ENC_B); encX.setCount(0);
  encY.attachHalfQuad(Y_ENC_A, Y_ENC_B); encY.setCount(0);

  ledcAttach(X_PWM_1, 5000, 8); ledcAttach(X_PWM_2, 5000, 8);
  ledcAttach(Y_PWM_1, 5000, 8); ledcAttach(Y_PWM_2, 5000, 8);
  pinMode(LED_PIN, OUTPUT);
  stopAll();

  BLEDevice::init("VibeChess-Board");
  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());
  BLEService* service = g_server->createService(SERVICE_UUID);
  g_ackChar = service->createCharacteristic(ACK_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  g_ackChar->addDescriptor(new BLE2902());
  g_cmdChar = service->createCharacteristic(CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  g_cmdChar->setCallbacks(new CmdWriteCB());
  service->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::getAdvertising()->start();

  Serial.println("\n=== VIBECHESS XY CALIBRATED ===");
  Serial.println("Commands: x1-x8, y1-y8, x5000, y5000, f/b, u/d, z");
  lastHeartbeat = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    printStatus("ready");
    lastHeartbeat = now;
  }

  static long lx=0, ly=0;
  static unsigned long lp=0;
  long cx = encX.getCount(); long cy = encY.getCount();
  if ((cx!=lx || cy!=ly) && (millis()-lp > 200)) {
    Serial.print("X: "); Serial.print(cx); Serial.print(" | Y: "); Serial.println(cy);
    lx=cx; ly=cy; lp=millis();
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char first = input.charAt(0);
    
    // --- PARSER ---
    // If user types 'y1' through 'y8'
    if (first == 'y' || first == 'Y') {
      if (input.length() == 2 && isDigit(input.charAt(1))) {
        int sq = input.substring(1).toInt();
        Serial.print(">>> Cmd: Y-Square "); Serial.println(sq);
        moveToTarget(encY, calculateTargetY(sq), 1);
      } else {
        long target = input.substring(1).toInt();
        Serial.print(">>> Cmd: Y-Raw "); Serial.println(target);
        moveToTarget(encY, target, 1);
      }
    }
    // If user types 'x1' through 'x8'
    else if (first == 'x' || first == 'X') {
      if (input.length() == 2 && isDigit(input.charAt(1))) {
        int sq = input.substring(1).toInt();
        Serial.print(">>> Cmd: X-Square "); Serial.println(sq);
        moveToTarget(encX, calculateTargetX(sq), 0);
      } else {
        long target = input.substring(1).toInt();
        Serial.print(">>> Cmd: X-Raw "); Serial.println(target);
        moveToTarget(encX, target, 0);
      }
    }
    
    // Manual Jogging
    else if (input == "f") setX(MAX_SPEED, true);
    else if (input == "b") setX(MAX_SPEED, false);
    else if (input == "u") setY(MAX_SPEED, true);
    else if (input == "d") setY(MAX_SPEED, false);
    else if (input == "s") stopAll();
    
    else if (input == "z") { 
      encX.setCount(0); encY.setCount(0); 
      Serial.println(">>> ZEROED ALL."); 
    }
  }
}