#include <Arduino.h>
#include <ESP32Encoder.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =================================================================================
//                             1. HARDWARE CONFIGURATION
// =================================================================================

// --- PINS ---
#define BIN_1 26     
#define BIN_2 25     
#define LED_PIN 13   

// --- PHYSICS CONSTANTS ---
const float COUNTS_PER_INCH = 720.12;  
const float DIST_TO_SQ1     = 0.965;   
const float SQUARE_PITCH    = 1.500;   

// --- MOTOR SETTINGS ---
const int SPEED_FAST = 255;   
const int SPEED_SLOW = 150;   
const int HOMING_SPEED = 120;  // Slower speed for auto-homing
const int SLOW_ZONE = 400;     
const int TOLERANCE = 10;      
const int CRITICAL_FAULT = 50; 
const int TIMEOUT_MS = 10000;  

// --- LIVE VARIABLES ---
bool invertDirection = true; 
bool isSystemFaulted = false; 

// --- STATE TRACKING ---
long lastFailedTarget = 0; // Remembers where we WANTED to go

// --- OBJECTS ---
ESP32Encoder encoder;

// =================================================================================
//                             2. BLE CONFIGURATION
// =================================================================================

static const char* SERVICE_UUID   = "b5b10001-6f3b-4c90-8e2b-7d1a2fa9c001";
static const char* CMD_CHAR_UUID  = "b5b10002-6f3b-4c90-8e2b-7d1a2fa9c002"; 
static const char* ACK_CHAR_UUID  = "b5b10003-6f3b-4c90-8e2b-7d1a2fa9c003"; 

const unsigned long HEARTBEAT_INTERVAL_MS = 5000;
unsigned long lastHeartbeat = 0;

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
  String notation;
  bool   valid = false;
};

// =================================================================================
//                             3. HARDWARE HELPER FUNCTIONS
// =================================================================================

long calculateSquareTarget(int squareNumber) {
  if (squareNumber < 1) squareNumber = 1; 
  if (squareNumber > 8) squareNumber = 8;
  float inches = DIST_TO_SQ1 + ((squareNumber - 1) * SQUARE_PITCH);
  return (long)(inches * COUNTS_PER_INCH);
}

void setMotor(int pwm, bool forward) {
  bool actualDirection = forward;
  if (invertDirection) actualDirection = !forward;
  if (actualDirection) {
    ledcWrite(BIN_1, LOW); ledcWrite(BIN_2, pwm);
  } else {
    ledcWrite(BIN_1, pwm); ledcWrite(BIN_2, LOW);
  }
}

void motorStop() {
  ledcWrite(BIN_1, LOW); ledcWrite(BIN_2, LOW);
}

// --- AUTO HOMING ROUTINE ---
// Drives backwards for 3 seconds to hit the wall, then zeroes.
void performAutoHome() {
  Serial.println(">>> ACTION: Starting Auto-Homing Sequence...");
  digitalWrite(LED_PIN, HIGH);
  
  // Drive towards 0 (Reverse) at low speed
  setMotor(HOMING_SPEED, false); 
  
  // Run for 3 seconds (assuming this is enough to hit the wall from anywhere)
  delay(3000); 
  
  motorStop();
  encoder.setCount(0);
  isSystemFaulted = false; // CLEAR FAULT
  
  Serial.println(">>> ACTION: Auto-Home Complete. Encoder Zeroed.");
  digitalWrite(LED_PIN, LOW);
}

bool moveToTarget(long target) {
  digitalWrite(LED_PIN, HIGH);
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0; 
  bool success = false;

  lastFailedTarget = target; // Save for recovery

  while (true) {
    // Emergency Stop
    if (Serial.available()) {
      if (Serial.peek() == 's') {
        motorStop(); Serial.read(); return false; 
      }
    }

    long current = encoder.getCount();
    long error = target - current; 

    if (millis() - lastPrintTime > 500) {
      lastPrintTime = millis();
      Serial.print("   Current X: "); Serial.print(current);
      Serial.print(" | Distance: "); Serial.println(error);
    }

    if (abs(error) <= TOLERANCE) {
      motorStop();
      Serial.print("   >>> ARRIVED! Final Position: "); Serial.println(current);
      success = true;
      break;
    }

    if (millis() - startTime > TIMEOUT_MS) {
      motorStop();
      Serial.println("   >>> TIMEOUT ERROR! Motor stuck?");
      return false; 
    }

    int speed = (abs(error) > SLOW_ZONE) ? SPEED_FAST : SPEED_SLOW;
    bool direction = (error > 0); 
    setMotor(speed, direction);
    delay(5); 
  }
  digitalWrite(LED_PIN, LOW);
  
  // Final Validation
  long finalPos = encoder.getCount();
  if (abs(target - finalPos) > CRITICAL_FAULT) {
    return false;
  }

  return success;
}

// =================================================================================
//                             4. BLE HELPER FUNCTIONS
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
  Serial.println("ERROR Sent to App: " + reason);
  bleNotify("ack:error " + id + " " + reason);
}

String getPieceFullName(String letter) {
  letter.toUpperCase();
  if (letter == "P") return "Pawn";
  if (letter == "N") return "Knight";
  if (letter == "B") return "Bishop";
  if (letter == "R") return "Rook";
  if (letter == "Q") return "Queen";
  if (letter == "K") return "King";
  return letter; 
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
        else if (key == "from")      m.fromSq    = val;
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

// =================================================================================
//                             5. BLE CALLBACKS
// =================================================================================

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    g_clientConnected = true;
    printStatus("connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    g_clientConnected = false;
    pServer->getAdvertising()->start();
  }
};

class CmdWriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String line = String(ch->getValue().c_str());
    line.trim();
    if (line.length() == 0) return;

    MoveCmd cmd;
    bool ok = parseStandardCommand(line, cmd);
    if (!ok) {
      printAckError("unknown", "bad_format");
      return;
    }

    // --- SPECIAL COMMAND: HOME ---
    if (cmd.type == "home") {
      printAckAccepted(cmd.id);
      performAutoHome();
      printAckDone(cmd.id);
      return;
    }

    // --- SPECIAL COMMAND: CONFIRM (Manual Override) ---
    if (cmd.type == "confirm") {
      printAckAccepted(cmd.id);
      // We trust the user moved it to the last failed target
      // Setting encoder count here ensures next move is calculated correctly
      encoder.setCount(lastFailedTarget); 
      isSystemFaulted = false;
      Serial.println(">>> MANUAL CONFIRM: Unlocked. Position set to " + String(lastFailedTarget));
      printAckDone(cmd.id);
      return;
    }

    // --- CHECK FAULT STATE ---
    if (isSystemFaulted) {
      printAckError(cmd.id, "FAULT_HOMING_REQUIRED");
      return;
    }

    // --- NORMAL MOVE ---
    if (cmd.type == "move") {
      printAckAccepted(cmd.id);

      // Narrative Log
      String fullPiece = getPieceFullName(cmd.piece);
      Serial.println("\n================================================");
      Serial.print("User Input: User moved "); Serial.print(fullPiece); 
      Serial.print(" from "); Serial.print(cmd.fromSq);
      Serial.print(" to "); Serial.println(cmd.toSq);
      Serial.println("================================================");

      char fileChar = cmd.toSq.charAt(0); 
      int fileIndex = 0;
      if (fileChar >= 'a' && fileChar <= 'h') fileIndex = fileChar - 'a' + 1;
      else if (fileChar >= 'A' && fileChar <= 'H') fileIndex = fileChar - 'A' + 1;

      if (fileIndex >= 1 && fileIndex <= 8) {
        long targetCounts = calculateSquareTarget(fileIndex);
        
        Serial.println("Sending Logic to X-Axis Motor...");
        Serial.print("ACTION (X-Direction): Moving Motor to ");
        Serial.println(targetCounts);
        
        bool moveSuccess = moveToTarget(targetCounts); 

        if (moveSuccess) {
          Serial.println("Sending Logic to Y-Axis Motor...");
          Serial.println("ACTION (Y-Direction): NOT IMPLEMENTED YET");
          printAckDone(cmd.id);
          Serial.println("Move Complete.");
          Serial.println("------------------------------------------------");
        } 
        else {
          isSystemFaulted = true; 
          printAckError(cmd.id, "FAULT_HOMING_REQUIRED");
          Serial.println("!!! CRITICAL ERROR: Motor stuck.");
        }
      } else {
        printAckError(cmd.id, "invalid_file");
      }
    }
  }
};

// =================================================================================
//                             6. SETUP & LOOP
// =================================================================================

void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(27, 33); 
  encoder.setCount(0);
  
  pinMode(LED_PIN, OUTPUT);
  ledcAttach(BIN_1, 5000, 8);
  ledcAttach(BIN_2, 5000, 8);
  motorStop();

  BLEDevice::init("VibeChess-Board");
  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());

  BLEService* service = g_server->createService(SERVICE_UUID);
  g_ackChar = service->createCharacteristic(ACK_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  g_ackChar->addDescriptor(new BLE2902());
  g_cmdChar = service->createCharacteristic(CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  g_cmdChar->setCallbacks(new CmdWriteCB());
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06); 
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  printStatus("ready");
  Serial.println("SYSTEM READY.");
  lastHeartbeat = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    printStatus("ready");
    lastHeartbeat = now;
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    if (input.length() > 0) {
      char firstChar = input.charAt(0);
      // --- MANUAL COMMANDS ---
      if (firstChar == 'z') { 
        encoder.setCount(0); 
        isSystemFaulted = false; // RESET FAULT ON ZERO
        Serial.println(">>> ZEROED. System Unlocked."); 
      }
      else if (firstChar == 'i') { invertDirection = !invertDirection; Serial.println("Inverted."); }
      else if (firstChar == 'f') { setMotor(SPEED_FAST, true); }
      else if (firstChar == 'r') { setMotor(SPEED_FAST, false); }
      else if (firstChar == 's') { motorStop(); }
      else if (isDigit(firstChar)) {
        long val = input.toInt();
        if (val >= 1 && val <= 8) {
             moveToTarget(calculateSquareTarget((int)val));
        } else {
             moveToTarget(val);
        }
      }
    }
  }
}