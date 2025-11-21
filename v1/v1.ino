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

// --- PHYSICS CONSTANTS (EXACT) ---
const float COUNTS_PER_INCH = 720.12;  
const float DIST_TO_SQ1     = 0.965;   // Distance from Home Wall to Center of Square 1
const float SQUARE_PITCH    = 1.500;   // Distance from Center to Center of next square

// --- MOTOR SETTINGS ---
const int SPEED_FAST = 255;   
const int SPEED_SLOW = 150;   
const int SLOW_ZONE = 400;     // Start slowing down 400 counts early
const int TOLERANCE = 10;      // +/- 10 counts error allowed
const int TIMEOUT_MS = 10000;  // 10 Sec Safety Cutoff

// --- LIVE VARIABLES ---
bool invertDirection = true; 

// --- OBJECTS ---
ESP32Encoder encoder;

// =================================================================================
//                             2. BLE CONFIGURATION
// =================================================================================

// --- UUIDS ---
static const char* SERVICE_UUID   = "b5b10001-6f3b-4c90-8e2b-7d1a2fa9c001";
static const char* CMD_CHAR_UUID  = "b5b10002-6f3b-4c90-8e2b-7d1a2fa9c002"; // write
static const char* ACK_CHAR_UUID  = "b5b10003-6f3b-4c90-8e2b-7d1a2fa9c003"; // notify

// --- TIMING & STATE ---
const unsigned long HEARTBEAT_INTERVAL_MS = 5000;
unsigned long lastHeartbeat = 0;

// --- BLE GLOBALS ---
BLEServer* g_server          = nullptr;
BLECharacteristic* g_cmdChar         = nullptr;  // write-only
BLECharacteristic* g_ackChar         = nullptr;  // notify-only
bool               g_clientConnected = false;

// --- DATA STRUCTURES ---
struct MoveCmd {
  String id;
  String notation;
  String fromSq;
  String toSq;
  String piece;    
  String type;     
  String source;   
  String timestamp;
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
  if (invertDirection) {
    actualDirection = !forward;
  }

  if (actualDirection) {
    ledcWrite(BIN_1, LOW);
    ledcWrite(BIN_2, pwm);
  } else {
    ledcWrite(BIN_1, pwm);
    ledcWrite(BIN_2, LOW);
  }
}

void motorStop() {
  ledcWrite(BIN_1, LOW);
  ledcWrite(BIN_2, LOW);
}

// The Core Movement Logic (Blocking with timeout)
void moveToTarget(long target) {
  // CHANGED LOG: Narrative style
  // We don't print "Moving to target" here because the main loop does it more descriptively.
  digitalWrite(LED_PIN, HIGH);

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0; 

  while (true) {
    // 1. Emergency Stop Check from Serial
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 's') {
        motorStop();
        Serial.read(); 
        Serial.println(">>> EMERGENCY STOP TRIGGERED");
        break;
      }
    }

    // 2. Get Status
    long current = encoder.getCount();
    long error = target - current; 

    // 3. Debug Print (Every 500ms) -> CHANGED LOG FORMAT
    if (millis() - lastPrintTime > 500) {
      lastPrintTime = millis();
      Serial.print("   Current X Position: "); 
      Serial.print(current);
      Serial.print(" | Distance to Travel: "); 
      Serial.println(error);
    }

    // 4. Success?
    if (abs(error) <= TOLERANCE) {
      motorStop();
      Serial.print("   >>> ARRIVED! Final Position: "); Serial.println(current);
      break;
    }

    // 5. Timeout?
    if (millis() - startTime > TIMEOUT_MS) {
      motorStop();
      Serial.println("   >>> TIMEOUT ERROR! Motor stuck?");
      break;
    }

    // 6. Move
    int speed = (abs(error) > SLOW_ZONE) ? SPEED_FAST : SPEED_SLOW;
    bool direction = (error > 0); 
    setMotor(speed, direction);
    
    delay(5); // Stability delay
  }
  digitalWrite(LED_PIN, LOW);
}

// =================================================================================
//                             4. BLE HELPER FUNCTIONS
// =================================================================================

void bleNotify(const String& msg) {
  if (!g_clientConnected || !g_ackChar) return;
  g_ackChar->setValue(msg.c_str());
  g_ackChar->notify();
}

void printStatus(const String& s) {
  bleNotify("status:" + s);
}

void printAckAccepted(const String& id) {
  String m = "ack:accepted " + id;
  // Serial.println(">>> BLE: Sent " + m); // Optional: Hide technical ACK log
  bleNotify(m);
}

void printAckDone(const String& id) {
  String m = "ack:done " + id;
  // Serial.println(">>> BLE: Sent " + m); // Optional: Hide technical ACK log
  bleNotify(m);
}

void printAckError(const String& id, const String& reason) {
  String m = "ack:error " + id + " " + reason;
  Serial.println("ERROR: " + m);
  bleNotify(m);
}

bool parseStandardCommand(const String& line, MoveCmd& out) {
  if (!line.startsWith("CMD ")) return false;

  MoveCmd m;
  int pos = 4; // skip "CMD "
  while (pos < line.length()) {
    int space = line.indexOf(' ', pos);
    String token = (space == -1) ? line.substring(pos) : line.substring(pos, space);
    token.trim();
    if (token.length() > 0) {
      int eq = token.indexOf('=');
      if (eq > 0) {
        String key = token.substring(0, eq);
        String val = token.substring(eq + 1);
        key.trim(); val.trim();
        key.toLowerCase();

        if      (key == "id")        m.id        = val;
        else if (key == "notation")  m.notation  = val;
        else if (key == "from")      m.fromSq    = val;
        else if (key == "to")        m.toSq      = val;
        else if (key == "piece")     m.piece     = val;
        else if (key == "type")      m.type      = val;
        else if (key == "source")    m.source    = val;
        else if (key == "timestamp") m.timestamp = val;
      }
    }
    if (space == -1) break;
    pos = space + 1;
  }

  if (m.id.isEmpty() || m.toSq.isEmpty()) {
    return false;
  }
  m.valid = true;
  out = m;
  return true;
}

// =================================================================================
//                             5. BLE CALLBACKS (THE BRIDGE)
// =================================================================================

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    g_clientConnected = true;
    Serial.println("\n>>> CONNECTION: App Connected Successfully.");
    printStatus("connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    g_clientConnected = false;
    Serial.println("\n>>> CONNECTION: App Disconnected.");
    printStatus("disconnected");
    pServer->getAdvertising()->start();
  }
};

class CmdWriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String line = String(ch->getValue().c_str());
    line.trim();
    if (line.length() == 0) return;

    // 1. Parse Command
    MoveCmd cmd;
    bool ok = parseStandardCommand(line, cmd);
    if (!ok) {
      printAckError("unknown", "bad_format");
      Serial.println("ERROR: Bad Command Format Received: " + line);
      return;
    }

    // 2. Acknowledge Immediately (Fixes UI "undefined")
    printAckAccepted(cmd.id);

    // =============================================================
    //                 CHANGED: NARRATIVE LOGGING
    // =============================================================
    
    Serial.println("\n================================================");
    Serial.print("User Input: User moved ");
    Serial.print(cmd.piece); // e.g. "P"
    Serial.print(" from ");
    Serial.print(cmd.fromSq);
    Serial.print(" to ");
    Serial.println(cmd.toSq);
    Serial.println("================================================");

    // 4. EXTRACT COORDINATES
    char fileChar = cmd.toSq.charAt(0); 
    char rankChar = cmd.toSq.charAt(1);

    // --- X-AXIS (FILES) ---
    int fileIndex = 0;
    if (fileChar >= 'a' && fileChar <= 'h') fileIndex = fileChar - 'a' + 1;
    else if (fileChar >= 'A' && fileChar <= 'H') fileIndex = fileChar - 'A' + 1;

    // --- Y-AXIS (RANKS) ---
    int rankIndex = 0;
    if (rankChar >= '1' && rankChar <= '8') rankIndex = rankChar - '0';

    // 5. EXECUTION LOGIC
    if (fileIndex >= 1 && fileIndex <= 8) {
      
      long targetCounts = calculateSquareTarget(fileIndex);

      // --- X-AXIS LOGIC ---
      Serial.println("Sending Logic to X-Axis Motor...");
      Serial.print("ACTION (X-Direction): Moving Motor to ");
      Serial.println(targetCounts);
      
      // Execute X Move
      moveToTarget(targetCounts); 

      // --- Y-AXIS LOGIC ---
      Serial.println("Sending Logic to Y-Axis Motor...");
      Serial.println("ACTION (Y-Direction): NOT IMPLEMENTED YET");
      // (No physical move here yet)
      
      printAckDone(cmd.id);
      Serial.println("Move Complete. Awaiting next input.");
      Serial.println("------------------------------------------------");
      
    } else {
      printAckError(cmd.id, "invalid_file");
      Serial.println("ERROR: Invalid File Character: " + String(fileChar));
    }
  }
};

// =================================================================================
//                             6. SETUP
// =================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== VIBECHESS CONTROLLER ===");

  // --- HARDWARE INIT ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(27, 33); 
  encoder.setCount(0);

  pinMode(LED_PIN, OUTPUT);
  ledcAttach(BIN_1, 5000, 8);
  ledcAttach(BIN_2, 5000, 8);
  motorStop();

  // --- BLE INIT ---
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

  // --- READY ---
  printStatus("ready");
  Serial.println("SYSTEM READY. Waiting for connection...");
  lastHeartbeat = millis();
}

// =================================================================================
//                             7. MAIN LOOP
// =================================================================================

void loop() {
  unsigned long now = millis();

  // 1. HEARTBEAT
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    printStatus("ready");
    lastHeartbeat = now;
  }

  // 2. MANUAL SERIAL INPUT
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 

    if (input.length() > 0) {
      char firstChar = input.charAt(0);

      if (firstChar == 'i') {
        invertDirection = !invertDirection;
        motorStop();
        Serial.println(invertDirection ? ">>> CONFIG: INVERTED" : ">>> CONFIG: NORMAL");
      }
      else if (firstChar == 'f') { Serial.println("Jog FWD"); setMotor(SPEED_FAST, true); }
      else if (firstChar == 'r') { Serial.println("Jog REV"); setMotor(SPEED_FAST, false); }
      else if (firstChar == 's') { Serial.println("STOP"); motorStop(); }
      else if (firstChar == 'z') { encoder.setCount(0); Serial.println(">>> ZEROED"); }
      
      else if (isDigit(firstChar) || firstChar == '-') {
        long val = input.toInt();
        if (val >= 1 && val <= 8) {
          Serial.print(">>> MANUAL CMD: Square "); Serial.print(val);
          long squareTarget = calculateSquareTarget((int)val);
          Serial.print(" -> Target: "); Serial.println(squareTarget);
          moveToTarget(squareTarget);
        }
        else {
           Serial.print(">>> MANUAL CMD: Raw Counts "); Serial.println(val);
           moveToTarget(val);
        }
      }
    }
  }
}