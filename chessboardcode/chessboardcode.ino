#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ble uuids
static const char* SERVICE_UUID   = "b5b10001-6f3b-4c90-8e2b-7d1a2fa9c001";
static const char* CMD_CHAR_UUID  = "b5b10002-6f3b-4c90-8e2b-7d1a2fa9c002"; // write
static const char* ACK_CHAR_UUID  = "b5b10003-6f3b-4c90-8e2b-7d1a2fa9c003"; // notify

// intervals
const unsigned long HEARTBEAT_INTERVAL_MS = 5000;
unsigned long lastHeartbeat = 0;

// stage timing
const unsigned long STAGE2_TRAVEL_MS = 2500;   // moving magnet across board
const unsigned long STAGE3_MOVE_MS   = 2500;   // moving piece to destination
const unsigned long COOLDOWN_MS      = 5000;   // cooldown before next input

// stage fsm
enum StageState {
  STAGE_IDLE = 0,          // stage 1
  STAGE_MOVE_TRAVEL,       // stage 2
  STAGE_MOVE_PICKPLACE,    // stage 3
  STAGE_COOLDOWN
};
StageState stageState = STAGE_IDLE;
unsigned long stageStartMs = 0;

// gatt global vars
BLEServer*         g_server          = nullptr;
BLECharacteristic* g_cmdChar         = nullptr;  // write-only (from client)
BLECharacteristic* g_ackChar         = nullptr;  // notify-only (to client)
bool               g_clientConnected = false;

// standardized data structure
struct MoveCmd {
  String id;
  String notation;
  String fromSq;
  String toSq;
  String piece;    // normalized uppercase P,N,B,R,Q,K
  String type;     // optional "move"
  String source;   // optional "ui" | "voice"
  String timestamp;
  bool   valid = false;
};

// service function: send ble notify (acks/status) safely if a client is connected
void bleNotify(const String& msg) {
  if (!g_clientConnected || !g_ackChar) return;
  g_ackChar->setValue(msg.c_str());
  g_ackChar->notify();
}

// service function: print + notify a status line
void printStatus(const String& s) {
  Serial.print("status:");
  Serial.println(s);
  bleNotify("status:" + s);
}

// service function: print + notify ack accepted
void printAckAccepted(const String& id) {
  String m = "ack:accepted " + id;
  Serial.println(m);
  bleNotify(m);
}

// service function: print + notify ack done
void printAckDone(const String& id) {
  String m = "ack:done " + id;
  Serial.println(m);
  bleNotify(m);
}

// service function: print + notify ack error with reason
void printAckError(const String& id, const String& reason) {
  String m = "ack:error " + id + " " + reason;
  Serial.println(m);
  bleNotify(m);
}

// service function: print stage 1 banner
void printStage1() {
  Serial.println("Stage 1: Ready for Input. Electromagnet and Motor OFF.");
}

// service function: enter a new stage and emit the one-time banner for that stage
void enterStage(StageState s) {
  stageState = s;
  stageStartMs = millis();
  switch (s) {
    case STAGE_IDLE:
      printStage1();
      break;
    case STAGE_MOVE_TRAVEL:
      Serial.println("Motors ON. Electromagnet OFF. Moving magnet across the board.");
      break;
    case STAGE_MOVE_PICKPLACE:
      Serial.println("Motors ON. Electromagnet ON. Moving chess piece to destination.");
      break;
    case STAGE_COOLDOWN:
      Serial.println("Move complete. Returning to idle state.");
      Serial.println("Cooling down for 5 seconds before next move...");
      break;
  }
}

// service function: parse "CMD key=value key=value ..." into MoveCmd
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
        // unknown keys ignored safely (everything else ignored)
      }
    }
    if (space == -1) break;
    pos = space + 1;
  }

  // min required fields
  if (m.id.isEmpty() || m.notation.isEmpty() || m.fromSq.isEmpty() || m.toSq.isEmpty() || m.piece.isEmpty()) {
    return false;
  }

  // optional
  if (m.type.length() && !(m.type == "move" || m.type == "MOVE")) {
    return false;
  }

  // validate squares
  auto isSquare = [](const String& sq)->bool{
    if (sq.length() != 2) return false;
    char f = sq.charAt(0), r = sq.charAt(1);
    return (f >= 'a' && f <= 'h') && (r >= '1' && r <= '8');
  };
  if (!isSquare(m.fromSq) || !isSquare(m.toSq)) return false;

  // normalize piece to uppercase P/N/B/R/Q/K
  char p = m.piece.charAt(0);
  if (p >= 'a' && p <= 'z') p -= 32;
  if (!(p=='P'||p=='N'||p=='B'||p=='R'||p=='Q'||p=='K')) return false;
  m.piece = String(p);

  m.valid = true;
  out = m;
  return true;
}

// service class: server connection callbacks (connect/disconnect events)
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    g_clientConnected = true;
    printStatus("connected");
    printStage1(); // show we are in stage 1
  }
  void onDisconnect(BLEServer* pServer) override {
    g_clientConnected = false;
    printStatus("disconnected");
    pServer->getAdvertising()->start();
  }
};

// service class: characteristic write callback (incoming cmd event)
class CmdWriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    // event checker: incoming write -> read payload into string
    String line = String(ch->getValue().c_str());
    line.trim();
    if (line.length() == 0) return;

    // event checker: standardized command parse
    MoveCmd cmd;
    bool ok = parseStandardCommand(line, cmd);
    if (!ok) {
      printAckError("unknown", "bad_format");
      return;
    }

    // event checker: reject if stage is not idle (busy)
    if (stageState != STAGE_IDLE) {
      printAckError(cmd.id, "busy");
      return;
    }

    // service function: acknowledge acceptance
    printAckAccepted(cmd.id);

    // service function: print parsed contents (unchanged)
    Serial.println("----- RECEIVED STANDARDIZED MOVE (BLE) -----");
    Serial.print("id: ");        Serial.println(cmd.id);
    Serial.print("type: ");      Serial.println(cmd.type.length() ? cmd.type : "move");
    Serial.print("notation: ");  Serial.println(cmd.notation);
    Serial.print("from: ");      Serial.println(cmd.fromSq);
    Serial.print("to: ");        Serial.println(cmd.toSq);
    Serial.print("piece: ");     Serial.println(cmd.piece);
    Serial.print("source: ");    Serial.println(cmd.source.length() ? cmd.source : "(unspecified)");
    Serial.print("timestamp: "); Serial.println(cmd.timestamp.length() ? cmd.timestamp : "(unspecified)");
    Serial.println("------------------------------------------------");

    // service function: immediate done ack (preserves previous behavior)
    printAckDone(cmd.id);

    // service function: start staged flow (non-blocking)
    enterStage(STAGE_MOVE_TRAVEL);
  }
};

// service function: device setup and gatt init
void setup() {
  Serial.begin(115200);
  Serial.println("\nVibeChess – BLE Receiver Init");

  BLEDevice::init("VibeChess-Board");
  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());

  BLEService* service = g_server->createService(SERVICE_UUID);

  g_ackChar = service->createCharacteristic(
      ACK_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  g_ackChar->addDescriptor(new BLE2902());

  g_cmdChar = service->createCharacteristic(
      CMD_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  g_cmdChar->setCallbacks(new CmdWriteCB());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // service function: initialize fsm into stage 1 and print banner
  enterStage(STAGE_IDLE);

  // service function: first heartbeat setup
  printStatus("ready");
  lastHeartbeat = millis();
}

// event checker: main loop tick (timers, heartbeats, and fsm transitions)
void loop() {
  unsigned long now = millis();

  // event checker: heartbeat interval
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    printStatus("ready");
    lastHeartbeat = now;
  }

  // event checker: stage timers and transitions (non-blocking fsm)
  switch (stageState) {
    case STAGE_IDLE:
      // idle accepts new commands (no action here)
      break;

    case STAGE_MOVE_TRAVEL:
      if (now - stageStartMs >= STAGE2_TRAVEL_MS) {
        enterStage(STAGE_MOVE_PICKPLACE);
      }
      break;

    case STAGE_MOVE_PICKPLACE:
      if (now - stageStartMs >= STAGE3_MOVE_MS) {
        enterStage(STAGE_COOLDOWN);
      }
      break;

    case STAGE_COOLDOWN:
      if (now - stageStartMs >= COOLDOWN_MS) {
        enterStage(STAGE_IDLE);
      }
      break;
  }
}