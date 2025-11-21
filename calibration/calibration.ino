#include <ESP32Encoder.h>

// ===================== HARDWARE PINS =====================
#define BIN_1 26     
#define BIN_2 25     
#define LED_PIN 13   

// ===================== PHYSICS CONFIGURATION (EXACT) =====================
const float COUNTS_PER_INCH = 720.12;  
const float DIST_TO_SQ1     = 0.965;   // Distance from Home Wall to Center of Square 1
const float SQUARE_PITCH    = 1.500;   // Distance from Center to Center of next square

// ===================== SETTINGS =====================
const int SPEED_FAST = 255;   
const int SPEED_SLOW = 150;   
const int SLOW_ZONE = 400;     // Start slowing down 400 counts early
const int TOLERANCE = 10;      // +/- 10 counts error allowed
const int TIMEOUT_MS = 10000;  // 10 Sec Safety Cutoff

// ===================== LIVE VARIABLES =====================
bool invertDirection = true; 

// ===================== OBJECTS =====================
ESP32Encoder encoder;

void setup() {
  Serial.begin(115200);

  // --- Encoder Setup ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(27, 33); 
  encoder.setCount(0);

  // --- Motor Setup ---
  pinMode(LED_PIN, OUTPUT);
  ledcAttach(BIN_1, 5000, 8);
  ledcAttach(BIN_2, 5000, 8);
  motorStop();

  Serial.println("==============================================");
  Serial.println("      CALIBRATED CHESS CONTROLLER");
  Serial.println("==============================================");
  Serial.println("COMMANDS:");
  Serial.println("  [1-8] -> Go to Square 1-8");
  Serial.println("  'z'   -> ZERO at Home Wall");
  Serial.println("  'f'   -> Jog Forward");
  Serial.println("  'r'   -> Jog Reverse");
  Serial.println("  's'   -> STOP");
  Serial.println("  'i'   -> FLIP DIRECTION");
  Serial.println("----------------------------------------------");
  
  // Verify Math on Startup
  Serial.print("Math Check: Sq 1 target = "); 
  Serial.print(calculateSquareTarget(1)); Serial.println(" counts");
  Serial.print("Math Check: Sq 2 target = "); 
  Serial.println(calculateSquareTarget(2));
  Serial.println("==============================================");
}

// ===================== HELPER: CALCULATE SQUARE POSITION =====================
long calculateSquareTarget(int squareNumber) {
  // Formula: 
  // Distance = (Distance to First Center) + (Number of Jumps * 1.5 inches)
  // Note: Square 1 has 0 jumps. Square 2 has 1 jump.
  
  if (squareNumber < 1) squareNumber = 1; // Safety
  
  float inches = DIST_TO_SQ1 + ((squareNumber - 1) * SQUARE_PITCH);
  
  return (long)(inches * COUNTS_PER_INCH);
}

// ===================== MOTOR HELPERS =====================
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

// ===================== SMART MOVE LOGIC =====================
void moveToTarget(long target) {
  
  Serial.print(">>> MOVING to Target: "); Serial.println(target);
  digitalWrite(LED_PIN, HIGH);

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0; 

  while (true) {
    // 1. Emergency Stop Check
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 's') {
        motorStop();
        Serial.read(); 
        Serial.println(">>> EMERGENCY STOP");
        break;
      }
    }

    // 2. Get Status
    long current = encoder.getCount();
    long error = target - current; 

    // 3. Debug Print (Every 500ms)
    if (millis() - lastPrintTime > 500) {
      lastPrintTime = millis();
      Serial.print("   Pos: "); Serial.print(current);
      Serial.print(" | Err: "); Serial.println(error);
    }

    // 4. Success?
    if (abs(error) <= TOLERANCE) {
      motorStop();
      Serial.print(">>> ARRIVED! Final Pos: "); Serial.println(current);
      break;
    }

    // 5. Timeout?
    if (millis() - startTime > TIMEOUT_MS) {
      motorStop();
      Serial.println(">>> TIMEOUT ERROR!");
      break;
    }

    // 6. Move
    int speed = (abs(error) > SLOW_ZONE) ? SPEED_FAST : SPEED_SLOW;
    bool direction = (error > 0); 
    setMotor(speed, direction);
    
    delay(5);
  }
  digitalWrite(LED_PIN, LOW);
}

// ===================== MAIN LOOP =====================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 

    if (input.length() > 0) {
      char firstChar = input.charAt(0);

      // 1. COMMANDS
      if (firstChar == 'i') {
        invertDirection = !invertDirection;
        motorStop();
        Serial.println(invertDirection ? ">>> CONFIG: INVERTED" : ">>> CONFIG: NORMAL");
      }
      else if (firstChar == 'f') { Serial.println("Jog FWD"); setMotor(SPEED_FAST, true); }
      else if (firstChar == 'r') { Serial.println("Jog REV"); setMotor(SPEED_FAST, false); }
      else if (firstChar == 's') { Serial.println("STOP"); motorStop(); }
      else if (firstChar == 'z') { encoder.setCount(0); Serial.println(">>> ZEROED"); }
      
      // 2. NUMERIC INPUT
      else if (isDigit(firstChar) || firstChar == '-') {
        long val = input.toInt();

        // LOGIC: 
        // If input is 1-8, it's a SQUARE index.
        // If input is larger (e.g. 500, 695), it's RAW COUNTS.
        
        if (val >= 1 && val <= 8) {
          Serial.print(">>> SQUARE COMMAND: "); Serial.print(val);
          long squareTarget = calculateSquareTarget((int)val);
          Serial.print(" (Target Count: "); Serial.print(squareTarget); Serial.println(")");
          moveToTarget(squareTarget);
        }
        else {
           Serial.print(">>> RAW COUNT COMMAND: "); Serial.println(val);
           moveToTarget(val);
        }
      }
    }
  }
}