#include <ESP32Encoder.h>

// ===================== PINS =====================
#define BIN_1 26     
#define BIN_2 25     
#define ENC_A 27
#define ENC_B 33
#define LED_PIN 13

// ===================== SETTINGS =====================
int currentSpeed = 110; // Start conservative
bool invertDirection = false; 

// ===================== OBJECTS =====================
ESP32Encoder encoder;

void setup() {
  Serial.begin(115200);

  // --- Encoder Setup ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(ENC_A, ENC_B); 
  encoder.setCount(0);

  // --- Motor Setup ---
  pinMode(LED_PIN, OUTPUT);
  ledcAttach(BIN_1, 5000, 8);
  ledcAttach(BIN_2, 5000, 8);
  
  motorStop();

  Serial.println("==============================================");
  Serial.println("      Z-AXIS SPEED TUNER");
  Serial.println("==============================================");
  Serial.println("CONTROLS:");
  Serial.println("  'w' = Move UP (Current Speed)");
  Serial.println("  's' = Move DOWN (Current Speed)");
  Serial.println("  'x' = STOP");
  Serial.println("----------------------------------------------");
  Serial.println("SPEED ADJUST:");
  Serial.println("  '+' = Increase Speed (+10)");
  Serial.println("  '-' = Decrease Speed (-10)");
  Serial.println("==============================================");
  Serial.print(">>> INITIAL SPEED: "); Serial.println(currentSpeed);
}

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

void loop() {
  // Live Feedback
  static long lastPos = 0;
  long currentPos = encoder.getCount();
  if (currentPos != lastPos) {
    Serial.print("Pos: "); Serial.println(currentPos);
    lastPos = currentPos;
  }

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

    switch (cmd) {
      case 'w': 
        Serial.print("Moving UP at "); Serial.println(currentSpeed);
        setMotor(currentSpeed, true); 
        break;
      case 's': 
        Serial.print("Moving DOWN at "); Serial.println(currentSpeed);
        setMotor(currentSpeed, false); 
        break;
      case 'x': 
        motorStop(); 
        Serial.println("STOPPED.");
        break;
      
      // SPEED ADJUSTMENT
      case '+':
        currentSpeed += 10;
        if (currentSpeed > 255) currentSpeed = 255;
        Serial.print(">>> SPEED INCREASED: "); Serial.println(currentSpeed);
        break;
      case '-':
        currentSpeed -= 10;
        if (currentSpeed < 50) currentSpeed = 50;
        Serial.print(">>> SPEED DECREASED: "); Serial.println(currentSpeed);
        break;
        
      case 'z': 
        encoder.setCount(0); 
        Serial.println(">>> ZEROED."); 
        break;
      case 'i':
        invertDirection = !invertDirection;
        Serial.println(">>> Direction Inverted.");
        break;
    }
  }
}