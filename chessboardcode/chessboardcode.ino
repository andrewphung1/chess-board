#include <WiFi.h>
#include <WiFiClientSecure.h> // 1. ADD THIS LINE
#include <HTTPClient.h>     // For making web requests
#include <ArduinoJson.h>    // For parsing Google's response
#include <Base64.h>         // For encoding the audio
#include "driver/i2s.h"

// ===================================
// 1. ENTER YOUR WI-FI INFO
// ===================================
const char* ssid = "";
const char* password = "";

// ===================================
// 2. ENTER YOUR GOOGLE API KEY
// ===================================
String GOOGLE_API_KEY = "";

// ===================================
// 3. YOUR PINS
// ===================================
#define I2S_WS 13
#define I2S_SD 27
#define I2S_SCK 25
#define I2S_PORT I2S_NUM_0

// ===================================
// 4. AUDIO RECORDING SETTINGS
// ===================================
#define SAMPLE_RATE 16000
#define RECORD_DURATION_SECONDS 1 // 1 second
#define RECORD_BUFFER_SIZE (SAMPLE_RATE * RECORD_DURATION_SECONDS)

// Audio buffer
int16_t audioBuffer[RECORD_BUFFER_SIZE];

// The HTTP client objects
WiFiClientSecure client; // 2. CHANGE THIS LINE
HTTPClient http;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting up...");
  
  // Tell the client to skip certificate verification.
  client.setInsecure(); 
  
  setup_wifi(); 

  Serial.println("Setting up I2S (16000 Hz)...");
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  
  Serial.println("\nReady!");
  Serial.println("Type 'r' and press Enter to start recording...");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'r') {
      recordAudio();
      sendToGoogle();
      Serial.println("\nType 'r' and press Enter to start recording...");
    }
  }
}

void recordAudio() {
  Serial.println("Recording...");
  
  size_t bytes_read = 0;
  esp_err_t result = i2s_read(I2S_PORT, 
                              &audioBuffer, 
                              sizeof(audioBuffer),
                              &bytes_read, 
                              portMAX_DELAY);

  if (result != ESP_OK) {
    Serial.println("Error reading from I2S");
  }

  int samples_read = bytes_read / 2;
  Serial.printf("Recording complete! Read %d samples.\n", samples_read);
}

void sendToGoogle() {
  Serial.println("Connecting to Google Speech-to-Text...");
  
  String url = "https://speech.googleapis.com/v1/speech:recognize?key=" + GOOGLE_API_KEY;
  
  if (http.begin(client, url)) { 
    http.addHeader("Content-Type", "application/json");

    Serial.println("Encoding audio...");
    String base64Audio;
    uint8_t* audioBytes = (uint8_t*)audioBuffer; // Corrected type
    int audioBytesLength = sizeof(audioBuffer);
    
    base64Audio = base64::encode(audioBytes, audioBytesLength);

    Serial.println("Creating JSON request...");
    
    DynamicJsonDocument jsonRequest(1024 + base64Audio.length());
    
    JsonObject config = jsonRequest.createNestedObject("config");
    config["encoding"] = "LINEAR16";
    config["sampleRateHertz"] = SAMPLE_RATE;
    config["languageCode"] = "en-US";
    
    JsonArray speechContexts = config.createNestedArray("speechContexts");
    JsonObject context = speechContexts.createNestedObject();
    JsonArray phrases = context.createNestedArray("phrases");
    phrases.add("knight"); phrases.add("bishop"); phrases.add("rook");
    phrases.add("queen"); phrases.add("king"); phrases.add("pawn");
    phrases.add("to");
    phrases.add("a"); phrases.add("b"); phrases.add("c"); phrases.add("d");
    phrases.add("e"); phrases.add("f"); phrases.add("g"); phrases.add("h");
    phrases.add("one"); phrases.add("two"); phrases.add("three"); phrases.add("four");
    phrases.add("five"); phrases.add("six"); phrases.add("seven"); phrases.add("eight");

    JsonObject audio = jsonRequest.createNestedObject("audio");
    audio["content"] = base64Audio;

    String requestBody;
    serializeJson(jsonRequest, requestBody);
    
    Serial.println("Sending request to Google...");
    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode > 0) {
      Serial.printf("HTTP Response code: %d\n", httpResponseCode);
      String responseBody = http.getString();
      
      Serial.println("Parsing response...");
      jsonRequest.clear();
      
      DeserializationError error = deserializeJson(jsonRequest, responseBody);
      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        Serial.println("Full response:");
        Serial.println(responseBody);
        http.end();
        return;
      }

      const char* transcript = jsonRequest["results"][0]["alternatives"][0]["transcript"];
      
      if (transcript) {
        Serial.println("========================================");
        Serial.printf("Transcript: %s\n", transcript);
        Serial.println("========================================");
      } else {
        Serial.println("Error: Could not find transcript in response.");
        Serial.println(responseBody);
      }
      
    } else {
      Serial.printf("HTTP Error: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
  } else {
    Serial.println("Failed to connect to Google API");
  }
}

// ===================================
// WI-FI SETUP FUNCTION
// ===================================
void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ===================================
// I2S FUNCTIONS (TYPO FIXED)
// ===================================
void i2s_install(){
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE, 
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 128, // This is the corrected typo
    .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin(){
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

and this is my issue right now

18:12:46.509 -> Sending request to Google...
18:12:46.674 -> HTTP Error: connection refused
18:12:46.674 -> 
18:12:46.674 -> Type 'r' and press Enter to start recording...
18:17:15.063 -> Recording...
18:17:15.985 -> Recording complete! Read 16000 samples.
18:17:15.985 -> Connecting to Google Speech-to-Text...
18:17:15.985 -> Encoding audio...
18:17:15.985 -> Creating JSON request...
18:17:17.009 -> Sending request to Google...
18:17:17.040 -> HTTP Error: connection refused
18:17:17.040 -> 
18:17:17.040 -> Type 'r' and press Enter to start recording...
