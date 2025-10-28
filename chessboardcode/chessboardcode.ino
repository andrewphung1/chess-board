/*
 * Basic test sketch for an INMP441 I2S Microphone with an ESP32.
 *
 * This code reads data from the INMP441 and calculates the
 * Root Mean Square (RMS) of the audio signal to get a basic
 * idea of the "volume" or "loudness" being detected.
 *
 * --- WIRING (18, 19, 32) ---
 * INMP441 VDD -> ESP32 3.3V
 * INMP441 GND -> ESP32 GND
 * INMP441 SCK -> ESP32 Pin 18  (D18, safe output)
 * INMP441 WS  -> ESP32 Pin 19  (D19, safe output)
 * INMP441 SD  -> ESP32 Pin 32  (D32, safe input)
 * INMP441 L/R -> ESP32 GND (Selects Left Channel)
 */

#include "driver/i2s.h"

// --- Pin Configuration ---
#define I2S_WS_PIN 19  // Word Select (WS)
#define I2S_SCK_PIN 18 // Serial Clock (SCK or BCLK)
#define I2S_SD_PIN 32  // Serial Data (SD or DIN)

// --- I2S Configuration ---
#define I2S_PORT I2S_NUM_0 // Use I2S Port 0
#define I2S_SAMPLE_RATE 16000 // Sample rate in Hz
#define I2S_SAMPLE_BITS I2S_BITS_PER_SAMPLE_32BIT // 24-bit mic, but read as 32-bit
#define I2S_CHANNEL_NUM (1) // 1 for Mono (L/R pin is grounded)
#define I2S_CHANNEL_FMT I2S_CHANNEL_FMT_ONLY_LEFT 

// --- Audio Buffer ---
#define READ_BUFFER_SIZE 1024 // Number of samples to read at a time
int32_t i2s_read_buffer[READ_BUFFER_SIZE]; // Buffer to store 32-bit samples

/**
 * @brief Initialize the I2S peripheral
 */
void i2s_init() {
  Serial.println("DEBUG: i2s_init() started.");

  // --- FINAL ATTEMPT: Forcefully uninstall driver first ---
  // This clears any bad state from a previous crash
  i2s_driver_uninstall(I2S_PORT); 
  Serial.println("DEBUG: Previous I2S driver uninstalled.");

  // 1. Configure I2S settings
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Master mode, Receive data
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S), // Standard I2S
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt priority
    .dma_buf_count = 8, // Number of DMA buffers
    .dma_buf_len = 64,  // Length of each DMA buffer in samples
    .use_apll = false, // Don't use APLL for clock
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  Serial.println("DEBUG: I2S config struct populated.");

  // 2. Configure I2S pins
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,     // Bit Clock (SCK)
    .ws_io_num = I2S_WS_PIN,      // Word Select (WS)
    .data_out_num = I2S_PIN_NO_CHANGE, // Not used for RX
    .data_in_num = I2S_SD_PIN       // Data In (SD)
  };
  Serial.println("DEBUG: I2S pin config struct populated.");

  // 3. Install and start I2S driver
  esp_err_t err;
  
  Serial.println("DEBUG: Attempting i2s_driver_install()...");
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed to install I2S driver: %d\n", err);
    while (true);
  }
  Serial.println("DEBUG: i2s_driver_install() OK.");

  Serial.println("DEBUG: Attempting i2s_set_pin()...");
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S pins: %d\n", err);
    while (true);
  }
  Serial.println("DEBUG: i2s_set_pin() OK.");


  Serial.println("DEBUG: Attempting i2s_set_clk()...");
  err = i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_SAMPLE_BITS, I2S_CHANNEL_MONO);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S clock: %d\n", err);
    while (true);
  }
  Serial.println("DEBUG: i2s_set_clk() OK.");

  Serial.println("I2S Driver initialized with SAFE pins (18, 19, 32).");
}

void setup() {
  // --- FINAL ATTEMPT: Add a delay for power to stabilize ---
  delay(2000); 

  // Start Serial Monitor
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to connect
  
  Serial.println("\n\nINMP441 I2S Test (v_stable)");

  // Initialize the I2S interface
  i2s_init();
}

void loop() {
  // Variable to store how many bytes were actually read
  size_t bytes_read = 0;

  // Read data from I2S
  esp_err_t result = i2s_read(I2S_PORT,
                              &i2s_read_buffer,
                              READ_BUFFER_SIZE * sizeof(int32_t), // Total bytes to read
                              &bytes_read,
                              pdMS_TO_TICKS(100)); // Wait for 100ms

  if (result != ESP_OK) {
    // Serial.printf("I2S read error: %d\n", result); // Can be spammy
    return;
  }

  if (bytes_read > 0) {
    int samples_read = bytes_read / sizeof(int32_t);

    // --- DEBUGGING: Print first 10 samples ---
    Serial.print("First 10 samples: ");
    for(int i = 0; i < 10 && i < samples_read; i++) {
      Serial.printf("%ld ", i2s_read_buffer[i]);
    }
    Serial.println();
    // --- END DEBUGGING ---

    // Calculate RMS
    double sum_sq = 0;
    for (int i = 0; i < samples_read; i++) {
      double sample = (double)i2s_read_buffer[i]; 
      sum_sq += sample * sample;
    }

    double rms = sqrt(sum_sq / samples_read);

    Serial.printf("RMS: %.0f\n\n", rms); // Added newline for readability
  } else {
    Serial.println("No data read from I2S (timeout).");
  }

  // Small delay to make the Serial Monitor readable and feed watchdog
  delay(100); 
}

