#include <driver/i2s.h>

// 1. YOUR PINS
// ===================================
#define I2S_WS 13
#define I2S_SD 27
#define I2S_SCK 25
// ===================================

#define I2S_PORT I2S_NUM_0
#define bufferLen 64 // Number of samples

// Buffer for 16-bit samples (64 samples * 2 bytes/sample = 128 bytes)
int16_t sBuffer[bufferLen];

void setup() {
  Serial.begin(115220); // Use 115200 baud
  Serial.println("Setup I2S (Corrected Code)...");

  delay(1000);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
}

void loop() {
  size_t bytesIn = 0;
  
  // BUG FIX #1: Read the full size of the buffer (sizeof(sBuffer) is 128 bytes)
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, sizeof(sBuffer), &bytesIn, portMAX_DELAY);
  
  if (result == ESP_OK)
  {
    // BUG FIX #2: Each sample is 2 bytes (int16_t)
    int samples_read = bytesIn / 2;

    if (samples_read > 0) {
      float mean = 0;
      for (int i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }
      mean /= samples_read;
      // Print the average value
      Serial.println(mean);
    }
  }
}

void i2s_install(){
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    
    // BUG FIX #3: dma_buf_len is in BYTES. Must be >= bufferLen * 2
    .dma_buf_len = bufferLen * 2, // 64 samples * 2 bytes = 128
    
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin(){
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,  // 25
    .ws_io_num = I2S_WS,    // 13
    .data_out_num = -1,
    .data_in_num = I2S_SD     // 27
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}