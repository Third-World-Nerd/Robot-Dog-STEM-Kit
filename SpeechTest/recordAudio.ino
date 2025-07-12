#include <driver/i2s.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 32
#define I2S_SCK 33

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// Define input buffer length
#define BUFFER_LEN 256
int32_t rawBuffer[BUFFER_LEN];  // 32-bit buffer for raw samples
int16_t sBuffer[BUFFER_LEN];    // 16-bit conversion buffer

// WiFi credentials
const char* ssid = "SOYAM_dhfibernet";
const char* password = "6153@6153";

// Server details
const char* serverHost = "192.168.18.202";
const int serverPort = 5000;
const char* serverPath = "/upload";

// Audio settings
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE 16
#define CHANNELS 1
#define RECORD_TIME 11      // 11 seconds (includes buffer)

// Calculate total bytes to send
#define TOTAL_BYTES (SAMPLE_RATE * RECORD_TIME * CHANNELS * BITS_PER_SAMPLE / 8)

void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true  // Changed to true for better clock stability
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection
  
  Serial.println("\n\nStarting Audio Streaming Recorder...");
  
  // Print memory info
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  
  // Initialize I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  
  // Connect to WiFi
  connectWiFi();
}

void loop() {
  Serial.println("\nStarting streaming to server...");
  
  // Create WiFi client
  WiFiClient client;
  
  // Connect to server
  Serial.print("Connecting to server...");
  if (!client.connect(serverHost, serverPort)) {
    Serial.println("Connection failed!");
    delay(2000);
    return;
  }
  Serial.println("Connected!");
  
  // Send HTTP headers
  client.print("POST ");
  client.print(serverPath);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(serverHost);
  client.println("Content-Type: audio/raw");
  client.print("Content-Length: ");
  client.println(TOTAL_BYTES);
  client.println("Connection: close");
  client.println();
  
  // Stream audio from I2S to server
  unsigned long startTime = millis();
  size_t bytesSent = 0;
  
  while (millis() - startTime < RECORD_TIME * 1000 && 
         bytesSent < TOTAL_BYTES && 
         client.connected()) {
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(
      I2S_PORT, 
      rawBuffer, 
      BUFFER_LEN * sizeof(int32_t),
      &bytesRead, 
      portMAX_DELAY
    );
    
    if (result == ESP_OK && bytesRead > 0) {
      // Convert 32-bit samples to 16-bit
      int samplesRead = bytesRead / sizeof(int32_t);
      for (int i = 0; i < samplesRead; i++) {
        // INMP441: 24-bit data in MSB of 32-bit word
        // Shift right by 16 to get the upper 16 bits
        sBuffer[i] = (int16_t)(rawBuffer[i] >> 16);

      }
      
      // Send converted 16-bit audio
      size_t bytesToSend = samplesRead * sizeof(int16_t);
      size_t bytesWritten = client.write((const uint8_t*)sBuffer, bytesToSend);
      
      if (bytesWritten > 0) {
        bytesSent += bytesWritten;
        
        // Debug: print first sample value periodically
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000) {
          lastPrint = millis();
          Serial.printf("Sample value: %d\n", sBuffer[0]);
        }
      } else {
        Serial.println("Failed to write data to server!");
        break;
      }
    }
  }
  
  // Flush to ensure all data is sent
  client.flush();
  Serial.printf("Streamed %d bytes to server\n", bytesSent);
  
  // Wait for server response
  unsigned long responseStart = millis();
  while (client.connected() && millis() - responseStart < 3000) {
    if (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    delay(10);
  }
  
  client.stop();
  Serial.println("Connection closed");
  
  // Add delay before next recording
  delay(1000);
}