#include <driver/i2s.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Update.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2S_WS 25
#define I2S_SD 26
#define I2S_SCK 33
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (44100)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (10) // Seconds per chunk
#define I2S_CHANNEL_NUM   (1)
#define AUDIO_BUFFER_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)
#define HEADER_SIZE       44

const char* ssid = "Vishnu";
const char* password = "vishnu@vishnu";
const char* presigned_url = "https://sound1624.s3.amazonaws.com/audio_file_1743078149321.wav?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAW3MD6NM2UL3D3767%2F20250327%2Fap-south-1%2Fs3%2Faws4_request&X-Amz-Date=20250327T122229Z&X-Amz-Expires=604800&X-Amz-SignedHeaders=host&X-Amz-Signature=99bce55d2a34fa455351634425946f36d3244139854feaf4859348b4046580cd";

#define GITHUB_USER "vishnukechem"
#define GITHUB_REPO "sound_2"
#define CURRENT_VERSION "v1.2"

unsigned long lastCheckTime = 0;
const unsigned long updateInterval = 1 * 60 * 1000;

uint8_t *audioBuffer;
TaskHandle_t recordTaskHandle, uploadTaskHandle;

void i2sInit();
void recordAudio(void *param);
void uploadToAWS(void *param);
void checkForUpdates();
void updateFirmware(String version);
String getLatestVersion();
void wavHeader(byte* header, int wavSize);

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\n✅ Connected to WiFi!");

    i2sInit();
    
    audioBuffer = (uint8_t *)ps_malloc(AUDIO_BUFFER_SIZE + HEADER_SIZE);
    if (!audioBuffer) {
        Serial.println("Failed to allocate PSRAM for audio buffer!");
        return;
    }

    xTaskCreatePinnedToCore(recordAudio, "RecordTask", 8192, NULL, 1, &recordTaskHandle, 0);
    xTaskCreatePinnedToCore(uploadToAWS, "UploadTask", 8192, NULL, 1, &uploadTaskHandle, 1);
}

void loop() {
    if (millis() - lastCheckTime >= updateInterval) {
        lastCheckTime = millis();
        checkForUpdates();
    }
    vTaskDelay(pdMS_TO_TICKS(10000)); // Let FreeRTOS handle timing
}

void recordAudio(void *param) {
    while (true) {
        Serial.println("🎙 Recording...");
        wavHeader(audioBuffer, AUDIO_BUFFER_SIZE);
        size_t bytesRead;
        i2s_read(I2S_PORT, audioBuffer + HEADER_SIZE, AUDIO_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
        Serial.println("✅ Recording complete.");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void uploadToAWS(void *param) {
    while (true) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("📤 Uploading to AWS...");
            HTTPClient http;
            http.begin(presigned_url);
            http.addHeader("Content-Type", "audio/wav");
            int httpResponseCode = http.PUT(audioBuffer, AUDIO_BUFFER_SIZE + HEADER_SIZE);
            Serial.printf("🔄 HTTP Response: %d\n", httpResponseCode);
            http.end();
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void checkForUpdates() {
    String latestVersion = getLatestVersion();
    if (!latestVersion.isEmpty() && latestVersion != CURRENT_VERSION) { 
        Serial.println("🚀 New version found! Updating...");
        updateFirmware(latestVersion);
    } else {
        Serial.println("✔ Already up to date by sound1624  (Version: " CURRENT_VERSION ")");
    }
}

String getLatestVersion() {
    HTTPClient http;
    String latestVersion = "";
    String url = "https://api.github.com/repos/" GITHUB_USER "/" GITHUB_REPO "/releases/latest";

    Serial.println("🔍 Checking for updates...");
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, http.getString());
        latestVersion = doc["tag_name"].as<String>();
        Serial.println("🔄 Latest Version: " + latestVersion);
    } else {
        Serial.println("❌ Failed to check updates. HTTP: " + String(httpCode));
    }

    http.end();
    return latestVersion;
}

void updateFirmware(String version) {
    HTTPClient http;
    String firmwareURL = "https://github.com/" GITHUB_USER "/" GITHUB_REPO "/releases/download/" + version + "/firmware.bin";

    Serial.println("⬇ Downloading: " + firmwareURL);
    http.begin(firmwareURL);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        WiFiClient& stream = http.getStream();
        size_t totalSize = http.getSize();

        if (Update.begin(totalSize)) {
            Serial.println("⚡ Updating firmware...");
            size_t written = Update.writeStream(stream);
            if (written == totalSize && Update.end()) {
                Serial.println("✅ Update Successful! Restarting...");
                ESP.restart();
            } else {
                Serial.println("❌ Update Failed!");
            }
        }
    } else {
        Serial.println("❌ Firmware fetch failed. HTTP: " + String(httpCode));
    }

    http.end();
}

void i2sInit() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false
    };
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };
    i2s_set_pin(I2S_PORT, &pin_config);
}

void wavHeader(byte* header, int wavSize) {
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    unsigned int fileSize = wavSize + HEADER_SIZE - 8;
    header[4] = fileSize & 0xFF;
    header[5] = (fileSize >> 8) & 0xFF;
    header[6] = (fileSize >> 16) & 0xFF;
    header[7] = (fileSize >> 24) & 0xFF;
    header[8] = 'W';
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f';
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 0x10;
    header[17] = 0x00;
    header[18] = 0x00;
    header[19] = 0x00;
    header[20] = 0x01;
    header[21] = 0x00;
    header[22] = 0x01;
    header[23] = 0x00;
    header[24] = 0x44;
    header[25] = 0xAC;
    header[26] = 0x00;
    header[27] = 0x00;
    header[28] = 0x10;
    header[29] = 0xB1;
    header[30] = 0x02;
    header[31] = 0x00;
    header[32] = 0x02;
    header[33] = 0x00;
    header[34] = 0x10;
    header[35] = 0x00;
    header[36] = 'd';
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    header[40] = wavSize & 0xFF;
    header[41] = (wavSize >> 8) & 0xFF;
    header[42] = (wavSize >> 16) & 0xFF;
    header[43] = (wavSize >> 24) & 0xFF;
}
