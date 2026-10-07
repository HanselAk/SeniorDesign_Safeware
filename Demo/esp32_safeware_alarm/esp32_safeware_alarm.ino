#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <atomic>

struct Sample {
  uint32_t us;
  float a[3];
  float g[3];
  float m[3];
  bool hasMag;
};

// Explicit declaration avoids Arduino prototype errors.
void pushSample(const Sample& sample);

// ================= YOUR SETTINGS =================
const char* WIFI_SSID = "Matteo";
const char* WIFI_PASS = "Lean165?";

const char* SERVER_URL =
    "http://172.20.10.12:5000/api/imu";

const char* API_TOKEN = "";
const char* DEVICE_ID = "esp32-01";

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const uint32_t SAMPLE_US = 10000;
const uint32_t SEND_MS = 100;

// ================= ALARM SETTINGS =================
const int LED_PIN = 26;
const int BUZZER_PIN = 25;

const bool LED_ACTIVE_HIGH = true;
const bool ENABLE_LED = true;

// Enable only after connecting a suitable buzzer transistor driver.
const bool ENABLE_BUZZER = true;

const uint32_t BUZZER_HZ = 2000;

std::atomic<bool> fallAlarm(false);
bool buzzerReady = false;

// ================= SENSOR SETTINGS =================
// MPU9250 wiring:
// VCC -> 3V3
// GND -> GND
// SDA -> GPIO21
// SCL -> GPIO22
// AD0 -> GND
// NCS/CS, if present -> 3V3

const uint8_t MPU = 0x68;
const uint8_t MAG = 0x0C;

const uint32_t BUF_SIZE = 512;
const uint32_t MAX_BATCH = 60;

Sample sampleBuffer[BUF_SIZE];
Sample batch[MAX_BATCH];

uint32_t head = 0;
uint32_t tail = 0;
uint32_t dropped = 0;

portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool imuOk = false;
volatile bool magOk = false;

float asa[3] = {1.0f, 1.0f, 1.0f};

uint32_t bootId = 0;
uint32_t lastSampleUs = 0;

char body[MAX_BATCH * 160 + 256];

// ================= ALARM OUTPUTS =================

void setupAlarm() {
  if (ENABLE_LED) {
    digitalWrite(LED_PIN, LED_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(LED_PIN, OUTPUT);
  }

  if (ENABLE_BUZZER) {
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT);

    buzzerReady = ledcAttach(BUZZER_PIN, BUZZER_HZ, 8);

    if (buzzerReady) {
      ledcWriteTone(BUZZER_PIN, 0);
    } else {
      Serial.println("Buzzer PWM initialization failed.");
    }
  }
}

void updateAlarm() {
  bool active = fallAlarm.load();

  if (ENABLE_LED) {
    digitalWrite(
        LED_PIN,
        (active == LED_ACTIVE_HIGH) ? HIGH : LOW);
  }

  bool beep = active && (millis() % 1000 < 500);
  static bool previousBeep = false;

  if (buzzerReady && beep != previousBeep) {
    ledcWriteTone(BUZZER_PIN, beep ? BUZZER_HZ : 0);
    previousBeep = beep;
  }
}

// ================= I2C HELPERS =================

bool writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readRegs(
    uint8_t address,
    uint8_t reg,
    uint8_t* output,
    uint8_t length) {

  Wire.beginTransmission(address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(address, length) != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    output[i] = Wire.read();
  }

  return true;
}

int16_t signedBE(const uint8_t* data) {
  return (int16_t)(
      ((uint16_t)data[0] << 8) | data[1]);
}

int16_t signedLE(const uint8_t* data) {
  return (int16_t)(
      ((uint16_t)data[1] << 8) | data[0]);
}

// ================= INITIALIZE IMU =================

bool initIMU() {
  if (!writeReg(MPU, 0x6B, 0x80)) {
    Serial.println("IMU not responding at 0x68.");
    return false;
  }

  delay(100);

  if (!writeReg(MPU, 0x6B, 0x01)) {
    return false;
  }

  delay(50);

  uint8_t who = 0;

  if (!readRegs(MPU, 0x75, &who, 1)) {
    return false;
  }

  Serial.printf("IMU WHO_AM_I: 0x%02X\n", who);

  if (who != 0x70 && who != 0x71 && who != 0x73) {
    Serial.println("Unsupported sensor ID.");
    return false;
  }

  if (!writeReg(MPU, 0x1A, 0x03)) return false;
  if (!writeReg(MPU, 0x19, 0x09)) return false;
  if (!writeReg(MPU, 0x1B, 0x08)) return false;
  if (!writeReg(MPU, 0x1C, 0x10)) return false;
  if (!writeReg(MPU, 0x1D, 0x03)) return false;

  return true;
}

// ================= INITIALIZE MAGNETOMETER =================

bool initMag() {
  if (!writeReg(MPU, 0x6A, 0x00)) return false;
  if (!writeReg(MPU, 0x37, 0x02)) return false;

  delay(10);

  if (!writeReg(MAG, 0x0B, 0x01)) return false;

  delay(10);

  uint8_t identity = 0;

  if (!readRegs(MAG, 0x00, &identity, 1) ||
      identity != 0x48) {
    return false;
  }

  if (!writeReg(MAG, 0x0A, 0x00)) return false;
  delay(10);

  if (!writeReg(MAG, 0x0A, 0x0F)) return false;
  delay(10);

  uint8_t raw[3] = {};

  if (!readRegs(MAG, 0x10, raw, 3)) {
    writeReg(MAG, 0x0A, 0x00);
    return false;
  }

  for (int i = 0; i < 3; i++) {
    asa[i] = (raw[i] - 128) / 256.0f + 1.0f;
  }

  if (!writeReg(MAG, 0x0A, 0x00)) return false;
  delay(10);

  if (!writeReg(MAG, 0x0A, 0x16)) return false;
  delay(10);

  return true;
}

// ================= READ IMU =================

bool readIMU(float accel[3], float gyro[3]) {
  uint8_t data[14];

  if (!readRegs(MPU, 0x3B, data, 14)) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    accel[i] = signedBE(&data[i * 2]) / 4096.0f;
    gyro[i] = signedBE(&data[8 + i * 2]) / 65.5f;
  }

  return true;
}

// ================= READ MAGNETOMETER =================

bool readMag(float values[3]) {
  uint8_t status = 0;

  if (!readRegs(MAG, 0x02, &status, 1) ||
      !(status & 0x01)) {
    return false;
  }

  uint8_t data[7];

  if (!readRegs(MAG, 0x03, data, 7)) {
    return false;
  }

  if (data[6] & 0x08) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    values[i] =
        signedLE(&data[i * 2]) * asa[i] * 0.15f;
  }

  return true;
}

// ================= SAMPLE BUFFER =================

void pushSample(const Sample& sample) {
  portENTER_CRITICAL(&bufferMux);

  if (head - tail >= BUF_SIZE) {
    tail++;
    dropped++;
  }

  sampleBuffer[head % BUF_SIZE] = sample;
  head++;

  portEXIT_CRITICAL(&bufferMux);
}

// ================= BUILD JSON =================

size_t buildBody(uint32_t count, uint32_t droppedCount) {
  int written = snprintf(
      body,
      sizeof(body),
      "{\"device\":\"%s\",\"boot\":%lu,"
      "\"imu\":%s,\"mag\":%s,\"dropped\":%lu,\"s\":[",
      DEVICE_ID,
      (unsigned long)bootId,
      imuOk ? "true" : "false",
      magOk ? "true" : "false",
      (unsigned long)droppedCount);

  if (written < 0 || (size_t)written >= sizeof(body)) {
    return 0;
  }

  size_t len = (size_t)written;

  for (uint32_t i = 0; i < count; i++) {
    const Sample& sample = batch[i];

    written = snprintf(
        body + len,
        sizeof(body) - len,
        "%s[%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
        i ? "," : "",
        (unsigned long)sample.us,
        sample.a[0],
        sample.a[1],
        sample.a[2],
        sample.g[0],
        sample.g[1],
        sample.g[2]);

    if (written < 0 ||
        (size_t)written >= sizeof(body) - len) {
      return 0;
    }

    len += (size_t)written;

    if (sample.hasMag) {
      written = snprintf(
          body + len,
          sizeof(body) - len,
          ",%.1f,%.1f,%.1f",
          sample.m[0],
          sample.m[1],
          sample.m[2]);

      if (written < 0 ||
          (size_t)written >= sizeof(body) - len) {
        return 0;
      }

      len += (size_t)written;
    }

    if (len + 1 >= sizeof(body)) return 0;

    body[len++] = ']';
    body[len] = '\0';
  }

  if (len + 2 >= sizeof(body)) return 0;

  body[len++] = ']';
  body[len++] = '}';
  body[len] = '\0';

  return len;
}

// ================= NETWORK TASK =================

void netTask(void*) {
  WiFiClient client;
  HTTPClient http;

  uint32_t lastSend = 0;
  uint32_t lastReconnect = millis();
  uint32_t lastPrint = 0;

  int responseCode = 0;

  for (;;) {
    uint32_t now = millis();

    uint32_t buffered;
    uint32_t droppedCount;

    portENTER_CRITICAL(&bufferMux);
    buffered = head - tail;
    droppedCount = dropped;
    portEXIT_CRITICAL(&bufferMux);

    if (now - lastPrint >= 2000) {
      lastPrint = now;

      String address =
          WiFi.status() == WL_CONNECTED
              ? WiFi.localIP().toString()
              : String("down");

      Serial.printf(
          "wifi %s | imu %s mag %s | last http %d | "
          "buffered %lu dropped %lu\n",
          address.c_str(),
          imuOk ? "ok" : "FAIL",
          magOk ? "ok" : "none",
          responseCode,
          (unsigned long)buffered,
          (unsigned long)droppedCount);
    }

    if (WiFi.status() != WL_CONNECTED) {
      if (now - lastReconnect >= 15000) {
        lastReconnect = now;
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (now - lastSend < SEND_MS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    lastSend = now;

    portENTER_CRITICAL(&bufferMux);

    uint32_t start = tail;
    uint32_t count = head - tail;
    droppedCount = dropped;

    if (count > MAX_BATCH) {
      count = MAX_BATCH;
    }

    for (uint32_t i = 0; i < count; i++) {
      batch[i] = sampleBuffer[(start + i) % BUF_SIZE];
    }

    portEXIT_CRITICAL(&bufferMux);

    if (count == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    size_t len = buildBody(count, droppedCount);

    if (len == 0) {
      Serial.println("JSON buffer too small.");
      continue;
    }

    if (!http.begin(client, SERVER_URL)) {
      responseCode = -1;
      Serial.println("Could not initialize HTTP.");
      continue;
    }

    http.setConnectTimeout(1000);
    http.setTimeout(2000);
    http.addHeader("Content-Type", "application/json");

    if (API_TOKEN[0]) {
      http.addHeader("X-Safeware-Token", API_TOKEN);
    }

    responseCode = http.POST((uint8_t*)body, len);

    if (responseCode == 200) {
      JsonDocument reply;

      DeserializationError error =
          deserializeJson(reply, http.getString());

      if (!error && reply["fall_active"].is<bool>()) {
        bool active = reply["fall_active"].as<bool>();
        bool previous = fallAlarm.exchange(active);

        if (active != previous) {
          Serial.println(
              active ? "FALL ALARM ON" : "FALL ALARM CLEARED");
        }
      } else {
        Serial.println(
            "Missing/invalid fall_active reply. "
            "Check Jetson server update.");
      }
    } else {
      if (responseCode > 0) {
        Serial.printf(
            "HTTP %d: %s\n",
            responseCode,
            http.getString().c_str());
      } else {
        Serial.printf(
            "HTTP error: %s\n",
            HTTPClient::errorToString(responseCode).c_str());
      }
    }

    http.end();

    if (responseCode == 200) {
      portENTER_CRITICAL(&bufferMux);

      if ((int32_t)(start + count - tail) > 0) {
        tail = start + count;
      }

      portEXIT_CRITICAL(&bufferMux);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(500);

  setupAlarm();

  Serial.println();
  Serial.println("Starting Safeware IMU wearable.");

  bootId = esp_random();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  imuOk = initIMU();
  magOk = imuOk && initMag();

  if (!imuOk) {
    Serial.println("IMU initialization failed.");
  } else if (!magOk) {
    Serial.println(
        "IMU ready. Magnetometer unavailable; "
        "streaming acceleration and gyro.");
  } else {
    Serial.println("IMU and magnetometer ready.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
  Serial.printf("Jetson endpoint: %s\n", SERVER_URL);

  BaseType_t taskCreated = xTaskCreatePinnedToCore(
      netTask,
      "network",
      8192,
      nullptr,
      1,
      nullptr,
      0);

  if (taskCreated != pdPASS) {
    Serial.println("ERROR: Could not start network task.");
  }

  Serial.println(
      "Hold the sensor still for a few seconds "
      "while the Jetson calibrates.");

  lastSampleUs = micros();
}

// ================= LOOP =================

void loop() {
  updateAlarm();

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastSampleUs) < SAMPLE_US) {
    delay(1);
    return;
  }

  lastSampleUs = nowUs;

  if (!imuOk) {
    static uint32_t lastInit = 0;

    if (millis() - lastInit >= 1000) {
      lastInit = millis();
      imuOk = initIMU();
      magOk = imuOk && initMag();
    }

    return;
  }

  Sample sample = {};

  if (!readIMU(sample.a, sample.g)) {
    imuOk = false;
    magOk = false;
    return;
  }

  sample.us = nowUs;
  sample.hasMag = magOk && readMag(sample.m);

  pushSample(sample);
}