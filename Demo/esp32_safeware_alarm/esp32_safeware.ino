#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <esp_system.h>
#include <math.h>
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
const char* WIFI_SSID = "";
const char* WIFI_PASS = "?";

const char* SERVER_URL =
    "http://172.20.10.12:5000/api/imu";

const char* API_TOKEN = "";
const char* DEVICE_ID = "worker-2";

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
//
// MAX30102 wiring (same I2C bus):
// VIN -> 3V3
// GND -> GND
// SDA -> GPIO21
// SCL -> GPIO22

const uint8_t MPU = 0x68;
const uint8_t MAG = 0x0C;
const uint8_t PPG = 0x57;

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

// ================= HEART RATE SETTINGS (MAX30102, CHEST) =================
// Heart rate is worked out on the ESP32 and sent with every IMU batch:
//   "hr"    bpm, 0 when there is no reading
//   "hr_q"  0..1, how clearly the pulse repeats
//   "hr_st" status text, e.g. "ok", "motion", "no skin contact"
//
// Chest skin gives a pulse ~10x weaker than a fingertip, so:
//  - LED current adjusts itself to keep the raw IR level mid-range.
//  - Rate comes from autocorrelation over 6 s, not single peaks.
//  - The MPU9250 accelerometer flags movement; while moving, the last
//    good reading is held instead of sending a wrong one.

// 0.2 mA per step. Starts low: at 0x7F the IR channel clipped on every
// spot in the chest tests. The control loop raises it as needed.
const uint8_t LED_START = 0x24;
const uint8_t LED_MIN = 0x04;
const uint8_t LED_MAX = 0xFF;

// Keep the raw IR level between these (18-bit max is 262143).
const float IR_LOW = 100000;
const float IR_HIGH = 210000;
const float IR_TARGET = 155000;

const float PPG_FS = 100.0f;            // samples per second from the sensor
const uint32_t CONTACT_MIN_IR = 20000;  // below this: not on skin
const uint32_t SATURATED_IR = 260000;
const int MIN_LAG = 30;                 // samples, 200 bpm
const int MAX_LAG = 150;                // samples, 40 bpm
const int HR_WINDOW = 600;              // 6 s of filtered signal
const int HR_SETTLE = 150;              // samples ignored after a reset
const float GOOD_QUALITY = 0.55f;       // autocorrelation needed for "ok"
const float WEAK_QUALITY = 0.4f;        // below this: no rate at all
const uint32_t HOLD_MS = 15000;         // how long a held reading stays
const float MOTION_G = 0.05f;           // g of movement that spoils the pulse
const uint32_t PPG_POLL_MS = 20;

struct Biquad {
  float b0, b1, b2, a1, a2;
  float z1, z2;

  float run(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

// 3.5 Hz 2nd-order lowpass + 0.7 Hz 4th-order highpass (42-210 bpm),
// Butterworth at 100 Hz. The steep highpass removes breathing.
Biquad bandpass[3] = {
    {0.0104324134f, 0.0208648267f, 0.0104324134f,
     -1.6909963769f, 0.7327260304f, 0, 0},
    {0.9441492060f, -1.8882984120f, 0.9441492060f,
     -1.9200702270f, 0.9219288578f, 0, 0},
    {1.0f, -2.0f, 1.0f,
     -1.9650029838f, 0.9669051095f, 0, 0},
};

bool ppgOk = false;
uint8_t ledPa = LED_START;

uint32_t ppgIndex = 0;
uint32_t saturatedCount = 0;

bool onSkin = false;
bool needDc = true;
float dcLevel = 0;
int hrSettle = HR_SETTLE;

float hrWindow[HR_WINDOW];
int hrNext = 0;
int hrCount = 0;

float gravity = 1.0f;
float motionLevel = 0;
uint32_t lastMotionIndex = 0;
bool sawMotion = false;

float lastGoodBpm = 0;
uint32_t lastGoodMs = 0;
float recentBpm[3];
int recentCount = 0;

// Written by loop(), read by the network task.
volatile float hrBpm = 0;
volatile float hrQuality = 0;
const char* volatile hrStatus = "starting";

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

uint32_t sample18(const uint8_t* data) {
  return (((uint32_t)data[0] << 16) |
          ((uint32_t)data[1] << 8) |
          data[2]) & 0x3FFFF;
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

// ================= HEART RATE =================

// Called on contact changes, LED changes and FIFO overflow: anything
// that puts a step in the signal and would fake a periodic pattern.
void resetHeartRate() {
  for (Biquad& stage : bandpass) {
    stage.z1 = 0;
    stage.z2 = 0;
  }

  needDc = true;
  hrSettle = HR_SETTLE;
  hrNext = 0;
  hrCount = 0;
}

void processPpgSample(uint32_t ir) {
  if (ir > SATURATED_IR) {
    saturatedCount++;
  }

  if (ir < CONTACT_MIN_IR) {
    if (onSkin) {
      resetHeartRate();
    }

    onSkin = false;
    dcLevel = ir;  // still tracked so the LED control can ramp up
    return;
  }

  onSkin = true;

  if (needDc) {
    dcLevel = ir;  // start at the skin level so the filter sees no jump
    needDc = false;
  }

  // Remove the steady light level. Each beat is a dip in IR: invert it.
  dcLevel += 0.01f * (ir - dcLevel);
  float x = -(ir - dcLevel);

  for (Biquad& stage : bandpass) {
    x = stage.run(x);
  }

  if (hrSettle > 0) {
    hrSettle--;
    return;
  }

  hrWindow[hrNext] = x;
  hrNext = (hrNext + 1) % HR_WINDOW;

  if (hrCount < HR_WINDOW) {
    hrCount++;
  }
}

// Finds the beat period as the lag where the pulse wave best matches a
// shifted copy of itself. Returns bpm, or 0; quality is 0..1.
float autocorrelationBpm(float& quality) {
  quality = 0;

  static float x[HR_WINDOW];

  for (int i = 0; i < HR_WINDOW; i++) {
    x[i] = hrWindow[(hrNext + i) % HR_WINDOW];
  }

  float energy = 0;

  for (int i = 0; i < HR_WINDOW; i++) {
    energy += x[i] * x[i];
  }

  if (energy <= 0) {
    return 0;
  }

  static float r[MAX_LAG + 2];

  for (int lag = MIN_LAG - 1; lag <= MAX_LAG + 1; lag++) {
    float sum = 0;

    for (int i = 0; i + lag < HR_WINDOW; i++) {
      sum += x[i] * x[i + lag];
    }

    r[lag] = (sum / (HR_WINDOW - lag)) / (energy / HR_WINDOW);
  }

  // Only local peaks count: just past lag 0 the curve is still falling.
  float best = 0;

  for (int lag = MIN_LAG; lag <= MAX_LAG; lag++) {
    if (r[lag] >= r[lag - 1] && r[lag] >= r[lag + 1]) {
      best = fmaxf(best, r[lag]);
    }
  }

  if (best < WEAK_QUALITY) {
    quality = best;
    return 0;
  }

  // Take the SHORTEST lag that is a local peak close to the best one.
  // Double the true period also matches well and would halve the rate.
  for (int lag = MIN_LAG; lag <= MAX_LAG; lag++) {
    if (r[lag] >= 0.6f * best &&
        r[lag] >= r[lag - 1] && r[lag] >= r[lag + 1]) {
      float a = r[lag - 1];
      float b = r[lag];
      float c = r[lag + 1];
      float denom = a - 2 * b + c;
      float shift = denom != 0 ? 0.5f * (a - c) / denom : 0;

      quality = b;
      return 60.0f * PPG_FS / (lag + shift);
    }
  }

  return 0;
}

// Runs once a second and publishes hrBpm / hrQuality / hrStatus.
void updateHeartRate() {
  const char* status;
  float bpm = 0;
  float quality = 0;
  bool moving = imuOk && sawMotion &&
                ppgIndex - lastMotionIndex < (uint32_t)HR_WINDOW;

  if (!onSkin) {
    status = "no skin contact";
  } else if (saturatedCount > 1) {
    status = "adjusting LED";
  } else if (hrCount < HR_WINDOW) {
    status = "collecting";
  } else if (moving) {
    status = "motion";
  } else {
    float raw = autocorrelationBpm(quality);

    if (raw <= 0) {
      status = "weak signal";
    } else {
      // Median of the last 3 estimates hides single bad windows.
      recentBpm[recentCount % 3] = raw;
      recentCount++;

      int n = recentCount < 3 ? recentCount : 3;
      float a = recentBpm[0];
      float b = n > 1 ? recentBpm[1] : a;
      float c = n > 2 ? recentBpm[2] : b;

      bpm = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
      status = quality >= GOOD_QUALITY ? "ok" : "low quality";
      lastGoodBpm = bpm;
      lastGoodMs = millis();
    }
  }

  if (bpm == 0 && onSkin && lastGoodBpm > 0 &&
      millis() - lastGoodMs < HOLD_MS) {
    // Keep sending the last good value through short gaps.
    bpm = lastGoodBpm;
    status = moving ? "motion, holding" : "holding";
  }

  if (!onSkin) {
    lastGoodBpm = 0;
    recentCount = 0;
  }

  saturatedCount = 0;

  hrBpm = bpm;
  hrQuality = quality;
  hrStatus = status;
}

// Runs twice a second. Only changes the LED when the level leaves the
// good band, because every change restarts the 6 s window.
void adjustLed() {
  float level = dcLevel;
  int next = ledPa;

  if (saturatedCount > 1) {
    next = ledPa * 0.7f;
  } else if (level < IR_LOW || level > IR_HIGH) {
    next = ledPa * (IR_TARGET / fmaxf(level, 1000.0f));
  } else {
    return;
  }

  next = constrain(next, LED_MIN, LED_MAX);

  if (next == ledPa) {
    return;  // already at the limit
  }

  ledPa = next;
  writeReg(PPG, 0x0C, ledPa);
  writeReg(PPG, 0x0D, ledPa);
  resetHeartRate();
}

// Uses the accel the IMU code already reads (in g), so the MPU9250
// keeps its +-8 g fall-detection setup. Magnitude only, so the board
// can be mounted at any angle (vertical, upside down).
void updateMotion(const float accel[3]) {
  float mag = sqrtf(accel[0] * accel[0] +
                    accel[1] * accel[1] +
                    accel[2] * accel[2]);

  // Slow average is gravity; what is left over is movement.
  gravity += 0.02f * (mag - gravity);
  motionLevel = fmaxf(fabsf(mag - gravity), motionLevel * 0.9f);

  if (motionLevel > MOTION_G) {
    lastMotionIndex = ppgIndex;
    sawMotion = true;
  }
}

// ================= INITIALIZE MAX30102 =================

bool initPPG() {
  if (!writeReg(PPG, 0x09, 0x40)) {
    return false;
  }

  delay(10);

  uint8_t id = 0;

  if (!readRegs(PPG, 0xFF, &id, 1) || id != 0x15) {
    Serial.printf("Unexpected MAX30102 part ID: 0x%02X\n", id);
    return false;
  }

  // Average 4 samples on-chip, FIFO rollover on.
  if (!writeReg(PPG, 0x08, 0x50)) return false;

  // 4096 nA ADC range, 400 Hz, 411 us pulse (18-bit).
  // 400 Hz / 4 averaged = 100 Hz output.
  if (!writeReg(PPG, 0x0A, 0x2F)) return false;

  if (!writeReg(PPG, 0x0C, ledPa)) return false;
  if (!writeReg(PPG, 0x0D, ledPa)) return false;

  // Clear FIFO write pointer, overflow counter, read pointer.
  if (!writeReg(PPG, 0x04, 0x00)) return false;
  if (!writeReg(PPG, 0x05, 0x00)) return false;
  if (!writeReg(PPG, 0x06, 0x00)) return false;

  // SpO2 mode: red + IR.
  if (!writeReg(PPG, 0x09, 0x03)) return false;

  resetHeartRate();
  return true;
}

// ================= READ MAX30102 FIFO =================

bool drainPPG() {
  // FIFO_WR_PTR, OVF_COUNTER, FIFO_RD_PTR are registers 0x04-0x06.
  uint8_t ptr[3];

  if (!readRegs(PPG, 0x04, ptr, 3)) {
    return false;
  }

  uint8_t count = (ptr[0] - ptr[2]) & 0x1F;

  if (ptr[1]) {
    // Lost samples break beat timing.
    resetHeartRate();

    if (count == 0) {
      count = 32;
    }
  }

  while (count) {
    // 20 samples * 6 bytes = 120, fits the 128-byte Wire buffer.
    uint8_t n = count > 20 ? 20 : count;
    uint8_t data[120];

    if (!readRegs(PPG, 0x07, data, n * 6)) {
      return false;
    }

    for (uint8_t i = 0; i < n; i++) {
      // Each sample is red (bytes 0-2) then IR (bytes 3-5).
      processPpgSample(sample18(&data[i * 6 + 3]));
      ppgIndex++;

      if (ppgIndex % 50 == 0) {
        adjustLed();
      }

      if (ppgIndex % 100 == 0) {
        updateHeartRate();
      }
    }

    count -= n;
  }

  return true;
}

void servicePPG() {
  static uint32_t lastPoll = 0;
  static uint32_t lastInit = 0;

  if (!ppgOk) {
    if (millis() - lastInit >= 2000) {
      lastInit = millis();
      ppgOk = initPPG();
      hrStatus = ppgOk ? "starting" : "sensor missing";
    }

    return;
  }

  if (millis() - lastPoll < PPG_POLL_MS) {
    return;
  }

  lastPoll = millis();

  if (!drainPPG()) {
    Serial.println("MAX30102 read failed, reinitializing.");
    ppgOk = false;
    hrBpm = 0;
    hrStatus = "sensor missing";
  }
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
      "\"imu\":%s,\"mag\":%s,\"dropped\":%lu,"
      "\"ppg\":%s,\"hr\":%.1f,\"hr_q\":%.2f,\"hr_st\":\"%s\",\"s\":[",
      DEVICE_ID,
      (unsigned long)bootId,
      imuOk ? "true" : "false",
      magOk ? "true" : "false",
      (unsigned long)droppedCount,
      ppgOk ? "true" : "false",
      (float)hrBpm,
      (float)hrQuality,
      (const char*)hrStatus);

  if (written < 0 || (size_t)written >= sizeof(body)) {
    return 0;
  }

  size_t len = (size_t)written;

  for (uint32_t i = 0; i < count; i++) {
    const Sample& sample = batch[i];

    // Without a magnetometer reading the shorter format simply ignores the m[] arguments.
    written = snprintf(
        body + len,
        sizeof(body) - len,
        sample.hasMag
            ? "%s[%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f]"
            : "%s[%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f]",
        i ? "," : "",
        (unsigned long)sample.us,
        sample.a[0], sample.a[1], sample.a[2],
        sample.g[0], sample.g[1], sample.g[2],
        sample.m[0], sample.m[1], sample.m[2]);

    if (written < 0 ||
        (size_t)written >= sizeof(body) - len) {
      return 0;
    }

    len += (size_t)written;
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
          "buffered %lu dropped %lu | "
          "hr %.1f q %.2f led 0x%02X %s\n",
          address.c_str(),
          imuOk ? "ok" : "FAIL",
          magOk ? "ok" : "none",
          responseCode,
          (unsigned long)buffered,
          (unsigned long)droppedCount,
          (float)hrBpm,
          (float)hrQuality,
          ledPa,
          (const char*)hrStatus);
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
      // Flask's jsonify reply is compact, e.g. {"fall_active":true,"status":"ok"}.
      String reply = http.getString();
      bool active = reply.indexOf("\"fall_active\":true") >= 0;

      if (active || reply.indexOf("\"fall_active\":false") >= 0) {
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
  Serial.println("Starting Safeware IMU + heart rate wearable.");

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

  ppgOk = initPPG();
  hrStatus = ppgOk ? "starting" : "sensor missing";

  Serial.println(
      ppgOk ? "MAX30102 ready. First heart rate in ~8 s."
            : "MAX30102 not found; sending IMU only.");

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

  // Heart rate shares the I2C bus, so it runs here on the same core
  // as the IMU reads, never from the network task.
  servicePPG();

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

  updateMotion(sample.a);
  pushSample(sample);
}
