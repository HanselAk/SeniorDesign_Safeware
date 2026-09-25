// ESP32 -> USB serial -> Jetson fall_to_dashboard.py
// Generic ESP32: MPU6050 VCC=3V3, GND=GND, SDA=GPIO21, SCL=GPIO22, AD0=GND.
// Uses Arduino-ESP32 Wire library, no third-party MPU library required.
#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t MPU_ADDR = 0x68;  // Use 0x69 if AD0 is HIGH.
constexpr float G = 9.80665f;
constexpr float COUNTS_PER_G = 4096.0f;  // ACCEL_CONFIG ±8g.
uint32_t lastSampleMs = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool readAccel(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H, then six consecutive bytes.
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  x = (int16_t)((Wire.read() << 8) | Wire.read());
  y = (int16_t)((Wire.read() << 8) | Wire.read());
  z = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 100000);
  delay(150);
  uint8_t who = 0;
  if (!readReg(0x75, who) || (who & 0x7E) != 0x68) {
    Serial.println("# MPU6050 not found at 0x68. Check 3V3, GND, SDA, SCL, AD0.");
    while (true) delay(1000);
  }
  if (!writeReg(0x6B, 0x00) || !writeReg(0x1C, 0x10)) {
    Serial.println("# Could not configure MPU6050.");
    while (true) delay(1000);
  }
  delay(100);
  Serial.println("# MPU6050 ready; ±8g, ~50 samples/sec, SI units m/s^2");
}

void loop() {
  if (millis() - lastSampleMs < 20) return;
  lastSampleMs = millis();
  int16_t x, y, z;
  if (!readAccel(x, y, z)) {
    Serial.println("# MPU6050 read error");
    return;
  }
  Serial.printf("{\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f}\n",
                x * G / COUNTS_PER_G, y * G / COUNTS_PER_G, z * G / COUNTS_PER_G);
}
