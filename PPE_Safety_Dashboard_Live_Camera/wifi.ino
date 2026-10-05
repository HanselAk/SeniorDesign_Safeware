// ============================================================
//  wifi.ino  (second tab, same sketch folder as esp32_motion.ino)
//
//  Sends telemetry and confirmed falls to the Jetson dashboard over WiFi.
//  Sensing stays on core 1 (loop), networking runs on core 0, so a slow
//  HTTP request can never make the 100 Hz fall detector miss samples.
//
//  Edits to esp32_motion.ino (your MPU9250 file, the main tab):
//    1. Next to the other statics:   static float lastA[3] = {0, 0, 1};
//    2. In imuUpdate(), right after "imuOk = true;":
//           lastA[0] = a[0]; lastA[1] = a[1]; lastA[2] = a[2];
//    3. In the getters section:      float lastAccelG(int i) { return lastA[i]; }
//    4. Near the top (PlatformIO only):  void wifiBegin();
//    5. In setup(), after imuBegin() succeeds:  wifiBegin();
//  Keep the rest of setup() and loop() as they are.
// ============================================================
#include <WiFi.h>
#include <HTTPClient.h>

// ---------------- Config: change these ----------------
static const char* WIFI_SSID  = "YOUR_SSID";
static const char* WIFI_PASS  = "YOUR_PASSWORD";
static const char* SERVER_URL = "http://192.168.1.50:5000/api/fall/update";  // Jetson IP: run `hostname -I` on it
static const char* API_TOKEN  = "change-me";                                  // must match SAFEWARE_TOKEN on the Jetson
static const char* DEVICE_ID  = "esp32-01";
static const unsigned long SEND_MS = 200;                                     // 5 updates per second
static const float G_TO_MS2   = 9.80665f;

float lastAccelG(int i);   // defined in esp32_motion.ino

static void netTask(void*) {
  WiFiClient client;
  HTTPClient http;
  http.setReuse(true);
  unsigned long lastSend = 0, lastReconnect = 0, lastPrint = 0;
  int code = 0;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastReconnect > 3000) {
        lastReconnect = millis();
        WiFi.reconnect();
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    bool fall = fallActive();
    unsigned long now = millis();
    if (fall || now - lastSend >= SEND_MS) {
      lastSend = now;

      char body[360];
      snprintf(body, sizeof(body),
        "{\"device\":\"%s\",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"event\":\"%s\",\"severity\":%d,\"peak_g\":%.2f,"
        "\"x\":%.2f,\"y\":%.2f,\"heading\":%.1f,\"steps\":%lu,\"distance\":%.2f,\"walking\":%s}",
        DEVICE_ID,
        lastAccelG(0) * G_TO_MS2, lastAccelG(1) * G_TO_MS2, lastAccelG(2) * G_TO_MS2,
        fall ? "FALL_DETECTED" : "", fall ? fallSeverity() : 0, fallPeakG(),
        pdrX(), pdrY(), pdrHeading(), pdrSteps(), pdrDistance(),
        pdrWalking() ? "true" : "false");

      http.begin(client, SERVER_URL);
      http.setConnectTimeout(800);
      http.setTimeout(1000);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("X-Safeware-Token", API_TOKEN);
      code = http.POST((uint8_t*)body, strlen(body));
      http.end();

      // Keep resending a confirmed fall until the server accepts it.
      if (fall && code == 200) fallAck();
    }

    if (now - lastPrint > 2000) {
      lastPrint = now;
      Serial.printf("wifi ok, last http code %d\n", code);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Call once from setup(), after imuBegin() succeeds.
void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower latency
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 1, nullptr, 0);
}
