#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <vector>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WS2812FX.h>
#include <A1301.h>


// -------------------- Pin definitions --------------------
#define LED_PIN 2
#define LED_COUNT 16
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22
#define HALL_PIN 34

// -------------------- Servers --------------------
WebSocketsServer webSocket(81);   // WebSocket on port 81
DNSServer dnsServer;
AsyncWebServer server(80);

// -------------------- LED Strip --------------------
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // Example LED strip

// -------------------- IMU --------------------
LSM6DS3 imu(I2C_MODE, 0x6B);

// -------------------- Hall effect sensor --------------------
A1324 hallSensor(HALL_PIN);

// -------------------- WiFi --------------------
const byte DNS_PORT = 53;
const char* ssid = "BigAssCantilever";
const char* password = "";

// -------------------- Data --------------------
struct AccelData { float x, y, z; };
AccelData linearAccel = {0,0,0}, offsetAccel = {0,0,0};

std::vector<float> raw_xs, raw_ys, raw_zs;
std::vector<float> accel_x_history, accel_y_history, accel_z_history;

int nphonon_x = 0, nphonon_y = 0, nphonon_z = 0, nphonon_tot = 0;
unsigned long lastUpdate = 0;

// Tunable via admin panel
float phonon_scale = 300.0;
float nphonon_period_ms = 750.0;

// -------------------- Math helpers --------------------
float mean(const std::vector<float>& data) {
  float sum = 0;
  for (float v : data) sum += v;
  return data.empty() ? 0 : sum / data.size();
}

float stddev(const std::vector<float>& data, float m) {
  float sum = 0;
  for (float v : data) sum += (v - m) * (v - m);
  return data.empty() ? 0 : sqrt(sum / data.size());
}

// -------------------- Phonon calc --------------------
int calc_nphonon(const std::vector<float>& accels) {
  int samples = nphonon_period_ms / 20.0; // 50Hz → 20ms
  if (samples <= 0 || samples >= accels.size()) return 0;

  float sum_sq = 0;
  for (size_t i = accels.size() - samples; i < accels.size(); i++) {
    float d = accels[i] - accels[i - samples];
    sum_sq += d * d;
  }

  float mean_sq = sum_sq / (accels.size() - samples);
  return static_cast<int>(mean_sq * phonon_scale);
}

void calc_nphonons() {
  nphonon_x = calc_nphonon(accel_x_history);
  nphonon_y = calc_nphonon(accel_y_history);
  nphonon_z = calc_nphonon(accel_z_history);
  nphonon_tot = nphonon_x + nphonon_y + nphonon_z;
}

// -------------------- Sensor update --------------------
void updateSensor() {
  unsigned long now = micros();
  if (now - lastUpdate < 20000) return; // 50Hz
  lastUpdate = now;

  float rx = imu.readFloatAccelX();
  float ry = imu.readFloatAccelY();
  float rz = imu.readFloatAccelZ();

  raw_xs.push_back(rx);
  raw_ys.push_back(ry);
  raw_zs.push_back(rz);

  if (raw_xs.size() > 20) {
    raw_xs.erase(raw_xs.begin());
    raw_ys.erase(raw_ys.begin());
    raw_zs.erase(raw_zs.begin());
  }

  float mx = mean(raw_xs);
  float my = mean(raw_ys);
  float mz = mean(raw_zs);

  float sx = stddev(raw_xs, mx);
  float sy = stddev(raw_ys, my);
  float sz = stddev(raw_zs, mz);

  if (sx < 0.02 && sy < 0.02 && sz < 0.02) {
    offsetAccel = {mx, my, mz};
  }

  linearAccel = {rx - offsetAccel.x, ry - offsetAccel.y, rz - offsetAccel.z};

  accel_x_history.push_back(linearAccel.x);
  accel_y_history.push_back(linearAccel.y);
  accel_z_history.push_back(linearAccel.z);

  if (accel_x_history.size() > 1000) {
    accel_x_history.erase(accel_x_history.begin());
    accel_y_history.erase(accel_y_history.begin());
    accel_z_history.erase(accel_z_history.begin());
  }
}

// -------------------- WebSocket --------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t*, size_t) {
  if (type == WStype_CONNECTED) {
    Serial.printf("Client %u connected\n", num);
  }
}

// Admin → player message
void sendMessageToPlayers(const String &msg) {
  JsonDocument doc;
  doc["type"] = "adminMessage";
  doc["text"] = msg;

  String payload;
  serializeJson(doc, payload);
  webSocket.broadcastTXT(payload);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  LittleFS.begin();

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  ws2812fx.init();
  ws2812fx.setBrightness(50);
  ws2812fx.start();

  hallSensor.begin(5.0, 1023);
  hallSensor.autoMidPoint();

  imu.settings.accelBandWidth = 200;
  if (imu.begin() != 0) {
    Serial.println("IMU not found");
    while (1);
  }
  Serial.println("IMU connected");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  Serial.println("DNS server started at " + WiFi.softAPIP().toString());

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/index.html", "text/html");
  });

  // Admin API
  server.on("/admin", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("cmd")) {
      req->send(400, "text/plain", "Missing cmd");
      return;
    }

    String cmd = req->getParam("cmd")->value();

    if (cmd == "setParams") {
      if (req->hasParam("period"))
        nphonon_period_ms = req->getParam("period")->value().toFloat();
      if (req->hasParam("phonon"))
        phonon_scale = req->getParam("phonon")->value().toFloat();

      req->send(200, "text/plain", "Parameters updated");
    }

    else if (cmd == "message") {
      if (!req->hasParam("text")) {
        req->send(400, "text/plain", "Missing text");
        return;
      }
      sendMessageToPlayers(req->getParam("text")->value());
      req->send(200, "text/plain", "Message sent");
    }

    else {
      req->send(400, "text/plain", "Unknown command");
    }
  });

  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/index.html", "text/html");
  });

  server.begin();
  Serial.println("Async HTTP server started");
}

// -------------------- Loop --------------------
void loop() {
  ws2812fx.service();
  dnsServer.processNextRequest();
  webSocket.loop();
  updateSensor();

  static unsigned long lastPush = 0;
  if (micros() - lastPush > 20000) {
    lastPush = micros();
    calc_nphonons();

    String json = "{\"x\":" + String(linearAccel.x,2) +
                  ",\"y\":" + String(linearAccel.y,2) +
                  ",\"z\":" + String(linearAccel.z,2) +
                  ",\"nphonon_x\":" + String(nphonon_x) +
                  ",\"nphonon_y\":" + String(nphonon_y) +
                  ",\"nphonon_z\":" + String(nphonon_z) +
                  ",\"nphonon_tot\":" + String(nphonon_tot) + "}";

    webSocket.broadcastTXT(json);
  }
}
