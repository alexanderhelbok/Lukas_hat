#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <vector>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WS2812FX.h>
#include <A1301.h>

// --- Pin definitions ---
#define LED_EYES_PIN 23
#define LED_LEFT_PIN 19
#define LED_RIGHT_PIN 33

// --- LED definitions ---
#define LED_EYES_COUNT 2
#define LED_LEFT_COUNT 67
#define LED_RIGHT_COUNT 67
#define BRIGHTNESS_EYES 50
#define BRIGHTNESS_LEFT 50
#define BRIGHTNESS_RIGHT 50
#define SPEED_LEFT 20
#define SPEED_RIGHT 20

#define MyRed 0xD62728
#define MyGreen 0x2CA02C

// --- Servers ---
WebSocketsServer webSocket(81);   // WebSocket on port 81
DNSServer dnsServer;
AsyncWebServer server(80);

// --- LED strips ---
WS2812FX ws2812fx_eyes = WS2812FX(LED_EYES_COUNT, LED_EYES_PIN, NEO_GRB + NEO_KHZ800); // Example LED strip
WS2812FX ws2812fx_left = WS2812FX(LED_LEFT_COUNT, LED_LEFT_PIN, NEO_GRB + NEO_KHZ800); // Example LED strip
WS2812FX ws2812fx_right = WS2812FX(LED_RIGHT_COUNT, LED_RIGHT_PIN, NEO_GRB + NEO_KHZ800); // Example LED strip

// --- IMU (compass) ---
// LSM6DS3 imu(I2C_MODE, 0x6B);
DFRobot_QMC5883 compass(&Wire, HMC5883L_ADDRESS);

// --- WiFi ---
const byte DNS_PORT = 53;
const char* ssid = "BigAssCantilever";
const char* password = "";

// --- Data ---
struct AccelData { float x, y, z; };
AccelData linearAccel = {0,0,0}, offsetAccel = {0,0,0};

std::vector<float> raw_xs, raw_ys, raw_zs;
std::vector<float> accel_x_history, accel_y_history, accel_z_history;

int nphonon_x = 0, nphonon_y = 0, nphonon_z = 0, nphonon_tot = 0;
unsigned long lastUpdate = 0;

// Current player stage (1 = magnitude stage, 2 = phonon stage). Shared with web client.
int current_stage = 1;

void clamp(int16_t &value, int16_t min, int16_t max) {
  if (value < min) value = min;
  if (value > max) value = max;
}

// --- Tunables (admin) ---
float phonon_scale = 0.1;
float nphonon_period_ms = 750.0;
// Magnitude threshold stage
float magnitude_threshold = 0.4; // default threshold
unsigned long magnitude_hold_ms = 1000; // default hold time in ms

// --- Math helpers ---
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

// --- Phonon calculation ---
int calc_nphonon(const std::vector<float>& accels) {
  int samples = nphonon_period_ms / 13.333; // 75Hz → 13.333ms
  if (samples <= 0 || samples >= accels.size()) return 0;

  float sum_sq = 0;
  for (size_t i = accels.size() - samples; i < accels.size(); i++) {
    float d = accels[i] - accels[i - samples];
    sum_sq += d * d;
  }

  float mean_sq = sum_sq / samples; // average over the number of samples, not total history
  return static_cast<int>(mean_sq * phonon_scale);
}

void calc_nphonons() {
  nphonon_x = calc_nphonon(accel_x_history);
  nphonon_y = calc_nphonon(accel_y_history);
  nphonon_z = calc_nphonon(accel_z_history);
  nphonon_tot = nphonon_x + nphonon_y + nphonon_z;
}

// --- Sensor update ---
void updateSensor() {
  unsigned long now = micros();
  if (now - lastUpdate < 13333) return; // 75Hz
  lastUpdate = now;

  float declinationAngle = (4.0 + (11.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);

  sVector_t mag = compass.readRaw();

  raw_xs.push_back(mag.XAxis);
  raw_ys.push_back(mag.YAxis);
  raw_zs.push_back(mag.ZAxis);

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

  linearAccel = {mag.XAxis - offsetAccel.x, mag.YAxis - offsetAccel.y, mag.ZAxis - offsetAccel.z};

  accel_x_history.push_back(linearAccel.x);
  accel_y_history.push_back(linearAccel.y);
  accel_z_history.push_back(linearAccel.z);

  if (accel_x_history.size() > 1000) {
    accel_x_history.erase(accel_x_history.begin());
    accel_y_history.erase(accel_y_history.begin());
    accel_z_history.erase(accel_z_history.begin());
  }
}

void init_LEDs() {
  ws2812fx_eyes.init();
  ws2812fx_left.init();
  ws2812fx_right.init();
  ws2812fx_eyes.setBrightness(BRIGHTNESS_EYES);
  ws2812fx_left.setBrightness(BRIGHTNESS_LEFT);
  ws2812fx_right.setBrightness(BRIGHTNESS_RIGHT);
  ws2812fx_left.setSpeed(SPEED_LEFT);
  ws2812fx_right.setSpeed(SPEED_RIGHT);
  ws2812fx_eyes.start();
  ws2812fx_left.start();
  ws2812fx_right.start();

  // Set individual eye colors: left = red, right = green.
  ws2812fx_eyes.setMode(FX_MODE_STATIC);
  ws2812fx_eyes.setPixelColor(0, (uint32_t)MyRed); // left eye red
  ws2812fx_eyes.setPixelColor(1, (uint32_t)MyGreen); // right eye green
  ws2812fx_eyes.show();

  // Control left and right strips manually via updateStripBlend().
  ws2812fx_left.setMode(FX_MODE_STATIC);
  ws2812fx_left.setColor((uint32_t)MyRed); // initial red tint
  ws2812fx_right.setMode(FX_MODE_STATIC);
  ws2812fx_right.setColor((uint32_t)MyGreen); // initial green tint
}

// --- Blended LED globals ---
float left_wipe_progress = 0.0f;
float right_wipe_progress = 0.0f;
unsigned long last_blend_ms = 0;

// --- Win / lights-off handling ---
unsigned long win_hold_ms = 3000; // ms to hold zero-phonons before turning lights off.
unsigned long win_timer_start = 0;
bool lights_off = false;
bool lights_off_applied = false;

// Track intended brightness for restore.
int brightness_eyes = 50;
int brightness_left = 50;
int brightness_right = 50;

// Note: Gaussian noise removed. Blending uses only wipe color.

// Update a strip with a wipe effect. Alpha derived from `nphonon_tot`.
// `baseR/baseG/baseB` are the per-strip wipe color components.
void updateStripBlend(WS2812FX &strip, int count, float &progress, int direction,
                      uint8_t baseR, uint8_t baseG, uint8_t baseB) {
  // how many phonons until fully wiped
  const float nphonon_scale = 10.0f;

  // compute alpha from global nphonon_tot (0..1)
  float alpha = (float)nphonon_tot / nphonon_scale;
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;

  unsigned long now = millis();
  float dt = 0.0f;
  if (last_blend_ms == 0) last_blend_ms = now;
  dt = (now - last_blend_ms) / 1000.0f;
  last_blend_ms = now;

  // advance wipe progress faster when alpha is higher
  float speed = 0.15f + alpha * 0.6f; // fraction per second
  progress += speed * dt;
  if (progress > 1.0f) progress -= 1.0f; // loop

  int wipe_pos = (int)floor(progress * (float)count);

  for (int i = 0; i < count; i++) {
    // Without noise: pixel is either wipe color or black.
    bool wiped;
    if (direction >= 0) wiped = (i < wipe_pos);
    else wiped = ((count - 1 - i) < wipe_pos);

    // If wiped, scale base color by alpha (so stronger phonons -> brighter wipe)
    uint8_t fr = wiped ? (uint8_t)((float)baseR * alpha) : 0;
    uint8_t fg = wiped ? (uint8_t)((float)baseG * alpha) : 0;
    uint8_t fb = wiped ? (uint8_t)((float)baseB * alpha) : 0;

    strip.setPixelColor(i, WS2812FX::Color(fr, fg, fb));
  }

  strip.show();
}

void init_IMU() {
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  if(compass.isHMC())
  {
    Serial.println("Initialize HMC5883");

    // Set/get the compass signal gain range (default 1.3 Ga).
    compass.setRange(HMC5883L_RANGE_8_1GA);
    Serial.print("compass range is:");
    Serial.println(compass.getRange());

    // Set/get measurement mode.
    // compass.setMeasurementMode(HMC5883L_CONTINOUS);
    Serial.print("compass measurement mode is:");
    Serial.println(compass.getMeasurementMode());

    // Set/get data collection frequency.
    compass.setDataRate(HMC5883L_DATARATE_75HZ);
    Serial.print("compass data rate is:");
    Serial.println(compass.getDataRate());

    // Set/get sensor samples.
    compass.setSamples(HMC5883L_SAMPLES_4);
    Serial.print("compass samples is:");
    Serial.println(compass.getSamples());
  }
}
// --- WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("Client %u connected\n", num);
    // send current stage to newly connected client
    String stageMsg = String("{\"type\":\"stage_sync\",\"stage\":") + String(current_stage) + String("}");
    webSocket.sendTXT(num, stageMsg);
    return;
  }

  if (type == WStype_TEXT && payload != nullptr && length > 0) {
    // parse incoming JSON messages from client
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
      Serial.println("WS: JSON parse error");
      return;
    }

    const char *t = doc["type"];
    if (t && strcmp(t, "stage") == 0) {
      int s = doc["stage"] | 1;
      Serial.printf("WS: stage set to %d\n", s);
      current_stage = s;
      // If stage moves back to 1, ensure lights are restored
      if (current_stage != 2) {
        lights_off = false;
        win_timer_start = 0;
      }
    }
  }
}

// --- Admin to player messages ---
void sendMessageToPlayers(const String &msg) {
  JsonDocument doc;
  doc["type"] = "adminMessage";
  doc["text"] = msg;

  String payload;
  serializeJson(doc, payload);
  webSocket.broadcastTXT(payload);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  LittleFS.begin();

  init_LEDs();
  init_IMU();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  Serial.println("DNS server started at " + WiFi.softAPIP().toString());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Serve root page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(LittleFS, "/index.html", "text/html");
  });

  // Admin API endpoints
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
      if (req->hasParam("threshold"))
        magnitude_threshold = req->getParam("threshold")->value().toFloat();
      if (req->hasParam("hold"))
        magnitude_hold_ms = (unsigned long)req->getParam("hold")->value().toInt();
      if (req->hasParam("win_hold"))
        win_hold_ms = (unsigned long)req->getParam("win_hold")->value().toInt();

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

    else if (cmd == "setStage") {
      if (!req->hasParam("stage")) {
        req->send(400, "text/plain", "Missing stage");
        return;
      }
      int s = req->getParam("stage")->value().toInt();
      current_stage = s;
      if (current_stage == 3) {
        lights_off = true;
      } else {
        lights_off = false;
        win_timer_start = 0;
      }
      // broadcast new stage to clients
      StaticJsonDocument<64> out;
      out["type"] = "stage";
      out["stage"] = current_stage;
      String payload;
      serializeJson(out, payload);
      webSocket.broadcastTXT(payload);

      req->send(200, "text/plain", "Stage updated");
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

// --- Loop ---
void loop() {
  // Apply/restore lights-off brightness.
  if (lights_off && !lights_off_applied) {
    ws2812fx_eyes.setBrightness(0);
    ws2812fx_left.setBrightness(0);
    ws2812fx_right.setBrightness(0);
    ws2812fx_eyes.show();
    ws2812fx_left.show();
    ws2812fx_right.show();
    lights_off_applied = true;
  } else if (!lights_off && lights_off_applied) {
    ws2812fx_eyes.setBrightness(brightness_eyes);
    ws2812fx_left.setBrightness(brightness_left);
    ws2812fx_right.setBrightness(brightness_right);
    lights_off_applied = false;
  }

  ws2812fx_eyes.service();
  // Enforce eye colors; service() may overwrite static pixel data.
  if (!lights_off) {
    ws2812fx_eyes.setPixelColor(0, (uint32_t)MyRed); // left eye red (#d62728)
    ws2812fx_eyes.setPixelColor(1, (uint32_t)MyGreen); // right eye green (#2ca02c)
    ws2812fx_eyes.show();
  }

  // Update left/right strips (alternating swipe directions)
  updateStripBlend(ws2812fx_left, LED_LEFT_COUNT, left_wipe_progress, 1, 
                    (uint8_t)((MyRed >> 16) & 0xFF), (uint8_t)((MyRed >> 8) & 0xFF), (uint8_t)(MyRed & 0xFF));
  updateStripBlend(ws2812fx_right, LED_RIGHT_COUNT, right_wipe_progress, -1, 
                    (uint8_t)((MyGreen >> 16) & 0xFF), (uint8_t)((MyGreen >> 8) & 0xFF), (uint8_t)(MyGreen & 0xFF));
  dnsServer.processNextRequest();
  webSocket.loop();
  updateSensor();

  static unsigned long lastPush = 0;
  if (micros() - lastPush > 20000) {
    lastPush = micros();
    calc_nphonons();

    // Only consider win/lights-off when client is in stage 2 (phonon stage).
    if (current_stage == 2 && nphonon_tot == 0) {
      if (win_timer_start == 0) {
        win_timer_start = millis();
      } else if (millis() - win_timer_start >= win_hold_ms) {
        // Enter persistent win stage (3) once target reached.
        if (current_stage != 3) {
          current_stage = 3;
          lights_off = true;
          // broadcast stage change to all clients
          StaticJsonDocument<64> out;
          out["type"] = "stage";
          out["stage"] = current_stage;
          String payload;
          serializeJson(out, payload);
          webSocket.broadcastTXT(payload);
          Serial.println("Entered win stage (3)");
        }
      }
    } else {
      // If already in persistent win stage, keep it until explicitly changed.
      if (current_stage == 3) {
        lights_off = true;
      } else {
        // reset timer and restore lights if phonons return or stage isn't 2
        win_timer_start = 0;
        lights_off = false;
      }
    }

    String json = "{\"x\":" + String(linearAccel.x,2) +
                  ",\"y\":" + String(linearAccel.y,2) +
                  ",\"z\":" + String(linearAccel.z,2) +
                  ",\"nphonon_x\":" + String(nphonon_x) +
                  ",\"nphonon_y\":" + String(nphonon_y) +
                  ",\"nphonon_z\":" + String(nphonon_z) +
                  ",\"nphonon_tot\":" + String(nphonon_tot) +
                  ",\"mag_threshold\":" + String(magnitude_threshold) +
                  ",\"mag_hold\":" + String(magnitude_hold_ms) + "}";

    webSocket.broadcastTXT(json);
  }
}
