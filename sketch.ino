#include <WiFi.h>
#include <time.h>

// WiFi Wokwi
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// Broker public
const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;

// Topic-uri
const char* TOPIC_CMD    = "adelina_salavastru/feed/command";
const char* TOPIC_STATUS = "adelina_salavastru/feed/status";
const char* TOPIC_LOG    = "adelina_salavastru/feed/log";

// LED hrănire
const int LED_PIN = 2;
const int FEED_MS = 2000;

// Hrănire automată 1/zi (default 08:00)
int feedHour = 8;
int feedMinute = 0;
int lastFedDayOfYear = -1;

// NTP (Romania: UTC+2 iarna; dacă ora e decalată, pune 3*3600)
const long GMT_OFFSET_SEC = 2 * 3600;
const int  DAYLIGHT_OFFSET_SEC = 0;

// MQTT minimal
WiFiClient client;
uint16_t mqttPacketId = 1;
unsigned long lastPingMs = 0;

// --- MQTT Remaining Length encoder ---
int encRL(uint8_t* out, int len) {
  int i = 0;
  do {
    uint8_t d = len % 128;
    len /= 128;
    if (len > 0) d |= 0x80;
    out[i++] = d;
  } while (len > 0);
  return i;
}

void putStr(uint8_t* buf, int &idx, const char* s) {
  uint16_t n = (uint16_t)strlen(s);
  buf[idx++] = (n >> 8) & 0xFF;
  buf[idx++] = n & 0xFF;
  memcpy(buf + idx, s, n);
  idx += n;
}

bool sendAll(const uint8_t* data, int len) {
  return client.write(data, len) == len;
}

bool mqttConnect() {
  Serial.print("MQTT TCP connect to ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  if (!client.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.println("TCP connect failed.");
    return false;
  }

  uint8_t body[256]; int b = 0;

  // Protocol Name "MQTT"
  body[b++] = 0x00; body[b++] = 0x04;
  body[b++] = 'M'; body[b++] = 'Q'; body[b++] = 'T'; body[b++] = 'T';
  // Level 4 (3.1.1)
  body[b++] = 0x04;
  // Clean session
  body[b++] = 0x02;
  // KeepAlive 60s
  body[b++] = 0x00; body[b++] = 60;

  String cid = "esp32-adelina-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  putStr(body, b, cid.c_str());

  uint8_t pkt[300]; int p = 0;
  pkt[p++] = 0x10; // CONNECT
  uint8_t rl[4]; int rlLen = encRL(rl, b);
  memcpy(pkt + p, rl, rlLen); p += rlLen;
  memcpy(pkt + p, body, b); p += b;

  if (!sendAll(pkt, p)) { client.stop(); return false; }

  // Wait CONNACK
  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 3000) delay(10);
  if (!client.available()) { Serial.println("CONNACK timeout"); client.stop(); return false; }

  uint8_t resp[4];
  int r = client.read(resp, 4);
  if (r < 4 || resp[0] != 0x20 || resp[3] != 0x00) {
    Serial.println("CONNACK failed");
    client.stop();
    return false;
  }

  Serial.println("MQTT connected.");
  lastPingMs = millis();
  return true;
}

bool mqttSubscribe(const char* topic) {
  uint8_t body[256]; int b = 0;
  uint16_t pid = mqttPacketId++;
  body[b++] = (pid >> 8) & 0xFF;
  body[b++] = pid & 0xFF;
  putStr(body, b, topic);
  body[b++] = 0x00; // QoS0

  uint8_t pkt[300]; int p = 0;
  pkt[p++] = 0x82; // SUBSCRIBE (QoS1)
  uint8_t rl[4]; int rlLen = encRL(rl, b);
  memcpy(pkt + p, rl, rlLen); p += rlLen;
  memcpy(pkt + p, body, b); p += b;

  bool ok = sendAll(pkt, p);
  Serial.print("Subscribed: "); Serial.println(topic);
  return ok;
}

bool mqttPublish(const char* topic, const String& payload) {
  uint8_t body[512]; int b = 0;
  putStr(body, b, topic);
  int pl = payload.length();
  memcpy(body + b, payload.c_str(), pl); b += pl;

  uint8_t pkt[600]; int p = 0;
  pkt[p++] = 0x30; // PUBLISH QoS0
  uint8_t rl[4]; int rlLen = encRL(rl, b);
  memcpy(pkt + p, rl, rlLen); p += rlLen;
  memcpy(pkt + p, body, b); p += b;

  return sendAll(pkt, p);
}

void mqttPing() {
  if (millis() - lastPingMs < 30000) return;
  uint8_t p[2] = {0xC0, 0x00}; // PINGREQ
  sendAll(p, 2);
  lastPingMs = millis();
}

bool getTimeSafe(struct tm &t) { return getLocalTime(&t, 2000); }

String tsNow() {
  struct tm t;
  if (!getTimeSafe(t)) return "unknown";
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &t);
  return String(buf);
}

void doFeed(const String& type) {
  digitalWrite(LED_PIN, HIGH);
  delay(FEED_MS);
  digitalWrite(LED_PIN, LOW);

  String ts = tsNow();
  mqttPublish(TOPIC_STATUS, "FEED_OK (" + type + ")");
  mqttPublish(TOPIC_LOG, "{\"time\":\"" + ts + "\",\"type\":\"" + type + "\",\"result\":\"OK\"}");
  Serial.println("[FEED] " + type + " at " + ts);
}

void parseCmd(const String& msg) {
  if (msg == "FEED_NOW") { doFeed("MANUAL"); return; }

  if (msg.startsWith("SET_TIME:") && msg.length() >= 14) {
    int hh = msg.substring(9, 11).toInt();
    int mm = msg.substring(12, 14).toInt();
    if (hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59) {
      feedHour = hh; feedMinute = mm;
      String ok = "TIME_SET_OK " + String(feedHour) + ":" + (feedMinute < 10 ? "0" : "") + String(feedMinute);
      mqttPublish(TOPIC_STATUS, ok);
      Serial.println(ok);
      return;
    }
    mqttPublish(TOPIC_STATUS, "TIME_SET_FAIL");
    return;
  }

  mqttPublish(TOPIC_STATUS, "CMD_UNKNOWN");
}

void readPackets() {
  while (client.available()) {
    uint8_t h = client.read();

    // Remaining length decode
    int mult = 1, val = 0;
    uint8_t d;
    do {
      while (!client.available()) delay(1);
      d = client.read();
      val += (d & 127) * mult;
      mult *= 128;
    } while (d & 128);

    uint8_t rb[512];
    int toRead = min(val, 512);
    int got = 0;
    unsigned long t0 = millis();
    while (got < toRead && millis() - t0 < 2000) {
      if (client.available()) rb[got++] = client.read();
      else delay(1);
    }

    uint8_t type = h >> 4;
    if (type == 3 && got >= 2) { // PUBLISH
      int pos = 0;
      uint16_t tlen = (rb[pos] << 8) | rb[pos + 1];
      pos += 2;
      if (pos + tlen > got) return;

      String topic = "";
      for (int i = 0; i < tlen; i++) topic += (char)rb[pos + i];
      pos += tlen;

      String payload = "";
      for (int i = pos; i < got; i++) payload += (char)rb[i];
      payload.trim();

      Serial.print("[MQTT IN] "); Serial.print(topic); Serial.print(" => "); Serial.println(payload);

      if (topic == TOPIC_CMD) parseCmd(payload);
    }
  }
}

void connectWiFi() {
  Serial.print("WiFi connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK. IP=" + WiFi.localIP().toString());
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  connectWiFi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");
  Serial.println("NTP OK.");

  if (mqttConnect()) {
    mqttSubscribe(TOPIC_CMD);
    mqttPublish(TOPIC_STATUS, "DEVICE_ONLINE");
    Serial.println("Ready. Send FEED_NOW / SET_TIME:HH:MM");
  }
}

void loop() {
  if (!client.connected()) {
    client.stop();
    delay(1000);
    if (mqttConnect()) {
      mqttSubscribe(TOPIC_CMD);
      mqttPublish(TOPIC_STATUS, "DEVICE_ONLINE");
    }
  }

  mqttPing();
  readPackets();

  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 1000) {
    lastCheck = millis();
    struct tm now;
    if (getTimeSafe(now)) {
      int day = now.tm_yday;
      if (now.tm_hour == feedHour && now.tm_min == feedMinute && lastFedDayOfYear != day) {
        Serial.println("Auto feed trigger: " + tsNow());
        doFeed("AUTO");
        lastFedDayOfYear = day;
      }
    }
  }
}
