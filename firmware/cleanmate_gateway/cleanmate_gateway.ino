// CleanMate - ESP-NOW gateway (USB serial <-> robots)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const uint8_t LEADER_MAC[6]   = {0xEC, 0xE3, 0x34, 0x22, 0x25, 0x50};
static const uint8_t FOLLOWER_MAC[6] = {0xF0, 0x24, 0xF9, 0x0E, 0x2B, 0xE4};
#define WIFI_CHANNEL 1

typedef struct __attribute__((packed)) {
  uint8_t robot_id; uint8_t state; uint32_t seq; uint32_t t_ms;
  int32_t enc_left; int32_t enc_right;
  float heading_deg; float dist_travelled_m; float v_actual_mps;
  uint16_t lateral_gap_mm; uint16_t dist_fl_mm; uint16_t dist_fr_mm; uint16_t batt_mv;
  uint8_t flags;
} RobotTelemetry;
typedef struct __attribute__((packed)) { char txt[25]; } TextCmd;

QueueHandle_t q;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len == sizeof(RobotTelemetry)) {
    RobotTelemetry t; memcpy(&t, data, len);
    xQueueSend(q, &t, 0);
  }
}

void addPeer(const uint8_t* mac) {
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6); p.channel = WIFI_CHANNEL; p.encrypt = false;
  esp_now_add_peer(&p);
}

void setup() {
  Serial.begin(115200);
  q = xQueueCreate(32, sizeof(RobotTelemetry));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("G,espnow-init-failed"); return; }
  esp_now_register_recv_cb(onRecv);
  addPeer(LEADER_MAC); addPeer(FOLLOWER_MAC);
  Serial.printf("G,ready,%s\n", WiFi.macAddress().c_str());
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) {
        TextCmd tc = {}; buf.toCharArray(tc.txt, sizeof(tc.txt));
        esp_now_send(LEADER_MAC, (uint8_t*)&tc, sizeof(tc));
        esp_now_send(FOLLOWER_MAC, (uint8_t*)&tc, sizeof(tc));
        if (strcmp(tc.txt, "h")) Serial.printf("G,tx,%s\n", tc.txt);
        buf = "";
      }
    } else if (buf.length() < 24) buf += c;
  }
  static uint8_t cnt[3] = {0, 0, 0};
  RobotTelemetry t;
  while (xQueueReceive(q, &t, 0) == pdTRUE) {
    if (t.robot_id > 2 || ++cnt[t.robot_id] < 5) continue;
    cnt[t.robot_id] = 0;
    Serial.printf("T,%u,%u,%lu,%lu,%ld,%ld,%.1f,%.3f,%.3f,%u,%u,%u,%u,%u\n",
      t.robot_id, t.state, (unsigned long)t.seq, (unsigned long)t.t_ms, (long)t.enc_left, (long)t.enc_right,
      t.heading_deg, t.dist_travelled_m, t.v_actual_mps, t.lateral_gap_mm, t.dist_fl_mm, t.dist_fr_mm,
      t.batt_mv, t.flags);
  }
}
