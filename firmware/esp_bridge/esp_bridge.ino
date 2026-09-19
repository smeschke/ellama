// ===== Llama Robot: Motor Receiver Test Sender (ESP-NOW) =====
//
// Bench tool for motor_receiver.ino. Type "<left> <right>" into the Serial
// Monitor (e.g. "255 255", "-255 0", "0 0" or just "stop") and it streams
// that DrivePacket to the motor receiver every SEND_INTERVAL_MS, so you can
// watch ramping, direction, and stop behavior live.
//
// Flash this onto a second ESP32 (or the ESP32 bridge you plug into a
// computer) -- sends via ESP-NOW broadcast, so any motor_receiver.ino
// listening on the same channel picks it up with no MAC pairing needed.

#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t SEND_INTERVAL_MS = 20; // well under motor_receiver.ino's CMD_TIMEOUT_MS (300ms)

typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

int cmdL = 0, cmdR = 0;
uint32_t lastSendMs = 0;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // no-op; add Serial.println(status) here if you need send-failure debugging
}

void readSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0 || line.equalsIgnoreCase("stop")) {
    cmdL = 0; cmdR = 0;
    Serial.println("-> stop");
    return;
  }

  int spaceIdx = line.indexOf(' ');
  if (spaceIdx < 0) {
    Serial.println("format: <left> <right>   e.g. 255 -255, or 'stop'");
    return;
  }

  int l = line.substring(0, spaceIdx).toInt();
  int r = line.substring(spaceIdx + 1).toInt();
  cmdL = constrain(l, -255, 255);
  cmdR = constrain(r, -255, 255);
  Serial.printf("-> left=%d right=%d\n", cmdL, cmdR);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- Motor Receiver Test Sender ---");
  Serial.println("Type: <left> <right>   e.g. 255 255 | -255 0 | -255 255");
  Serial.println("Type 'stop' or press enter with nothing typed to stop.");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  peer.channel = 1;
  peer.encrypt = false;
  memcpy(peer.peer_addr, broadcastMac, 6);
  esp_now_add_peer(&peer);
}

void loop() {
  readSerialCommand();

  uint32_t now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    DrivePacket p{ (int16_t)cmdL, (int16_t)cmdR };
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }
}
