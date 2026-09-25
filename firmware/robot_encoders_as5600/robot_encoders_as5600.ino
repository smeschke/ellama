// ===== Llama Robot: Dual Wheel Encoder Sender (2x AS5600, ESP-NOW) =====
//
// Runs on the robot's encoder ESP32. Reads two AS5600 magnetic encoders and
// broadcasts an EncoderPacket over ESP-NOW every SEND_DT_MS. The bridge
// (computer_bridge.ino) relays it to the PC as "ENC <left> <right> <ms>".
//
// The AS5600's I2C address (0x36) is fixed, so each encoder gets its own bus:
//   Left  encoder (enc[0], Wire,  SDA=GPIO21, SCL=GPIO22)
//   Right encoder (enc[1], Wire1, SDA=GPIO18, SCL=GPIO19)
//   Both: VCC->3.3V, GND->GND.
// If left/right come out swapped on your robot, swap the two bus entries in
// enc[] below rather than rewiring.
//
// The packet carries raw cumulative counts; gear ratio, wheel size and speed
// are applied on the PC side, so they can be tuned without reflashing.
//
// Boot check and the 10 Hz USB status line are identical to
// bench_encoders_as5600_test.ino, so bench_encoders_as5600_test/visualize.py
// also works against this sketch over USB. Counts here start at 0 at boot
// (relative travel); the bench test starts them at the raw angle.
//
// The bridge tells packet types apart by length: Drive = 4, Encoder = 12,
// Imu = 16. Keep this struct's size distinct from those.

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

// ================= I2C pins =================
#define SDA_L 21
#define SCL_L 22
#define SDA_R 18
#define SCL_R 19

// ================= AS5600 =================
const uint8_t AS5600_ADDR   = 0x36;
const uint8_t REG_STATUS    = 0x0B;
const uint8_t REG_RAW_ANGLE = 0x0C; // 12-bit, 0..4095, unfiltered
const uint8_t REG_AGC       = 0x1A;
const uint8_t REG_MAGNITUDE = 0x1B;

const uint8_t STATUS_MD = 0x20; // magnet detected
const uint8_t STATUS_ML = 0x10; // magnet too weak
const uint8_t STATUS_MH = 0x08; // magnet too strong

// Encoder-shaft turns per wheel turn, same empirical value as the bench test.
// Only used for the USB wheel_revs/wheel_rpm figures; the packet stays raw.
const float GEAR_RATIO = 6.33f;

// ================= ESP-NOW =================
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// NOTE: layout and size (12 bytes) must match the copy in computer_bridge.ino exactly.
typedef struct __attribute__((packed)) {
  int32_t left;   // cumulative unwrapped AS5600 counts (4096 counts = 1 encoder-shaft rev)
  int32_t right;
  uint32_t ms;    // this ESP32's millis() at send time
} EncoderPacket;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// ================= Settings =================
const uint32_t SEND_DT_MS  = 20;  // 50 Hz sample + ESP-NOW rate
const uint32_t PRINT_DT_MS = 100; // 10 Hz USB status

// ================= State =================
struct Encoder {
  const char *name;
  TwoWire *bus;
  int32_t counts;          // cumulative 12-bit counts, no wraparound
  int32_t lastPrintCounts;
  int lastRaw;
};
Encoder enc[2] = { {"left", &Wire, 0, 0, -1}, {"right", &Wire1, 0, 0, -1} }; // [0] = left, [1] = right

uint32_t lastSendMs = 0, lastPrintMs = 0;

uint16_t readAs5600(TwoWire &w, uint8_t reg, uint8_t n) {
  w.beginTransmission(AS5600_ADDR);
  w.write(reg);
  w.endTransmission(false);
  w.requestFrom((int)AS5600_ADDR, (int)n);
  uint16_t hi = w.available() ? w.read() : 0;
  if (n == 1) return hi;
  uint16_t lo = w.available() ? w.read() : 0;
  return ((hi << 8) | lo) & 0x0FFF;
}

void updateEncoder(int i) {
  Encoder &e = enc[i];
  int raw = readAs5600(*e.bus, REG_RAW_ANGLE, 2);
  int delta = raw - e.lastRaw;
  if (delta > 2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  e.counts += delta;
  e.lastRaw = raw;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_L, SCL_L, 400000);
  Wire1.begin(SDA_R, SCL_R, 400000);

  Serial.println("===== Robot Dual Encoder Sender =====");

  for (int i = 0; i < 2; i++) {
    Encoder &e = enc[i];
    uint8_t status = readAs5600(*e.bus, REG_STATUS, 1);
    Serial.printf("[%s] ", e.name);
    if (!(status & STATUS_MD)) {
      Serial.println("WARNING: magnet not detected -- check wiring/alignment.");
    } else {
      Serial.print("Magnet detected.");
      if (status & STATUS_ML) Serial.print(" Too weak, move magnet closer.");
      if (status & STATUS_MH) Serial.print(" Too strong, move magnet farther.");
      Serial.println();
    }
    Serial.printf("[%s] AGC: %u  Magnitude: %u\n", e.name,
                  readAs5600(*e.bus, REG_AGC, 1), readAs5600(*e.bus, REG_MAGNITUDE, 2));

    // Start counts at 0 so each run reports relative travel.
    e.lastRaw = readAs5600(*e.bus, REG_RAW_ANGLE, 2);
    e.counts = 0;
    e.lastPrintCounts = 0;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  } else {
    esp_now_register_send_cb(onSent);
    esp_now_peer_info_t peer{};
    peer.channel = 1;
    peer.encrypt = false;
    memcpy(peer.peer_addr, broadcastMac, 6);
    esp_now_add_peer(&peer);
    Serial.println("ESP-NOW broadcasting EncoderPacket on channel 1.");
  }

  lastSendMs = lastPrintMs = millis();
}

void loop() {
  // Poll both encoders every pass so fast spins never skip more than half a
  // rev between reads (unwrap breaks past that).
  updateEncoder(0);
  updateEncoder(1);

  uint32_t now = millis();
  if (now - lastSendMs >= SEND_DT_MS) {
    lastSendMs = now;
    EncoderPacket p{ enc[0].counts, enc[1].counts, now };
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }

  // Same per-encoder line format as bench_encoders_as5600_test.ino.
  if (now - lastPrintMs >= PRINT_DT_MS) {
    float dt_s = (now - lastPrintMs) / 1000.0f;
    for (int i = 0; i < 2; i++) {
      Encoder &e = enc[i];
      float angleDeg = e.lastRaw * 360.0f / 4096.0f;
      float revs = e.counts / 4096.0f;
      float rpm = ((e.counts - e.lastPrintCounts) / 4096.0f) / dt_s * 60.0f;
      Serial.printf("%-5s angle=%6.1f deg  revs=%8.3f  rpm=%7.1f  raw=%4d  || wheel_revs=%8.3f  wheel_rpm=%6.1f\n",
                    e.name, angleDeg, revs, rpm, e.lastRaw, revs / GEAR_RATIO, rpm / GEAR_RATIO);
      e.lastPrintCounts = e.counts;
    }
    lastPrintMs = now;
  }
}
