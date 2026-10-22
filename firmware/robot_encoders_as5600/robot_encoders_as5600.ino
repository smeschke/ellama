// ===== Llama Robot: Dual Wheel Encoder Sender (2x AS5600, ESP-NOW) =====
//
// Runs on the Llama Encoder Board (currently a breadboard prototype). Reads two AS5600 magnetic encoders and
// broadcasts a SensorPacket named "ENC" over ESP-NOW every SEND_DT_MS. The bridge
// (computer_bridge.ino) relays it to the PC as "ENC <left> <right> <ms>", unchanged
// from the old packet format.
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
// Counts are int32 and cumulative, so they only wrap after a very long one-way run:
// at 4096 counts per encoder-shaft turn and a 6.33:1 gear ratio, a wheel turning a
// steady 10 rpm takes about 5 days of continuous rotation in a single direction to
// overflow, and driving both ways cancels out. No scaling is needed to avoid wrapping.

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LlamaSensors.h>

// ================= I2C pins =================
// Left encoder on Wire, right on Wire1. If left/right come out swapped on the robot,
// swap the two bus entries in enc[] below rather than rewiring. If an encoder reports
// "no magnet" at boot, check SDA/SCL are not transposed for that bus before suspecting
// the magnet -- a swapped pair looks exactly like a missing sensor.
#define SDA_L 18
#define SCL_L 19
#define SDA_R 33
#define SCL_R 32

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
// Broadcast as a SensorPacket (LlamaSensors.h) rather than the old fixed-length
// EncoderPacket, so the packet identifies itself by name instead of by its length.
//
// values[0] = left, values[1] = right: cumulative unwrapped AS5600 counts, where 4096
// counts = one turn of the ENCODER shaft (not the wheel). Counts stay raw on the wire --
// the gear ratio and wheel size are applied on the PC side, so they can be re-measured
// without reflashing a board bolted to the robot. An encoder that is not responding sends
// SENSOR_VALUE_UNAVAILABLE in its slot, so losing one costs you that wheel and nothing
// else -- the packet keeps its shape and the other wheel keeps reporting.
//
// The name is "ENC" deliberately: the bridge prints "<name> <values...> <ms>", so this
// comes out as "ENC <left> <right> <ms>" -- byte-identical to what the old 12-byte
// EncoderPacket produced. calibration/bridge.py (including its encoder_sign handling)
// and live_view.py therefore need no changes at all.
static const char SENSOR_NAME[] = "ENC";

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// ================= Settings =================
const uint32_t SEND_DT_MS  = 20;  // 50 Hz sample + ESP-NOW rate
const uint32_t PRINT_DT_MS = 100; // 10 Hz USB status

// ================= State =================
struct Encoder {
  const char *name;
  TwoWire *bus;
  int sda, scl;
  int32_t counts;          // cumulative 12-bit counts, no wraparound
  int32_t lastPrintCounts;
  int lastRaw;
  bool present;            // false = nothing answering on this bus
};
Encoder enc[2] = {   // [0] = left, [1] = right
  {"left",  &Wire,  SDA_L, SCL_L, 0, 0, -1, false},
  {"right", &Wire1, SDA_R, SCL_R, 0, 0, -1, false},
};

// The encoders sit next to the wheels and get damaged, so one being gone is a normal
// state rather than a fault to stop for. A missing encoder is re-probed periodically so
// a repaired or replaced one rejoins without a reboot, and its slot on the wire carries
// SENSOR_VALUE_UNAVAILABLE -- never a fabricated 0, which the PC could not tell apart
// from a wheel that simply is not turning.
const uint32_t RECHECK_DT_MS = 2000;
uint32_t lastRecheckMs = 0;

uint32_t lastSendMs = 0, lastPrintMs = 0;

// Does an AS5600 answer on this bus with these pins? The AS5600's address is fixed at
// 0x36, so a simple address probe settles it -- and distinguishes the two faults that
// look identical in the magnet report: a transposed SDA/SCL pair (nothing answers) and a
// present sensor with no magnet near it (answers, but AGC/magnitude read 0).
bool busHasEncoder(TwoWire &w, int sda, int scl) {
  w.end();
  w.begin(sda, scl);
  w.setClock(400000);
  w.beginTransmission(AS5600_ADDR);
  return w.endTransmission() == 0;
}

// Bring up one bus, trying the configured pin order first and the swapped order second.
// SDA/SCL transposed against the wiring comment has now caused two separate "sensor is
// dead" hunts in this firmware, so the sketch checks rather than trusting the #defines.
// Returns true if an AS5600 answered, and leaves the bus initialised on whichever order
// worked.
bool openEncoderBus(TwoWire &w, const char *name, int sda, int scl) {
  if (busHasEncoder(w, sda, scl)) {
    Serial.printf("[%s] AS5600 at 0x%02X on SDA=%d SCL=%d\n", name, AS5600_ADDR, sda, scl);
    return true;
  }
  if (busHasEncoder(w, scl, sda)) {
    Serial.printf("[%s] AS5600 found with SDA/SCL SWAPPED: SDA=%d SCL=%d -- "
                  "update the #defines to match\n", name, scl, sda);
    return true;
  }
  Serial.printf("[%s] no AS5600 on SDA=%d SCL=%d (either order) -- check VCC/GND/wiring\n",
                name, sda, scl);
  // Leave it on the configured order so later reads fail quietly rather than hanging.
  busHasEncoder(w, sda, scl);
  return false;
}

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

  for (int i = 0; i < 2; i++) {
    Encoder &e = enc[i];
    e.present = openEncoderBus(*e.bus, e.name, e.sda, e.scl);
  }

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
    Serial.printf("ESP-NOW broadcasting SensorPacket \"%s\" (%u bytes) on channel 1.\n",
                  SENSOR_NAME, (unsigned)sizeof(SensorPacket));
  }

  lastSendMs = lastPrintMs = millis();
}

void loop() {
  // Poll both encoders every pass so fast spins never skip more than half a
  // rev between reads (unwrap breaks past that).
  for (int i = 0; i < 2; i++) {
    if (enc[i].present) updateEncoder(i);
  }

  uint32_t now = millis();

  // Re-probe anything missing, so a replaced encoder comes back on its own.
  if (now - lastRecheckMs >= RECHECK_DT_MS) {
    lastRecheckMs = now;
    for (int i = 0; i < 2; i++) {
      Encoder &e = enc[i];
      if (e.present) continue;
      if (busHasEncoder(*e.bus, e.sda, e.scl)) {
        e.present = true;
        e.lastRaw = -1;              // resync the unwrap on the next read
        e.counts = 0;                // counts restart from here; the PC re-zeros anyway
        Serial.printf("[%s] came back on SDA=%d SCL=%d\n", e.name, e.sda, e.scl);
      }
    }
  }
  if (now - lastSendMs >= SEND_DT_MS) {
    lastSendMs = now;
    SensorPacket p;
    sensorPacketInit(&p, SENSOR_NAME, 2, now);
    for (int i = 0; i < 2; i++) {
      p.values[i] = enc[i].present ? enc[i].counts : SENSOR_VALUE_UNAVAILABLE;
    }
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }

  // Same per-encoder line format as bench_encoders_as5600_test.ino.
  if (now - lastPrintMs >= PRINT_DT_MS) {
    float dt_s = (now - lastPrintMs) / 1000.0f;
    for (int i = 0; i < 2; i++) {
      Encoder &e = enc[i];
      if (!e.present) {
        Serial.printf("%-5s NOT_RESPONDING\n", e.name);
        continue;
      }
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
