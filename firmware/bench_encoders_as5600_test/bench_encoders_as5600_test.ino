// ===== Llama Robot: Dual AS5600 Encoder Bench Test =====
//
// Standalone sanity check for two AS5600 magnetic encoders wired to an ESP32
// over I2C. Confirms each magnet is present and well aligned, then streams
// angle, unwrapped position, and RPM for both so you can spin each wheel by
// hand and watch the right numbers move. No libraries beyond Wire.h -- talks
// to each AS5600 directly over its I2C register map.
//
// Same pins as robot_encoders_as5600.ino, one bus per encoder because the
// AS5600's address (0x36) is fixed:
//   Left  (Wire,  SDA=GPIO21, SCL=GPIO22)
//   Right (Wire1, SDA=GPIO18, SCL=GPIO19)
//   Both: VCC->3.3V, GND->GND.
// Most AS5600 breakout boards already have SDA/SCL pull-ups on board.
//
// An encoder that isn't connected just reports a missing magnet; the other
// one keeps working, so this also tests a single encoder.

#include <Wire.h>

// ================= I2C pins =================
#define SDA_L 18
#define SCL_L 19
#define SDA_R 33
#define SCL_R 32

// ================= AS5600 registers =================
const uint8_t AS5600_ADDR   = 0x36;
const uint8_t REG_STATUS    = 0x0B;
const uint8_t REG_RAW_ANGLE = 0x0C; // 12-bit, 0..4095, unfiltered
const uint8_t REG_AGC       = 0x1A;
const uint8_t REG_MAGNITUDE = 0x1B;

const uint8_t STATUS_MD = 0x20; // magnet detected
const uint8_t STATUS_ML = 0x10; // magnet too weak
const uint8_t STATUS_MH = 0x08; // magnet too strong

// ================= Settings =================
const uint32_t PRINT_DT_MS = 100; // ~10 Hz status output

// Gear reduction between the AS5600's shaft and the wheel: wheel turns are
// the encoder's turns divided by this ratio. Measured empirically: 10 real
// wheel revs read as 6.3 at GEAR_RATIO=7.0, so true ratio = 7.0*(6.3/10).
const float GEAR_RATIO = 6.33f;

// ================= State =================
struct Encoder {
  const char *name;
  TwoWire *bus;
  int sda, scl;
  int32_t counts;      // cumulative 12-bit counts, no wraparound
  int32_t lastPrintCounts;
  int lastRaw;
  bool present;        // false = nothing answering on this bus
};
Encoder enc[2] = {
  {"left",  &Wire,  SDA_L, SCL_L, 0, 0, -1, false},
  {"right", &Wire1, SDA_R, SCL_R, 0, 0, -1, false},
};

// The encoders sit next to the wheels and get damaged, so one of them being gone is a
// normal state, not an error. Everything below carries on with whatever is still there:
// a missing encoder is re-probed periodically (so it reappears on its own if reconnected
// or replaced) and reported as absent rather than reading as a stationary wheel, which is
// what a fabricated 0 would look like.
const uint32_t RECHECK_DT_MS = 2000;
uint32_t lastRecheckMs = 0;

uint32_t lastPrintMs = 0;

uint8_t readReg8(TwoWire &w, uint8_t reg) {
  w.beginTransmission(AS5600_ADDR);
  w.write(reg);
  w.endTransmission(false);
  w.requestFrom((int)AS5600_ADDR, 1);
  return w.available() ? w.read() : 0;
}

uint16_t readReg16(TwoWire &w, uint8_t reg) {
  w.beginTransmission(AS5600_ADDR);
  w.write(reg);
  w.endTransmission(false);
  w.requestFrom((int)AS5600_ADDR, 2);
  uint16_t hi = w.available() ? w.read() : 0;
  uint16_t lo = w.available() ? w.read() : 0;
  return ((hi << 8) | lo) & 0x0FFF;
}

// The AS5600's address is fixed at 0x36, so an address ping says whether anything is on
// this bus at all -- which separates a dead/unplugged encoder from a live one whose
// magnet is missing or misaligned (the latter answers, but reads AGC/magnitude 0).
bool busHasEncoder(TwoWire &w, int sda, int scl) {
  w.end();
  w.begin(sda, scl);
  w.setClock(400000);
  w.beginTransmission(AS5600_ADDR);
  return w.endTransmission() == 0;
}

// Try the configured pin order, then the swapped order. SDA/SCL transposed against a
// sketch's own wiring comment has caused more than one "the sensor is dead" hunt in this
// firmware, so check rather than trust the #defines.
bool probeEncoder(Encoder &e) {
  if (busHasEncoder(*e.bus, e.sda, e.scl)) return true;
  if (busHasEncoder(*e.bus, e.scl, e.sda)) {
    Serial.printf("[%s] found with SDA/SCL SWAPPED (SDA=%d SCL=%d) -- fix the #defines\n",
                  e.name, e.scl, e.sda);
    return true;
  }
  busHasEncoder(*e.bus, e.sda, e.scl);   // leave the bus on the configured order
  return false;
}

// Read the magnet status and seed the counter. Only called for an encoder that answered.
void reportMagnet(Encoder &e) {
  uint8_t status = readReg8(*e.bus, REG_STATUS);
  Serial.printf("[%s] ", e.name);
  if (!(status & STATUS_MD)) {
    Serial.println("responding, but NO MAGNET detected -- check alignment.");
  } else {
    Serial.print("Magnet detected.");
    if (status & STATUS_ML) Serial.print(" Too weak, move magnet closer.");
    if (status & STATUS_MH) Serial.print(" Too strong, move magnet farther.");
    Serial.println();
  }
  Serial.printf("[%s] AGC: %u  Magnitude: %u\n", e.name,
                readReg8(*e.bus, REG_AGC), readReg16(*e.bus, REG_MAGNITUDE));
  e.lastRaw = readReg16(*e.bus, REG_RAW_ANGLE);
  e.counts = e.lastRaw;
  e.lastPrintCounts = e.counts;
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("===== Dual AS5600 Encoder Test =====");

  uint8_t found = 0;
  for (int i = 0; i < 2; i++) {
    Encoder &e = enc[i];
    e.present = probeEncoder(e);
    if (e.present) {
      Serial.printf("[%s] AS5600 at 0x%02X on SDA=%d SCL=%d\n",
                    e.name, AS5600_ADDR, e.sda, e.scl);
      reportMagnet(e);
      found++;
    } else {
      Serial.printf("[%s] NOT RESPONDING on SDA=%d SCL=%d (either order) -- "
                    "check VCC/GND/wiring\n", e.name, e.sda, e.scl);
    }
  }
  Serial.printf("%u of 2 encoders responding; re-probing missing ones every %lu ms.\n",
                found, (unsigned long)RECHECK_DT_MS);

  lastPrintMs = millis();
  lastRecheckMs = millis();
}

// ================= Loop =================
void loop() {
  // Poll every pass so fast spins never skip more than half a rev between
  // reads (unwrap breaks past that).
  int raws[2];
  for (int i = 0; i < 2; i++) {
    Encoder &e = enc[i];
    if (!e.present) {
      raws[i] = -1;
      continue;       // nothing to read; do not invent a count
    }
    int raw = readReg16(*e.bus, REG_RAW_ANGLE);
    int delta = raw - e.lastRaw;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    e.counts += delta;
    e.lastRaw = raw;
    raws[i] = raw;
  }

  // Re-probe anything missing, so a reconnected or replaced encoder comes back without
  // a reboot. Costs one address ping per bus every RECHECK_DT_MS.
  if (millis() - lastRecheckMs >= RECHECK_DT_MS) {
    lastRecheckMs = millis();
    for (int i = 0; i < 2; i++) {
      Encoder &e = enc[i];
      if (e.present) continue;
      if (probeEncoder(e)) {
        e.present = true;
        Serial.printf("[%s] came back on SDA=%d SCL=%d\n", e.name, e.sda, e.scl);
        reportMagnet(e);
      }
    }
  }

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_DT_MS) {
    float dt_s = (now - lastPrintMs) / 1000.0f;
    for (int i = 0; i < 2; i++) {
      Encoder &e = enc[i];
      if (!e.present) {
        // A distinct line, not a row of zeros: zeros are indistinguishable from a wheel
        // that simply is not turning, which is the one reading that must never be faked.
        Serial.printf("%-5s NOT_RESPONDING\n", e.name);
        continue;
      }
      float angleDeg = raws[i] * 360.0f / 4096.0f;
      float revs = e.counts / 4096.0f;
      float rpm = ((e.counts - e.lastPrintCounts) / 4096.0f) / dt_s * 60.0f;
      Serial.printf("%-5s angle=%6.1f deg  revs=%8.3f  rpm=%7.1f  raw=%4d  || wheel_revs=%8.3f  wheel_rpm=%6.1f\n",
                    enc[i].name, angleDeg, revs, rpm, raws[i], revs / GEAR_RATIO, rpm / GEAR_RATIO);
      e.lastPrintCounts = e.counts;
    }
    lastPrintMs = now;
  }
}
