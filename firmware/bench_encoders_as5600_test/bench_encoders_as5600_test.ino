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
#define SDA_L 21
#define SCL_L 22
#define SDA_R 18
#define SCL_R 19

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
  int32_t counts;      // cumulative 12-bit counts, no wraparound
  int32_t lastPrintCounts;
  int lastRaw;
};
Encoder enc[2] = { {"left", &Wire, 0, 0, -1}, {"right", &Wire1, 0, 0, -1} };

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

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_L, SCL_L, 400000);
  Wire1.begin(SDA_R, SCL_R, 400000);

  Serial.println("===== Dual AS5600 Encoder Test =====");

  for (int i = 0; i < 2; i++) {
    Encoder &e = enc[i];
    uint8_t status = readReg8(*e.bus, REG_STATUS);
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
                  readReg8(*e.bus, REG_AGC), readReg16(*e.bus, REG_MAGNITUDE));

    e.lastRaw = readReg16(*e.bus, REG_RAW_ANGLE);
    e.counts = e.lastRaw;
    e.lastPrintCounts = e.counts;
  }
  lastPrintMs = millis();
}

// ================= Loop =================
void loop() {
  // Poll every pass so fast spins never skip more than half a rev between
  // reads (unwrap breaks past that).
  int raws[2];
  for (int i = 0; i < 2; i++) {
    Encoder &e = enc[i];
    int raw = readReg16(*e.bus, REG_RAW_ANGLE);
    int delta = raw - e.lastRaw;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    e.counts += delta;
    e.lastRaw = raw;
    raws[i] = raw;
  }

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_DT_MS) {
    float dt_s = (now - lastPrintMs) / 1000.0f;
    for (int i = 0; i < 2; i++) {
      Encoder &e = enc[i];
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
