// ===== Llama Robot: GY-ICM20945 V2 IMU Bench Test =====
//
// Standalone sanity check for a GY-ICM20945 V2 breakout (ICM-20945/20948-
// family accel+gyro+mag) wired to an ESP32 over I2C. Same role as
// bench_imu_mpu6050_test.ino for the (now dead) GY-521/MPU6050 board -- confirm the chip
// is alive and well-mounted before trusting it broadcasting blind on the
// robot. No libraries beyond Wire.h -- talks to the chip directly over its
// I2C register map, same style as bench_imu_mpu6050_test.ino/bench_encoders_as5600_test.ino.
//
// This is NOT a drop-in register-compatible swap for the MPU6050: the
// ICM-20945 splits its registers across 4 banks (selected via REG_BANK_SEL),
// WHO_AM_I lives at a different address and returns a different value
// (0xEA, not MPU6050's 0x68), and the accel+gyro burst-read block is a
// different address/length. Scale factors (LSB/g, LSB/dps) happen to match
// the MPU6050 test's values exactly, since both parts use a 16-bit ADC with
// the same +-2g/+-250dps full-scale conventions.
//
// Wiring: GY-ICM20945 VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22,
// AD0->GND (selects I2C address 0x68; tie AD0 high instead for 0x69 if
// that's already taken on this bus), NCS->3.3V (REQUIRED -- this chip
// defaults to SPI mode unless chip-select is held high; some breakouts have
// an onboard pull-up on NCS and some don't, so wire it explicitly rather
// than leaving it floating). FSYNC and INT are unused here -- leave
// unconnected. EDA/ECL are the chip's internal aux-I2C pins (used
// internally to reach the onboard AK09916 magnetometer, or to cascade
// further external I2C sensors through it) -- also unconnected, since this
// test only reads accel+gyro, same scope as the MPU6050 test it replaces.
//
// Note: this board has a real magnetometer on it (AK09916, reachable via
// the aux-I2C bus above), which the MPU6050 never had. Reading it would let
// yaw reference true heading instead of drifting from gyro integration
// alone -- worth revisiting later, but out of scope for this bench test.
//
// Keep the board still and level for the first ~1s after boot -- that's
// the gyro bias calibration window (see calibrateGyroBias()). Then rock it
// by hand and watch pitch/roll track; yaw will drift over time since
// there's no magnetometer reference wired up yet, which is expected.

#include <Wire.h>

// ================= I2C pins =================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= ICM-20945 address & registers =================
const uint8_t ICM_ADDR = 0x68; // AD0->GND; use 0x69 if AD0 is tied high instead

const uint8_t REG_BANK_SEL = 0x7F; // present in every bank, selects the active one

// Bank 0
const uint8_t REG_WHO_AM_I     = 0x00;
const uint8_t REG_PWR_MGMT_1   = 0x06;
const uint8_t REG_PWR_MGMT_2   = 0x07;
const uint8_t REG_ACCEL_XOUT_H = 0x2D; // 12 contiguous bytes: accel x/y/z, gyro x/y/z
                                        // (no temp registers in between, unlike MPU6050)
// Bank 2
const uint8_t REG_GYRO_CONFIG_1 = 0x01;
const uint8_t REG_ACCEL_CONFIG  = 0x14;

const uint8_t WHO_AM_I_EXPECTED = 0xEA;

// ================= Scale factors (register configs left at power-on default: =================
// accel +-2g, gyro +-250 deg/s -- see setup(), written explicitly for clarity)
const float ACCEL_LSB_PER_G  = 16384.0f;
const float GYRO_LSB_PER_DPS = 131.0f;

// ================= Settings =================
const uint32_t PRINT_DT_MS = 50; // ~20 Hz status output
const int GYRO_BIAS_SAMPLES = 200; // ~1s at the raw-read rate below, robot must be still
const float COMPLEMENTARY_ALPHA = 0.98f; // weight on gyro-integrated angle vs. accel angle

// ================= State =================
uint8_t currentBank = 0xFF; // invalid sentinel -- forces a bank-select on the first access
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0; // deg/s, subtracted from every gyro read
float pitch = 0, roll = 0, yaw = 0; // deg; yaw is gyro-integration only, will drift
uint32_t lastPrintMs = 0;
uint32_t lastSampleUs = 0;

void selectBank(uint8_t bank) {
  if (bank == currentBank) return;
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_BANK_SEL);
  Wire.write(bank << 4);
  Wire.endTransmission();
  currentBank = bank;
}

void writeReg8(uint8_t bank, uint8_t reg, uint8_t val) {
  selectBank(bank);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg8(uint8_t bank, uint8_t reg) {
  selectBank(bank);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ICM_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

// Reads the 12-byte accel+gyro burst starting at REG_ACCEL_XOUT_H (bank 0).
void readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  selectBank(0);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ICM_ADDR, 12);

  uint8_t buf[12];
  for (int i = 0; i < 12 && Wire.available(); i++) buf[i] = Wire.read();

  ax = (buf[0] << 8) | buf[1];
  ay = (buf[2] << 8) | buf[3];
  az = (buf[4] << 8) | buf[5];
  gx = (buf[6] << 8) | buf[7];
  gy = (buf[8] << 8) | buf[9];
  gz = (buf[10] << 8) | buf[11];
}

// Averages GYRO_BIAS_SAMPLES raw gyro reads to find the at-rest offset.
// Board MUST be still and unshaken during this -- any motion here bakes a
// wrong bias into every reading for the rest of the run.
void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias -- keep the board still...");
  int16_t ax, ay, az, gx, gy, gz;
  double sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    readRaw(ax, ay, az, gx, gy, gz);
    sumX += gx; sumY += gy; sumZ += gz;
    delay(5);
  }
  gyroBiasX = (sumX / GYRO_BIAS_SAMPLES) / GYRO_LSB_PER_DPS;
  gyroBiasY = (sumY / GYRO_BIAS_SAMPLES) / GYRO_LSB_PER_DPS;
  gyroBiasZ = (sumZ / GYRO_BIAS_SAMPLES) / GYRO_LSB_PER_DPS;
  Serial.print("Gyro bias (deg/s): x="); Serial.print(gyroBiasX, 3);
  Serial.print(" y="); Serial.print(gyroBiasY, 3);
  Serial.print(" z="); Serial.println(gyroBiasZ, 3);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("===== GY-ICM20945 V2 IMU Test =====");

  uint8_t whoAmI = readReg8(0, REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x"); Serial.println(whoAmI, HEX);
  if (whoAmI != WHO_AM_I_EXPECTED) {
    Serial.println("WARNING: unexpected WHO_AM_I -- check wiring/address (AD0 pin), "
                    "and that NCS is tied to 3.3V (I2C mode, not SPI).");
  }

  writeReg8(0, REG_PWR_MGMT_1, 0x01); // wake from sleep, auto-select best clock source
  delay(50);
  writeReg8(0, REG_PWR_MGMT_2, 0x00); // enable all accel+gyro axes
  writeReg8(2, REG_GYRO_CONFIG_1, 0x00);  // +-250 deg/s
  writeReg8(2, REG_ACCEL_CONFIG, 0x00);   // +-2g

  calibrateGyroBias();

  lastPrintMs = millis();
  lastSampleUs = micros();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);

  uint32_t nowUs = micros();
  float dt_s = (nowUs - lastSampleUs) / 1000000.0f;
  lastSampleUs = nowUs;

  float accelGx = ax / ACCEL_LSB_PER_G;
  float accelGy = ay / ACCEL_LSB_PER_G;
  float accelGz = az / ACCEL_LSB_PER_G;
  float gyroDpsX = gx / GYRO_LSB_PER_DPS - gyroBiasX;
  float gyroDpsY = gy / GYRO_LSB_PER_DPS - gyroBiasY;
  float gyroDpsZ = gz / GYRO_LSB_PER_DPS - gyroBiasZ;

  // Accel-only tilt estimate (only valid when not accelerating linearly --
  // that's exactly what the gyro half of the filter is for).
  float accPitch = atan2(-accelGx, sqrt(accelGy * accelGy + accelGz * accelGz)) * 180.0f / PI;
  float accRoll  = atan2(accelGy, accelGz) * 180.0f / PI;

  pitch = COMPLEMENTARY_ALPHA * (pitch + gyroDpsY * dt_s) + (1 - COMPLEMENTARY_ALPHA) * accPitch;
  roll  = COMPLEMENTARY_ALPHA * (roll  + gyroDpsX * dt_s) + (1 - COMPLEMENTARY_ALPHA) * accRoll;
  yaw  += gyroDpsZ * dt_s; // no magnetometer reference wired up -- integration only, will drift

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_DT_MS) {
    lastPrintMs = now;
    Serial.print("accel(g)="); Serial.print(accelGx, 2); Serial.print(",");
    Serial.print(accelGy, 2); Serial.print(","); Serial.print(accelGz, 2);
    Serial.print("  gyro(dps)="); Serial.print(gyroDpsX, 1); Serial.print(",");
    Serial.print(gyroDpsY, 1); Serial.print(","); Serial.print(gyroDpsZ, 1);
    Serial.print("  || pitch="); Serial.print(pitch, 1);
    Serial.print(" roll="); Serial.print(roll, 1);
    Serial.print(" yaw="); Serial.println(yaw, 1);
  }
}
