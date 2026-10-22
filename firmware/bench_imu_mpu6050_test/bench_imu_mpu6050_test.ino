// ===== Llama Robot: GY-521 (MPU6050) IMU Bench Test =====
//
// Standalone sanity check for a GY-521 breakout (MPU6050 accel+gyro) wired
// to an ESP32 over I2C. Same role as bench_encoders_as5600_test.ino: confirm the chip is
// alive and well-mounted before trusting it broadcasting blind on the
// robot. No libraries beyond Wire.h -- talks to the MPU6050 directly over
// its I2C register map, same style as bench_encoders_as5600_test.ino's AS5600 reads.
//
// Wiring: GY-521 VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22, AD0->GND
// (selects I2C address 0x68; tie AD0 high instead for 0x69 if that's
// already taken on this bus).
//
// Keep the board still and level for the first ~1s after boot -- that's
// the gyro bias calibration window (see calibrateGyroBias()). Then rock it
// by hand and watch pitch/roll track; yaw will drift over time since
// there's no magnetometer to reference it against, which is expected.
//
// Prints two lines per cycle: the human-readable one below for the Serial Monitor, and a
// "RAW ax=.. ay=.. az=.. gx=.. gy=.. gz=.. pitch=.. roll=.. yaw=.. t=.." line that
// visualize.py in this folder parses into a live display -- same arrangement as
// bench_tof_vl53l0x_test.ino.
//
// The six raw values on that line are in the SAME ORDER, with the same bias correction,
// that robot_imu_gy521.ino puts in its SensorPacket. So what you verify here is exactly
// what goes on the wire, and the orientation maths on the PC side
// (calibration/bridge.py's OrientationState) consumes identical numbers.

#include <Wire.h>

// ================= I2C pins =================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= MPU6050 registers =================
// AD0 low = 0x68, AD0 high = 0x69. Probed at boot rather than hardcoded, because a
// floating or pulled-up AD0 pin is one of the two usual reasons a GY-521 looks dead, and
// robot_imu_gy521.ino already auto-detects -- the bench test should not be the fussier of
// the two.
const uint8_t ADDR_AD0_LOW  = 0x68;
const uint8_t ADDR_AD0_HIGH = 0x69;
uint8_t MPU_ADDR = ADDR_AD0_LOW;   // set by detectAddress()

// Genuine MPU6050 reports 0x68; relabelled GY-521 clones commonly report one of these.
// Same list as robot_imu_gy521.ino.
const uint8_t WHO_AM_I_OK[] = {0x68, 0x70, 0x71, 0x72, 0x73, 0x98};
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I   = 0x75;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // 14 contiguous bytes: accel x/y/z, temp, gyro x/y/z

// ================= Scale factors (register configs left at power-on default: =================
// accel +-2g, gyro +-250 deg/s -- see setup(), written explicitly for clarity)
const float ACCEL_LSB_PER_G   = 16384.0f;
const float GYRO_LSB_PER_DPS  = 131.0f;

// ================= Settings =================
const uint32_t PRINT_DT_MS = 50; // ~20 Hz status output
const int GYRO_BIAS_SAMPLES = 200; // ~1s at the raw-read rate below, robot must be still
const float COMPLEMENTARY_ALPHA = 0.98f; // weight on gyro-integrated angle vs. accel angle

// ================= State =================
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0; // deg/s, subtracted from every gyro read
float pitch = 0, roll = 0, yaw = 0; // deg; yaw is gyro-integration only, will drift
uint32_t lastPrintMs = 0;
uint32_t lastSampleUs = 0;

void writeReg8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

// Reads the 14-byte accel+temp+gyro burst starting at REG_ACCEL_XOUT_H.
void readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14);

  uint8_t buf[14];
  for (int i = 0; i < 14 && Wire.available(); i++) buf[i] = Wire.read();

  ax = (buf[0] << 8) | buf[1];
  ay = (buf[2] << 8) | buf[3];
  az = (buf[4] << 8) | buf[5];
  // buf[6],buf[7] = temperature -- unused here
  gx = (buf[8] << 8) | buf[9];
  gy = (buf[10] << 8) | buf[11];
  gz = (buf[12] << 8) | buf[13];
}

// Averages GYRO_BIAS_SAMPLES raw gyro reads to find the at-rest offset.
// Robot/board MUST be still and unshaken during this -- any motion here
// bakes a wrong bias into every reading for the rest of the run.
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

// List everything answering on the bus. A bench test that only says "WHO_AM_I: 0x0" tells
// you the chip did not answer but not whether ANYTHING is out there -- and "nothing at
// all" (power or wiring) needs a different fix from "something, but at the other address".
void scanBus() {
  Serial.print("I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x"); Serial.print(a, HEX);
      found++;
    }
  }
  if (!found) Serial.print(" nothing");
  Serial.println();
}

bool deviceAt(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool whoAmIRecognised(uint8_t id) {
  for (uint8_t ok : WHO_AM_I_OK) if (id == ok) return true;
  return false;
}

// Returns true and sets MPU_ADDR if a device answers at either address.
bool detectAddress() {
  for (uint8_t addr : {ADDR_AD0_LOW, ADDR_AD0_HIGH}) {
    if (deviceAt(addr)) {
      MPU_ADDR = addr;
      Serial.print("GY-521 found at 0x"); Serial.println(addr, HEX);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("===== GY-521 (MPU6050) IMU Test =====");

  // Find the chip before talking to it: reading WHO_AM_I at a hardcoded address reports
  // 0x0 whether the board is at the other address, unpowered, or absent.
  scanBus();
  if (!detectAddress()) {
    Serial.println("ERROR: nothing at 0x68 or 0x69 -- check VCC/GND/SDA/SCL and AD0.");
    Serial.println("Halting: calibrating a gyro bias from a dead bus produces a bogus");
    Serial.println("offset that silently corrupts every reading afterwards.");
    while (true) delay(1000);
  }

  uint8_t whoAmI = readReg8(REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x"); Serial.print(whoAmI, HEX);
  Serial.println(whoAmIRecognised(whoAmI) ? " (ok)" : " (unrecognised -- continuing anyway)");

  writeReg8(REG_PWR_MGMT_1, 0x00);   // wake from sleep, use internal 8MHz oscillator
  delay(50);
  writeReg8(REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  writeReg8(REG_ACCEL_CONFIG, 0x00); // +-2g

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
  yaw  += gyroDpsZ * dt_s; // no magnetometer -- integration only, will drift over time

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

    // Machine-readable twin for visualize.py. ax..gz are the raw LSBs the packet
    // carries: accel as read (+-2g => 16384 LSB/g), gyro bias-corrected and converted
    // back to LSBs (+-250 dps => 131 LSB/dps), matching robot_imu_gy521.ino exactly.
    // pitch/roll/yaw are this sketch's own filter output, for comparison only -- they
    // are not transmitted; the PC recomputes them from the raw values.
    Serial.printf("RAW ax=%d ay=%d az=%d gx=%d gy=%d gz=%d "
                  "pitch=%.2f roll=%.2f yaw=%.2f t=%lu\n",
                  ax, ay, az,
                  (int)lroundf(gyroDpsX * GYRO_LSB_PER_DPS),
                  (int)lroundf(gyroDpsY * GYRO_LSB_PER_DPS),
                  (int)lroundf(gyroDpsZ * GYRO_LSB_PER_DPS),
                  pitch, roll, yaw, (unsigned long)now);
  }
}
