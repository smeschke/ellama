// ===== Llama Robot: GY-521 IMU Sender (ESP-NOW) =====
//
// Runs on the IMU ESP32. Dedicated GY-521 breakout sender -- a hardened
// rewrite of robot_imu_mpu6050.ino. Same 16-byte ImuPacket, same ESP-NOW
// broadcast, same scale factors, so computer_bridge.ino and the PC side need
// no changes. What's different from robot_imu_mpu6050.ino:
//   - auto-detects the I2C address (0x68 with AD0->GND, 0x69 with AD0->3.3V)
//   - accepts the WHO_AM_I values common GY-521 clones report (0x68, 0x70,
//     0x71, 0x72, 0x73, 0x98) instead of warning on anything but 0x68
//   - full device reset + PLL clock source at init (more stable than the
//     internal 8 MHz oscillator the chip wakes up on)
//   - enables the on-chip digital low-pass filter (~44 Hz) to cut motor
//     vibration noise before it reaches the packet
//   - checks every I2C read; a failed read is skipped (no garbage packet)
//     and the sensor is re-initialised after repeated failures, e.g. a
//     loose jumper while the robot is driving
//   - gyro bias calibration rejects the window and retries if the robot is
//     moved during it
//
// Wiring: GY-521 VCC->3.3V (the board's onboard regulator also accepts 5V),
// GND->GND, SDA->GPIO21, SCL->GPIO22, AD0->GND (address 0x68). XDA, XCL and
// INT unconnected.
//
// Keep the robot still for ~1s after boot -- that's the gyro bias
// calibration window (see calibrateGyroBias()). The packet sends
// bias-corrected raw gyro counts and uncorrected raw accel counts; scale
// factors (ACCEL_LSB_PER_G = 16384, GYRO_LSB_PER_DPS = 131) are applied on
// the PC side, same pattern robot_encoders_as5600.ino uses for gear ratio --
// keep the firmware dumb, tune scale/calibration without reflashing.

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

// ================= I2C pins =================
#define SDA_PIN 22
#define SCL_PIN 21

// ================= MPU6050 registers =================
const uint8_t ADDR_AD0_LOW  = 0x68;
const uint8_t ADDR_AD0_HIGH = 0x69;

const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B; // 14 contiguous bytes: accel x/y/z, temp, gyro x/y/z
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

// Genuine MPU6050 = 0x68; clone/relabelled GY-521 boards often carry an
// MPU6500/9250/6886-family die that answers with one of the others. All of
// them share the register layout used below.
const uint8_t WHO_AM_I_OK[] = {0x68, 0x70, 0x71, 0x72, 0x73, 0x98};

const float GYRO_LSB_PER_DPS = 131.0f; // +-250 deg/s, see REG_GYRO_CONFIG below

// ================= ESP-NOW =================
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// NOTE: this struct's layout and size must match the copy in
// computer_bridge.ino exactly -- packet type is told apart purely by length
// (4 = DrivePacket, 12 = EncoderPacket, 16 = ImuPacket).
typedef struct __attribute__((packed)) {
  int16_t ax, ay, az; // raw accel LSB, +-2g full scale, gravity included (not bias-calibrated)
  int16_t gx, gy, gz; // raw gyro LSB, +-250 deg/s full scale, bias-corrected at boot
  uint32_t ms;         // this ESP32's millis() at send time
} ImuPacket;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // no-op; add Serial.println(status) here if you need send-failure debugging
}

// ================= Settings =================
const uint32_t PRINT_DT_MS = 20;       // ~50 Hz status output AND ESP-NOW send rate
const int GYRO_BIAS_SAMPLES = 200;
const int GYRO_STILL_MAX_SPREAD = 300; // raw LSB (~2.3 deg/s); larger min-max spread = robot moved
const int GYRO_BIAS_MAX_TRIES = 5;
const int MAX_READ_FAILS = 10;         // consecutive failed reads before re-init

// ================= State =================
uint8_t imuAddr = ADDR_AD0_LOW;
int16_t gyroBiasXRaw = 0, gyroBiasYRaw = 0, gyroBiasZRaw = 0; // raw LSB, subtracted before send
int readFails = 0;
uint32_t lastPrintMs = 0;

bool writeReg8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readReg8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)imuAddr, 1) != 1) return false;
  val = Wire.read();
  return true;
}

bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool whoAmIRecognised(uint8_t id) {
  for (uint8_t ok : WHO_AM_I_OK) if (id == ok) return true;
  return false;
}

// Returns false if the burst read came back short -- caller should skip the sample.
bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(imuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)imuAddr, 14) != 14) return false;

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  // buf[6..7] = temperature, unused
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

// Finds the GY-521 on the bus and configures it. Returns false if no sensor answers.
bool initImu() {
  if (probe(ADDR_AD0_LOW))       imuAddr = ADDR_AD0_LOW;
  else if (probe(ADDR_AD0_HIGH)) imuAddr = ADDR_AD0_HIGH;
  else {
    Serial.println("ERROR: no GY-521 at 0x68 or 0x69 -- check VCC/GND/SDA/SCL wiring.");
    return false;
  }
  Serial.print("GY-521 found at 0x"); Serial.println(imuAddr, HEX);

  uint8_t whoAmI = 0;
  readReg8(REG_WHO_AM_I, whoAmI);
  Serial.print("WHO_AM_I = 0x"); Serial.print(whoAmI, HEX);
  Serial.println(whoAmIRecognised(whoAmI) ? " (ok)" : " (unrecognised -- continuing anyway)");

  writeReg8(REG_PWR_MGMT_1, 0x80);   // device reset
  delay(100);
  writeReg8(REG_PWR_MGMT_1, 0x01);   // wake, clock = PLL with X-gyro reference
  delay(50);
  writeReg8(REG_CONFIG, 0x03);       // DLPF ~44 Hz accel / ~42 Hz gyro, 1 kHz internal rate
  writeReg8(REG_SMPLRT_DIV, 0x04);   // 1 kHz / (1+4) = 200 Hz sample rate
  writeReg8(REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  writeReg8(REG_ACCEL_CONFIG, 0x00); // +-2g
  delay(50);                         // let the filter settle before calibrating

  readFails = 0;
  return true;
}

// Averages GYRO_BIAS_SAMPLES readings; retries if the robot wasn't held still.
void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias -- keep the robot still...");
  for (int attempt = 1; attempt <= GYRO_BIAS_MAX_TRIES; attempt++) {
    int16_t ax, ay, az, gx, gy, gz;
    long sumX = 0, sumY = 0, sumZ = 0;
    int16_t minX = INT16_MAX, minY = INT16_MAX, minZ = INT16_MAX;
    int16_t maxX = INT16_MIN, maxY = INT16_MIN, maxZ = INT16_MIN;
    int n = 0;

    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
      if (readRaw(ax, ay, az, gx, gy, gz)) {
        sumX += gx; sumY += gy; sumZ += gz;
        minX = min(minX, gx); maxX = max(maxX, gx);
        minY = min(minY, gy); maxY = max(maxY, gy);
        minZ = min(minZ, gz); maxZ = max(maxZ, gz);
        n++;
      }
      delay(5);
    }

    bool still = n > GYRO_BIAS_SAMPLES / 2
              && (maxX - minX) < GYRO_STILL_MAX_SPREAD
              && (maxY - minY) < GYRO_STILL_MAX_SPREAD
              && (maxZ - minZ) < GYRO_STILL_MAX_SPREAD;

    if (still || attempt == GYRO_BIAS_MAX_TRIES) {
      if (n > 0) {
        gyroBiasXRaw = sumX / n;
        gyroBiasYRaw = sumY / n;
        gyroBiasZRaw = sumZ / n;
      }
      if (!still) Serial.println("WARNING: robot never held still -- using last bias estimate.");
      Serial.print("Gyro bias (raw LSB): x="); Serial.print(gyroBiasXRaw);
      Serial.print(" y="); Serial.print(gyroBiasYRaw);
      Serial.print(" z="); Serial.println(gyroBiasZRaw);
      return;
    }
    Serial.print("Movement detected during calibration, retrying ("); Serial.print(attempt);
    Serial.print("/"); Serial.print(GYRO_BIAS_MAX_TRIES); Serial.println(")...");
  }
}

int16_t subtractBias(int16_t raw, int16_t bias) {
  return (int16_t)constrain((int32_t)raw - bias, INT16_MIN, INT16_MAX);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("===== GY-521 IMU Sender =====");

  while (!initImu()) delay(1000); // keep retrying until the sensor shows up
  calibrateGyroBias();

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
    Serial.println("ESP-NOW broadcasting ImuPacket on channel 1.");
  }

  lastPrintMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastPrintMs < PRINT_DT_MS) return;
  lastPrintMs = now;

  int16_t ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) {
    if (++readFails >= MAX_READ_FAILS) {
      Serial.println("GY-521 not responding -- re-initialising (bias kept).");
      Wire.end();
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(400000);
      initImu();
    }
    return; // don't send a stale/garbage packet
  }
  readFails = 0;

  int16_t gxCorr = subtractBias(gx, gyroBiasXRaw);
  int16_t gyCorr = subtractBias(gy, gyroBiasYRaw);
  int16_t gzCorr = subtractBias(gz, gyroBiasZRaw);

  Serial.print("accel(raw)="); Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(","); Serial.print(az);
  Serial.print("  gyro(dps)="); Serial.print(gxCorr / GYRO_LSB_PER_DPS, 1); Serial.print(",");
  Serial.print(gyCorr / GYRO_LSB_PER_DPS, 1); Serial.print(",");
  Serial.println(gzCorr / GYRO_LSB_PER_DPS, 1);

  ImuPacket p{ ax, ay, az, gxCorr, gyCorr, gzCorr, now };
  esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
}
