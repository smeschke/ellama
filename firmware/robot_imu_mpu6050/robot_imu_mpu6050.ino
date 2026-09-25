// ===== Llama Robot: GY-521 (MPU6050) IMU Sender (ESP-NOW) =====
//
// Runs on the IMU ESP32, temporarily taped to the robot for bench testing.
// Same MPU6050 read/bias-calibration logic as bench_imu_mpu6050_test.ino, but instead
// of only printing over USB it also broadcasts an ImuPacket over ESP-NOW
// every PRINT_DT_MS. The bridge ESP32 (computer_bridge.ino) listens for these
// and relays them over serial to the PC, alongside encoder and drive
// traffic.
//
// Wiring: GY-521 VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22, AD0->GND.
//
// Keep the robot still for ~1s after boot -- that's the gyro bias
// calibration window (see calibrateGyroBias() in bench_imu_mpu6050_test.ino, same
// logic here). The packet sends bias-corrected raw gyro counts and
// uncorrected raw accel counts; scale factors (ACCEL_LSB_PER_G,
// GYRO_LSB_PER_DPS) live in bench_imu_mpu6050_test.ino's comments and are applied on
// the PC side, same pattern robot_encoders_as5600.ino uses for gear ratio --
// keep the firmware dumb, tune scale/calibration without reflashing.

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

// ================= I2C pins =================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= MPU6050 registers =================
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I   = 0x75;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

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
const uint32_t PRINT_DT_MS = 20; // ~50 Hz status output AND ESP-NOW send rate
const int GYRO_BIAS_SAMPLES = 200;

// ================= State =================
int16_t gyroBiasXRaw = 0, gyroBiasYRaw = 0, gyroBiasZRaw = 0; // raw LSB, subtracted before send
uint32_t lastPrintMs = 0;

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
  gx = (buf[8] << 8) | buf[9];
  gy = (buf[10] << 8) | buf[11];
  gz = (buf[12] << 8) | buf[13];
}

void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias -- keep the robot still...");
  int16_t ax, ay, az, gx, gy, gz;
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    readRaw(ax, ay, az, gx, gy, gz);
    sumX += gx; sumY += gy; sumZ += gz;
    delay(5);
  }
  gyroBiasXRaw = sumX / GYRO_BIAS_SAMPLES;
  gyroBiasYRaw = sumY / GYRO_BIAS_SAMPLES;
  gyroBiasZRaw = sumZ / GYRO_BIAS_SAMPLES;
  Serial.print("Gyro bias (raw LSB): x="); Serial.print(gyroBiasXRaw);
  Serial.print(" y="); Serial.print(gyroBiasYRaw);
  Serial.print(" z="); Serial.println(gyroBiasZRaw);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("===== GY-521 IMU Sender =====");

  uint8_t whoAmI = readReg8(REG_WHO_AM_I);
  if (whoAmI != 0x68) {
    Serial.print("WARNING: unexpected WHO_AM_I 0x"); Serial.println(whoAmI, HEX);
  }

  writeReg8(REG_PWR_MGMT_1, 0x00);
  delay(50);
  writeReg8(REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  writeReg8(REG_ACCEL_CONFIG, 0x00); // +-2g

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
  int16_t ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);

  int16_t gxCorr = gx - gyroBiasXRaw;
  int16_t gyCorr = gy - gyroBiasYRaw;
  int16_t gzCorr = gz - gyroBiasZRaw;

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_DT_MS) {
    lastPrintMs = now;

    Serial.print("accel(raw)="); Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(","); Serial.print(az);
    Serial.print("  gyro(dps)="); Serial.print(gxCorr / GYRO_LSB_PER_DPS, 1); Serial.print(",");
    Serial.print(gyCorr / GYRO_LSB_PER_DPS, 1); Serial.print(",");
    Serial.println(gzCorr / GYRO_LSB_PER_DPS, 1);

    ImuPacket p{ ax, ay, az, gxCorr, gyCorr, gzCorr, now };
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }
}
