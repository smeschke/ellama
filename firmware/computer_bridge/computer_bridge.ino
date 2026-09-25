// ===== Llama Robot: Computer Bridge (ESP-NOW <-> USB serial) =====
//
// Bench tool for robot_motor_bts7960.ino. Type "<left> <right>" into the Serial
// Monitor (e.g. "255 255", "-255 0", "0 0" or just "stop") and it streams
// that DrivePacket to the motor receiver every SEND_INTERVAL_MS, so you can
// watch ramping, direction, and stop behavior live.
//
// Flash this onto a second ESP32 (or the ESP32 bridge you plug into a
// computer) -- sends via ESP-NOW broadcast, so any robot_motor_bts7960.ino
// listening on the same channel picks it up with no MAC pairing needed.
//
// Also relays wheel-encoder telemetry: robot_encoders_as5600.ino (on the
// robot's encoder ESP32) broadcasts an EncoderPacket over the same ESP-NOW
// channel. This bridge listens for it (packet type told apart from
// DrivePacket purely by length, per the firmware README) and prints it to
// Serial as "ENC <left> <right> <ms>" so a PC-side script can capture it
// alongside the drive commands it's sending out.
//
// Also relays IMU telemetry the same way: robot_imu_icm20945.ino (or
// robot_imu_mpu6050.ino) broadcasts an ImuPacket, told apart from
// DrivePacket/EncoderPacket by its own distinct length (16 bytes), printed as
// "IMU <ax> <ay> <az> <gx> <gy> <gz> <ms>".
//
// Packet lengths: Drive = 4, Encoder = 12, Imu = 16.
//
// Listen-only by default: after boot this bridge transmits NOTHING, so it
// can sit on the same channel as controller_stick.ino and just relay
// telemetry while you drive by hand. It only starts streaming a DrivePacket
// every SEND_INTERVAL_MS once it receives a "<left> <right>" or "stop" line
// over serial. Send "listen" to go silent again (robot_motor_bts7960.ino then
// fails safe after CMD_TIMEOUT_MS unless something else is driving).
// Never send a drive command while the stick is also driving -- they'd
// fight on the same channel.

#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t SEND_INTERVAL_MS = 20; // well under robot_motor_bts7960.ino's CMD_TIMEOUT_MS (300ms)

typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

// Must match robot_encoders_as5600.ino's copy exactly (12 bytes).
typedef struct __attribute__((packed)) {
  int32_t left;   // cumulative unwrapped AS5600 counts (4096 counts = 1 encoder-shaft rev)
  int32_t right;
  uint32_t ms;    // sender's millis() at send time
} EncoderPacket;

// Must match robot_imu_*.ino's copy exactly -- see that file for field meaning.
typedef struct __attribute__((packed)) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint32_t ms;
} ImuPacket;

int cmdL = 0, cmdR = 0;
bool transmitting = false; // false = listen-only; set by a serial drive command, cleared by "listen"
uint32_t lastSendMs = 0;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // no-op; add Serial.println(status) here if you need send-failure debugging
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == sizeof(EncoderPacket)) {
    EncoderPacket p;
    memcpy(&p, data, sizeof(p));
    Serial.printf("ENC %ld %ld %lu\n", (long)p.left, (long)p.right, (unsigned long)p.ms);
  } else if (len == sizeof(ImuPacket)) {
    ImuPacket p;
    memcpy(&p, data, sizeof(p));
    Serial.printf("IMU %d %d %d %d %d %d %lu\n",
                  p.ax, p.ay, p.az, p.gx, p.gy, p.gz, (unsigned long)p.ms);
  }
  // anything else (e.g. a stray DrivePacket echo) is ignored
}

void readSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.equalsIgnoreCase("listen")) {
    transmitting = false;
    cmdL = 0; cmdR = 0;
    Serial.println("-> listen-only (not transmitting)");
    return;
  }

  if (line.length() == 0 || line.equalsIgnoreCase("stop")) {
    cmdL = 0; cmdR = 0;
    transmitting = true;
    Serial.println("-> stop (transmitting 0 0)");
    return;
  }

  int spaceIdx = line.indexOf(' ');
  if (spaceIdx < 0) {
    Serial.println("format: <left> <right>   e.g. 255 -255, 'stop', or 'listen'");
    return;
  }

  int l = line.substring(0, spaceIdx).toInt();
  int r = line.substring(spaceIdx + 1).toInt();
  cmdL = constrain(l, -255, 255);
  cmdR = constrain(r, -255, 255);
  transmitting = true;
  Serial.printf("-> left=%d right=%d\n", cmdL, cmdR);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- Motor Receiver Test Sender ---");
  Serial.println("Type: <left> <right>   e.g. 255 255 | -255 0 | -255 255");
  Serial.println("Type 'stop' or press enter with nothing typed to stop.");
  Serial.println("Listen-only until a drive command is sent; type 'listen' to go silent again.");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer{};
  peer.channel = 1;
  peer.encrypt = false;
  memcpy(peer.peer_addr, broadcastMac, 6);
  esp_now_add_peer(&peer);
}

void loop() {
  readSerialCommand();

  uint32_t now = millis();
  if (transmitting && now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    DrivePacket p{ (int16_t)cmdL, (int16_t)cmdR };
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }
}
