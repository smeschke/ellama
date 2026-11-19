// ===== Llama Robot: Computer Bridge (ESP-NOW <-> USB serial) v2 =====
//
// Framing v2 (breaks v1 text format on purpose):
//   Every telemetry line is checksummed and sequenced so a torn USB line is
//   dropped instead of becoming a fake sensor:
//
//     @ <seq> <NAME> <fields...> *<CC>\n
//
//   <seq> is a bridge-global uint32 incremented per printed frame (wraps).
//   <CC> is two uppercase hex digits of XOR over every byte of
//   "<seq> <NAME> <fields...>" (i.e. everything between "@ " and " *").
//   NAME is ENC, IMU, TOF_FRONT, ..., CMD, DROPPED, FOREIGN.
//   Fields are identical to v1: ENC/IMU/SENSOR values then sender ms,
//   CMD left right, DROPPED/FOREIGN a single count.
//
//   Example: "@ 123 TOF_FRONT 812 -1 48213 *5A"
//
// Why: at 115200 baud ~10 kB/s of text through an 11.5 kB/s link, torn
// lines were routine and the PC invented sensors like "MU" from a split
// "IMU" line. v2 runs at 921600 baud with a 64-frame queue and rejects
// any line whose checksum fails, so the registry no longer needs to
// distrust first sightings as hard (SAMPLES_TO_TRUST stays as a belt).
//
// Serial downlink (unchanged): "<left> <right>", "stop", "listen",
// plus "status" (prints queue/seq/baud stats, stays listen-only).
// Listen-only by default; transmitting fails safe to listen-only after
// SERIAL_IDLE_TIMEOUT_MS with no serial line. Only one drive sender at a
// time: the motor follows the last DrivePacket it hears.

#include <WiFi.h>
#include <esp_now.h>
#include <LlamaSensors.h>

// v2 serial rate: 8x headroom over v1's 115200. The Python side must use
// the same value (bridge.BAUD_RATE). Reflash + restart both together.
const uint32_t BAUD_RATE = 921600;

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t SEND_INTERVAL_MS = 20; // well under robot_motor_bts7960.ino's CMD_TIMEOUT_MS (300ms)

typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

// ---- Legacy packets, for senders not yet converted to SensorPacket. ----
typedef struct __attribute__((packed)) {
  int32_t left;
  int32_t right;
  uint32_t ms;
} EncoderPacket;

typedef struct __attribute__((packed)) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint32_t ms;
} ImuPacket;

int cmdL = 0, cmdR = 0;
bool transmitting = false;
uint32_t lastSendMs = 0;
uint32_t lastRxMs = 0;

const uint32_t SERIAL_IDLE_TIMEOUT_MS = 500;

// ================= receive ring buffer =================
const uint8_t RX_FRAME_CAP = sizeof(SensorPacket);
const uint8_t RX_QUEUE_LEN = 64;   // v2: ~1.3 s of 50 Hz telemetry, was 16

struct RxFrame {
  uint8_t len;
  uint8_t data[RX_FRAME_CAP];
};

RxFrame rxQueue[RX_QUEUE_LEN];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint32_t rxDropped = 0;
uint32_t rxDroppedReported = 0;

uint32_t foreignFrames = 0;
uint32_t foreignReported = 0;
uint32_t lastForeignReportMs = 0;
const uint32_t FOREIGN_REPORT_DT_MS = 1000;

uint32_t txSeq = 0;  // bridge-global frame counter for v2 lines

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len <= 0 || len > RX_FRAME_CAP) {
    rxDropped++;
    return;
  }
  uint8_t next = (rxHead + 1) % RX_QUEUE_LEN;
  if (next == rxTail) {
    rxDropped++;
    return;
  }
  rxQueue[rxHead].len = (uint8_t)len;
  memcpy(rxQueue[rxHead].data, data, len);
  rxHead = next;
}

// XOR checksum over the payload text between "@ " and " *".
uint8_t frameChecksum(const char* payload) {
  uint8_t cs = 0;
  for (const char* p = payload; *p; p++) cs ^= (uint8_t)*p;
  return cs;
}

// Print one v2 line: "@ <seq> <payload> *<CC>". payload is already built
// without the "@ " prefix. Runs in loop(), where blocking is fine.
void emitLine(const char* payload) {
  char line[192];
  snprintf(line, sizeof(line), "%lu %s", (unsigned long)txSeq++, payload);
  Serial.printf("@ %s *%02X\n", line, frameChecksum(line));
}

void printFrame(const RxFrame& f) {
  char payload[160];
  if (f.len == sizeof(SensorPacket)) {
    SensorPacket p;
    memcpy(&p, f.data, sizeof(p));
    if (!sensorPacketPlausible(&p)) {
      foreignFrames++;
      return;
    }
    char name[SENSOR_NAME_LEN];
    sensorPacketSafeName(&p, name);
    uint8_t count = (p.count > SENSOR_MAX_VALUES) ? SENSOR_MAX_VALUES : p.count;
    int n = snprintf(payload, sizeof(payload), "%s", name);
    for (uint8_t i = 0; i < count && n < (int)sizeof(payload) - 14; i++)
      n += snprintf(payload + n, sizeof(payload) - n, " %ld", (long)p.values[i]);
    snprintf(payload + n, sizeof(payload) - n, " %lu", (unsigned long)p.ms);
    emitLine(payload);
    return;
  }

  if (f.len == sizeof(ImuPacket)) {
    ImuPacket p;
    memcpy(&p, f.data, sizeof(p));
    snprintf(payload, sizeof(payload), "IMU %d %d %d %d %d %d %lu",
             p.ax, p.ay, p.az, p.gx, p.gy, p.gz, (unsigned long)p.ms);
    emitLine(payload);
    return;
  }

  if (f.len == sizeof(EncoderPacket)) {
    EncoderPacket p;
    memcpy(&p, f.data, sizeof(p));
    snprintf(payload, sizeof(payload), "ENC %ld %ld %lu",
             (long)p.left, (long)p.right, (unsigned long)p.ms);
    emitLine(payload);
    return;
  }

  if (f.len == sizeof(DrivePacket)) {
    DrivePacket p;
    memcpy(&p, f.data, sizeof(p));
    snprintf(payload, sizeof(payload), "CMD %d %d", p.left, p.right);
    emitLine(payload);
    return;
  }

  foreignFrames++;
}

void drainRxQueue() {
  while (rxTail != rxHead) {
    printFrame(rxQueue[rxTail]);
    rxTail = (rxTail + 1) % RX_QUEUE_LEN;
  }
  uint32_t dropped = rxDropped;
  if (dropped != rxDroppedReported) {
    char payload[64];
    snprintf(payload, sizeof(payload), "DROPPED %lu",
             (unsigned long)(dropped - rxDroppedReported));
    emitLine(payload);
    rxDroppedReported = dropped;
  }

  uint32_t now = millis();
  if (foreignFrames != foreignReported && now - lastForeignReportMs >= FOREIGN_REPORT_DT_MS) {
    lastForeignReportMs = now;
    char payload[64];
    snprintf(payload, sizeof(payload), "FOREIGN %lu",
             (unsigned long)(foreignFrames - foreignReported));
    emitLine(payload);
    foreignReported = foreignFrames;
  }
}

void printStatus() {
  Serial.printf("@ status seq=%lu dropped=%lu foreign=%lu q=%u/%u baud=%lu *00\n",
                (unsigned long)txSeq, (unsigned long)rxDropped,
                (unsigned long)foreignFrames,
                (unsigned)((rxHead + RX_QUEUE_LEN - rxTail) % RX_QUEUE_LEN),
                (unsigned)RX_QUEUE_LEN, (unsigned long)BAUD_RATE);
}

void readSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  lastRxMs = millis();

  if (line.equalsIgnoreCase("listen")) {
    transmitting = false;
    cmdL = 0; cmdR = 0;
    Serial.println("@ status listen-only *00");
    return;
  }

  if (line.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }

  if (line.length() == 0 || line.equalsIgnoreCase("stop")) {
    cmdL = 0; cmdR = 0;
    transmitting = true;
    Serial.println("@ status transmitting-stop *00");
    return;
  }

  int spaceIdx = line.indexOf(' ');
  if (spaceIdx < 0) {
    Serial.println("@ status bad-format *00");
    return;
  }

  int l = line.substring(0, spaceIdx).toInt();
  int r = line.substring(spaceIdx + 1).toInt();
  cmdL = constrain(l, -255, 255);
  cmdR = constrain(r, -255, 255);
  transmitting = true;
  Serial.printf("@ status driving %d %d *00\n", cmdL, cmdR);
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(300);
  Serial.println("@ status bridge-v2 *00");
  Serial.printf("@ status sensor-packet=%u baud=%lu q=%u *00\n",
                (unsigned)sizeof(SensorPacket),
                (unsigned long)BAUD_RATE, (unsigned)RX_QUEUE_LEN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("@ status esp-now-init-failed *00");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer{};
  peer.channel = 1;
  peer.encrypt = false;
  memcpy(peer.peer_addr, broadcastMac, 6);
  esp_now_add_peer(&peer);

  lastRxMs = millis();
}

void loop() {
  readSerialCommand();
  drainRxQueue();

  uint32_t now = millis();

  if (transmitting && now - lastRxMs > SERIAL_IDLE_TIMEOUT_MS) {
    transmitting = false;
    cmdL = 0; cmdR = 0;
    Serial.println("@ status serial-idle-timeout *00");
  }

  if (transmitting && now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    DrivePacket p{ (int16_t)cmdL, (int16_t)cmdR };
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }
}
