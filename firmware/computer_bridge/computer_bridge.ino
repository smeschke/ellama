// ===== Llama Robot: Computer Bridge (ESP-NOW <-> USB serial) =====
//
// Sits on a bench/PC USB port and relays between ESP-NOW and serial in both directions:
// telemetry from the robot comes out on serial, and drive commands typed in on serial go
// out over the air.
//
// Flash this onto a spare ESP32 plugged into a computer. It broadcasts, so any
// robot_motor_bts7960.ino on the same channel picks up its drive commands with no MAC
// pairing needed.
//
// ---- Receiving telemetry ----
//
// Two packet families arrive here, told apart by length:
//
//   SensorPacket (41 bytes, LlamaSensors.h) -- the current scheme. The packet carries its
//     own name, so this bridge needs no table of sensors and no change when a new one is
//     added. Printed as "<NAME> <value>... <ms>", e.g.
//        TOF_FRONT 812 -1 48213
//     Senders: robot_tof_vl53l0x.ino, and everything else as it is converted.
//
//   EncoderPacket (12) / ImuPacket (16) -- the OLD length-is-the-type scheme, kept so the
//     encoder and IMU boards keep working without being reflashed. Printed exactly as
//     before, "ENC <left> <right> <ms>" and "IMU <ax> <ay> <az> <gx> <gy> <gz> <ms>", so
//     calibration/bridge.py and live_view.py are unaffected.
//
// Those two are mutually exclusive per sender, not per bridge: a sender either uses the
// old struct or the new one. Both paths exist here only for the changeover. Once the
// encoder and IMU sketches send SensorPacket, delete the legacy structs and their two
// branches -- nothing else here depends on them.
//
// Drive commands overheard from other senders (the stick) still print as
// "CMD <left> <right>", so a PC-side recorder can log what the human commanded.
//
// Packet lengths: Drive = 4, Encoder = 12 (legacy), Imu = 16 (legacy), Sensor = 41.
//
// ---- Printing happens in loop(), not in the receive callback ----
//
// esp_now_register_recv_cb's callback runs on the WiFi task. A Serial.printf() there
// blocks until the UART drains, which stalls the callback and silently drops incoming
// packets -- and the symptom is telemetry that gets choppier as you add sensors, which
// looks like a radio problem and is not. So the callback only copies the frame into a
// ring buffer, and loop() does the formatting and printing. Overruns are counted and
// reported rather than hidden.
//
// ---- Transmitting drive commands ----
//
// Type "<left> <right>" into the Serial Monitor (e.g. "255 255", "-255 0", "0 0" or just
// "stop") and it streams that DrivePacket every SEND_INTERVAL_MS, so you can watch
// ramping, direction and stop behavior live.
//
// Listen-only by default: after boot this bridge transmits NOTHING, so it can sit on the
// same channel as controller_stick.ino and just relay telemetry while you drive by hand.
// It only starts streaming once it receives a "<left> <right>" or "stop" line. Send
// "listen" to go silent again (robot_motor_bts7960.ino then fails safe after its
// CMD_TIMEOUT_MS unless something else is driving).
//
// Never send a drive command while the stick is also driving -- they fight on the same
// channel and the motor board follows whichever DrivePacket arrived last.
//
// Fails safe to listen-only if no serial line arrives for SERIAL_IDLE_TIMEOUT_MS while
// transmitting -- e.g. the controlling script dies or the USB link drops. Without this
// the last DrivePacket would repeat forever, since transmitting only ever changes when a
// serial line comes in.
//
// ---- Serial rate ----
//
// 115200 baud is ~11.5 kB/s, and a busy robot (50 Hz encoders + 50 Hz IMU + ToF) puts
// roughly 10 kB/s of text through it, so it is close to full. Raising BAUD_RATE to 921600
// gives 8x headroom -- but calibration/bridge.py's BAUD_RATE must be changed to match in
// the same commit, or the PC side reads garbage.

#include <WiFi.h>
#include <esp_now.h>
#include <LlamaSensors.h>

const uint32_t BAUD_RATE = 115200;  // must match calibration/bridge.py's BAUD_RATE

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t SEND_INTERVAL_MS = 20; // well under robot_motor_bts7960.ino's CMD_TIMEOUT_MS (300ms)

typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

// ---- Legacy packets, for senders not yet converted to SensorPacket. ----
// Must match robot_encoders_as5600.ino's copy exactly (12 bytes).
typedef struct __attribute__((packed)) {
  int32_t left;   // cumulative unwrapped AS5600 counts (4096 counts = 1 encoder-shaft rev)
  int32_t right;
  uint32_t ms;    // sender's millis() at send time
} EncoderPacket;

// Must match robot_imu_*.ino's copy exactly (16 bytes) -- see that file for field meaning.
typedef struct __attribute__((packed)) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint32_t ms;
} ImuPacket;

int cmdL = 0, cmdR = 0;
bool transmitting = false; // false = listen-only; set by a serial drive command, cleared by "listen"
uint32_t lastSendMs = 0;
uint32_t lastRxMs = 0;

// If we're transmitting and no serial line has arrived in this long, fail safe to
// listen-only. Covers the case a controlling process (script, MCP server) dies or the
// USB link drops while a drive command is still active.
const uint32_t SERIAL_IDLE_TIMEOUT_MS = 500;

// ================= receive ring buffer =================
// Written only by the ESP-NOW callback (WiFi task), read only by loop(). Single producer,
// single consumer, so the two indices need no lock -- but they must be volatile, since
// each side only ever advances its own.
const uint8_t RX_FRAME_CAP = sizeof(SensorPacket);  // the largest frame we decode
const uint8_t RX_QUEUE_LEN = 16;                    // ~2.5 s of ToF, ~0.3 s of 50 Hz telemetry

struct RxFrame {
  uint8_t len;
  uint8_t data[RX_FRAME_CAP];
};

RxFrame rxQueue[RX_QUEUE_LEN];
volatile uint8_t rxHead = 0;   // next slot the callback will write
volatile uint8_t rxTail = 0;   // next slot loop() will read
volatile uint32_t rxDropped = 0;
uint32_t rxDroppedReported = 0;

// Frames that are not ours. The bridge hears every ESP-NOW broadcast on the channel, and
// in a room with other ESP-NOW devices that is a continuous stream -- measured at ~77/s
// here. Printing a line each would swamp the serial link and bury the real telemetry, so
// they are counted and summarised at most once a second.
uint32_t foreignFrames = 0;
uint32_t foreignReported = 0;
uint32_t lastForeignReportMs = 0;
const uint32_t FOREIGN_REPORT_DT_MS = 1000;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // no-op; add a counter here if you need send-failure debugging
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  // Keep this function short and non-blocking: no Serial, no formatting.
  if (len <= 0 || len > RX_FRAME_CAP) {
    rxDropped++;   // not a packet we decode, or too big to store
    return;
  }
  uint8_t next = (rxHead + 1) % RX_QUEUE_LEN;
  if (next == rxTail) {
    rxDropped++;   // queue full: loop() is behind, most likely blocked on the UART
    return;
  }
  rxQueue[rxHead].len = (uint8_t)len;
  memcpy(rxQueue[rxHead].data, data, len);
  rxHead = next;
}

// Format and print one received frame. Runs in loop(), where blocking is fine.
void printFrame(const RxFrame& f) {
  if (f.len == sizeof(SensorPacket)) {
    SensorPacket p;
    memcpy(&p, f.data, sizeof(p));
    // A foreign frame can be exactly this long by coincidence; without this check it
    // would print as a sensor reading with a junk name and meaningless values, which the
    // PC would then treat as real data.
    if (!sensorPacketPlausible(&p)) {
      foreignFrames++;
      return;
    }
    // A name from a half-flashed or mis-wired board could still contain odd bytes, which
    // would corrupt this whitespace-delimited line and desync the PC parser.
    char name[SENSOR_NAME_LEN];
    sensorPacketSafeName(&p, name);
    uint8_t count = (p.count > SENSOR_MAX_VALUES) ? SENSOR_MAX_VALUES : p.count;
    Serial.print(name);
    for (uint8_t i = 0; i < count; i++) Serial.printf(" %ld", (long)p.values[i]);
    Serial.printf(" %lu\n", (unsigned long)p.ms);
    return;
  }

  if (f.len == sizeof(ImuPacket)) {           // legacy
    ImuPacket p;
    memcpy(&p, f.data, sizeof(p));
    Serial.printf("IMU %d %d %d %d %d %d %lu\n",
                  p.ax, p.ay, p.az, p.gx, p.gy, p.gz, (unsigned long)p.ms);
    return;
  }

  if (f.len == sizeof(EncoderPacket)) {       // legacy
    EncoderPacket p;
    memcpy(&p, f.data, sizeof(p));
    Serial.printf("ENC %ld %ld %lu\n", (long)p.left, (long)p.right, (unsigned long)p.ms);
    return;
  }

  if (f.len == sizeof(DrivePacket)) {
    // The stick's (or anyone's) drive command, heard on the same channel. ESP-NOW never
    // delivers a sender its own packets, so this is never our own transmission. Lets the
    // computer log what the human commanded alongside the telemetry.
    DrivePacket p;
    memcpy(&p, f.data, sizeof(p));
    Serial.printf("CMD %d %d\n", p.left, p.right);
    return;
  }

  // An unrecognised length: unrelated ESP-NOW traffic on this channel, or a struct that
  // changed on one side only. Counted rather than printed per frame -- see foreignFrames.
  foreignFrames++;
}

void drainRxQueue() {
  while (rxTail != rxHead) {
    printFrame(rxQueue[rxTail]);
    rxTail = (rxTail + 1) % RX_QUEUE_LEN;
  }
  uint32_t dropped = rxDropped;
  if (dropped != rxDroppedReported) {
    Serial.printf("DROPPED %lu\n", (unsigned long)(dropped - rxDroppedReported));
    rxDroppedReported = dropped;
  }

  // Summarise foreign traffic periodically: still visible (a struct mismatch would show
  // up as a sudden non-zero count), without one line per frame.
  uint32_t now = millis();
  if (foreignFrames != foreignReported && now - lastForeignReportMs >= FOREIGN_REPORT_DT_MS) {
    lastForeignReportMs = now;
    Serial.printf("FOREIGN %lu\n", (unsigned long)(foreignFrames - foreignReported));
    foreignReported = foreignFrames;
  }
}

void readSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  lastRxMs = millis(); // any line at all counts as the link being alive

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
  Serial.begin(BAUD_RATE);
  delay(300);
  Serial.println("\n--- Llama Computer Bridge ---");
  Serial.printf("Relaying SensorPacket (%u bytes) by name, plus legacy ENC/IMU.\n",
                (unsigned)sizeof(SensorPacket));
  Serial.println("Type: <left> <right>   e.g. 255 255 | -255 0 | -255 255");
  Serial.println("Type 'stop' or press enter with nothing typed to stop.");
  Serial.println("Listen-only until a drive command is sent; type 'listen' to go silent again.");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(1);
  Serial.printf("WiFi MAC: %s\n", WiFi.macAddress().c_str());

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

  lastRxMs = millis();
}

void loop() {
  readSerialCommand();
  drainRxQueue();

  uint32_t now = millis();

  if (transmitting && now - lastRxMs > SERIAL_IDLE_TIMEOUT_MS) {
    transmitting = false;
    cmdL = 0; cmdR = 0;
    Serial.println("-> serial idle timeout, falling back to listen-only");
  }

  if (transmitting && now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    DrivePacket p{ (int16_t)cmdL, (int16_t)cmdR };
    esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p));
  }
}
