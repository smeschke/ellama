// ===== Llama Robot: Time-of-Flight Range Sender (2x VL53L0X, ESP-NOW) =====
//
// Runs on a robot ESP32-C3 carrying two VL53L0X range sensors. Reads them one at a time
// and broadcasts both distances in a single SensorPacket over ESP-NOW. The bridge
// (computer_bridge.ino) relays it to the PC as "TOF_FRONT <mm_a> <mm_b> <ms>".
//
// Bring-up, pin assignment and the measurement schedule are identical to
// bench_tof_vl53l0x_test.ino -- debug there over USB first, then flash this. This sketch
// also prints a USB status line, so it stays readable over the cable while transmitting.
//
// Wiring (ESP32-C3, shared I2C bus) -- MAC ac:27:6e:45:c5:9c is the front board:
//   SDA -> GPIO4, SCL -> GPIO3   (both sensors)
//   XSHUT: sensor A -> GPIO2, sensor B -> GPIO9
//   VCC -> 3.3V, GND -> GND
//
// ---- What goes on the wire ----
//
// One packet per cycle carrying BOTH sensors, not one packet each. An ESP-NOW packet
// costs roughly the same airtime whatever its size, because framing and preamble
// dominate, so two sensors in one packet is about half the airtime of two packets. See
// LlamaSensors.h. Values are millimetres, in sensors[] order:
//
//   values[0] = sensor A distance, mm   (SENSOR_VALUE_INVALID if it could not measure)
//   values[1] = sensor B distance, mm
//
// A sensor that is present but cannot range sends SENSOR_VALUE_INVALID rather than a
// plausible-looking number, so the PC can tell "nothing in range" from "board is dead".
// The sensor's raw 8191 mm no-return sentinel never reaches the wire.
//
// SENSOR_NAME identifies this board. Change it to "TOF_REAR" before flashing the second
// ToF board -- that string is the only difference between the two, and because the name
// travels inside the packet there is no id table anywhere to keep in step.
//
// ---- Measurement schedule ----
//
// Each sensor fires alone and the bus goes quiet for INTER_SENSOR_GAP_MS before the next
// laser fires, including after the last one so the wrap-around gets dead time too. Two
// VL53L0X units with overlapping fields of view otherwise see each other's pulses and
// report short distances or refuse to range. 50 ms is more settling time than strictly
// needed; it is cheap here because the packet rate is set by the cycle, and a full cycle
// of 2 x (33 ms budget + 50 ms gap) is ~166 ms, so this board transmits at ~6 Hz. Lower
// the gap if you want a faster update and the readings stay clean.
//
// IMPORTANT: Adafruit_VL53L0X must never be copied -- it holds a pointer to its own
// member, so a copy drives the ORIGINAL's device state and both sensors silently share
// one device struct. Hence named globals and pointers in sensors[].
//
// Also: clearInterruptMask() after every measurement is load-bearing. The data-ready
// flag survives a measurement that timed out, and the ST API then returns the PREVIOUS
// reading instantly -- reporting success after a 2 ms "measurement" and serving stale
// data forever after one timeout. Keep the timing budget near the 33 ms default too; a
// much longer budget exhausts the API's fixed-iteration wait loop and causes exactly
// that timeout.
//
// NOTE: GPIO2 and GPIO9 are ESP32-C3 strapping pins (GPIO9 is BOOT). The breakouts pull
// XSHUT high, so the board still boots -- but suspect these pins first if an upload ever
// fails. GPIO5/6/7/10 are free.

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_VL53L0X.h>
#include <LlamaSensors.h>
#include "esp_mac.h"

// ================= identity =================
// "TOF_REAR" on the second ToF board. Must fit in SENSOR_NAME_LEN - 1 characters.
static const char SENSOR_NAME[] = "TOF_FRONT";

// ================= I2C pins =================
#define PIN_SDA 4
#define PIN_SCL 3

// ================= schedule =================
const uint32_t INTER_SENSOR_GAP_MS = 50;   // quiet time between one laser and the next
const uint32_t TIMING_BUDGET_US = 33000;   // keep near the library default -- see above
const uint32_t STATUS_DT_MS = 1000;        // USB status line, for watching over the cable

// ================= ESP-NOW =================
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t ESPNOW_CHANNEL = 1;          // same channel as every other sketch here

// ================= sensors =================
// One named instance per sensor -- never copied.
Adafruit_VL53L0X devA;
Adafruit_VL53L0X devB;

struct Sensor {
  const char* name;
  uint8_t xshut;            // GPIO holding this sensor in reset until it's its turn
  uint8_t addr;             // address assigned at boot; must be unique and not 0x29
  Adafruit_VL53L0X* dev;    // pointer, never a copy
  bool ok;                  // false if it never came up
  int32_t mm;               // last distance, or SENSOR_VALUE_INVALID
  // Diagnostics, printed on the USB status line only -- never transmitted.
  uint16_t rawMm;           // what the sensor reported, before the validity check
  uint8_t status;           // RangeStatus: why a reading was rejected
  float sig, amb;           // return signal / ambient IR, MCps
};

Sensor sensors[] = {
  { "A", 2, 0x30, &devA, false, SENSOR_VALUE_INVALID, 0, 0, 0, 0 },
  { "B", 9, 0x31, &devB, false, SENSOR_VALUE_INVALID, 0, 0, 0, 0 },
};
const uint8_t N_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// What counts as a usable reading is the sensor's PHYSICAL range, nothing else.
//
// ST's RangeStatus is deliberately not consulted. Its limit checks are tuned for
// metrology: they reject a measurement whose signal-to-noise is poor, which happens
// constantly in ordinary light, and the rejected reading is usually a perfectly good
// distance. Honouring the status made every packet read -1 even while the bench viewer,
// which draws the raw millimetres regardless, showed sensible numbers.
//
// A signal-rate threshold was the obvious next idea and is worse: the signal rate depends
// on ambient IR, so any constant is calibrated to the room it was measured in -- near zero
// in the dark, ten times higher in sunlight. This robot goes anywhere, so that constant
// would miss real obstacles in one place and invent them in another.
//
// So the wire carries any distance inside the sensor's range, and the PC decides what to
// trust -- the same split robot_encoders_as5600.ino uses for gear ratio and wheel size,
// and for the same reason: it can be retuned without pulling a board off the robot. The
// bound still excludes the 8190/8191 mm no-return sentinel, and 0 mm is excluded because
// the driver reports it when a measurement did not really happen.
//
// The cost is honest: noisy and occasional short readings reach the PC, where a median
// over the last few samples removes them. st/sig/amb stay on the USB status line for
// debugging, but no longer gate what is transmitted.
const int32_t RANGE_MIN_MM = 1;
const int32_t RANGE_MAX_MM = 2000;         // the sensor's ceiling

uint32_t packetsSent = 0, sendFailures = 0;
uint32_t lastStatusMs = 0;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) sendFailures++;
}

void scanBus(const char* when) {
  Serial.printf("I2C scan (%s):", when);
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02x", a);
      found++;
    }
  }
  if (!found) Serial.print(" nothing");
  Serial.println();
}

// Wake the sensors one at a time, giving each its own address before releasing the next.
// The VL53L0X's 0x29 is fixed in hardware and only changeable in software, so they would
// collide if brought up together. The reassignment lives in RAM: it runs every boot.
void bringUpSensors() {
  for (uint8_t i = 0; i < N_SENSORS; i++) {
    pinMode(sensors[i].xshut, OUTPUT);
    digitalWrite(sensors[i].xshut, LOW);
  }
  delay(20);
  scanBus("all in reset");

  for (uint8_t i = 0; i < N_SENSORS; i++) {
    Sensor& s = sensors[i];
    digitalWrite(s.xshut, HIGH);
    delay(50);  // boot + firmware load before it answers on 0x29

    s.ok = s.dev->begin(s.addr, false, &Wire);
    if (s.ok) {
      // The long-range profile lowers the final-range signal-rate limit and lengthens
      // the VCSEL pulse. Without it, these sensors measure a target correctly but the
      // default limit check rejects the result as "signal fail" (RangeStatus 2) whenever
      // ambient IR is high -- around 1.5 MCps of background here, which is enough to
      // push the final-range SNR under the default threshold. The distance was always
      // being computed; it was only ever being thrown away.
      // configSensor() installs its own timing budget, so set ours afterwards.
      bool cfg = s.dev->configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
      s.dev->setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
      Serial.printf("sensor %s: up, moved to 0x%02x (XSHUT GPIO%u), long-range cfg=%d\n",
                    s.name, s.addr, s.xshut, cfg ? 1 : 0);
    } else {
      // Left awake on 0x29 on failure, which would collide with the next sensor's
      // bring-up -- so put it back into reset and carry on with the others.
      Serial.printf("sensor %s: NOT FOUND on XSHUT GPIO%u -- check wiring/power\n",
                    s.name, s.xshut);
      digitalWrite(s.xshut, LOW);
    }
  }
  scanBus("after bring-up");
}

// Fire one sensor's laser with nothing else ranging, and store the result.
void measure(Sensor& s) {
  if (!s.ok) {
    s.mm = SENSOR_VALUE_INVALID;
    return;
  }

  VL53L0X_RangingMeasurementData_t m;
  VL53L0X_Error err = s.dev->rangingTest(&m, false);  // fires the laser; blocks

  // Never let a failed measurement leave the data-ready flag set -- see the header note.
  if (err != VL53L0X_ERROR_NONE) s.dev->stopMeasurement();
  s.dev->clearInterruptMask(false);

  int32_t mm = (int32_t)m.RangeMilliMeter;
  bool valid = (err == VL53L0X_ERROR_NONE) && (mm >= RANGE_MIN_MM) && (mm <= RANGE_MAX_MM);
  s.mm = valid ? mm : SENSOR_VALUE_INVALID;
  s.rawMm = m.RangeMilliMeter;
  s.status = m.RangeStatus;                        // USB diagnostics only
  s.sig = m.SignalRateRtnMegaCps / 65536.0f;       // FixPoint1616 -> float
  s.amb = m.AmbientRateRtnMegaCps / 65536.0f;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n--- Llama ToF Sender (%s) ---\n", SENSOR_NAME);
  // Read from eFuse rather than WiFi.macAddress(), which returns all zeros until the
  // WiFi stack is up -- and the stack is started further down, after the sensors.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("WiFi MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("I2C: SDA=GPIO%d SCL=GPIO%d\n", PIN_SDA, PIN_SCL);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  bringUpSensors();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // sleep would drop incoming/outgoing ESP-NOW frames
  WiFi.setChannel(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed -- will keep measuring but not transmit");
    return;
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  memcpy(peer.peer_addr, broadcastMac, 6);
  esp_now_add_peer(&peer);

  uint8_t up = 0;
  for (uint8_t i = 0; i < N_SENSORS; i++) if (sensors[i].ok) up++;
  Serial.printf("%u of %u sensors up. Broadcasting SensorPacket \"%s\" (%u bytes) "
                "on channel %u.\n",
                up, N_SENSORS, SENSOR_NAME, (unsigned)sizeof(SensorPacket), ESPNOW_CHANNEL);
  Serial.printf("One laser at a time, %lu ms quiet between them; -1 mm = could not "
                "measure.\n", (unsigned long)INTER_SENSOR_GAP_MS);
  Serial.printf("Sending any distance in %ld..%ld mm; RangeStatus is diagnostics only.\n",
                (long)RANGE_MIN_MM, (long)RANGE_MAX_MM);

  lastStatusMs = millis();
}

void loop() {
  // Read every sensor in turn, with dead air after each -- including the last, so the
  // wrap-around back to sensor A has a gap too.
  for (uint8_t i = 0; i < N_SENSORS; i++) {
    measure(sensors[i]);
    delay(INTER_SENSOR_GAP_MS);
  }

  // Both distances in one packet: one send instead of two, for about half the airtime.
  SensorPacket p;
  sensorPacketInit(&p, SENSOR_NAME, N_SENSORS, millis());
  for (uint8_t i = 0; i < N_SENSORS; i++) p.values[i] = sensors[i].mm;
  if (esp_now_send(broadcastMac, (uint8_t*)&p, sizeof(p)) == ESP_OK) packetsSent++;
  else sendFailures++;

  uint32_t now = millis();
  if (now - lastStatusMs >= STATUS_DT_MS) {
    lastStatusMs = now;
    Serial.printf("%s", SENSOR_NAME);
    for (uint8_t i = 0; i < N_SENSORS; i++) {
      Serial.printf(" | %s ", sensors[i].name);
      Sensor& s = sensors[i];
      if (!s.ok) {
        Serial.print("absent");
        continue;
      }
      if (s.mm == SENSOR_VALUE_INVALID) Serial.printf("rej raw=%5u", s.rawMm);
      else                              Serial.printf("%4ld mm %5.1f in", (long)s.mm, s.mm / 25.4f);
      Serial.printf(" st%u sig %4.2f amb %4.2f", s.status, s.sig, s.amb);
    }
    Serial.printf(" | sent=%lu fail=%lu\n",
                  (unsigned long)packetsSent, (unsigned long)sendFailures);
  }
}
