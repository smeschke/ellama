// ===== Llama Robot: Dual Time-of-Flight Range Bench Test (2x VL53L0X, Serial only) =====
//
// Standalone sanity check for two VL53L0X range sensors on one I2C bus.
// USB-tethered, Serial-only, no ESP-NOW -- the same role bench_encoders_as5600_test.ino
// plays for the AS5600s. Flash it, open the Serial Monitor at 115200, wave your hand in
// front of each sensor and watch its distance move.
//
// Wiring (ESP32-C3, shared I2C bus):
//   SDA -> GPIO4, SCL -> GPIO3   (both sensors)
//   XSHUT: sensor A -> GPIO2, sensor B -> GPIO9
//   VCC -> 3.3V, GND -> GND
//
// ---- Measurement schedule (one laser at a time, with dead air between) ----
//
// Each sensor fires alone and reports its own line as soon as it has a reading, then the
// bus goes quiet for INTER_SENSOR_GAP_MS before the next laser fires:
//
//   [A fires, TIMING_BUDGET_US] -> "A ..." -> 50 ms quiet
//   [B fires, TIMING_BUDGET_US] -> "B ..." -> 50 ms quiet  -> repeat
//
// So A always reports first and B reports roughly 50 ms + one timing budget later, and
// there is never less than 50 ms between one sensor's laser going quiet and the next
// one's firing. Two VL53L0X units with overlapping fields of view can otherwise see each
// other's pulses and report implausibly short distances, or refuse to range at all; the
// gap also lets the previous measurement's photons and the I2C traffic settle. The gap
// after the LAST sensor matters too -- without it the wrap-around from B back to A would
// have no dead time.
//
// Every line carries gap=<ms>, measured as the real quiet time between the previous
// sensor's laser stopping and this one's starting, so you can confirm the schedule is
// actually being honoured rather than trusting the constant.
//
// The two knobs, and how they trade:
//   TIMING_BUDGET_US   longer = more photons per measurement, so weak returns (dark or
//                      steeply angled targets) resolve instead of coming back as signal
//                      fail -- but slower.
//   INTER_SENSOR_GAP_MS  longer = more isolation between the two sensors -- but slower.
// At the defaults below a full cycle is 2 x (100 ms + 50 ms) = 300 ms, so each sensor
// reports at about 3.3 Hz. Change one at a time when diagnosing.
//
// ---- The VL53L0X address dance ----
//
// The VL53L0X's I2C address is 0x29 and, unlike the AS5600's, it CAN be changed -- but
// only in software, and only while the other sensors are held in reset. So both start
// shut down, then each is woken one at a time and moved to its own address (A -> 0x30,
// B -> 0x31) before the next one is released. This runs every boot: the reassignment
// lives in RAM, not in the sensor, so a power cycle puts both back at 0x29.
//
// That XSHUT dance is also why two sensors need only one bus, where the two AS5600s
// needed two. It scales to three or more sensors -- add a device, a row to sensors[],
// and a spare GPIO.
//
// IMPORTANT: Adafruit_VL53L0X must never be copied. It holds
// `VL53L0X_Dev_t MyDevice; VL53L0X_Dev_t* pMyDevice = &MyDevice;` -- a pointer to its own
// member -- and the implicit copy constructor copies that pointer verbatim, so a copy
// ends up driving the ORIGINAL's device state. Putting instances directly in an array
// initializer silently does exactly that, and the symptom is subtle: both sensors come up
// fine and answer on their own addresses, but every reading comes back weak and identical
// between them, because both objects are really talking to one device struct. Hence the
// devices are named globals here and sensors[] holds pointers to them.
//
// NOTE: GPIO2 and GPIO9 are ESP32-C3 strapping pins (GPIO9 is BOOT). VL53L0X breakouts
// normally pull XSHUT high on-board, so the chip still boots -- but if an upload ever
// fails or the board drops into download mode on its own, these pins are the first
// suspect. GPIO5/6/7/10 are free if you want to move them.
//
// Output is one line per sensor per cycle, in key=value form so it reads fine in the
// Serial Monitor and parses trivially in visualize.py in this folder:
//   A mm=812 in=32.0 st=0 sig=3.21 amb=1.05 t=48213 gap=50 dur=35 err=0 ok=1
// dur= is how long the measurement actually took and should track TIMING_BUDGET_US; a
// dur far below it means the API returned stale data instead of ranging. err= is the
// VL53L0X_Error (0 = success, -7 = timeout).
// mm is the raw reading and is printed even when the status rejects it, so you can see
// what the sensor thought it saw; ok=1 only when st=0 and the distance is in range.
//
// Also prints this ESP32's WiFi MAC at boot, since the robot sketches identify boards by
// MAC when you need to tell two of them apart.

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "esp_mac.h"

// ================= I2C pins =================
#define PIN_SDA 4
#define PIN_SCL 3

// ================= schedule =================
// Quiet time after each sensor's measurement, before the next laser fires.
const uint32_t INTER_SENSOR_GAP_MS = 50;

// Integration window per measurement. The ST API waits for completion by polling a
// fixed number of times rather than against a clock, so a budget far above the 33 ms
// default can exhaust that loop and return VL53L0X_ERROR_TIME_OUT (-7) even though the
// sensor is fine. 33 ms is the library default and completes well inside the loop.
// Raise it for weak returns only if dur= in the output still tracks it.
const uint32_t TIMING_BUDGET_US = 33000;

// ================= sensors =================
// One named instance per sensor -- see the no-copy warning above.
Adafruit_VL53L0X devA;
Adafruit_VL53L0X devB;

struct Sensor {
  const char* name;
  uint8_t xshut;            // GPIO holding this sensor in reset until it's its turn
  uint8_t addr;             // address assigned at boot; must be unique and not 0x29
  Adafruit_VL53L0X* dev;    // pointer, never a copy
  bool ok;                  // false if it never came up
};

Sensor sensors[] = {
  { "A", 2, 0x30, &devA, false },
  { "B", 9, 0x31, &devB, false },
};
const uint8_t N_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// Only RangeStatus 0 is a real measurement. Adafruit's examples test for "!= 4" (phase
// fail), which lets the 8191 mm no-return sentinel through as if it were a distance --
// so check for 0 instead, and sanity-cap at the sensor's 2 m ceiling.
const uint8_t RANGE_STATUS_VALID = 0;
const uint16_t RANGE_MAX_MM = 2000;

// When any sensor's laser last stopped. Used to measure the real gap, not assume it.
uint32_t lastMeasureEndMs = 0;

void printMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("WiFi MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
void bringUpSensors() {
  // Everyone into reset first, so the bus has nothing on 0x29.
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

    // begin() talks to the sensor at 0x29 and leaves it on s.addr.
    s.ok = s.dev->begin(s.addr, false, &Wire);
    if (s.ok) {
      // Long-range profile: lowers the final-range signal-rate limit and lengthens the
      // VCSEL pulse. With the default profile these sensors range correctly but the limit
      // check rejects it as signal fail (st2) under the ambient IR in this room.
      // configSensor() installs its own timing budget, so set ours afterwards.
      s.dev->configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
      s.dev->setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
      Serial.printf("sensor %s: up, moved to 0x%02x (XSHUT GPIO%u), timing budget %lu us\n",
                    s.name, s.addr, s.xshut,
                    (unsigned long)s.dev->getMeasurementTimingBudgetMicroSeconds());
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

// Fire one sensor's laser, with nothing else ranging, and report what came back.
void measureAndReport(Sensor& s) {
  uint32_t startMs = millis();
  // Real quiet time since the previous sensor's laser stopped. On the very first
  // measurement there is no previous one, so this is just time since boot.
  uint32_t gapMs = startMs - lastMeasureEndMs;

  VL53L0X_RangingMeasurementData_t m;
  // rangingTest() returns a VL53L0X_Error -- 0 is success, negative is a fault. Ignoring
  // it hides the worst failure mode: on an I2C error the ST API gives up waiting for the
  // measurement and hands back whatever is in the struct, which reads as a plausible-
  // looking distance. A measurement that returns far faster than the timing budget never
  // happened, so dur= is printed alongside to make that visible.
  VL53L0X_Error err = s.dev->rangingTest(&m, false);
  lastMeasureEndMs = millis();
  uint32_t durMs = lastMeasureEndMs - startMs;

  // The data-ready flag survives a measurement that timed out or was abandoned, and the
  // ST API's wait-for-completion loop then sees it already set and returns the PREVIOUS
  // reading immediately -- reporting err=0 after a 2 ms "measurement". One early timeout
  // therefore serves stale, one-cycle-late data forever after, which looks like flaky
  // sensors rather than a stuck flag. Clearing the interrupt after every measurement
  // keeps each read independent; a hard error also gets an explicit stop first.
  if (err != VL53L0X_ERROR_NONE) s.dev->stopMeasurement();
  s.dev->clearInterruptMask(false);

  float sig = m.SignalRateRtnMegaCps / 65536.0f;   // FixPoint1616 -> float
  float amb = m.AmbientRateRtnMegaCps / 65536.0f;
  bool valid = (m.RangeStatus == RANGE_STATUS_VALID) && (m.RangeMilliMeter <= RANGE_MAX_MM);

  // sig/amb are the diagnostic pair: sig ~0 = nothing returning (blocked lens, dead
  // VCSEL, target too far or too dark); amb high = ambient IR swamping the return.
  Serial.printf("%s mm=%u in=%.1f st=%u sig=%.2f amb=%.2f t=%lu gap=%lu dur=%lu err=%d ok=%d\n",
                s.name, m.RangeMilliMeter, m.RangeMilliMeter / 25.4f, m.RangeStatus,
                sig, amb, (unsigned long)startMs, (unsigned long)gapMs,
                (unsigned long)durMs, (int)err, valid ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- Dual VL53L0X Bench Test ---");
  printMac();
  Serial.printf("I2C: SDA=GPIO%d SCL=GPIO%d\n", PIN_SDA, PIN_SCL);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  bringUpSensors();

  uint8_t up = 0;
  for (uint8_t i = 0; i < N_SENSORS; i++) if (sensors[i].ok) up++;
  if (!up) {
    Serial.println("No sensors came up. Nothing to report -- fix the wiring and reset.");
  } else {
    Serial.printf("%u of %u sensors up. One laser at a time, %lu ms quiet between them.\n",
                  up, N_SENSORS, (unsigned long)INTER_SENSOR_GAP_MS);
    Serial.println("st: 0=valid 1=sigma fail 2=signal fail 3=min range 4=no return 5=hw fail");
    Serial.println("sig/amb in MCps: sig ~0 = nothing coming back, amb high = ambient IR.");
    Serial.println("Hold a hand or a wall within 2 m of a sensor to see it read.");
  }
  Serial.println();

  lastMeasureEndMs = millis();
}

void loop() {
  // A, then a gap, then B, then a gap. The trailing gap is what keeps dead air between
  // B's laser and A's on the next pass round.
  for (uint8_t i = 0; i < N_SENSORS; i++) {
    Sensor& s = sensors[i];
    if (!s.ok) continue;
    measureAndReport(s);
    delay(INTER_SENSOR_GAP_MS);
  }
}
