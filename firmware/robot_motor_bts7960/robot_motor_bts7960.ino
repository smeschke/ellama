// ===== Llama Robot: Motor Receiver (ESP-NOW, LEDC PWM) =====
//
// Runs on the Llama Motor Board.
//
// Wire format: DrivePacket{left, right} -- signed PWM per side, -255..255.
// Works with either a human joystick remote (which does its own arcade-
// drive mixing before sending) or a computer plugged into an ESP32 bridge
// (which can send raw left/right values directly). This receiver does no
// mixing -- it only ramps toward the commanded values (to protect the
// gearboxes from instant direction reversals) and writes PWM to the
// motors via the ESP32 LEDC peripheral. Stopping is just PWM 0; with R_EN/L_EN
// hardwired enabled, that holds both motor terminals near ground through the
// BTS7960 low-side switches, which actively (dynamically) brakes the motor.

#include <WiFi.h>
#include <esp_now.h>

// ================= Motor pins =================

#define FL_RPWM 16
#define FL_LPWM 17
#define BL_RPWM 18
#define BL_LPWM 19
#define FR_RPWM 32
#define FR_LPWM 33
#define BR_RPWM 25
#define BR_LPWM 26

// ================= LEDC settings =================
const int LEDC_FREQ_HZ  = 20000; // above audible range
const int LEDC_RES_BITS = 8;     // 0..255 duty, matches PWM range below

// ================= Settings =================
const int      RAMP_STEP      = 1;   // max PWM change per ramp tick
const uint32_t RAMP_DT_MS     = 3;   // ramp tick period
const uint32_t CMD_TIMEOUT_MS = 300; // stop if no packet received in this long

// ================= Wire format =================
typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

// ================= State =================
int curL = 0, curR = 0;
volatile int tgtL = 0, tgtR = 0;
volatile uint32_t lastRecvMs = 0;
uint32_t lastRampMs = 0;

void setAll(int l, int r) {
  if (l >= 0) {
    ledcWrite(FL_LPWM, l); ledcWrite(FL_RPWM, 0);
    ledcWrite(BL_LPWM, l); ledcWrite(BL_RPWM, 0);
  } else {
    ledcWrite(FL_LPWM, 0); ledcWrite(FL_RPWM, -l);
    ledcWrite(BL_LPWM, 0); ledcWrite(BL_RPWM, -l);
  }

  if (r >= 0) {
    ledcWrite(FR_LPWM, r); ledcWrite(FR_RPWM, 0);
    ledcWrite(BR_LPWM, r); ledcWrite(BR_RPWM, 0);
  } else {
    ledcWrite(FR_LPWM, 0); ledcWrite(FR_RPWM, -r);
    ledcWrite(BR_LPWM, 0); ledcWrite(BR_RPWM, -r);
  }
}

// ================= ESP-NOW receive =================
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(DrivePacket)) return;

  DrivePacket p;
  memcpy(&p, data, sizeof(p));

  tgtL = constrain((int)p.left, -255, 255);
  tgtR = constrain((int)p.right, -255, 255);
  lastRecvMs = millis();
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);

  ledcAttach(FL_LPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(FL_RPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(BL_LPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(BL_RPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(FR_LPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(FR_RPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(BR_LPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(BR_RPWM, LEDC_FREQ_HZ, LEDC_RES_BITS);

  setAll(0, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  Serial.println("Robot Ready.");
}

// ================= Loop =================
void loop() {
  uint32_t now = millis();

  if (now - lastRecvMs > CMD_TIMEOUT_MS) {
    tgtL = 0;
    tgtR = 0;
  }

  if (now - lastRampMs >= RAMP_DT_MS) {
    lastRampMs = now;
    curL += constrain(tgtL - curL, -RAMP_STEP, RAMP_STEP);
    curR += constrain(tgtR - curR, -RAMP_STEP, RAMP_STEP);
    setAll(curL, curR);
  }
}
