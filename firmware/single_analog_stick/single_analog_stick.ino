// ===== Llama Robot: Single-Stick Controller (ESP-NOW sender) =====
//
// Reads a KY-023-style dual-axis analog joystick (VRx, VRy, SW) and mixes
// it into left/right drive values using arcade-style mixing, then sends a
// DrivePacket{left, right} to the robot over ESP-NOW broadcast. This wire
// format must exactly match motor_receiver.ino's DrivePacket -- that
// receiver does no mixing of its own, it just ramps toward whatever
// left/right values it's sent.
//
// Broadcast is used instead of pairing to a specific MAC address, so this
// will drive ANY robot receiver running motor_receiver.ino that's
// listening on the same WiFi channel. If you build more than one robot,
// give each a distinct peer MAC and unicast instead.

#include <WiFi.h>
#include <esp_now.h>

// ================= Joystick pins =================
#define PIN_VRX 34   // turn axis (left/right)
#define PIN_VRY 35   // throttle axis (forward/back)
#define PIN_SW  33   // press switch, active LOW -- used here as a turbo button

// ================= Tuning =================
const int   ADC_MAX      = 4095;  // ESP32 ADC is 12-bit
const int   DEADZONE     = 50;   // ADC counts around center to ignore (cheap sticks drift)
const int   PWM_MAX      = 255;   // matches robot's -255..255 wire range
const float NORMAL_SCALE = 0.4f;  // top speed during normal driving (0..1)
const float TURBO_SCALE  = 0.8f;  // top speed while the switch is held down

// If forward or turning feels backward once you test it, flip these instead
// of rewiring anything.
const bool INVERT_X = true; // turn axis
const bool INVERT_Y = true; // throttle axis

// ================= Wire format (must match motor_receiver.ino) =================
typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const uint32_t SEND_PERIOD_MS = 40; // well under the robot's 300ms failsafe timeout
uint32_t lastSendMs = 0;

int centerX, centerY; // measured at startup, see calibrate()

// Maps a raw ADC reading to -255..255, treating anything within DEADZONE
// of center as zero and rescaling the remainder so full stick travel still
// reaches -255/255.
int applyDeadzone(int raw, int center) {
  int delta = raw - center;
  if (abs(delta) < DEADZONE) return 0;

  int span = (delta > 0) ? (ADC_MAX - center) : center;
  span -= DEADZONE;
  if (span <= 0) span = 1;

  delta = (delta > 0) ? (delta - DEADZONE) : (delta + DEADZONE);
  float scaled = constrain((float)delta / (float)span, -1.0f, 1.0f);
  return (int)(scaled * PWM_MAX);
}

// Cheap joysticks rarely rest at exact half-scale, so read the actual
// resting position instead of assuming 2048. Keep the stick untouched
// while the controller powers on.
void calibrate() {
  long sumX = 0, sumY = 0;
  const int SAMPLES = 50;
  for (int i = 0; i < SAMPLES; i++) {
    sumX += analogRead(PIN_VRX);
    sumY += analogRead(PIN_VRY);
    delay(2);
  }
  centerX = sumX / SAMPLES;
  centerY = sumY / SAMPLES;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);
  analogReadResolution(12);

  calibrate();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0; // use whatever WiFi channel we're already on
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("Controller ready. center=(%d,%d)\n", centerX, centerY);
}

void loop() {
  uint32_t now = millis();
  if (now - lastSendMs < SEND_PERIOD_MS) return;
  lastSendMs = now;

  int rawX = analogRead(PIN_VRX);
  int rawY = analogRead(PIN_VRY);
  bool turbo = (digitalRead(PIN_SW) == LOW);

  int turn = applyDeadzone(rawX, centerX); // +right
  int fwd  = applyDeadzone(rawY, centerY); // +forward

  if (INVERT_X) turn = -turn;
  if (INVERT_Y) fwd  = -fwd;

  // Arcade mix.
  int left  = fwd + turn;
  int right = fwd - turn;

  // If the mix overshoots +-255 (e.g. full forward + full turn), scale
  // both sides down together so the turn ratio is preserved instead of
  // clipping one side and distorting the turn.
  int maxMag = max(abs(left), abs(right));
  if (maxMag > PWM_MAX) {
    left  = left  * PWM_MAX / maxMag;
    right = right * PWM_MAX / maxMag;
  }

  float scale = turbo ? TURBO_SCALE : NORMAL_SCALE;
  left  = (int)(left  * scale);
  right = (int)(right * scale);

  DrivePacket pkt;
  pkt.left  = (int16_t)constrain(left,  -PWM_MAX, PWM_MAX);
  pkt.right = (int16_t)constrain(right, -PWM_MAX, PWM_MAX);

  esp_now_send(broadcastAddr, (uint8_t*)&pkt, sizeof(pkt));
}
