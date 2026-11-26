// ===== Llama Robot: Drive-packet LED status bar (ESP32-C3 + 7x WS2812) =====
//
// Listen-only ESP-NOW eavesdropper. It receives the same 4-byte DrivePacket
// {int16 left, int16 right} (-255..255) the stick controller and the computer
// bridge broadcast on WiFi channel 1, and shows it on 7 addressable LEDs.
// It NEVER transmits, so it is safe to leave running while anyone drives.
//
// Layout (10 LEDs in a row, index 0..9):
//   [0][1][2][3][4] left-motor bar | [5][6][7][8][9] right-motor bar
//
//   - Straight forward : bars green, length + brightness grow with speed.
//   - Straight reverse : bars red, same scaling.
//   - Spin (sides oppose): all ten LEDs blink blue together.
//   - Arcing turn (sides differ a lot): fast side keeps its green/red bar,
//     slow (inside) side blinks blue.
//   - Stop: all ten breathe purple (red if the link itself is stale).
//
// Wiring: 7x WS2812 data-in to LED_PIN through a ~330 ohm series resistor,
// strip 5V/GND to a 5V supply shared with the C3's GND. Default LED_PIN 8.
//
// Flash (ESP32-C3 needs USB CDC on boot for Serial):
//   arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
//     firmware/robot_led_status
//   arduino-cli upload -p /dev/ttyACM0 \
//     --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc firmware/robot_led_status

#include <WiFi.h>
#include <esp_now.h>
#include <FastLED.h>

#define LED_PIN 4  // data-in wire: 10x WS2812 strip, ~330 ohm in series
#define NUM_LEDS 10

#define LEFT_FIRST 0
#define LEFT_N 5
#define RIGHT_FIRST 5
#define RIGHT_N 5

// Same wire format as computer_bridge.ino / robot_motor_bts7960.ino.
typedef struct __attribute__((packed)) {
  int16_t left;   // -255..255
  int16_t right;  // -255..255
} DrivePacket;

CRGB leds[NUM_LEDS];

volatile int16_t gLeft = 0;
volatile int16_t gRight = 0;
volatile uint32_t gLastPacketMs = 0;
volatile bool gEverSeen = false;

const uint32_t STALE_MS = 500;   // stick sends every 40 ms, bridge every 20 ms
const int TURN_MIX = 60;         // |L-R| above this counts as an arcing turn
const uint8_t MAX_BRIGHT = 160;  // keep the strip polite indoors

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(DrivePacket)) return;  // telemetry (41 B) etc. ignored
  DrivePacket p;
  memcpy(&p, data, sizeof(p));
  // Clamp before trusting: a foreign 4-byte frame is just noise.
  if (p.left < -255 || p.left > 255 || p.right < -255 || p.right > 255) return;
  gLeft = p.left;
  gRight = p.right;
  gLastPacketMs = millis();
  gEverSeen = true;
}

// How many of a 5-LED bar light for |pwm|: fifths of the PWM range real
// sticks actually send (observed tops out ~100-130, not 255).
uint8_t barCount(int pwm) {
  int m = abs(pwm);
  if (m <= 0) return 0;
  if (m < 25) return 1;
  if (m < 45) return 2;
  if (m < 65) return 3;
  if (m < 85) return 4;
  return 5;
}

// Brightness grows with speed so "green brighter for faster" reads at a glance.
uint8_t speedBright(int pwm) {
  int m = abs(pwm);
  if (m <= 0) return 0;
  return (uint8_t)(70 + (MAX_BRIGHT - 70) * m / 255);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // Self-test: chase R, G, B down the strip so a dead pixel or a
  // backwards/mirrored strip shows itself every reboot. Index 0 lights
  // first; count physical LEDs from the data-wire end to match them up.
  const CRGB testCols[3] = {CRGB::Red, CRGB::Green, CRGB::Blue};
  for (uint8_t c = 0; c < 3; c++) {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      leds[i] = testCols[c];
      FastLED.show();
      delay(150);
    }
  }
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  delay(400);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(1);  // same channel the stick/bridge broadcast on

  if (esp_now_init() != ESP_OK) {
    // No radio, no show: blink red so the failure is visible, not silent.
    while (true) {
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      FastLED.show();
      delay(300);
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      delay(300);
    }
  }
  esp_now_register_recv_cb(onRecv);
  // NOTE: no peer is added and nothing is ever sent. Listen-only by design.

  Serial.println("@ status led-status-v1 listen-only *00");
}

int16_t snapLeft, snapRight;
bool linkAlive;

void paintBars() {
  bool spin = (snapLeft != 0 && snapRight != 0 &&
               ((snapLeft > 0) != (snapRight > 0)));
  bool turn = !spin && abs(snapLeft - snapRight) > TURN_MIX &&
              (snapLeft != 0 || snapRight != 0);
  uint32_t now = millis();

  CRGB leftCol = snapLeft >= 0 ? CRGB::Green : CRGB::Red;
  CRGB rightCol = snapRight >= 0 ? CRGB::Green : CRGB::Red;

  if (spin) {
    // Blink blue together at ~4 Hz while spinning in place -- all 10.
    bool on = (now / 125) % 2 == 0;
    CRGB c = on ? CRGB::Blue : CRGB::Black;
    fill_solid(leds, NUM_LEDS, c);
    return;
  }

  // Per-side bars; the slower side of an arcing turn blinks blue instead.
  bool leftSlow = turn && abs(snapLeft) < abs(snapRight);
  bool rightSlow = turn && abs(snapRight) < abs(snapLeft);
  bool blinkOn = (now / 200) % 2 == 0;

  uint8_t ln = barCount(snapLeft), rn = barCount(snapRight);
  uint8_t lb = speedBright(snapLeft), rb = speedBright(snapRight);
  for (uint8_t i = 0; i < LEFT_N; i++) {
    if (leftSlow) {
      leds[LEFT_FIRST + i] = (blinkOn && i < (ln ? ln : 1)) ? CRGB::Blue : CRGB::Black;
    } else {
      leds[LEFT_FIRST + i] = (i < ln) ? leftCol : CRGB::Black;
      if (i < ln) leds[LEFT_FIRST + i].nscale8_video(lb == 0 ? 1 : lb);
    }
  }
  for (uint8_t i = 0; i < RIGHT_N; i++) {
    if (rightSlow) {
      leds[RIGHT_FIRST + i] = (blinkOn && i < (rn ? rn : 1)) ? CRGB::Blue : CRGB::Black;
    } else {
      leds[RIGHT_FIRST + i] = (i < rn) ? rightCol : CRGB::Black;
      if (i < rn) leds[RIGHT_FIRST + i].nscale8_video(rb == 0 ? 1 : rb);
    }
  }
}

// Stopped: the ONLY heartbeat left. All 10 LEDs breathe purple together,
// whether that is a commanded (0,0) or a stale link. Stale breathes red
// instead of purple, so "nobody is broadcasting" still reads differently
// from "stop commanded".
void paintStopGlow() {
  uint32_t now = millis();
  float t = (now % 1500) / 1500.0f;
  float s = 0.5f - 0.5f * cos(t * 2.0f * 3.14159f);  // 0..1..0
  CRGB c = linkAlive ? CRGB::Purple : CRGB::Red;
  uint8_t b = (uint8_t)(s * 123);  // breathe 0..123
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = c;
    leds[i].nscale8_video(b);
  }
}

int16_t lastShownL = 1, lastShownR = 1;  // force first print
bool lastAlive = false;
uint32_t lastSerialBeat = 0;
const uint32_t SERIAL_BEAT_MS = 2000;  // periodic line so a late-opened
                                       // monitor still sees proof of life

void loop() {
  noInterrupts();
  snapLeft = gLeft;
  snapRight = gRight;
  uint32_t age = millis() - gLastPacketMs;
  bool ever = gEverSeen;
  interrupts();
  linkAlive = ever && age <= STALE_MS;

  if (!linkAlive) {
    snapLeft = 0;  // stale link shows stop, never a frozen command
    snapRight = 0;
  }

  if (snapLeft == 0 && snapRight == 0)
    paintStopGlow();  // the only heartbeat: stop, on all 10 LEDs
  else
    paintBars();      // driving: center joins the bars, no heartbeat
  FastLED.show();

  if (snapLeft != lastShownL || snapRight != lastShownR || linkAlive != lastAlive) {
    lastShownL = snapLeft;
    lastShownR = snapRight;
    lastAlive = linkAlive;
    Serial.printf("@ CMD %d %d alive=%d *00\n", snapLeft, snapRight, linkAlive ? 1 : 0);
    lastSerialBeat = millis();
  }
  if (millis() - lastSerialBeat >= SERIAL_BEAT_MS) {
    lastSerialBeat = millis();
    Serial.printf("@ beat L=%d R=%d alive=%d age=%lu *00\n",
                  snapLeft, snapRight, linkAlive ? 1 : 0,
                  (unsigned long)(ever ? age : 0xFFFFFFFF));
  }

  delay(20);
}
