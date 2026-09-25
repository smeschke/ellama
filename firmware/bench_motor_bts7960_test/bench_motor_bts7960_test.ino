// ===== Llama Robot: BTS7960 Motor Bench Test =====
//
// Gently exercise ONE motor channel at a time from the Serial Monitor (115200
// baud, line ending "Newline"). No ESP-NOW, no radio -- USB only. Use it to
// confirm each BTS7960 driver, motor and wire harness is connected to the pin
// you think it is and spins the direction you think it does, before trusting
// robot_motor_bts7960.ino with all four at once.
//
// Same pins, LEDC setup and direction convention as robot_motor_bts7960.ino:
// positive PWM drives LPWM ("forward"), negative drives RPWM ("reverse").
//
// Safety, by design:
//   - Only one channel can run at a time; a new command stops the old one first.
//   - PWM is clamped to +-MAX_PWM (low on purpose -- raise it only if the motor
//     doesn't move at all, and only a little at a time).
//   - Every command runs for a fixed number of seconds (default DEFAULT_RUN_S,
//     hard cap MAX_RUN_S) and then stops by itself. Nothing "stays on".
//   - Speed ramps up slowly (RAMP_STEP per RAMP_DT_MS) instead of stepping.
//   - Everything is at 0 on boot, and "stop" (or any bad command) zeroes it
//     immediately. PWM 0 with R_EN/L_EN hardwired enabled actively brakes the
//     motor, same as the robot firmware.
//   - Put the robot on blocks (wheels off the ground) before running a
//     drive-train motor, and keep the power switch/kill within reach.
//
// Commands:
//   <ch> <pwm>            run channel for DEFAULT_RUN_S, e.g.  fl 30   or   br -30
//   <ch> <pwm> <seconds>  run for a set time (max MAX_RUN_S), e.g.  fr 25 3
//   stop  (or empty line) stop everything immediately
//   pins                  print the channel -> pin map
//   help                  print this list
// Channels: fl (front left), bl (back left), fr (front right), br (back right).

// ================= Motor pins (match robot_motor_bts7960.ino) =================
#define FL_RPWM 16
#define FL_LPWM 17
#define BL_RPWM 18
#define BL_LPWM 19
#define FR_RPWM 32
#define FR_LPWM 33
#define BR_RPWM 25
#define BR_LPWM 26

struct Channel {
  const char *name;
  const char *label;
  uint8_t rpwm;
  uint8_t lpwm;
};
const Channel CHANNELS[4] = {
  {"fl", "front left",  FL_RPWM, FL_LPWM},
  {"bl", "back left",   BL_RPWM, BL_LPWM},
  {"fr", "front right", FR_RPWM, FR_LPWM},
  {"br", "back right",  BR_RPWM, BR_LPWM},
};

// ================= LEDC settings =================
const int LEDC_FREQ_HZ  = 20000; // above audible range
const int LEDC_RES_BITS = 8;     // 0..255 duty

// ================= Safety limits =================
const int      MAX_PWM       = 60;    // hard ceiling on |PWM|, out of 255
const uint32_t DEFAULT_RUN_S = 2;     // seconds a command runs if none is given
const uint32_t MAX_RUN_S     = 5;     // longest a single command may run
const int      RAMP_STEP     = 1;     // max PWM change per ramp tick
const uint32_t RAMP_DT_MS    = 10;    // ramp tick period (slower than the robot's 3 ms)

// ================= State =================
int activeCh = -1;       // index into CHANNELS, -1 = none
int curPwm = 0;          // signed, what's actually being output right now
int tgtPwm = 0;          // signed, where the ramp is heading
uint32_t runUntilMs = 0; // when the active run ends and the target drops to 0
uint32_t lastRampMs = 0;

void writeChannel(int ch, int pwm) {
  if (pwm >= 0) {
    ledcWrite(CHANNELS[ch].lpwm, pwm);
    ledcWrite(CHANNELS[ch].rpwm, 0);
  } else {
    ledcWrite(CHANNELS[ch].lpwm, 0);
    ledcWrite(CHANNELS[ch].rpwm, -pwm);
  }
}

// Immediately zero every output (brake) and forget the active run.
void stopAll() {
  for (int i = 0; i < 4; i++) {
    ledcWrite(CHANNELS[i].lpwm, 0);
    ledcWrite(CHANNELS[i].rpwm, 0);
  }
  activeCh = -1;
  curPwm = 0;
  tgtPwm = 0;
}

int findChannel(const String &name) {
  for (int i = 0; i < 4; i++) {
    if (name.equalsIgnoreCase(CHANNELS[i].name)) return i;
  }
  return -1;
}

void printPins() {
  for (int i = 0; i < 4; i++) {
    Serial.printf("  %s (%s): RPWM=GPIO%d (reverse)  LPWM=GPIO%d (forward)\n",
                  CHANNELS[i].name, CHANNELS[i].label, CHANNELS[i].rpwm, CHANNELS[i].lpwm);
  }
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  <ch> <pwm> [seconds]   e.g. 'fl 30', 'br -30', 'fr 25 3'");
  Serial.println("  stop / empty line      stop everything now");
  Serial.println("  pins                   channel -> pin map");
  Serial.printf ("Channels: fl bl fr br. |pwm| <= %d, run time <= %lu s, one channel at a time.\n",
                 MAX_PWM, (unsigned long)MAX_RUN_S);
}

void handleLine(String line) {
  line.trim();

  if (line.length() == 0 || line.equalsIgnoreCase("stop")) {
    stopAll();
    Serial.println("-> stopped");
    return;
  }
  if (line.equalsIgnoreCase("help") || line == "?") { printHelp(); return; }
  if (line.equalsIgnoreCase("pins")) { printPins(); return; }

  // Parse "<ch> <pwm> [seconds]"
  int s1 = line.indexOf(' ');
  if (s1 < 0) { stopAll(); Serial.println("bad command -> stopped. Type 'help'."); return; }
  String chName = line.substring(0, s1);
  String rest = line.substring(s1 + 1);
  rest.trim();
  int s2 = rest.indexOf(' ');
  String pwmStr = (s2 < 0) ? rest : rest.substring(0, s2);
  String secStr = (s2 < 0) ? "" : rest.substring(s2 + 1);

  int ch = findChannel(chName);
  if (ch < 0) { stopAll(); Serial.println("unknown channel -> stopped. Use fl, bl, fr or br."); return; }

  int pwm = pwmStr.toInt();
  if (pwm > MAX_PWM || pwm < -MAX_PWM) {
    Serial.printf("(clamped %d to +-%d)\n", pwm, MAX_PWM);
    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  }
  uint32_t runS = DEFAULT_RUN_S;
  if (secStr.length() > 0) {
    long v = secStr.toInt();
    if (v < 1) v = 1;
    if (v > (long)MAX_RUN_S) { Serial.printf("(clamped run time to %lu s)\n", (unsigned long)MAX_RUN_S); v = MAX_RUN_S; }
    runS = v;
  }

  // One channel at a time: kill whatever was running before starting.
  stopAll();
  if (pwm == 0) { Serial.println("-> pwm 0, nothing to run"); return; }

  activeCh = ch;
  tgtPwm = pwm;
  runUntilMs = millis() + runS * 1000UL;
  Serial.printf("-> %s (%s) pwm=%d for %lus (ramping up)\n",
                CHANNELS[ch].name, CHANNELS[ch].label, pwm, (unsigned long)runS);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 4; i++) {
    ledcAttach(CHANNELS[i].lpwm, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttach(CHANNELS[i].rpwm, LEDC_FREQ_HZ, LEDC_RES_BITS);
  }
  stopAll();

  Serial.println("\n--- BTS7960 Motor Bench Test ---");
  Serial.println("All channels off. Put the robot on blocks before running a motor.");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    handleLine(Serial.readStringUntil('\n'));
  }

  uint32_t now = millis();

  // Run time over: start ramping down to 0.
  if (activeCh >= 0 && (int32_t)(now - runUntilMs) >= 0 && tgtPwm != 0) {
    tgtPwm = 0;
    Serial.println("-> run time over, ramping down");
  }

  if (activeCh >= 0 && now - lastRampMs >= RAMP_DT_MS) {
    lastRampMs = now;
    curPwm += constrain(tgtPwm - curPwm, -RAMP_STEP, RAMP_STEP);
    writeChannel(activeCh, curPwm);
    if (tgtPwm == 0 && curPwm == 0) {
      stopAll();
      Serial.println("-> done");
    }
  }
}
