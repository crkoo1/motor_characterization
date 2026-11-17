// piano_firmware: the piano's microcontroller firmware (Nano Every + PCA9685 for now).
//
//   Pi --USB serial--> this board --I2C--> PCA9685 boards --> MOSFETs --> solenoids
//
// Keys: MIDI notes 21 (A0) to 108 (C8). Note n uses output (n - 21):
//   PCA9685 at 0x40 drives notes 21-36, 0x41 drives 37-52, ... 0x45 drives 101-108.
//   (Set the address jumpers on each extra PCA9685 board to match.)
//
// Serial, 115200 baud, one command per line. Pi -> board:
//   ON <note> <velocity>   strike a key, velocity 1-127 (harder = louder)
//   OFF <note>             release a key
//   ALLOFF                 release everything
//   PING                   "I'm still here"; send at least every 100 ms
//   DEBUG <0|1>            1 = report every strike and release (for testing)
//   TUNE <note> <min> <max>  this key's strike length in ms at velocity 1 and 127
//                          (keys differ; this evens them out). Lost at power-off:
//                          the Pi re-sends its saved tuning on connect.
//   Bench only (see "Characterization" below; blocks for ~150 ms):
//   TEST <note> <ms> [duty] fire once and measure the hit on the force sensor
//   TESTUS <note> <us> [duty] the same with the pulse in microseconds
//   FSR                    read the force sensor at rest (for calibration)
//   THRESH <counts>        how far above resting counts as contact (default 20)
// Board -> Pi:
//   READY <boards found>   after power-up or reset
//   OK                     reply to PING / ALLOFF
//   LIMIT <note>           a key was held too long and was released
//   ERR <reason>           something was wrong (bad command, missing board, watchdog)
//
// How a strike works:
//   1. STRIKE: full power for a time set by velocity, between that key's tuned
//      min and max (default STRIKE_MIN_MS to STRIKE_MAX_MS).
//      Longer = the plunger is pushed for longer = faster key = louder note.
//   2. Then, if HOLD_DUTY is 0: release (the note won't sustain).
//      If HOLD_DUTY > 0: hold the key down at that reduced power until OFF, so the
//      note sustains without overheating the coil. This uses PWM, which needs a
//      flyback diode across every solenoid. Don't enable it without one.
//
// Safety, enforced here whatever the Pi sends:
//   * every output is off at power-up, before anything else
//   * no PING or command for WATCHDOG_MS -> everything off ("ERR watchdog")
//   * a key held longer than MAX_HOLD_MS is released ("LIMIT <note>")
#include <Wire.h>

// ---- Tuning -------------------------------------------------------------
const uint16_t STRIKE_MIN_MS = 10;     // velocity 1
const uint16_t STRIKE_MAX_MS = 80;     // velocity 127
const uint8_t STRIKE_LIMIT_MS = 150;   // TUNE can't go past this
const uint16_t HOLD_DUTY = 0;          // 0 = no hold. With diodes fitted, try ~1200 (30%) of 4095.
const uint16_t MAX_HOLD_MS = 5000;
const uint16_t WATCHDOG_MS = 500;

// ---- Hardware -----------------------------------------------------------
const uint8_t FIRST_NOTE = 21, LAST_NOTE = 108;
const uint8_t NUM_KEYS = LAST_NOTE - FIRST_NOTE + 1;     // 88
const uint8_t PCA_BASE = 0x40, MAX_BOARDS = 6;           // 6 x 16 = 96 outputs
const uint8_t PRESCALE = 5;                              // 25 MHz / (4096 * 6) = about 1 kHz PWM

// PCA9685 registers
const uint8_t MODE1 = 0x00, MODE2 = 0x01, LED0_ON_L = 0x06, ALL_LED_ON_L = 0xFA, PRE_SCALE = 0xFE;
const uint8_t MODE1_SLEEP = 0x10, MODE1_AI = 0x20, MODE1_RESTART = 0x80, MODE2_OUTDRV = 0x04;
const uint8_t FULL = 0x10;

enum Phase : uint8_t { IDLE, STRIKE, HOLD };
struct Key {
  Phase phase;
  unsigned long since;      // when the current phase started
  uint16_t strikeMs;
};
Key keys[NUM_KEYS];
uint8_t tuneMin[NUM_KEYS], tuneMax[NUM_KEYS];
bool boardOk[MAX_BOARDS];
bool debug = false;
unsigned long lastHeard = 0;
bool watchdogTripped = false;
char line[40];
uint8_t lineLen = 0;

// ---- PCA9685 --------------------------------------------------------------
bool writeRegs(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data, n);
  return Wire.endTransmission() == 0;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  return writeRegs(addr, reg, &value, 1);
}

// duty: 0 = off, 4095 = fully on, in between = PWM
bool setOutput(uint8_t key, uint16_t duty) {
  uint8_t board = key / 16, ch = key % 16;
  if (!boardOk[board]) return false;
  uint8_t r[4] = {0, 0, 0, 0};
  if (duty == 0) r[3] = FULL;
  else if (duty >= 4095) r[1] = FULL;
  else { r[2] = duty & 0xFF; r[3] = duty >> 8; }
  return writeRegs(PCA_BASE + board, LED0_ON_L + 4 * ch, r, 4);
}

bool initBoard(uint8_t addr) {
  uint8_t allOff[4] = {0, 0, 0, FULL};
  return writeReg(addr, MODE1, MODE1_SLEEP | MODE1_AI)       // asleep: outputs can't fire
      && writeRegs(addr, ALL_LED_ON_L, allOff, 4)            // everything off before waking
      && writeReg(addr, MODE2, MODE2_OUTDRV)                 // push-pull, good for MOSFET gates
      && writeReg(addr, PRE_SCALE, PRESCALE)                 // PWM frequency, only settable asleep
      && writeReg(addr, MODE1, MODE1_AI)                     // wake
      && (delayMicroseconds(600), writeReg(addr, MODE1, MODE1_AI | MODE1_RESTART));
}

bool readReg(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

void printAddr(uint8_t b) {
  Serial.print("0x"); Serial.print(PCA_BASE + b, HEX);
}

// A PCA9685 that loses power (a loose wire, or a 5 V dip when solenoids fire)
// comes back asleep with every output off, yet still accepts writes. So once a
// second, read each board's MODE1: if it's asleep, set it up again.
void checkBoards() {
  for (uint8_t b = 0; b < MAX_BOARDS; b++) {
    uint8_t mode1;
    bool present = readReg(PCA_BASE + b, MODE1, mode1);
    if (boardOk[b] && !present) {
      boardOk[b] = false;
      for (uint8_t k = b * 16; k < b * 16 + 16 && k < NUM_KEYS; k++) keys[k].phase = IDLE;
      Serial.print("ERR board "); printAddr(b); Serial.println(" not answering: check its wiring");
    } else if (present && (!boardOk[b] || (mode1 & MODE1_SLEEP))) {
      bool wasOk = boardOk[b];
      for (uint8_t k = b * 16; k < b * 16 + 16 && k < NUM_KEYS; k++) keys[k].phase = IDLE;
      boardOk[b] = initBoard(PCA_BASE + b);
      if (!boardOk[b]) continue;
      Serial.print(wasOk ? "ERR board " : "BOARD "); printAddr(b);
      Serial.println(wasOk ? " had reset (power dip or loose 5V/GND?), set up again" : " found");
    }
  }
}

uint8_t initBoards() {
  uint8_t found = 0;
  for (uint8_t b = 0; b < MAX_BOARDS; b++) {
    boardOk[b] = false;
    for (uint8_t tries = 0; tries < 5 && !boardOk[b]; tries++) {
      boardOk[b] = initBoard(PCA_BASE + b);
      if (!boardOk[b]) delay(10);
    }
    if (boardOk[b]) found++;
  }
  return found;
}

// ---- Keys ---------------------------------------------------------------
void release(uint8_t k) {
  if (keys[k].phase == IDLE) return;
  setOutput(k, 0);
  if (debug) {
    Serial.print("REL "); Serial.print(k + FIRST_NOTE);
    Serial.print(' '); Serial.println(millis() - keys[k].since);
  }
  keys[k].phase = IDLE;
}

void releaseAll() {
  uint8_t allOff[4] = {0, 0, 0, FULL};
  for (uint8_t b = 0; b < MAX_BOARDS; b++)
    if (boardOk[b]) writeRegs(PCA_BASE + b, ALL_LED_ON_L, allOff, 4);
  for (uint8_t k = 0; k < NUM_KEYS; k++) keys[k].phase = IDLE;
}

void strike(int note, int velocity) {
  if (note < FIRST_NOTE || note > LAST_NOTE) { Serial.print("ERR note out of range "); Serial.println(note); return; }
  if (velocity < 1 || velocity > 127) { Serial.print("ERR velocity must be 1-127 "); Serial.println(note); return; }
  uint8_t k = note - FIRST_NOTE;
  if (!boardOk[k / 16]) { Serial.print("ERR no board for note "); Serial.println(note); return; }
  uint16_t ms = tuneMin[k] + (uint32_t)(velocity - 1) * (tuneMax[k] - tuneMin[k]) / 126;
  if (!setOutput(k, 4095)) { releaseAll(); Serial.println("ERR I2C write failed, all off"); return; }
  keys[k] = {STRIKE, millis(), ms};
  if (debug) { Serial.print("HIT "); Serial.print(note); Serial.print(' '); Serial.println(ms); }
}

void updateKeys(unsigned long now) {
  for (uint8_t k = 0; k < NUM_KEYS; k++) {
    Key &key = keys[k];
    if (key.phase == STRIKE && now - key.since >= key.strikeMs) {
      if (HOLD_DUTY == 0) {
        release(k);
      } else {
        setOutput(k, HOLD_DUTY);
        key.phase = HOLD;
      }
    }
    if (key.phase != IDLE && now - key.since >= MAX_HOLD_MS) {
      release(k);
      Serial.print("LIMIT "); Serial.println(k + FIRST_NOTE);
    }
  }
}

void tune(int note, int minMs, int maxMs) {
  if (note < FIRST_NOTE || note > LAST_NOTE) { Serial.print("ERR note out of range "); Serial.println(note); return; }
  if (minMs < 1 || maxMs < minMs || maxMs > STRIKE_LIMIT_MS) {
    Serial.print("ERR tune needs 1 <= min <= max <= "); Serial.println(STRIKE_LIMIT_MS); return;
  }
  tuneMin[note - FIRST_NOTE] = minMs;
  tuneMax[note - FIRST_NOTE] = maxMs;
  Serial.println("OK");
}

// ---- Characterization (bench only) --------------------------------------
// A force sensor (FSR) the plunger hits, read on A0:
//   5V -- FSR -- A0 -- 1k -- GND
// TEST fires one solenoid and samples A0 every SAMPLE_US for CAPTURE_SAMPLES,
// timed here on the Nano so the numbers don't depend on USB delays. It blocks
// for about 150 ms, so it's for the bench, not for playing.
const uint8_t FSR_PIN = A0;
const uint16_t SAMPLE_US = 200;                  // 5 kHz
const uint16_t CAPTURE_SAMPLES = 750;            // 150 ms
uint16_t trace[CAPTURE_SAMPLES];
uint16_t contactThreshold = 20;                  // ADC counts above resting level

uint16_t restingLevel() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 32; i++) { sum += analogRead(FSR_PIN); delayMicroseconds(100); }
  return sum / 32;
}

// TEST <note> <ms> [duty]   or   TESTUS <note> <us> [duty]
// Full power (or duty/4095) for that long, capture the impact. Times are from
// the moment the output turned on. Switching the PCA9685 over I2C takes a few
// hundred us, so turn-off is sent early by that much (measured on each hit).
// Replies:
//   RESULT <note> <pulse_us> <duty> <rest> <contact_us> <peak> <peak_us> <width_us> <area> <on_us> <i2c_us>
//     on_us: when the output actually turned off; i2c_us: one I2C switch command
//     contact_us: pulse start to first force (-1 = never touched the sensor)
//     peak: highest ADC reading; area: sum of (reading - rest) while in contact
//   TRACE <sample_us> <v0> <v1> ...   every sample, for plotting
void characterize(int note, long pulseUs, int duty) {
  if (note < FIRST_NOTE || note > LAST_NOTE || !boardOk[(note - FIRST_NOTE) / 16]) {
    Serial.println("ERR TEST needs a note with a board"); return;
  }
  if (pulseUs < 100 || pulseUs > STRIKE_LIMIT_MS * 1000L) {
    Serial.print("ERR TEST pulse must be 0.1-"); Serial.print(STRIKE_LIMIT_MS); Serial.println(" ms"); return;
  }
  if (duty < 1 || duty > 4095) { Serial.println("ERR TEST duty must be 1-4095"); return; }
  uint8_t k = note - FIRST_NOTE;
  release(k);
  uint16_t rest = restingLevel();

  unsigned long t0 = micros();
  if (!setOutput(k, duty)) { releaseAll(); Serial.println("ERR I2C write failed, all off"); return; }
  unsigned long start = micros();                 // the output switches as the write completes
  unsigned long i2cUs = start - t0;
  unsigned long offAt = (unsigned long)pulseUs > i2cUs ? pulseUs - i2cUs : 0;
  unsigned long onUs = 0;
  bool on = true;
  long contactUs = -1, peakUs = 0, widthUs = 0;
  uint16_t peak = 0;
  uint32_t area = 0;
  for (uint16_t i = 0; i < CAPTURE_SAMPLES; i++) {
    unsigned long due = start + (unsigned long)i * SAMPLE_US;
    while ((long)(micros() - due) < 0) {
      if (on && micros() - start >= offAt) { setOutput(k, 0); on = false; onUs = micros() - start; }
    }
    if (on && micros() - start >= offAt) { setOutput(k, 0); on = false; onUs = micros() - start; }
    unsigned long t = micros() - start;
    uint16_t v = analogRead(FSR_PIN);
    trace[i] = v;
    if (v > rest + contactThreshold) {
      if (contactUs < 0) contactUs = t;
      widthUs += SAMPLE_US;
      area += v - rest;
    }
    if (v > peak) { peak = v; peakUs = t; }
  }
  if (on) { setOutput(k, 0); onUs = micros() - start; }
  lastHeard = millis();                          // we were busy, not the Pi gone quiet

  Serial.print("RESULT "); Serial.print(note); Serial.print(' '); Serial.print(pulseUs);
  Serial.print(' '); Serial.print(duty); Serial.print(' '); Serial.print(rest);
  Serial.print(' '); Serial.print(contactUs); Serial.print(' '); Serial.print(peak);
  Serial.print(' '); Serial.print(peakUs); Serial.print(' '); Serial.print(widthUs);
  Serial.print(' '); Serial.print(area);
  Serial.print(' '); Serial.print(onUs); Serial.print(' '); Serial.println(i2cUs);
  Serial.print("TRACE "); Serial.print(SAMPLE_US);
  for (uint16_t i = 0; i < CAPTURE_SAMPLES; i++) { Serial.print(' '); Serial.print(trace[i]); }
  Serial.println();
}

// FSR: average, min and max of A0 over about 200 ms, for calibrating with weights.
void readFsr() {
  uint32_t sum = 0;
  uint16_t lo = 1023, hi = 0;
  for (uint16_t i = 0; i < 1000; i++) {
    uint16_t v = analogRead(FSR_PIN);
    sum += v; lo = min(lo, v); hi = max(hi, v);
    delayMicroseconds(200);
  }
  lastHeard = millis();
  Serial.print("FSR "); Serial.print(sum / 1000); Serial.print(' ');
  Serial.print(lo); Serial.print(' '); Serial.println(hi);
}

// ---- Serial -------------------------------------------------------------
void handle(char *cmd) {
  int note, vel, minMs, maxMs;
  lastHeard = millis();
  watchdogTripped = false;
  if (sscanf(cmd, "ON %d %d", &note, &vel) == 2) strike(note, vel);
  else if (sscanf(cmd, "TUNE %d %d %d", &note, &minMs, &maxMs) == 3) tune(note, minMs, maxMs);
  else if (!strncmp(cmd, "TEST ", 5) || !strncmp(cmd, "TESTUS ", 7)) {
    bool us = cmd[4] == 'U';
    long len;
    int duty = 4095, n = sscanf(cmd + (us ? 7 : 5), "%d %ld %d", &note, &len, &duty);
    if (n >= 2) characterize(note, us ? len : len * 1000L, duty);
    else Serial.println("ERR usage: TEST <note> <ms> [duty]  or  TESTUS <note> <us> [duty]");
  }
  else if (!strcmp(cmd, "FSR")) readFsr();
  else if (sscanf(cmd, "THRESH %d", &vel) == 1 && vel > 0 && vel < 1023) { contactThreshold = vel; Serial.println("OK"); }
  else if (sscanf(cmd, "OFF %d", &note) == 1) {
    if (note >= FIRST_NOTE && note <= LAST_NOTE) release(note - FIRST_NOTE);
  }
  else if (!strcmp(cmd, "PING")) Serial.println("OK");
  else if (!strcmp(cmd, "ALLOFF")) { releaseAll(); Serial.println("OK"); }
  else if (sscanf(cmd, "DEBUG %d", &vel) == 1) { debug = vel; Serial.println("OK"); }
  else { Serial.print("ERR unknown command: "); Serial.println(cmd); }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = 0;
      if (lineLen) handle(line);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

void setup() {
  for (uint8_t k = 0; k < NUM_KEYS; k++) { tuneMin[k] = STRIKE_MIN_MS; tuneMax[k] = STRIKE_MAX_MS; }
  Wire.begin();
  Wire.setClock(100000);       // jumper wires: keep it slow. Try 400000 on a real PCB.
  delay(20);
  uint8_t found = initBoards(); // first thing: outputs off
  Serial.begin(115200);
  Serial.print("READY "); Serial.println(found);
  lastHeard = millis();
}

void loop() {
  readSerial();
  unsigned long now = millis();
  updateKeys(now);
  static unsigned long lastCheck = 0;
  if (now - lastCheck >= 1000) {
    lastCheck = now;
    checkBoards();
  }
  if (!watchdogTripped && now - lastHeard > WATCHDOG_MS) {
    bool anyOn = false;
    for (uint8_t k = 0; k < NUM_KEYS; k++) anyOn |= keys[k].phase != IDLE;
    releaseAll();
    watchdogTripped = true;      // report once, until the Pi talks again
    if (anyOn) Serial.println("ERR watchdog: no message from Pi, all off");
  }
}
