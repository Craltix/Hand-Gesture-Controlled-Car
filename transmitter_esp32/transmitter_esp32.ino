// ============================================================
//  GESTURE CAR — TRANSMITTER (ESP32 + MPU6050 + 433 MHz TX)
//  Library needed: RadioHead by Mike McCauley
//  Install via: Arduino IDE → Tools → Manage Libraries → "RadioHead"
// ============================================================

#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h>          // Required by RadioHead even if unused

// ── Pin definitions ──────────────────────────────────────────
// IMPORTANT: GPIO2 (strapping — pulled LOW at boot) and GPIO15
// (strapping — pulled HIGH, affects SDIO) must NOT be used here.
// RadioHead also explicitly warns not to use pin 2 on ESP32.
// Using safe, non-strapping pins instead:
#define RF_TX_PIN    23   // RF TX DATA → GPIO23
#define RF_RX_PIN    4    // Dummy RX (not wired); required by RH_ASK constructor
#define LED_PIN      13   // Status LED: anode → 220Ω → GPIO13 → GND
#define SWITCH_PIN   16   // Toggle switch: one leg → GPIO16, other → GND

// ── MPU6050 I2C ──────────────────────────────────────────────
#define MPU_ADDR     0x68  // AD0 tied to GND → address 0x68
#define ACCEL_XOUT_H 0x3B  // First of 6 accel bytes
#define PWR_MGMT_1   0x6B  // Power register; write 0 to wake MPU

// ── Tilt thresholds (raw 16-bit signed at ±2 g) ──────────────
// 1 g = 16384 raw units.
// TILT_THRESH  = sin(20°) × 16384 ≈ 5600  → minimum tilt to register
// TILT_STRONG  = sin(42°) × 16384 ≈ 11000 → "fast" tilt → double LED blink
// Raise TILT_THRESH if the car drifts when held flat.
#define TILT_THRESH  5600
#define TILT_STRONG  11000

// ── Command bytes ─────────────────────────────────────────────
#define CMD_FORWARD  'F'
#define CMD_BACKWARD 'B'
#define CMD_LEFT     'L'
#define CMD_RIGHT    'R'
#define CMD_STOP     'S'

// ── RF driver ────────────────────────────────────────────────
// Three-argument form omits the PTT pin so no extra pin is
// configured as OUTPUT by the library's init().
RH_ASK rfDriver(2000, RF_RX_PIN, RF_TX_PIN);

// ── State ─────────────────────────────────────────────────────
char  lastCmd    = CMD_STOP;  // Initialise to STOP so watchdog starts correctly
bool  txEnabled  = true;
bool  lastSwitch = HIGH;

// ── Timing ───────────────────────────────────────────────────
unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL_MS = 80;  // ~12 Hz

// =============================================================
void setupMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);            // Clear sleep bit → wake up
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);            // ACCEL_CONFIG
  Wire.write(0x00);            // AFS_SEL = 0 → ±2 g
  Wire.endTransmission(true);
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
}

// =============================================================
// Gesture → command mapping
//   Flat (both axes below threshold)  → STOP
//   Y-axis dominant, Y < 0            → FORWARD  (nose tilts down)
//   Y-axis dominant, Y > 0            → BACKWARD
//   X-axis dominant, X < 0            → LEFT
//   X-axis dominant, X > 0            → RIGHT
//   On diagonals the stronger axis wins.
// =============================================================
char computeCommand(int16_t ax, int16_t ay) {
  int16_t absX = abs(ax);
  int16_t absY = abs(ay);

  if (absX < TILT_THRESH && absY < TILT_THRESH) return CMD_STOP;

  if (absY >= absX) {
    return (ay < 0) ? CMD_FORWARD : CMD_BACKWARD;
  } else {
    return (ax < 0) ? CMD_LEFT : CMD_RIGHT;
  }
}

// =============================================================
// LED feedback (called after every newly-transmitted command).
//   STOP         → no blink (steady-on LED already indicates TX active)
//   Normal tilt  → single short blink
//   Strong tilt  → double blink ("fast" signal to the driver)
// =============================================================
void blinkLED(char cmd, int16_t absMax) {
  if (cmd == CMD_STOP) return;

  int blinks = (absMax >= TILT_STRONG) ? 2 : 1;
  for (int i = 0; i < blinks; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(15);
    digitalWrite(LED_PIN, HIGH);
    delay(15);
  }
}

void sendCommand(char cmd) {
  uint8_t msg[1] = { (uint8_t)cmd };
  rfDriver.send(msg, 1);
  rfDriver.waitPacketSent();
}

// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Gesture Car Transmitter — Starting up...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Internal pull-up; switch shorts GPIO16 to GND when pressed/toggled
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  Wire.begin(21, 22);  // SDA=21, SCL=22 (ESP32 I2C defaults)
  setupMPU6050();
  Serial.println("MPU6050 initialized.");

  if (!rfDriver.init()) {
    Serial.println("ERROR: RF driver failed to initialize!");
    for (int i = 0; i < 6; i++) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  } else {
    Serial.println("RF driver OK.");
  }

  // Startup: three quick flashes
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(120);
    digitalWrite(LED_PIN, LOW);  delay(120);
  }
  Serial.println("Ready. Tilt to drive!");
}

// =============================================================
void loop() {
  // ── Toggle switch (active-LOW with internal pull-up) ─────
  // Triggers on the falling edge (HIGH → LOW) when the switch
  // closes the GPIO16-to-GND path.
  bool switchNow = digitalRead(SWITCH_PIN);
  if (lastSwitch == HIGH && switchNow == LOW) {
    txEnabled = !txEnabled;
    Serial.print("TX ");
    Serial.println(txEnabled ? "ENABLED" : "DISABLED");

    if (txEnabled) {
      // Triple fast blink = TX enabled
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(80);
        digitalWrite(LED_PIN, LOW);  delay(80);
      }
    } else {
      // Single slow blink = TX disabled; also stop the car immediately
      digitalWrite(LED_PIN, HIGH); delay(400);
      digitalWrite(LED_PIN, LOW);
      sendCommand(CMD_STOP);
      lastCmd = CMD_STOP;
    }
    delay(50);  // Simple debounce
  }
  lastSwitch = switchNow;

  // Steady LED = transmitter is active
  digitalWrite(LED_PIN, txEnabled ? HIGH : LOW);

  // ── Rate-limit ────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastTxTime < TX_INTERVAL_MS) return;
  lastTxTime = now;

  if (!txEnabled) return;

  // ── Read MPU6050 ─────────────────────────────────────────
  int16_t ax, ay, az;
  readAccel(ax, ay, az);

  // ── Determine command ────────────────────────────────────
  char cmd = computeCommand(ax, ay);
  Serial.printf("ax=%6d  ay=%6d  →  cmd=%c\n", ax, ay, cmd);

  // ── Transmit only on state change ────────────────────────
  // FIX: previously `cmd != lastCmd || cmd == CMD_STOP` caused STOP
  // to be re-sent every 80 ms even when already stopped, wasting RF
  // bandwidth. Now we only transmit when the command actually changes.
  if (cmd != lastCmd) {
    sendCommand(cmd);
    Serial.print("TX: ");
    Serial.println(cmd);

    int16_t absMax = max(abs(ax), abs(ay));
    blinkLED(cmd, absMax);

    lastCmd = cmd;
  }
}
