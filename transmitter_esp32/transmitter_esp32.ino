// ============================================================
//  GESTURE CAR — TRANSMITTER (ESP32 + MPU6050 + 433 MHz TX)
//  Library needed: RadioHead by Mike McCauley
//  Install via: Arduino IDE → Tools → Manage Libraries → "RadioHead"
// ============================================================

#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h>         // Required by RadioHead even if unused

// ── Pin definitions ──────────────────────────────────────────
#define RF_TX_PIN    23  // RF TX DATA → ESP32 GPIO23
#define RF_RX_PIN    4   // Dummy RX pin (unused; needed by RH_ASK constructor)
#define LED_PIN      2   // Status LED anode → 220Ω → GPIO2
#define SWITCH_PIN   15  // Toggle switch one leg → GPIO15, other leg → GND

// ── MPU6050 I2C ──────────────────────────────────────────────
#define MPU_ADDR     0x68  // AD0 tied to GND → address 0x68
#define ACCEL_XOUT_H 0x3B  // First of 6 accel bytes (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
#define PWR_MGMT_1   0x6B  // Power register; write 0 to wake MPU

// ── Tilt thresholds (raw 16-bit signed values at ±2 g) ───────
// ±32768 = ±2 g → 1 g = 16384.  At 2 g range, sin(θ) ≈ raw/16384
// TILT_THRESH ≈ 16384 × sin(20°) ≈ 5600  → adjust to taste
#define TILT_THRESH  5600   // Minimum tilt to register a direction
#define TILT_STRONG  11000  // "Fast" tilt (used for LED double-blink feedback)

// ── Command bytes sent over RF ────────────────────────────────
//   Single ASCII character; must match what the receiver expects
#define CMD_FORWARD  'F'
#define CMD_BACKWARD 'B'
#define CMD_LEFT     'L'
#define CMD_RIGHT    'R'
#define CMD_STOP     'S'

// ── RF driver  (bps=2000, rxPin=RF_RX_PIN, txPin=RF_TX_PIN) ──
RH_ASK rfDriver(2000, RF_RX_PIN, RF_TX_PIN, 0);

// ── State ─────────────────────────────────────────────────────
char     lastCmd    = 0;      // Avoid spamming identical packets
bool     txEnabled  = true;   // Toggle switch toggles this
bool     lastSwitch = HIGH;   // For edge detection on switch

// ── Timing ───────────────────────────────────────────────────
unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL_MS = 80;  // Send at ~12 Hz

// =============================================================
void setupMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);           // Clear sleep bit → wake up MPU
  Wire.endTransmission(true);
  delay(100);

  // Optional: set accel full-scale range to ±2 g (default, but explicit)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);           // ACCEL_CONFIG register
  Wire.write(0x00);           // AFS_SEL = 0 → ±2 g
  Wire.endTransmission(true);
}

// Returns raw 16-bit signed accel values
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
// Gesture logic
//
//   Hold your hand flat → Flat = STOP
//   Tilt FORWARD  (nose down, Y axis negative) → FORWARD
//   Tilt BACKWARD (nose up,   Y axis positive) → BACKWARD
//   Tilt LEFT     (left side down, X negative) → LEFT
//   Tilt RIGHT    (right side down, X positive) → RIGHT
//
//   Diagonal tilts: whichever axis is stronger wins.
//   Adjust TILT_THRESH if the car moves when you hold still.
// =============================================================
char computeCommand(int16_t ax, int16_t ay) {
  // Absolute magnitudes for comparison
  int16_t absX = abs(ax);
  int16_t absY = abs(ay);

  // Neither axis tilted enough → stop
  if (absX < TILT_THRESH && absY < TILT_THRESH) return CMD_STOP;

  // Dominant axis wins
  if (absY >= absX) {
    // Forward/backward tilt dominates
    return (ay < 0) ? CMD_FORWARD : CMD_BACKWARD;
  } else {
    // Left/right tilt dominates
    return (ax < 0) ? CMD_LEFT : CMD_RIGHT;
  }
}

// =============================================================
void sendCommand(char cmd) {
  uint8_t msg[1] = { (uint8_t)cmd };
  rfDriver.send(msg, 1);
  rfDriver.waitPacketSent();  // Block until transmission finishes (~10 ms at 2000 bps)
}

// LED feedback: quick blink on each transmitted command
void blinkLED(char cmd) {
  int blinks = 1;
  if (cmd == CMD_STOP)             blinks = 0;  // No blink when stopped
  else if (abs((int)cmd) > 0)      blinks = 1;

  for (int i = 0; i < blinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(15);
    digitalWrite(LED_PIN, LOW);
    delay(15);
  }
}

// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Gesture Car Transmitter — Starting up...");

  // GPIO setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Switch: internal pull-up; connect switch between GPIO and GND
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // I2C on default ESP32 pins (SDA=21, SCL=22)
  Wire.begin(21, 22);
  setupMPU6050();
  Serial.println("MPU6050 initialized.");

  // RF driver
  if (!rfDriver.init()) {
    Serial.println("ERROR: RF driver failed to initialize!");
    // Flash SOS on LED
    for (int i = 0; i < 6; i++) {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  } else {
    Serial.println("RF driver OK.");
  }

  // Startup blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(120);
    digitalWrite(LED_PIN, LOW);  delay(120);
  }
  Serial.println("Ready. Tilt to drive!");
}

// =============================================================
void loop() {
  // ── Toggle switch (active-LOW with pull-up) ──────────────
  bool switchNow = digitalRead(SWITCH_PIN);
  if (lastSwitch == HIGH && switchNow == LOW) {
    // Rising edge → toggle transmit enable
    txEnabled = !txEnabled;
    Serial.print("TX ");
    Serial.println(txEnabled ? "ENABLED" : "DISABLED");

    // Visual confirmation: fast triple blink = enabled, slow single = disabled
    if (txEnabled) {
      for (int i = 0; i < 3; i++) { digitalWrite(LED_PIN,HIGH); delay(80); digitalWrite(LED_PIN,LOW); delay(80); }
    } else {
      digitalWrite(LED_PIN, HIGH); delay(400); digitalWrite(LED_PIN, LOW);
      sendCommand(CMD_STOP);  // Safety: stop car when TX disabled
    }
    delay(50);  // Debounce
  }
  lastSwitch = switchNow;

  // ── Steady LED = TX active ───────────────────────────────
  digitalWrite(LED_PIN, txEnabled ? HIGH : LOW);

  // ── Throttle transmit rate ───────────────────────────────
  unsigned long now = millis();
  if (now - lastTxTime < TX_INTERVAL_MS) return;
  lastTxTime = now;

  if (!txEnabled) return;

  // ── Read MPU6050 ─────────────────────────────────────────
  int16_t ax, ay, az;
  readAccel(ax, ay, az);

  // ── Determine command ────────────────────────────────────
  char cmd = computeCommand(ax, ay);

  // Debug output (comment out to reduce serial noise)
  Serial.printf("ax=%6d  ay=%6d  →  cmd=%c\n", ax, ay, cmd);

  // ── Transmit (only if command changed OR periodically resend STOP) ──
  if (cmd != lastCmd || cmd == CMD_STOP) {
    sendCommand(cmd);
    Serial.print("TX: ");
    Serial.println(cmd);
    lastCmd = cmd;
  }
}
