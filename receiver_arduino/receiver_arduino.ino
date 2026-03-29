// ============================================================
//  GESTURE CAR — RECEIVER (Arduino Uno + L298N + 433 MHz RX)
//  Library needed: RadioHead by Mike McCauley
//  Install via: Arduino IDE → Tools → Manage Libraries → "RadioHead"
// ============================================================

#include <RH_ASK.h>
#include <SPI.h>   // Required by RadioHead even if unused

// ── RF Receiver pin ─────────────────────────────────────────
#define RF_RX_PIN  12   // RF DATA → Arduino D12
#define RF_TX_PIN  11   // Dummy TX pin (not connected to anything)

// ── L298N motor driver pins ──────────────────────────────────
// Left motor  → OUT1 / OUT2
#define ENA  3    // PWM speed control — left motor (D3 is PWM on Uno)
#define IN1  5    // Direction bit A — D5
#define IN2  6    // Direction bit B — D6
// Right motor → OUT3 / OUT4
#define ENB  9    // PWM speed control — right motor (D9 is PWM on Uno)
#define IN3  10   // Direction bit C — D10
#define IN4  11   // Direction bit D — no PWM needed for this, D11

// ── Speed settings (0–255) ────────────────────────────────────
#define SPEED_NORMAL   180  // Straight-line drive speed
#define SPEED_TURN     140  // Speed of the faster side while turning
#define SPEED_TURN_SLOW 60  // Speed of the slower side while turning
                             // Set to 0 for a tight pivot turn

// ── Watchdog timeout ─────────────────────────────────────────
// If no packet received within this many ms, stop motors (safety)
#define WATCHDOG_MS  400

// ── Command bytes (must match transmitter) ───────────────────
#define CMD_FORWARD  'F'
#define CMD_BACKWARD 'B'
#define CMD_LEFT     'L'
#define CMD_RIGHT    'R'
#define CMD_STOP     'S'

// ── RF driver (bps=2000, rxPin=RF_RX_PIN, txPin=RF_TX_PIN) ──
RH_ASK rfDriver(2000, RF_RX_PIN, RF_TX_PIN, 0);

// ── State ────────────────────────────────────────────────────
unsigned long lastPacketTime = 0;
char          lastCmd        = 0;

// =============================================================
// Motor helpers
// =============================================================

void motorsStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void motorsForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL);
}

void motorsBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL);
}

// Gentle arc left: right motor faster, left motor slower
void motorsLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN_SLOW);   // left side slows down
  analogWrite(ENB, SPEED_TURN);        // right side stays fast
}

// Gentle arc right: left motor faster, right motor slower
void motorsRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN);        // left side stays fast
  analogWrite(ENB, SPEED_TURN_SLOW);   // right side slows down
}

// =============================================================
// Execute a command character
// =============================================================
void executeCommand(char cmd) {
  if (cmd == lastCmd) return;  // Already in this state
  lastCmd = cmd;

  switch (cmd) {
    case CMD_FORWARD:  motorsForward();  break;
    case CMD_BACKWARD: motorsBackward(); break;
    case CMD_LEFT:     motorsLeft();     break;
    case CMD_RIGHT:    motorsRight();    break;
    case CMD_STOP:
    default:           motorsStop();     break;
  }

  Serial.print("CMD: ");
  Serial.println(cmd);
}

// =============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Gesture Car Receiver — Starting up...");

  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motorsStop();

  // RF driver
  if (!rfDriver.init()) {
    Serial.println("ERROR: RF driver failed to initialize!");
    // Halt; no point continuing
    while (true) { delay(1000); }
  }
  Serial.println("RF driver OK. Waiting for commands...");
}

// =============================================================
void loop() {
  uint8_t buf[4];
  uint8_t buflen = sizeof(buf);

  // ── Check for incoming RF packet ─────────────────────────
  if (rfDriver.recv(buf, &buflen)) {
    // RH_ASK appends a checksum; a valid 1-byte payload returns buflen=1
    if (buflen >= 1) {
      char cmd = (char)buf[0];

      // Validate: only accept our known command characters
      if (cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
          cmd == CMD_LEFT    || cmd == CMD_RIGHT    ||
          cmd == CMD_STOP) {

        lastPacketTime = millis();
        executeCommand(cmd);
      }
      // Unknown byte → ignore (could be noise)
    }
  }

  // ── Watchdog: stop motors if signal lost ─────────────────
  if (lastCmd != CMD_STOP && millis() - lastPacketTime > WATCHDOG_MS) {
    Serial.println("Signal lost — stopping motors.");
    executeCommand(CMD_STOP);
  }
}
