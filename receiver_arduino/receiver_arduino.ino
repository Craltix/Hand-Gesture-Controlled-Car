// ============================================================
//  GESTURE CAR — RECEIVER (Arduino Uno + L298N + 433 MHz RX)
//  Library needed: RadioHead by Mike McCauley
//  Install via: Arduino IDE → Tools → Manage Libraries → "RadioHead"
// ============================================================

#include <RH_ASK.h>
#include <SPI.h>   // Required by RadioHead even if unused

// ── RF Receiver pin ──────────────────────────────────────────
#define RF_RX_PIN  12   // RF DATA → Arduino D12
// NOTE: We do NOT define RF_TX_PIN separately because the previous
// code assigned it to pin 11 — the same pin used by IN4 below —
// causing a direct conflict. The dummy TX pin is passed inline as
// a literal to the RH_ASK constructor so it's clear it's unused.
// Pin 7 is free and does not conflict with any other assignment.
#define RF_TX_DUMMY  7

// ── L298N motor driver pins ──────────────────────────────────
// Left motor  → L298N OUT1 / OUT2
#define ENA  3    // PWM speed — left  (D3 is hardware PWM on Uno)
#define IN1  5    // Direction bit
#define IN2  6    // Direction bit
// Right motor → L298N OUT3 / OUT4
#define ENB  9    // PWM speed — right (D9 is hardware PWM on Uno)
#define IN3  10   // Direction bit
// FIX: IN4 was previously pin 11, which collided with RF_TX_PIN 11.
// Moved to D4 which is free and has no special function on the Uno.
#define IN4  4    // Direction bit  (was 11 — CONFLICT FIXED)

// ── Speed settings (0–255) ────────────────────────────────────
#define SPEED_NORMAL    180  // Both motors straight ahead
#define SPEED_TURN      160  // Faster side while turning
#define SPEED_TURN_SLOW  50  // Slower side while turning (set to 0 for pivot)

// ── Watchdog timeout ─────────────────────────────────────────
#define WATCHDOG_MS  400     // Stop motors if no valid packet for 400 ms

// ── Command bytes (must match transmitter exactly) ────────────
#define CMD_FORWARD  'F'
#define CMD_BACKWARD 'B'
#define CMD_LEFT     'L'
#define CMD_RIGHT    'R'
#define CMD_STOP     'S'

// ── RF driver ────────────────────────────────────────────────
// FIX 1: The original code passed 0 as a 4th (PTT) argument.
//   RH_ASK::init() calls pinMode(_pttPin, OUTPUT) on whatever pin
//   is passed, so passing 0 forced Arduino pin 0 (hardware Serial RX)
//   to OUTPUT, silently breaking all Serial communication.
// FIX 2: Three-argument constructor omits the PTT pin entirely so
//   the library never touches pin 0 or any other unintended pin.
RH_ASK rfDriver(2000, RF_RX_PIN, RF_TX_DUMMY);

// ── State ─────────────────────────────────────────────────────
unsigned long lastPacketTime = 0;
char          lastCmd        = CMD_STOP;  // Init to STOP, not 0

// =============================================================
// Motor helpers — all pin writes are explicit to avoid hidden state
// =============================================================

void motorsStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void motorsForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL);
}

void motorsBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL);
}

// Arc left: right motor faster, left motor slower
void motorsLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN_SLOW);
  analogWrite(ENB, SPEED_TURN);
}

// Arc right: left motor faster, right motor slower
void motorsRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_TURN);
  analogWrite(ENB, SPEED_TURN_SLOW);
}

// =============================================================
void executeCommand(char cmd) {
  if (cmd == lastCmd) return;  // Already in this state — no redundant writes
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
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motorsStop();

  // FIX: Seed lastPacketTime here so the watchdog timer doesn't
  // immediately fire on boot (previously lastPacketTime=0 and
  // lastCmd=0≠CMD_STOP meant the watchdog printed "Signal lost"
  // before any packet could arrive).
  lastPacketTime = millis();

  if (!rfDriver.init()) {
    Serial.println("ERROR: RF driver failed to initialize!");
    while (true) { delay(1000); }  // Halt — nothing works without RF
  }
  Serial.println("RF driver OK. Waiting for commands...");
}

// =============================================================
void loop() {
  uint8_t buf[4];
  uint8_t buflen = sizeof(buf);

  // ── Check for incoming RF packet ─────────────────────────
  if (rfDriver.recv(buf, &buflen)) {
    if (buflen >= 1) {
      char cmd = (char)buf[0];

      // Whitelist: only accept our five known command bytes;
      // any other byte (noise, interference) is silently dropped.
      if (cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
          cmd == CMD_LEFT    || cmd == CMD_RIGHT    ||
          cmd == CMD_STOP) {

        lastPacketTime = millis();
        executeCommand(cmd);
      }
    }
  }

  // ── Watchdog: stop motors on signal loss ─────────────────
  if (lastCmd != CMD_STOP && millis() - lastPacketTime > WATCHDOG_MS) {
    Serial.println("Signal lost — stopping motors.");
    executeCommand(CMD_STOP);
  }
}
