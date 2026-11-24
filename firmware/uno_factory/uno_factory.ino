// Smart Mini Factory: Arduino Uno firmware. Hardware layer only; the rules are in core.cpp.
#include <Wire.h>
#include <avr/wdt.h>
#include "config.h"
#include "core.h"

const uint8_t IR1 = 2, IR2 = 3, STEP = 4, DIR = 5, EN = 6, ESTOP = 7, RELAY = 8, BTN_START = 9,
              BTN_STOP = 10, HOOD = 11, BUZ = 12, LG = 13, LY = A2, LR = A3;
const uint8_t PCA = 0x40;
Core core;
Inputs in;
uint16_t sent[6];
uint16_t zeroAdc = 512;
volatile uint16_t stepInc = 0;

void sendLine(const char *s) { Serial.write(s); }

// 20 kHz: one step whenever the phase rolls over; the pulse lasts one interrupt (50 us).
ISR(TIMER1_COMPA_vect) {
  static uint16_t phase = 0;
  PORTD &= ~_BV(4);
  uint16_t inc = stepInc;
  if (!inc) { phase = 0; return; }
  uint16_t old = phase; phase += inc;
  if (phase < old) PORTD |= _BV(4);
}

void pcaWrite(uint8_t reg, uint8_t v) { Wire.beginTransmission(PCA); Wire.write(reg); Wire.write(v); Wire.endTransmission(); }

void pcaBegin() {
  Wire.begin(); Wire.setClock(400000); Wire.setWireTimeout(3000, true);
  pcaWrite(0x00, 0x10);        // sleep, so the prescaler can be set
  pcaWrite(0xFE, 121);         // 50 Hz from the 25 MHz oscillator
  pcaWrite(0x00, 0x20);        // wake, auto-increment
  delay(1);
  pcaWrite(0xFD, 0x10);        // all outputs fully off until the first SRV command
}

void pcaServos() {
  bool change = false;
  for (uint8_t i = 0; i < 6; i++) if (sent[i] != core.out.srv[i]) change = true;
  if (!core.out.servos_on || !change) return;
  Wire.beginTransmission(PCA); Wire.write(0x06);
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t c = (uint32_t)core.out.srv[i] * 4096UL / 20000UL;
    Wire.write((uint8_t)0); Wire.write((uint8_t)0); Wire.write((uint8_t)c); Wire.write((uint8_t)(c >> 8));
  }
  if (Wire.endTransmission() == 0) for (uint8_t i = 0; i < 6; i++) sent[i] = core.out.srv[i];
}

void setup() {
  wdt_disable();
  digitalWrite(RELAY, LOW); pinMode(RELAY, OUTPUT);     // motor power off first
  digitalWrite(EN, HIGH); pinMode(EN, OUTPUT);          // stepper disabled
  pinMode(STEP, OUTPUT); pinMode(DIR, OUTPUT); digitalWrite(DIR, LOW);
  pinMode(IR1, INPUT_PULLUP); pinMode(IR2, INPUT_PULLUP); pinMode(ESTOP, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(HOOD, OUTPUT); pinMode(BUZ, OUTPUT); pinMode(LG, OUTPUT); pinMode(LY, OUTPUT); pinMode(LR, OUTPUT);
  TCCR2A = _BV(COM2A1) | _BV(WGM20); TCCR2B = _BV(CS20); OCR2A = 0;   // D11 at 31.4 kHz
  Serial.begin(115200);
  pcaBegin();
  uint32_t sum = 0; for (uint8_t i = 0; i < 64; i++) sum += analogRead(A0);   // relay is off: no current
  zeroAdc = sum / 64;
  noInterrupts(); TCCR1A = 0; TCCR1B = _BV(WGM12) | _BV(CS10); OCR1A = 799; TIMSK1 = _BV(OCIE1A); interrupts();
  core.begin(millis(), sendLine);
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  uint32_t ms = millis();
  while (Serial.available()) core.rx(Serial.read(), ms);

  in.estop_open = digitalRead(ESTOP) == HIGH;           // NC chain to ground: open or broken reads HIGH
  in.ir1 = digitalRead(IR1) == IR_BLOCKED_LEVEL;
  in.ir2 = digitalRead(IR2) == IR_BLOCKED_LEVEL;
  in.start = digitalRead(BTN_START) == LOW;
  in.stop = digitalRead(BTN_STOP) == LOW;
  static uint32_t adcMs = 0;
  if (ms - adcMs >= 5) {
    adcMs = ms;
    int d = (int)analogRead(A0) - (int)zeroAdc; if (d < 0) d = -d;
    in.mA = (uint32_t)d * 4883UL / 100UL;               // ACS712-20A: 100 mV per A
    in.mV = (uint32_t)analogRead(A1) * 21UL;            // 5 V / 1024 * 43 / 10
  }
  in.analog_ok = !BENCH_MODE;
  core.tick(ms, in);

  digitalWrite(RELAY, core.out.relay);
  digitalWrite(EN, core.out.relay ? LOW : HIGH);
  uint16_t inc = (uint16_t)(core.out.belt * (STEPS_PER_MM / 10.0f) * 65536.0f / 20000.0f);
  noInterrupts(); stepInc = inc; interrupts();
  OCR2A = core.out.led;
  digitalWrite(LG, core.out.g); digitalWrite(LY, core.out.y); digitalWrite(LR, core.out.r); digitalWrite(BUZ, core.out.buzzer);
  static uint32_t pcaMs = 0;
  if (ms - pcaMs >= 20) { pcaMs = ms; pcaServos(); }
}
