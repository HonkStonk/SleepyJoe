#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <TM1637Display.h>
#include <Wire.h>
#include <EEPROM.h>          
#include "RTClib.h"

// -------------------- Debug: Serial dump --------------------
#define DEBUG_SERIAL 1         // set to 0 to compile out all Serial debug

#if DEBUG_SERIAL
static void serialInitOnce() {
  static bool inited = false;
  if (inited) return;

  Serial.begin(115200);
  // Optional small wait so the USB-serial bridge can catch up (doesn't block long)
  delay(10);
  inited = true;
}

static void dumpBatteryEEPROM() {
  serialInitOnce();

  uint16_t vIdle = 0, vSag = 0;
  uint8_t pct = 0;

  bool ok = loadBatteryStatsFromEEPROM(vIdle, vSag, pct);

  Serial.println(F("---- Battery EEPROM ----"));
  Serial.print(F("magic ok: ")); Serial.println(ok ? F("YES") : F("NO"));

  if (ok) {
    Serial.print(F("vIdle_mV: ")); Serial.println(vIdle);
    Serial.print(F("vSag_mV : ")); Serial.println(vSag);
    Serial.print(F("health% : ")); Serial.println(pct);

    // Also show derived things so you can sanity-check what your code *would* compute
    uint8_t basePct = basePercentFromIdle(vIdle);
    uint8_t sagPct  = sagHealthPercent(vIdle, vSag);
    Serial.print(F("base%   : ")); Serial.println(basePct);
    Serial.print(F("sag%    : ")); Serial.println(sagPct);

    uint16_t duty = marioDutyFromPercent(pct);
    Serial.print(F("marioDuty(permille): ")); Serial.println(duty);
  }

  Serial.println(F("------------------------"));
}
#endif

// -------------------- Button helpers (must be near top to avoid Arduino auto-prototype issues) --------------------

struct RepeatBtn {
  bool     prevPressed = false;
  uint32_t pressedAtMs = 0;
  uint32_t lastFireMs  = 0;
};

static int stepFromHoldMs(uint32_t heldMs);
static int updateRepeatButton(RepeatBtn &b, bool pressed, uint32_t nowMs);

RTC_DS3231 rtc;

// --- Displays ---
#define TM_CLK 6
#define TM_DIO_BAT 7
TM1637Display displayBat(TM_CLK, TM_DIO_BAT);

#define TM_DIO_TIME 8
TM1637Display displayCurrentTime(TM_CLK, TM_DIO_TIME);

#define TM_DIO_TRIG 12
TM1637Display displayTriggerTime(TM_CLK, TM_DIO_TRIG);

// --- Buttons ---
#define BUTTON_PIN 4                  // wake / "enter settings" button
#define BTN_TIME_NOW_MINUS    10      // D10
#define BTN_TIME_NOW_PLUS     A0      // A0
#define BTN_TIME_OPEN_MINUS   A1      // A1
#define BTN_TIME_OPEN_PLUS    A2      // A2

// --- Solenoid (IRL540N low-side on D8) ---
#define SOL_PIN 9

// --- Powers DS3231 VCC+I2C pullups ---
#define RTC_POWER_PIN 5


// --- EEPROM layout for open time ---
#define EEPROM_MAGIC_ADDR      0
#define EEPROM_OPEN_HOUR_ADDR  1
#define EEPROM_OPEN_MIN_ADDR   2
#define EEPROM_MAGIC_VALUE   0x42

// --- EEPROM layout for battery stats ---
#define EEPROM_BATT_MAGIC_ADDR   10
#define EEPROM_BATT_VIDLE_L      11
#define EEPROM_BATT_VIDLE_H      12
#define EEPROM_BATT_VSAG_L       13
#define EEPROM_BATT_VSAG_H       14
#define EEPROM_BATT_PCT_ADDR     15
#define EEPROM_BATT_MAGIC_VALUE  0xB7

// --- Open/alarm time (default 02:30) ---
uint8_t openHour   = 2;
uint8_t openMinute = 30;

volatile bool woke    = false;
volatile bool rtcWake = false;

// -------------------- Interrupt --------------------

ISR(PCINT2_vect) {
  uint8_t now = PIND;

  // D4: wake when it's currently HIGH (SW1 closed / +4V present)
  if (now & (1 << PD4)) {
    //PCMSK2 &= ~(1 << PD4); // debounce: ignore further edges on D4
    woke = true;
  }

  // D3: DS3231 INT/SQW is active LOW (0 = alarm)
  if (!(now & (1 << PD3))) {
    rtcWake = true;
  }
}

// -------------------- Power / Sleep --------------------

void enterSleep() {
  // reaches 0.22µA in sleep - requires BOD disabled by burning efuse = 0xFF
  ADCSRA = 0;
  MCUCR = (1 << BODS) | (1 << BODSE);
  MCUCR = (1 << BODS);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();
  power_all_disable();         // shut down peripherals

  PCICR |= (1 << PCIE2);       // enable PCINT for Port D
  PCMSK2 |= (1 << PD4);        // enable pin D4
  PCMSK2 |= (1 << PD3);        // enable pin D3 (RTC)

  sleep_enable();
  sei();

  // >>> IMPORTANT: if an interrupt already happened, don't go to sleep
  if (woke || rtcWake) {
    sleep_disable();
    power_all_enable();
    return;
  }

  sleep_cpu();                 // sleep here
  sleep_disable();

  power_all_enable();          // restore peripherals
}

// -------------------- Vcc/battery measurement --------------------

long readVccOnce() {
  power_adc_enable();
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // Vcc ref, 1.1V input
  ADCSRA |= _BV(ADEN);          // ensure ADC is enabled
  delay(2);                     // wait for ref to settle
  ADCSRA |= _BV(ADSC);          // start conversion
  while (bit_is_set(ADCSRA, ADSC));

  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  uint16_t result = (high << 8) | low;

  power_adc_disable();

  long vcc = 1125300L / result; // 1125300 = 1.1*1023*1000
  return vcc; // in mV - this is the lower 3xAA stack
}

long readVccFiltered(uint8_t samples) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += readVccOnce();
    delay(2);
  }
  return (long)(acc / samples);
}

uint8_t basePercentFromIdle(uint16_t vIdle_mV) {
  if (vIdle_mV > 4700) vIdle_mV = 4700;
  if (vIdle_mV < 3000) vIdle_mV = 3000;
  long pct = (long)(vIdle_mV - 3000) * 100L / 1700L;
  if (pct < 0)   pct = 0;
  if (pct > 100)  pct = 100;
  return (uint8_t)pct;
}

uint8_t sagHealthPercent(uint16_t vIdle_mV, uint16_t vSag_mV)
{
  if (vSag_mV >= vIdle_mV) {
    // No sag or measurement noise -> perfect
    return 100;
  }

  uint16_t drop_mV = vIdle_mV - vSag_mV;

  // Thresholds in mV for 4% and 20% sag
  uint16_t th4_mV  = (uint32_t)vIdle_mV * 4  / 100;  // 4% of idle
  uint16_t th45_mV = (uint32_t)vIdle_mV * 45 / 100;  // 45% of idle

  if (drop_mV <= th4_mV) {
    return 100;          // very little sag -> perfect
  }
  if (drop_mV >= th45_mV) {
    return 0;            // huge sag -> terrible
  }

  // Map drop_mV from [th4_mV..th20_mV] -> [100..0]
  uint16_t span   = th45_mV - th4_mV;        // > 0
  uint16_t offset = drop_mV - th4_mV;        // 0..span
  uint16_t drop   = offset * 100UL / span;   // 0..100

  return (uint8_t)(100 - drop);
}

uint8_t computeBatteryHealthPercent(uint16_t vIdle_mV, uint16_t vSag_mV)
{
  uint8_t base      = basePercentFromIdle(vIdle_mV);        // 0..100
  uint8_t sagHealth = sagHealthPercent(vIdle_mV, vSag_mV);  // 0..100

  // Weight: 70% idle voltage, 30% sag behavior (tune later) - also does measured temperature help here?
  uint16_t h = (uint16_t)base * 70 + (uint16_t)sagHealth * 30;
  h /= 100;
  if (h > 100) h = 100;

  return (uint8_t)h;
}

// -------------------- Display helpers --------------------

void showPercent(TM1637Display &disp, int percent) {
  uint8_t segs[4];

  if (percent < 0)  percent = 0;
  if (percent > 99) percent = 99;

  segs[0] = disp.encodeDigit((percent / 10) % 10); // tens
  segs[1] = disp.encodeDigit(percent % 10);        // ones
  segs[2] = 0b01100011; // ABGF
  segs[3] = 0b01011100; // GCDE
  disp.setSegments(segs);
}

void showTime(TM1637Display &disp, int hours, int minutes) {
  int displayTime = hours * 100 + minutes;
  disp.showNumberDecEx(displayTime, 0b01000000, true); // colon on
}

void showTriggerTime(TM1637Display &disp, int hours, int minutes) {
  int trigTime = hours * 100 + minutes;
  disp.showNumberDecEx(trigTime, 0b01000000, true);
}

// -------------------- Button helpers --------------------

static int stepFromHoldMs(uint32_t heldMs) {
  if (heldMs >= 2500) return 30;  // long hold
  if (heldMs >= 1200) return 10;  // medium hold
  return 1;                       // short hold
}

// returns 0 if no action, otherwise returns +step or -step (you decide sign outside)
static int updateRepeatButton(RepeatBtn &b, bool pressed, uint32_t nowMs) {
  // tuning knobs
  const uint16_t HOLD_START_MS   = 350; // start repeating after this (tap still works)
  const uint16_t REPEAT_1_MS     = 220; // repeat interval while step=1
  const uint16_t REPEAT_10_MS    = 180; // repeat interval while step=10
  const uint16_t REPEAT_30_MS    = 150; // repeat interval while step=30

  // Rising edge => one immediate 1-minute step
  if (pressed && !b.prevPressed) {
    b.pressedAtMs = nowMs;
    b.lastFireMs  = nowMs;
    b.prevPressed = true;
    return 1;
  }

  // Released
  if (!pressed && b.prevPressed) {
    b.prevPressed = false;
    return 0;
  }

  // Held
  if (pressed) {
    uint32_t heldMs = nowMs - b.pressedAtMs;
    if (heldMs < HOLD_START_MS) {
      return 0;
    }

    int step = stepFromHoldMs(heldMs);
    uint16_t interval =
      (step == 1)  ? REPEAT_1_MS :
      (step == 10) ? REPEAT_10_MS :
                     REPEAT_30_MS;

    if ((uint32_t)(nowMs - b.lastFireMs) >= interval) {
      b.lastFireMs = nowMs;
      return step;
    }
  }

  return 0;
}

// -------------------- EEPROM helpers (open time) ---------

void loadOpenTimeFromEEPROM() {
  uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
  if (magic == EEPROM_MAGIC_VALUE) {
    uint8_t h = EEPROM.read(EEPROM_OPEN_HOUR_ADDR);
    uint8_t m = EEPROM.read(EEPROM_OPEN_MIN_ADDR);
    if (h < 24 && m < 60) {     // basic sanity check
      openHour   = h;
      openMinute = m;
      return;
    }
  }

  // No valid data: write defaults (02:30)
  openHour   = 2;
  openMinute = 30;
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.update(EEPROM_OPEN_HOUR_ADDR, openHour);
  EEPROM.update(EEPROM_OPEN_MIN_ADDR, openMinute);
}

void saveOpenTimeToEEPROM() {
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.update(EEPROM_OPEN_HOUR_ADDR, openHour);
  EEPROM.update(EEPROM_OPEN_MIN_ADDR, openMinute);
}

// -------------------- EEPROM helpers (battery stats) ---------

void saveBatteryStatsToEEPROM(uint16_t vIdle, uint16_t vSag, uint8_t pct) {
  EEPROM.update(EEPROM_BATT_MAGIC_ADDR, EEPROM_BATT_MAGIC_VALUE);
  EEPROM.update(EEPROM_BATT_VIDLE_L, lowByte(vIdle));
  EEPROM.update(EEPROM_BATT_VIDLE_H, highByte(vIdle));
  EEPROM.update(EEPROM_BATT_VSAG_L,  lowByte(vSag));
  EEPROM.update(EEPROM_BATT_VSAG_H,  highByte(vSag));
  EEPROM.update(EEPROM_BATT_PCT_ADDR, pct);
}

bool loadBatteryStatsFromEEPROM(uint16_t &vIdle, uint16_t &vSag, uint8_t &pct) {
  if (EEPROM.read(EEPROM_BATT_MAGIC_ADDR) != EEPROM_BATT_MAGIC_VALUE) {
    return false;
  }
  uint8_t vl = EEPROM.read(EEPROM_BATT_VIDLE_L);
  uint8_t vh = EEPROM.read(EEPROM_BATT_VIDLE_H);
  uint8_t sl = EEPROM.read(EEPROM_BATT_VSAG_L);
  uint8_t sh = EEPROM.read(EEPROM_BATT_VSAG_H);
  vIdle = (uint16_t)vh << 8 | vl;
  vSag  = (uint16_t)sh << 8 | sl;
  pct   = EEPROM.read(EEPROM_BATT_PCT_ADDR);
  return true;
}

// -------------------- DS3231 low-level config ---------

void configureRTCForVBATAlarm() {
  const byte DS3231_ADDRESS = 0x68;

  // ---- Control register 0x0E ----
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x0E);  // control register
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  uint8_t ctrl = Wire.read();

  // Bit7 EOSC = 0 (oscillator on, even on VBAT)
  ctrl &= ~(1 << 7);
  // Bit6 BBSQW = 1 (INT/SQW active on VBAT)
  ctrl |=  (1 << 6);
  // Bits4-3 RS2/RS1 = 0 (1 Hz / off; we use interrupt, not square wave)
  ctrl &= ~((1 << 4) | (1 << 3));
  // Bit2 INTCN = 1 (use interrupt on alarms)
  ctrl |=  (1 << 2);
  // Bit1 A2IE = 1 (enable Alarm2 interrupt)
  ctrl |=  (1 << 1);
  // Bit0 A1IE left 0 (we don't use Alarm1)
  ctrl &= ~(1 << 0);

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x0E);
  Wire.write(ctrl);
  Wire.endTransmission();

  // ---- Status register 0x0F ----
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x0F);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, (uint8_t)1);
  uint8_t status = Wire.read();

  // Clear OSF, A2F, A1F bits
  status &= ~((1 << 7) | (1 << 1) | (1 << 0));

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x0F);
  Wire.write(status);
  Wire.endTransmission();
}

// -------------------- RTC alarm handling --------------------

void setupRTCAlarm() {
  // We won't rely on writeSqwPinMode any more; we manage the control register ourselves
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  DateTime now = rtc.now();

  // Create a DateTime for today at openHour:openMinute
  DateTime alarmTime(now.year(), now.month(), now.day(),
                     openHour, openMinute, 0);

  // If open time already passed today, schedule for tomorrow
  if (alarmTime <= now) {
    alarmTime = alarmTime + TimeSpan(1, 0, 0, 0); // add 1 day
  }

  // Set Alarm2: match hours + minutes
  rtc.setAlarm2(alarmTime, DS3231_A2_Hour);

  // Make sure DS3231 is configured so Alarm2 drives INT/SQW on VBAT
  configureRTCForVBATAlarm();   // <<< NEW
}

// Adjust RTC "current time" by deltaMinutes
void adjustCurrentTime(int deltaMinutes) {
  DateTime now = rtc.now();
  long totalMinutes = now.hour() * 60L + now.minute() + deltaMinutes;

  // wrap around 0..(24*60-1)
  long dayMinutes = 24L * 60L;
  while (totalMinutes < 0) totalMinutes += dayMinutes;
  totalMinutes %= dayMinutes;

  int h = totalMinutes / 60;
  int m = totalMinutes % 60;

  // Keep same date, change hour/min, zero seconds
  DateTime newTime(now.year(), now.month(), now.day(), h, m, 0);
  rtc.adjust(newTime);
}

// Adjust open time by deltaMinutes and re-program alarm
void adjustOpenTime(int deltaMinutes) {
  long totalMinutes = openHour * 60L + openMinute + deltaMinutes;
  long dayMinutes = 24L * 60L;
  while (totalMinutes < 0) totalMinutes += dayMinutes;
  totalMinutes %= dayMinutes;

  openHour   = totalMinutes / 60;
  openMinute = totalMinutes % 60;

  saveOpenTimeToEEPROM();   // <<< NEW: keep user setting non-volatile
  setupRTCAlarm();
}

// -------- Solenoid control --------

void solenoidOn() {
  pinMode(SOL_PIN, OUTPUT);
  digitalWrite(SOL_PIN, HIGH);   // IRL540 gate HIGH -> solenoid ON
}

void solenoidOff() {
  digitalWrite(SOL_PIN, LOW);    // gate LOW -> solenoid OFF
  pinMode(SOL_PIN, INPUT);
}

// Called once when RTC alarm wakes the MCU
void hammerSolenoid() {
  // Ensure pin is an output and off
  pinMode(SOL_PIN, OUTPUT);
  solenoidOff();
  delay(10);

  // Pulse 1: 120 ms ON, 200 ms OFF
  solenoidOn();
  delay(120);
  solenoidOff();
  delay(200);

  // Pulse 2: 200 ms ON, 250 ms OFF
  solenoidOn();
  delay(200);
  solenoidOff();
  delay(250);

  // Pulse 3: 280 ms ON
  solenoidOn();
  delay(280);
  solenoidOff();
}

void hammerSolenoidOnce() {
  solenoidOff();
  delay(10);
  solenoidOn();
  delay(120); // works 100% with 3xAA + 3xAA in series
  solenoidOff();
}

void hammerSolenoidBuzz() {
  const uint8_t  PULSE_COUNT = 50;  // try 40 first
  const uint16_t ON_MS       = 17;  // 17 ms on
  const uint16_t OFF_MS      = 11;  // 1 ms off

  for (uint8_t i = 0; i < PULSE_COUNT; i++) {
    solenoidOn();
    delay(ON_MS);
    solenoidOff();
    delay(OFF_MS);
  }
}

void fireSolenoidWithSagMeasure(uint16_t &vIdle, uint16_t &vSag) {
  // 1) Idle
  vIdle = (uint16_t)readVccFiltered(6);

  // 2) Start pulse 1
  uint32_t t0 = millis();
  solenoidOn();
  delay(20);  // let the sag settle

  // 3) Measure under load
  vSag = (uint16_t)readVccFiltered(4);

  // 4) Keep holding to reach 120 ms total
  while (millis() - t0 < 120) { /* wait */ }
  solenoidOff();

  // 5) Gap, then second 120 ms pulse (no more measuring needed)
  delay(300);
  solenoidOn();
  delay(120);
  solenoidOff();
}

// -------------------- Melody volume compensator --------------------

// Base duty at "good battery"
static const uint16_t MARIO_DUTY_BASE_PERMILLE = 310;   // 31.0% at >=86% health

// This is what the ISR will actually use (read inside TIMER2_COMPA ISR)
static volatile uint16_t gMarioDutyPermille = MARIO_DUTY_BASE_PERMILLE;

// Map battery percent -> duty (permille).
// Rule: >=86% => 310‰, below that => increase duty.
// Tune knobs:
//  - PER_PERCENT_UP: how many permille to add per missing percent below 86
//  - MAX_DUTY: absolute safety cap (timer1Start already caps at 900, but keep this sane)
static uint16_t marioDutyFromPercent(uint8_t pct) {
  const uint8_t  THRESH_PCT      = 86;
  const uint16_t PER_PERCENT_UP  = 6;    // 6‰ per % below 86 -> 0% => 310 + 86*6 = 826‰
  const uint16_t MAX_DUTY        = 850;  // keep a little headroom under 900

  if (pct >= THRESH_PCT) return MARIO_DUTY_BASE_PERMILLE;

  uint8_t deficit = (uint8_t)(THRESH_PCT - pct);  // 1..86
  uint32_t duty = (uint32_t)MARIO_DUTY_BASE_PERMILLE + (uint32_t)deficit * PER_PERCENT_UP;

  if (duty > MAX_DUTY) duty = MAX_DUTY;
  return (uint16_t)duty;
}

// -------------------- Mario melody on SOL_PIN (D9 / OC1A / Timer1) --------------------
// Interrupt-driven scheduler via Timer2 (1ms tick). Timer0 (millis) is untouched.
// D9 on ATmega328P = PB1 = OC1A

// ---------- [1] PWM generation on Timer1 (same as you had) ----------
static void timer1StartPwmOC1A(uint16_t freqHz, uint16_t dutyPermille) {
  if (freqHz < 1) freqHz = 1;
  if (dutyPermille > 900) dutyPermille = 900;

  const uint16_t presc = 8;

  uint32_t top = (F_CPU / (uint32_t)presc / (uint32_t)freqHz) - 1UL;
  if (top > 65535UL) top = 65535UL;
  if (top < 10UL)    top = 10UL;

  DDRB |= _BV(DDB1);      // PB1 / D9 output

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  ICR1 = (uint16_t)top;

  uint32_t ticks = (uint32_t)(top + 1UL) * (uint32_t)dutyPermille / 1000UL;
  if (ticks < 1UL) ticks = 1UL;
  if (ticks > top) ticks = top;
  OCR1A = (uint16_t)ticks;

  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // prescaler 8
}

static void timer1StopOC1A() {
  TCCR1A = 0;
  TCCR1B = 0;
  PORTB &= ~_BV(PORTB1);  // force D9 low (still output)
}

// ---------- [2] Melody data ----------
struct MarioNote { uint16_t freq; uint16_t durMs; uint16_t gapMs; };

static const MarioNote mario7[] = {
  {659,  90,  20},  // E5
  {659, 110, 160},  // E5
  {659, 110, 140},  // E5
  {523, 120,  80},  // C5
  {659, 120, 180},  // E5
  {784, 150, 330},  // G5
  {392, 160,   0},  // G4
};

// ---------- [3] Timer2-driven scheduler state (ALL used by ISR => volatile) ----------
static volatile bool     marioPlaying = false;
static volatile uint8_t  marioIdx = 0;
static volatile bool     marioTonePhase = false;   // false = next is "tone ON", true = next is "gap"
static volatile uint16_t marioRemainingMs = 0;     // countdown in ms for current phase

// Saved Timer1 + PB1 state so we can restore cleanly after melody
static uint8_t  savedTCCR1A, savedTCCR1B, savedTIMSK1;
static uint16_t savedOCR1A, savedOCR1B, savedICR1, savedTCNT1;
static uint8_t  savedDDRB,  savedPORTB;

// ---------- [4] Timer2 1ms tick (CTC) ----------
static void marioTimer2Start_1msTick() {
  // Timer2 in CTC mode, interrupt every 1ms.
  //
  // We pick prescaler = 64 and compute OCR2A from F_CPU:
  // tick_hz = F_CPU / presc / (OCR2A+1)
  // Want tick_hz = 1000 => OCR2A = F_CPU/(presc*1000) - 1
  //
  // For 16 MHz: OCR2A = 16000000/(64*1000)-1 = 249  (what I assumed earlier)
  // For  8 MHz: OCR2A =  8000000/(64*1000)-1 = 124  (your case)

  const uint32_t presc = 64UL;
  uint32_t ocr = (F_CPU / (presc * 1000UL)) - 1UL;

  // Keep within 8-bit range (0..255). For normal 8/16 MHz this is safe.
  if (ocr > 255UL) ocr = 255UL;

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;

  OCR2A  = (uint8_t)ocr;

  TCCR2A = _BV(WGM21);                 // CTC
  // Prescaler 64: CS22=1, CS21=0, CS20=0
  TCCR2B = _BV(CS22);
  TIMSK2 = _BV(OCIE2A);                // enable compare A interrupt
}

static void marioTimer2Stop() {
  TIMSK2 = 0;        // disable Timer2 interrupts
  TCCR2A = 0;
  TCCR2B = 0;
}

// ---------- [5] ISR: advances the note schedule exactly in ms ----------
ISR(TIMER2_COMPA_vect) {
  if (!marioPlaying) return;

  if (marioRemainingMs > 0) {
    marioRemainingMs--;
    return;
  }

  // Time to transition phase
  if (marioIdx >= (sizeof(mario7) / sizeof(mario7[0]))) {
    // finished
    timer1StopOC1A();
    PORTB &= ~_BV(PORTB1);
    marioPlaying = false;
    marioTimer2Stop();
    return;
  }

  const MarioNote n = mario7[marioIdx]; // copy (safe, small)

  if (!marioTonePhase) {
    // Start tone phase
    if (n.freq == 0) {
      timer1StopOC1A();
    } else {
      timer1StartPwmOC1A(n.freq, gMarioDutyPermille);
    }
    marioRemainingMs = n.durMs;     // countdown tone duration
    marioTonePhase = true;          // next transition will be "gap/off"
  } else {
    // Start gap phase (tone off)
    timer1StopOC1A();
    PORTB &= ~_BV(PORTB1);

    marioRemainingMs = n.gapMs;     // countdown gap
    marioTonePhase = false;         // next transition will be next note tone-on
    marioIdx++;
  }
}

// ---------- [6] Public controls (same names you already use) ----------
bool marioIsPlaying() { return marioPlaying; }

void marioStartNonBlocking() {
  if (marioPlaying) return;

  // Make sure solenoid is not driven while we sing
  solenoidOff();
  delay(10);

  // Save Timer1 + PB1 state
  savedTCCR1A = TCCR1A; savedTCCR1B = TCCR1B; savedTIMSK1 = TIMSK1;
  savedOCR1A  = OCR1A;  savedOCR1B  = OCR1B;  savedICR1   = ICR1;  savedTCNT1 = TCNT1;
  savedDDRB   = DDRB;   savedPORTB  = PORTB;

  // Stop Timer1 interrupts during melody (you already did this)
  TIMSK1 = 0;

  // Arm scheduler
  cli();
  marioIdx = 0;
  marioTonePhase = false;  // next ISR transition starts tone immediately
  marioRemainingMs = 0;    // force immediate start on next ISR tick
  marioPlaying = true;
  sei();

  marioTimer2Start_1msTick(); // start 1ms metronome
}

static void marioStopAndRestore() {
  // Stop ISR scheduler first
  marioPlaying = false;
  marioTimer2Stop();

  // Stop PWM and force low
  timer1StopOC1A();
  PORTB &= ~_BV(PORTB1);

  // Restore Timer1 state (start clock last)
  TCCR1A = savedTCCR1A;
  OCR1A  = savedOCR1A;
  OCR1B  = savedOCR1B;
  ICR1   = savedICR1;
  TCNT1  = savedTCNT1;
  TCCR1B = savedTCCR1B;
  TIMSK1 = savedTIMSK1;

  // Restore PB1 direction + output state
  DDRB  = (DDRB  & ~_BV(DDB1))   | (savedDDRB  & _BV(DDB1));
  PORTB = (PORTB & ~_BV(PORTB1)) | (savedPORTB & _BV(PORTB1));

  solenoidOff();
}

void marioAbort() {
  if (!marioPlaying) return;
  cli();
  marioStopAndRestore();
  sei();
}

// --------- display init ---------------------------

void initDisplays() {
  pinMode(TM_CLK,      OUTPUT); // this shared clk pin absolutely needs an external 10k pullup
  pinMode(TM_DIO_BAT,  OUTPUT);
  pinMode(TM_DIO_TIME, OUTPUT);
  pinMode(TM_DIO_TRIG, OUTPUT);

  displayBat.setBrightness(0x0f);   // full brightness, display ON
  displayBat.clear();

  displayCurrentTime.setBrightness(0x0f);
  displayCurrentTime.clear();

  displayTriggerTime.setBrightness(0x0f);
  displayTriggerTime.clear();
}

void preparePinsForSleep() {
  // TM1637 modules – make sure they are high-Z to avoid backfeeding +4V
  pinMode(TM_CLK,       INPUT); digitalWrite(TM_CLK,       LOW);
  pinMode(TM_DIO_BAT,   INPUT); digitalWrite(TM_DIO_BAT,   LOW);
  pinMode(TM_DIO_TIME,  INPUT); digitalWrite(TM_DIO_TIME,  LOW);
  pinMode(TM_DIO_TRIG,  INPUT); digitalWrite(TM_DIO_TRIG,  LOW);

  // I2C pins to RTC – also high-Z, no pullups from the AVR side
  pinMode(A4, INPUT); digitalWrite(A4, LOW);  // SDA
  pinMode(A5, INPUT); digitalWrite(A5, LOW);  // SCL

  // D4 wake input: input, no pullup; R1 handles pull-down
  pinMode(BUTTON_PIN, INPUT);

  pinMode(SOL_PIN, INPUT);
  digitalWrite(SOL_PIN, LOW);

  pinMode(RTC_POWER_PIN, INPUT);
  digitalWrite(RTC_POWER_PIN, LOW); // turns off DS3231 VCC+I2C pullups
  delay(10);  // settle
}

// -------------------- Setup --------------------

void setup() {
  // Default: all unused pins input (no pullup)
  for (uint8_t i = 0; i < 20; i++) {
    pinMode(i, INPUT);      // NO pullup
    digitalWrite(i, LOW);   // ensure pullup off
  }

  pinMode(RTC_POWER_PIN, OUTPUT);
  digitalWrite(RTC_POWER_PIN, HIGH);   // power on DS3231 VCC + I2C pullups
  delay(25);                           // let the rail/pullups settle

  Wire.begin();  // initialize I2C first
  rtc.begin();   // init RTC

  loadOpenTimeFromEEPROM();   // restore user alarm time

  // Override the four button pins: no pullups
  pinMode(BTN_TIME_NOW_MINUS,  INPUT);
  pinMode(BTN_TIME_NOW_PLUS,   INPUT);
  pinMode(BTN_TIME_OPEN_MINUS, INPUT);
  pinMode(BTN_TIME_OPEN_PLUS,  INPUT);

  // Wake button (D4)
  pinMode(BUTTON_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Solenoid gate
  pinMode(SOL_PIN, OUTPUT);
  solenoidOff();

  wdt_disable();

  initDisplays();

  setupRTCAlarm();
  preparePinsForSleep();
  enterSleep();
}

// -------------------- Main loop --------------------

void loop() {
  if (woke || rtcWake) {
    pinMode(RTC_POWER_PIN, OUTPUT);
    digitalWrite(RTC_POWER_PIN, HIGH); // turns on DS3231 VCC+I2C pullups
    pinMode(BUTTON_PIN, INPUT);
    delay(150);        // power rail settle
    Wire.begin();      // (already done in setup, but harmless)
    initDisplays();
    delay(150);        // I2C/RTC settle (or just “system settle”)

    uint16_t vIdle_mV = 0, vSag_mV = 0;
    uint8_t last_healthPct = 0;
    int percent = 0;

    // If we woke due to RTC alarm, clear it now
    if (rtcWake) {
      rtc.clearAlarm(2);   // clear the latched A2F flag (releases INT/SQW)
      if (marioIsPlaying()) marioAbort();
      fireSolenoidWithSagMeasure(vIdle_mV, vSag_mV);
      uint8_t health = computeBatteryHealthPercent(vIdle_mV, vSag_mV);
      saveBatteryStatsToEEPROM(vIdle_mV, vSag_mV, health);
      setupRTCAlarm();
    }

    if (loadBatteryStatsFromEEPROM(vIdle_mV, vSag_mV, last_healthPct)) {
      percent = last_healthPct;
    }
    else {
      vIdle_mV = (uint16_t)readVccFiltered(6);
      percent = basePercentFromIdle(vIdle_mV);
    }
    #if DEBUG_SERIAL
      dumpBatteryEEPROM();
    #endif

    // Set Mario PWM duty from the best-known battery percent (health or fallback idle-percent)
    gMarioDutyPermille = marioDutyFromPercent((uint8_t)percent);

    // --- Settings / display loop while main button held ---
    bool ledState = false;
    uint32_t lastBlink = millis();
    uint32_t nextUiUpdateMs = 0;
    bool currentTimeWasEdited = false;

    int buttonLowCount = 0;   // low-level filter counter

    RepeatBtn rbNowMinus, rbNowPlus, rbOpenMinus, rbOpenPlus;

    if(woke) { marioStartNonBlocking(); } // only mario on config button

    // --- Phase-lock RTC time to millis() on an RTC second tick ---
    DateTime  baseNow; 
    uint32_t  baseNowMs = 0;
    DateTime t0 = rtc.now();
    uint8_t s0 = t0.second();

    // Wait for RTC second to change (max ~1s)
    while (true) {
      DateTime t1 = rtc.now();
      if (t1.second() != s0) {
        baseNow = t1;          // exact moment just after the tick
        baseNowMs = millis();  // capture MCU time right then
        break;
      }
    }
    int32_t  editNowDeltaMin  = 0;               // shadow adjustment
    int32_t  editOpenDeltaMin = 0;               // shadow adjustment
    int32_t  shadowNowSec = (int32_t)baseNow.hour() * 3600
                         + (int32_t)baseNow.minute() * 60
                         + (int32_t)baseNow.second();
    uint8_t baseOpenHour = openHour;
    uint8_t baseOpenMin  = openMinute;

    // helper lambdas (Arduino GCC supports lambdas fine)
    auto wrapDayMin = [](int32_t m) -> int32_t {
      const int32_t day = 24 * 60;
      while (m < 0) m += day;
      return m % day;
    };

    while (true) {
      bool buttonHigh = (digitalRead(BUTTON_PIN) == HIGH);

      if (!buttonHigh) {
        // Button is LOW: count how many consecutive times we see it low.
        if (buttonLowCount < 200) buttonLowCount++;   // 200 * 1ms ≈ 200ms
      } else {
        buttonLowCount = 0; // any high resets the counter
      }

      if (buttonLowCount >= 200 && !marioIsPlaying()) {
        // Consider this a "real" release → exit config mode
        break;
      }

      // --- ONLY do the expensive RTC+display stuff at 20 Hz when SW1 is held---
      uint32_t nowMs = millis();
      if (buttonHigh && (int32_t)(nowMs - nextUiUpdateMs) >= 0) {
        nextUiUpdateMs = nowMs + (marioIsPlaying() ? 150 : 50); // 50 ms => 20 Hz
        
        // current time (shadowed)
        uint32_t elapsedSec = (millis() - baseNowMs) / 1000UL;
        int32_t baseSec = (int32_t)baseNow.hour() * 3600
                        + (int32_t)baseNow.minute() * 60
                        + (int32_t)baseNow.second();
        int32_t nowSec = baseSec + (int32_t)elapsedSec + (int32_t)editNowDeltaMin * 60;
        nowSec %= 86400; if (nowSec < 0) nowSec += 86400;
        shadowNowSec = nowSec;  // <<< save for commit
        int nowH = nowSec / 3600;
        int nowM = (nowSec % 3600) / 60;

        // open time (shadowed)
        int32_t baseOpenTot = baseOpenHour * 60 + baseOpenMin;
        int32_t openTot = wrapDayMin(baseOpenTot + editOpenDeltaMin);
        int openH = openTot / 60;
        int openM = openTot % 60;

        showTime(displayCurrentTime, nowH, nowM);
        showTriggerTime(displayTriggerTime, openH, openM);

        showPercent(displayBat, percent);

        // Read buttons
        bool nowMinus  = digitalRead(BTN_TIME_NOW_MINUS);
        bool nowPlus   = digitalRead(BTN_TIME_NOW_PLUS);
        bool openMinus = digitalRead(BTN_TIME_OPEN_MINUS);
        bool openPlus  = digitalRead(BTN_TIME_OPEN_PLUS);

        uint32_t t = millis();

        int s;

        bool nowChanged = false;

        // Current time -
        s = updateRepeatButton(rbNowMinus, nowMinus, t);
        if (s) { editNowDeltaMin -= s; nowChanged = true; }

        // Current time +
        s = updateRepeatButton(rbNowPlus, nowPlus, t);
        if (s) { editNowDeltaMin += s; nowChanged = true; }

        // If user touched "current time", re-anchor the shadow clock to :00 seconds NOW.
        // This makes the next minute change happen 60s later, not "whatever RTC seconds were".
        if (nowChanged) {
          // Recompute the intended displayed time using current base + elapsed + delta
          uint32_t elapsedSec = (millis() - baseNowMs) / 1000UL;
          int32_t baseSec = (int32_t)baseNow.hour() * 3600
                          + (int32_t)baseNow.minute() * 60
                          + (int32_t)baseNow.second();

          int32_t newSec = baseSec + (int32_t)elapsedSec + (int32_t)editNowDeltaMin * 60;
          newSec %= 86400; if (newSec < 0) newSec += 86400;

          int newH = newSec / 3600;
          int newM = (newSec % 3600) / 60;

          // Anchor to exactly HH:MM:00 starting right now
          baseNow   = DateTime(baseNow.year(), baseNow.month(), baseNow.day(), newH, newM, 0);
          baseNowMs = millis();

          editNowDeltaMin = 0;                 // baked into baseNow
          shadowNowSec    = newH * 3600 + newM * 60;  // keep commit state sane
          currentTimeWasEdited = true;
        }

        // Open time -
        s = updateRepeatButton(rbOpenMinus, openMinus, t);
        if (s) editOpenDeltaMin -= s;

        // Open time +
        s = updateRepeatButton(rbOpenPlus, openPlus, t);
        if (s) editOpenDeltaMin += s;

        // also here: both open time + and open time - at the same time = fire solenoid
      }

      // Blink LED every 500 ms without blocking (independent of button)
      if (nowMs - lastBlink >= 500) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = nowMs;
      }

      delay(1);  // small debounce / loop delay
    }

    // Leaving settings mode
    // Commit shadow changes once (fast, deterministic)
    if (currentTimeWasEdited) {
      // Commit the *actual* shadow time at exit: baseNow + elapsed since baseNowMs
      uint32_t elapsedSec = (millis() - baseNowMs) / 1000UL;

      int32_t baseSec = (int32_t)baseNow.hour() * 3600
                      + (int32_t)baseNow.minute() * 60
                      + (int32_t)baseNow.second();

      int32_t nowSec = baseSec + (int32_t)elapsedSec;
      nowSec %= 86400; if (nowSec < 0) nowSec += 86400;

      int nowH = nowSec / 3600;
      int nowM = (nowSec % 3600) / 60;
      int nowS = nowSec % 60;

      DateTime newTime(baseNow.year(), baseNow.month(), baseNow.day(), nowH, nowM, nowS);
      rtc.adjust(newTime);
      setupRTCAlarm(); // since RTC time was changed - also sync when to do the alarm
    }

    if (editOpenDeltaMin != 0) {
      int32_t baseOpenTot = baseOpenHour * 60 + baseOpenMin;
      int32_t openTot = wrapDayMin(baseOpenTot + editOpenDeltaMin);
      openHour   = openTot / 60;
      openMinute = openTot % 60;

      saveOpenTimeToEEPROM();
      setupRTCAlarm();   // one reprogram
    }

    digitalWrite(LED_BUILTIN, LOW);
    marioAbort();

    woke    = false;
    rtcWake = false;
    PCMSK2 |= (1 << PD4) | (1 << PD3);      // re-enable button/rtc interrupt
  }

  preparePinsForSleep();
  enterSleep();
}
