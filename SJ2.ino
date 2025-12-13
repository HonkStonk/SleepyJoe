#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <TM1637Display.h>
#include <Wire.h>
#include <EEPROM.h>          
#include "RTClib.h"

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
  uint16_t th20_mV = (uint32_t)vIdle_mV * 20 / 100;  // 20% of idle

  if (drop_mV <= th4_mV) {
    return 100;          // very little sag -> perfect
  }
  if (drop_mV >= th20_mV) {
    return 0;            // huge sag -> terrible
  }

  // Map drop_mV from [th4_mV..th20_mV] -> [100..0]
  uint16_t span   = th20_mV - th4_mV;        // > 0
  uint16_t offset = drop_mV - th4_mV;        // 0..span
  uint16_t drop   = offset * 100UL / span;   // 0..100

  return (uint8_t)(100 - drop);
}

uint8_t computeBatteryHealthPercent(uint16_t vIdle_mV, uint16_t vSag_mV)
{
  uint8_t base      = basePercentFromIdle(vIdle_mV);        // 0..100
  uint8_t sagHealth = sagHealthPercent(vIdle_mV, vSag_mV);  // 0..100

  // Weight: 70% idle voltage, 30% sag behavior (tune later)
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

// -------------------- Mario melody on SOL_PIN (D9 / OC1A / Timer1) --------------------
// D9 on ATmega328P = PB1 = OC1A

static void timer1StartToggleOC1A(uint16_t freqHz) {
  // CTC mode (TOP = OCR1A), toggle OC1A on compare match
  // f_out = F_CPU / (2 * prescaler * (1 + OCR1A))
  const uint16_t presc = 64;

  uint32_t ocr = (F_CPU / (2UL * presc * (uint32_t)freqHz)) - 1UL;
  if (ocr > 65535UL) ocr = 65535UL;
  if (ocr < 1UL)     ocr = 1UL;

  // Ensure OC1A pin is output (PB1)
  DDRB |= _BV(DDB1);

  // Stop timer
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = (uint16_t)ocr;

  // Toggle OC1A on compare match
  TCCR1A = _BV(COM1A0);

  // CTC mode (WGM12=1), prescaler = 64 (CS11|CS10)
  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
}

static void timer1StopOC1A() {
  // Stop timer and disconnect OC1A toggling
  TCCR1A = 0;
  TCCR1B = 0;
}

// Plays one "note" using Timer1 toggle
static void playNoteTimer1_OC1A(uint16_t freqHz, uint16_t onMs, uint16_t offMs) {
  if (freqHz == 0) {
    // rest
    timer1StopOC1A();
    delay(onMs + offMs);
    return;
  }

  timer1StartToggleOC1A(freqHz);
  delay(onMs);
  timer1StopOC1A();
  PORTB &= ~_BV(PORTB1); // drive D9 low (only if it’s still output)

  // Make sure the gate isn't left "half-driven" by peripheral
  // (after stopping timer, OC1A is disconnected; keep pin low if it was output)
  // We'll restore DDR/PORT later anyway.
  delay(offMs);
}

// Public function: plays 7 notes and restores everything exactly as it was
void playMario7OnSolenoidPin9() {
  // Make sure we don't accidentally do a long DC pull before buzzing
  solenoidOff();
  delay(10);

  // ---- Save Timer1 + PB1 (D9) pin state ----
  uint8_t  savedTCCR1A = TCCR1A;
  uint8_t  savedTCCR1B = TCCR1B;
  uint16_t savedOCR1A  = OCR1A;
  uint16_t savedTCNT1  = TCNT1;
  uint8_t  savedTIMSK1 = TIMSK1;

  uint8_t savedDDRB  = DDRB;
  uint8_t savedPORTB = PORTB;

  // Disable Timer1 interrupts (you aren't using them, but be clean)
  TIMSK1 = 0;

  // ---- "Mario intro" 7 notes (recognizable) ----
  // Frequencies (approx):
  // E5=659, C5=523, G5=784, G4=392
  // Pattern: E5, E5, E5, C5, E5, G5, G4  (with small gaps)
  playNoteTimer1_OC1A(659, 120, 80);   // E5
  playNoteTimer1_OC1A(659, 120, 160);  // E5
  playNoteTimer1_OC1A(659, 120, 160);  // E5
  playNoteTimer1_OC1A(523, 120, 80);   // C5
  playNoteTimer1_OC1A(659, 120, 220);  // E5
  playNoteTimer1_OC1A(784, 160, 260);  // G5
  playNoteTimer1_OC1A(392, 160, 0);    // G4

  // ---- Restore Timer1 + PB1 state ----
  TCCR1A = savedTCCR1A;
  OCR1A  = savedOCR1A;
  TCNT1  = savedTCNT1;
  TCCR1B = savedTCCR1B;
  TIMSK1 = savedTIMSK1;

  DDRB  = (DDRB  & ~_BV(DDB1))  | (savedDDRB  & _BV(DDB1));
  PORTB = (PORTB & ~_BV(PORTB1))| (savedPORTB & _BV(PORTB1));

  // Safety: leave solenoid off
  solenoidOff();
}

//--------- a little better music magic ------------
struct MarioNote { uint16_t freq; uint16_t durMs; uint16_t gapMs; };

static const MarioNote mario7[] = {
  {659, 120,  80},  // E5
  {659, 120, 160},  // E5
  {659, 120, 160},  // E5
  {523, 120,  80},  // C5
  {659, 120, 220},  // E5
  {784, 160, 260},  // G5
  {392, 160,   0},  // G4
};

static bool     marioPlaying = false;
static uint8_t  marioIdx = 0;
static bool     marioToneOnPhase = false;
static uint32_t marioNextMs = 0;

// saved state to restore when finished
static uint8_t  savedTCCR1A, savedTCCR1B, savedTIMSK1;
static uint16_t savedOCR1A, savedOCR1B, savedICR1, savedTCNT1;
static uint8_t  savedDDRB, savedPORTB;

void marioStartNonBlocking() {
  if (marioPlaying) return;

  solenoidOff();
  delay(10); // optional safety pause

  // Save Timer1 + PB1
  savedTCCR1A = TCCR1A; savedTCCR1B = TCCR1B; savedTIMSK1 = TIMSK1;
  savedOCR1A  = OCR1A;  savedOCR1B  = OCR1B;  savedICR1   = ICR1;  savedTCNT1 = TCNT1;
  savedDDRB   = DDRB;   savedPORTB  = PORTB;

  TIMSK1 = 0;

  marioPlaying = true;
  marioIdx = 0;
  marioToneOnPhase = false;      // first update will start tone
  marioNextMs = millis();        // start immediately
}

static void marioStopAndRestore() {
  timer1StopOC1A();
  PORTB &= ~_BV(PORTB1); // keep gate low if still output

  // Restore Timer1 (start clock last)
  TCCR1A = savedTCCR1A;
  OCR1A  = savedOCR1A;
  OCR1B  = savedOCR1B;
  ICR1   = savedICR1;
  TCNT1  = savedTCNT1;
  TCCR1B = savedTCCR1B;
  TIMSK1 = savedTIMSK1;

  DDRB  = (DDRB  & ~_BV(DDB1))   | (savedDDRB  & _BV(DDB1));
  PORTB = (PORTB & ~_BV(PORTB1)) | (savedPORTB & _BV(PORTB1));

  solenoidOff();
  marioPlaying = false;
}

void marioUpdateNonBlocking() {
  if (!marioPlaying) return;

  uint32_t now = millis();
  if ((int32_t)(now - marioNextMs) < 0) return;

  if (marioIdx >= (sizeof(mario7) / sizeof(mario7[0]))) {
    marioStopAndRestore();
    return;
  }

  const MarioNote &n = mario7[marioIdx];

  if (!marioToneOnPhase) {
    // start tone phase
    if (n.freq == 0) {
      timer1StopOC1A();
      marioNextMs = now + n.durMs;   // treat dur as rest length
      marioToneOnPhase = true;       // next phase will be gap/advance
    } else {
      timer1StartToggleOC1A(n.freq);
      marioNextMs = now + n.durMs;
      marioToneOnPhase = true;
    }
  } else {
    // stop tone + gap, then advance
    timer1StopOC1A();
    PORTB &= ~_BV(PORTB1);
    marioNextMs = now + n.gapMs;
    marioToneOnPhase = false;
    marioIdx++;
  }
}

bool marioIsPlaying() { return marioPlaying; }

void marioAbort() {
  if (!marioPlaying) return;
  marioStopAndRestore();
}

// -------- Blink to signal rtc alarm trigger worked ------
void blinkRtcTrigger() {
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
  }
}

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

      blinkRtcTrigger();
      setupRTCAlarm();
    }

    if (loadBatteryStatsFromEEPROM(vIdle_mV, vSag_mV, last_healthPct)) {
      percent = last_healthPct;
    }
    else {
      vIdle_mV = (uint16_t)readVccFiltered(6);
      percent = basePercentFromIdle(vIdle_mV);
    }

    // --- Settings / display loop while main button held ---
    bool ledState = false;
    uint32_t lastBlink = millis();

    // For edge-detect on the four adjust buttons
    bool prevNowMinus  = false;
    bool prevNowPlus   = false;
    bool prevOpenMinus = false;
    bool prevOpenPlus  = false;

    int buttonLowCount = 0;   // low-level filter counter

    marioStartNonBlocking();

    while (true) {
      marioUpdateNonBlocking();
      bool buttonHigh = (digitalRead(BUTTON_PIN) == HIGH);

      if (!buttonHigh) {
        // Button is LOW: count how many consecutive times we see it low.
        if (buttonLowCount < 100) buttonLowCount++;   // 100 * 10 ms ≈ 1 s
      } else {
        buttonLowCount = 0; // any high resets the counter
      }

      if (buttonLowCount >= 100 && !marioIsPlaying()) {
        // Consider this a "real" release → exit config mode
        break;
      }

      // --- ONLY talk to RTC & displays while SW1 is actually ON ---
      if (buttonHigh) {
        DateTime now = rtc.now();

        // Show current time and open time
        showTime(displayCurrentTime, now.hour(), now.minute());
        showTriggerTime(displayTriggerTime, openHour, openMinute);
        showPercent(displayBat, percent);

        // Read buttons
        bool nowMinus  = digitalRead(BTN_TIME_NOW_MINUS);
        bool nowPlus   = digitalRead(BTN_TIME_NOW_PLUS);
        bool openMinus = digitalRead(BTN_TIME_OPEN_MINUS);
        bool openPlus  = digitalRead(BTN_TIME_OPEN_PLUS);

        // Rising edges = one-step adjustment
        if (nowMinus && !prevNowMinus) {
          adjustCurrentTime(-1);
        }
        if (nowPlus && !prevNowPlus) {
          adjustCurrentTime(+1);
        }
        if (openMinus && !prevOpenMinus) {
          adjustOpenTime(-1);
        }
        if (openPlus && !prevOpenPlus) {
          adjustOpenTime(+1);
        }

        prevNowMinus  = nowMinus;
        prevNowPlus   = nowPlus;
        prevOpenMinus = openMinus;
        prevOpenPlus  = openPlus;
      }

      // Blink LED every 500 ms without blocking (independent of button)
      uint32_t nowMs = millis();
      if (nowMs - lastBlink >= 500) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = nowMs;
      }

      delay(10);  // small debounce / loop delay
    }

    // Leaving settings mode
    digitalWrite(LED_BUILTIN, LOW);
    marioAbort();

    woke    = false;
    rtcWake = false;
    PCMSK2 |= (1 << PD4) | (1 << PD3);      // re-enable button/rtc interrupt
  }

  preparePinsForSleep();
  enterSleep();
}
