#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <TM1637Display.h>
#include <Wire.h>
#include <EEPROM.h>          // <<< NEW
#include "RTClib.h"

RTC_DS3231 rtc;

// --- Displays ---
#define TM_CLK 6
#define TM_DIO 7
TM1637Display displayBat(TM_CLK, TM_DIO);

#define TM_CLK_TIME 6
#define TM_DIO_TIME 9
TM1637Display displayCurrentTime(TM_CLK_TIME, TM_DIO_TIME);

#define TM_CLK_TRIG 6
#define TM_DIO_TRIG 12
TM1637Display displayTriggerTime(TM_CLK_TRIG, TM_DIO_TRIG);

// --- Buttons ---
#define BUTTON_PIN 4                  // wake / "enter settings" button
#define BTN_TIME_NOW_MINUS    10      // D10
#define BTN_TIME_NOW_PLUS     A0      // A0
#define BTN_TIME_OPEN_MINUS   A1      // A1
#define BTN_TIME_OPEN_PLUS    A2      // A2

// --- EEPROM layout for open time ---  <<< NEW
#define EEPROM_MAGIC_ADDR      0
#define EEPROM_OPEN_HOUR_ADDR  1
#define EEPROM_OPEN_MIN_ADDR   2
#define EEPROM_MAGIC_VALUE   0x42

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

// -------------------- Vcc measurement --------------------

long readVcc() {
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
  return vcc; // in mV
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

int batteryPercent(long vcc_mv) {
  if (vcc_mv > 4700) vcc_mv = 4700;
  if (vcc_mv < 3000) vcc_mv = 3000;
  return (int)((vcc_mv - 3000) * 99L / (1700));
}

void showTime(TM1637Display &disp, int hours, int minutes) {
  int displayTime = hours * 100 + minutes;
  disp.showNumberDecEx(displayTime, 0b01000000, true); // colon on
}

void showTriggerTime(TM1637Display &disp, int hours, int minutes) {
  int trigTime = hours * 100 + minutes;
  disp.showNumberDecEx(trigTime, 0b01000000, true);
}

// -------------------- EEPROM helpers (open time) ---------  <<< NEW

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
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.write(EEPROM_OPEN_HOUR_ADDR, openHour);
  EEPROM.write(EEPROM_OPEN_MIN_ADDR, openMinute);
}

void saveOpenTimeToEEPROM() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.write(EEPROM_OPEN_HOUR_ADDR, openHour);
  EEPROM.write(EEPROM_OPEN_MIN_ADDR, openMinute);
}

// -------------------- DS3231 low-level config ---------  <<< NEW

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
  pinMode(TM_DIO,      OUTPUT);
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
  pinMode(TM_DIO,       INPUT); digitalWrite(TM_DIO,       LOW);
  pinMode(TM_CLK_TIME,  INPUT); digitalWrite(TM_CLK_TIME,  LOW);
  pinMode(TM_DIO_TIME,  INPUT); digitalWrite(TM_DIO_TIME,  LOW);
  pinMode(TM_CLK_TRIG,  INPUT); digitalWrite(TM_CLK_TRIG,  LOW);
  pinMode(TM_DIO_TRIG,  INPUT); digitalWrite(TM_DIO_TRIG,  LOW);

  // I2C pins to RTC – also high-Z, no pullups from the AVR side
  pinMode(A4, INPUT); digitalWrite(A4, LOW);  // SDA
  pinMode(A5, INPUT); digitalWrite(A5, LOW);  // SCL

  // D4 wake input: input, no pullup; R1 handles pull-down
  pinMode(BUTTON_PIN, INPUT);
}

// -------------------- Setup --------------------

void setup() {
  // Default: all unused pins input (no pullup)
  for (uint8_t i = 0; i < 20; i++) {
    pinMode(i, INPUT);      // NO pullup
    digitalWrite(i, LOW);   // ensure pullup off
  }

  Wire.begin();  // initialize I2C first
  rtc.begin();   // init RTC

  loadOpenTimeFromEEPROM();   // <<< NEW: restore user alarm time

  // Override the four button pins: no pullups
  pinMode(BTN_TIME_NOW_MINUS,  INPUT);
  pinMode(BTN_TIME_NOW_PLUS,   INPUT);
  pinMode(BTN_TIME_OPEN_MINUS, INPUT);
  pinMode(BTN_TIME_OPEN_PLUS,  INPUT);

  // Wake button (D4)
  pinMode(BUTTON_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  wdt_disable();

  initDisplays();

  setupRTCAlarm();
  preparePinsForSleep();
  enterSleep();
}

// -------------------- Main loop --------------------

void loop() {
  if (woke || rtcWake) {
    pinMode(BUTTON_PIN, INPUT);
    delay(400);  // settle

    initDisplays();

    long vcc_mv = readVcc();
    int percent = batteryPercent(vcc_mv);

    // If we woke due to RTC alarm, clear it now
    if (rtcWake) {
      blinkRtcTrigger();
      rtc.clearAlarm(2);
      configureRTCForVBATAlarm();  // re-enable A2IE / clear flags  <<< NEW safety
    }

    // --- Settings / display loop while main button held ---
    bool ledState = false;
    uint32_t lastBlink = millis();

    // For edge-detect on the four adjust buttons
    bool prevNowMinus  = false;
    bool prevNowPlus   = false;
    bool prevOpenMinus = false;
    bool prevOpenPlus  = false;

    int buttonLowCount = 0;   // <<< NEW: low-level filter counter

    while (true) {
      bool buttonHigh = (digitalRead(BUTTON_PIN) == HIGH);

      if (!buttonHigh) {
        // Button is LOW: count how many consecutive times we see it low.
        if (buttonLowCount < 50) buttonLowCount++;   // 50 * 20 ms ≈ 1 s
      } else {
        buttonLowCount = 0; // any high resets the counter
      }

      if (buttonLowCount >= 50) {
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

      delay(20);  // small debounce / loop delay
    }

    // Leaving settings mode
    digitalWrite(LED_BUILTIN, LOW);

    woke    = false;
    rtcWake = false;
    PCMSK2 |= (1 << PD4);      // re-enable button interrupt
  }

  preparePinsForSleep();
  enterSleep();
}
