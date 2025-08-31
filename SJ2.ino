#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <TM1637Display.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

#define TM_CLK 6
#define TM_DIO 7
TM1637Display displayBat(TM_CLK, TM_DIO);
#define TM_CLK_TIME 8
#define TM_DIO_TIME 9
TM1637Display displayCurrentTime(TM_CLK_TIME, TM_DIO_TIME);
#define TM_CLK_TRIG 11
#define TM_DIO_TRIG 12
TM1637Display displayTriggerTime(TM_CLK_TRIG, TM_DIO_TRIG);

#define BUTTON_PIN 4
volatile bool woke = false;
volatile bool rtcWake = false;

ISR(PCINT2_vect) {
  static uint8_t lastState = 0;
  uint8_t now = PIND;   // read port D state
  uint8_t changed = now ^ lastState;
  lastState = now;

  if (changed & (1 << PD4)) {
    // Button wake
    PCMSK2 &= ~(1 << PD4); // debounce disable
    woke = true;
  }

  if (changed & (1 << PD3)) {
    // DS3231 SQW wake
    rtcWake = true;
  }
}

void enterSleep() {
  // reaches 0.22µA in sleep - requires BOD disabled by burning efuse = 0xFF
  // disable ADC, BOD, peripherals
  ADCSRA = 0; 
  MCUCR = (1 << BODS) | (1 << BODSE);
  MCUCR = (1 << BODS);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();
  power_all_disable();         // shut down peripherals
  PCICR |= (1 << PCIE2);       // enable PCINT for Port D
  PCMSK2 |= (1 << PD4);        // enable pin D4
  PCMSK2 |= (1 << PD3);        // enable pin D3

  sleep_enable();
  sei();
  sleep_cpu();                 // sleep here
  sleep_disable();

  power_all_enable();          // restore peripherals
}

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

void showPercent(TM1637Display &disp, int percent) {
  uint8_t segs[4];

  // left 2 digits = percent value
  disp.encodeDigit((percent / 10) % 10); // tens
  segs[0] = disp.encodeDigit((percent / 10) % 10);
  segs[1] = disp.encodeDigit(percent % 10);
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
  disp.showNumberDecEx(displayTime, 0b01000000, true); // 0b01000000 = colon on
}

void showTriggerTime(TM1637Display &disp, int hours, int minutes) {
  int trigTime = hours * 100 + minutes;
  disp.showNumberDecEx(trigTime, 0b01000000, true); 
}

void setupRTCAlarm() {
  rtc.writeSqwPinMode(DS3231_OFF);  // make sure SQW is off
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  DateTime now = rtc.now();
  // Create a DateTime for today at 07:30
  DateTime alarmTime(now.year(), now.month(), now.day(), 7, 30, 0);
  // If 07:30 already passed, schedule for tomorrow
  if (alarmTime <= now) {
    alarmTime = alarmTime + TimeSpan(1,0,0,0); // add 1 day
  }
  // Set Alarm2: match hours + minutes
  rtc.setAlarm2(alarmTime, DS3231_A2_Hour);
}

void setup() {
  for (uint8_t i = 0; i < 20; i++) {
  pinMode(i, INPUT_PULLUP);
  }
  Wire.begin();  // initialize I2C first
  rtc.begin();   // init RTC
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  wdt_disable();
  displayBat.setBrightness(0x0f);   // full brightness
  displayBat.clear();
  displayCurrentTime.setBrightness(0x0f);
  displayCurrentTime.clear();
  displayTriggerTime.setBrightness(0x0f);
  displayTriggerTime.clear();
  setupRTCAlarm();
  enterSleep();
}

void loop() {
  if (woke) {
    delay(50);

    long vcc_mv = readVcc();
    int percent = batteryPercent(vcc_mv);
    showPercent(displayBat, percent);

    while (digitalRead(BUTTON_PIN) == HIGH) {
      showTime(displayCurrentTime, 00, 47);
      showTriggerTime(displayTriggerTime, 12, 34);
      digitalWrite(LED_BUILTIN, HIGH); delay(500);
      digitalWrite(LED_BUILTIN, LOW);  delay(500);
    }
    woke = false;
    PCMSK2 |= (1 << PD4);      // re-enable button
  }
  enterSleep();
}
