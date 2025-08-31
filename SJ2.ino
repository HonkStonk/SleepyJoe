#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <TM1637Display.h>

#define TM_CLK 6
#define TM_DIO 7
TM1637Display displayBat(TM_CLK, TM_DIO);

#define BUTTON_PIN 4
volatile bool woke = false;

ISR(PCINT2_vect) {
  PCMSK2 &= ~(1 << PD4); // disable interrupt mask (debounce)
  woke = true;
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

void setup() {
  for (uint8_t i = 0; i < 20; i++) {
  pinMode(i, INPUT_PULLUP);
  }
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  wdt_disable();
  displayBat.setBrightness(0x0f);   // full brightness
  displayBat.clear();
  enterSleep();
}

void loop() {
  if (woke) {
    delay(50);

    long vcc_mv = readVcc();
    int percent = batteryPercent(vcc_mv);
    showPercent(displayBat, percent);

    while (digitalRead(BUTTON_PIN) == HIGH) {
      digitalWrite(LED_BUILTIN, HIGH); delay(500);
      digitalWrite(LED_BUILTIN, LOW);  delay(500);
    }
    woke = false;
    PCMSK2 |= (1 << PD4);      // re-enable button
  }
  enterSleep();
}
