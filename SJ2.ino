#include <avr/sleep.h>
#include <avr/power.h>
#include <Arduino.h>
#include <avr/wdt.h>

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

void setup() {
  for (uint8_t i = 0; i < 20; i++) {
  pinMode(i, INPUT_PULLUP);
  }
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  wdt_disable();
  enterSleep();
}

void loop() {
  if (woke) {
    delay(50);
    while (digitalRead(BUTTON_PIN) == HIGH) {
      digitalWrite(LED_BUILTIN, HIGH); delay(500);
      digitalWrite(LED_BUILTIN, LOW);  delay(500);
    }
    woke = false;
    PCMSK2 |= (1 << PD4);      // re-enable button
  }
  enterSleep();
}
