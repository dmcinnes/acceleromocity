#include "accel.h"
#include <Math.h>
#include <FastLED.h>
#include <avr/sleep.h>

#define MAX_LED_BRIGHTNESS 32
#define SLEEP_THRESHOLD 3 * 60 * 1000L // 3 minutes
#define LED_PIN 3
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

#define X_PIN 0
#define Y_PIN 1
#define Z_PIN 2
#define ACC_MIN 285
#define ACC_MAX 430

unsigned long lastTime = millis();
unsigned long timeSinceLastCheck = 0;
unsigned long timeSinceLastEvent = 0;
double currentAcc[3] = {0.0, 0.0, 0.0};

const byte ledCount = 16;
CRGB leds[ledCount];

// normalized
float ledsX[ledCount];
float ledsY[ledCount];

static byte currentRoutine = 6;
static byte routineCount   = 10;
static void (*routines[10]) () = { twinkle, followSide, fadeCycle, chase, followMarquee, fade, followSingle, doubleChase, twinkleFade, followHorizon };

void setup() {
  // Enable interrupts on PCINT11 (pin 26) for tap interrupts
  /* PCMSK1 = 1<<PCINT11; */

  /* // Enable interrupts PCINT14..8 */
  /* PCICR  = 1<<PCIE1; */

  // Global interrupt enable
  /* sei(); */

  /* currentRoutine = random(routineCount); */

  float segment = (2 * PI) / ledCount;
  for (byte i = 0; i < ledCount; i++) {
    ledsX[i] = cos(i * segment);
    ledsY[i] = sin(i * segment);
  }

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, ledCount).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MAX_LED_BRIGHTNESS);

  Serial.begin(9600);
}

void loop() {
  unsigned int i;
  int output;

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  timeSinceLastCheck += elapsedTime;
  timeSinceLastEvent += elapsedTime;

  if (timeSinceLastCheck > 100) {
    updateAccelData();

    (*routines[currentRoutine])();

    timeSinceLastCheck = 0;

    /* if (digitalRead(A3) == LOW) { */
    /*   interruptHandler(); */
    /*   timeSinceLastEvent = 0; */
    /* } */
  }

  /* if (timeSinceLastEvent > SLEEP_THRESHOLD) { // 3 minutes */
  /*   goToSleep(); */
  /*   timeSinceLastEvent = 0; */
  /* } */
}

void updateAccelData() {
  int xRead = analogRead(X_PIN);
  int yRead = analogRead(Y_PIN);
  int zRead = analogRead(Z_PIN);

  int xAng = map(xRead, ACC_MIN, ACC_MAX, -90, 90);
  int yAng = map(yRead, ACC_MIN, ACC_MAX, -90, 90);
  int zAng = map(zRead, ACC_MIN, ACC_MAX, -90, 90);

  currentAcc[0] = xAng / 90.0;
  currentAcc[1] = yAng / 90.0;
  currentAcc[2] = zAng / 90.0;

  Serial.print("RAW x: ");
  Serial.print(xRead);
  Serial.print(" | y: ");
  Serial.print(yRead);
  Serial.print(" | z: ");
  Serial.print(zRead);
  Serial.print(" CALC x: ");
  Serial.print(currentAcc[0]);
  Serial.print(" | y: ");
  Serial.print(currentAcc[1]);
  Serial.print(" | z: ");
  Serial.println(currentAcc[2]);
}

void followSide() {
  byte i, value;
  float dot;

  for (i = 0; i < ledCount; i++) {
    dot = accelerationDotProduct(ledsX[i], ledsY[i]);

    // invert dot so the up pointing led is lit
    value = byte(255 * constrain(-dot, 0.0, 1.0));
    leds[i] = CRGB(value, value, value);
  }
  FastLED.show();
}

void followSingle() {
  byte i, lit;
  float max, dot;

  lit = 0;
  max = 0.0;
  for (i = 0; i < ledCount; i++) {
    dot = constrain(-accelerationDotProduct(ledsX[i], ledsY[i]), 0.0, 1.0);
    if (dot > max) {
      max = dot;
      lit = i;
    }
    /* SoftPWMSet(ledPins[i], 0); */
    leds[i] = CRGB::Black;
  }
  leds[lit] = CRGB::Blue;
  FastLED.show();
}

void followHorizon() {
  byte i, lit;
  float min, dot;

  lit = 0;
  min = 1.0;
  for (i = 0; i < ledCount; i++) {
    dot = abs(-accelerationDotProduct(ledsX[i], ledsY[i]));
    if (dot < min) {
      min = dot;
      lit = i;
    }
    /* SoftPWMSet(ledPins[i], 0); */
  }
  /* SoftPWMSet(ledPins[lit], MAX_LED_BRIGHTNESS); */
  lit = (lit + 6) % 12;
  /* SoftPWMSet(ledPins[lit], MAX_LED_BRIGHTNESS); */
}

void followMarquee() {
  byte i, lit;
  float max, dot;

  lit = 0;
  max = 0.0;
  for (i = 0; i < ledCount; i++) {
    dot = constrain(-accelerationDotProduct(ledsX[i], ledsY[i]), 0.0, 1.0);
    if (dot > max) {
      max = dot;
      lit = i;
    }
    /* SoftPWMSet(ledPins[i], 0); */
  }

  for (i = lit % 3; i < ledCount; i += 3) {
    /* SoftPWMSet(ledPins[i], MAX_LED_BRIGHTNESS); */
  }
}

byte led = 0;
void chase() {
  for (byte i = 0; i < ledCount; i++) {
    /* SoftPWMSet(ledPins[i], 0); */
  }
  led++;
  if (led >= ledCount) {
    led = 0;
  }
  /* SoftPWMSet(ledPins[led], MAX_LED_BRIGHTNESS); */
}

void doubleChase() {
  for (byte i = 0; i < ledCount; i++) {
    /* SoftPWMSet(ledPins[i], 0); */
  }
  led++;
  if (led >= ledCount) {
    led = 0;
  }
  /* SoftPWMSet(ledPins[led], MAX_LED_BRIGHTNESS); */
  /* SoftPWMSet(ledPins[(led + ledCount/2) % ledCount], MAX_LED_BRIGHTNESS); */
}

void twinkle() {
  byte newLed;
  for (byte i = 0; i < ledCount; i++) {
    leds[i] = CRGB::Black;
  }
  do {
    newLed = random(ledCount);
  } while (newLed == led);
  led = newLed;
  leds[led] = CRGB::White;
  FastLED.show();
}

byte fadeValue = 32;
void fade() {
  byte value = MAX_LED_BRIGHTNESS * (cos(2 * PI * fadeValue/64)+1)/2;
  for (byte i = 0; i < ledCount; i++) {
    leds[i] = CRGB(value, value, value);
  }
  fadeValue++;
  if (fadeValue >= 64) {
    fadeValue = 0;
  }
  FastLED.show();
}

void fadeCycle() {
  byte value;
  for (byte i = 0; i < ledCount; i++) {
    value = 255 * constrain((cos((2 * PI * (fadeValue + i*10))/60)+1)/2, 0.0, 1.0);
    leds[i] = CRGB(value, value, value);
  }
  fadeValue++;
  if (fadeValue >= 60) {
    fadeValue = 0;
  }
  FastLED.show();
}

void twinkleFade() {
  for (byte i = 0; i < ledCount; i++) {
    leds[i] = CRGB::Black;
  }

  byte value = MAX_LED_BRIGHTNESS * sin(PI * fadeValue/16);
  /* SoftPWMSet(ledPins[led], value); */

  fadeValue++;
  if (fadeValue >= 16) {
    fadeValue = 0;

    byte newLed;
    do {
      newLed = random(ledCount);
    } while (newLed == led);
    led = newLed;
  }
}

float accelerationDotProduct(float x, float y) {
  return currentAcc[0] * x + currentAcc[1] * y;
}

void goToSleep(void) {
  for (byte i = 0; i < ledCount; i++) {
    /* digitalWrite(ledPins[i], 0); */
  }

  ADCSRA &= ~_BV(ADEN); // disable ADC
  ACSR   |= _BV(ACD);   // disable the analog comparator

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  // turn off the brown-out detector.
  // must have an ATtiny45 or ATtiny85 rev C or later for software to be able to disable the BOD.
  // current while sleeping will be <0.5uA if BOD is disabled, <25uA if not.
  cli();
  uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  // turn off the brown-out detector
  uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;
  sei();                         // ensure interrupts enabled so we can wake up again
  sleep_cpu();                   // go to sleep
  cli();                         // wake up here, disable interrupts
  sleep_disable();

  sei();                         // enable interrupts again

  ADCSRA |= _BV(ADEN);   // enable ADC
  ACSR   &= ~_BV(ACD);   // enable the analog comparator
}

void nextRoutine() {
  currentRoutine++;
  if (currentRoutine >= routineCount) {
    currentRoutine = 0;
  }
}

// interrupts
EMPTY_INTERRUPT(PCINT1_vect); // Accelerometer I2
