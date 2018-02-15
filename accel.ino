#include "accel.h"
#include <Math.h>
#include <FastLED.h>
#include <avr/sleep.h>

#define MAX_LED_BRIGHTNESS 64
#define SLEEP_THRESHOLD 3 * 60 * 1000L // 3 minutes
#define LED_PIN 3
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

unsigned long lastTime = millis();
unsigned long timeSinceLastCheck = 0;
unsigned long timeSinceLastEvent = 0;
float currentAcc[3] = {0.0, 0.0, 0.0};

const byte ledCount = 16;
CRGB leds[ledCount];

// normalized
static float ledsX[12] = { -1.0, -0.92, -0.55,  0.0,  0.55,  0.92,  1.0,  0.92,  0.55,  0.0,  -0.55, -0.92 };
static float ledsY[12] = {  0.0, -0.39, -0.83, -1.0, -0.83, -0.39,  0.0,  0.39,  0.83,  1.0,   0.83,  0.39 };

static byte currentRoutine = 0;
static byte routineCount   = 10;
static void (*routines[10]) () = { followSide, fadeCycle, chase, followMarquee, twinkle, fade, followSingle, doubleChase, twinkleFade, followHorizon };

void setup() {
  // Enable interrupts on PCINT11 (pin 26) for tap interrupts
  /* PCMSK1 = 1<<PCINT11; */

  /* // Enable interrupts PCINT14..8 */
  /* PCICR  = 1<<PCIE1; */

  // Global interrupt enable
  /* sei(); */

  /* currentRoutine = random(routineCount); */

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, ledCount).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(MAX_LED_BRIGHTNESS);
}

void loop() {
  // Turn the first led red for 1 second
  leds[0] = CRGB::Red; 
  FastLED.show();
  delay(1000);

  // Set the first led back to black for 1 second
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(1000);
}

void xloop() {
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

    if (digitalRead(A3) == LOW) {
      interruptHandler();
      timeSinceLastEvent = 0;
    }
  }

  if (timeSinceLastEvent > SLEEP_THRESHOLD) { // 3 minutes
    goToSleep();
    timeSinceLastEvent = 0;
  }
}

void updateAccelData() {
  int accelCount[3];  // Stores the 12-bit signed value

  readAccelData(accelCount);  // Read the x/y/z adc values

  // Now we'll calculate the accleration value into actual g's
  float accelG[3];  // Stores the real accel value in g's
  for (byte i = 0; i < 3; i++) {
    accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set

    // use a rolling filter
    currentAcc[i] = 0.95 * accelG[i] + currentAcc[i] * 0.05;
  }
}

void followSide() {
  byte i;
  float dot;

  for (i = 0; i < ledCount; i++) {
    dot = accelerationDotProduct(ledsX[i], ledsY[i]);

    // invert dot so the up pointing led is lit
    /* SoftPWMSet(ledPins[i], byte(MAX_LED_BRIGHTNESS * constrain(-dot, 0.0, 1.0))); */
  }
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
  }
  /* SoftPWMSet(ledPins[lit], MAX_LED_BRIGHTNESS); */
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
    /* SoftPWMSet(ledPins[i], 0); */
  }
  do {
    newLed = random(ledCount);
  } while (newLed == led);
  led = newLed;
  /* SoftPWMSet(ledPins[led], MAX_LED_BRIGHTNESS); */
}

byte fadeValue = 32;
void fade() {
  byte value = MAX_LED_BRIGHTNESS * (cos(2 * PI * fadeValue/64)+1)/2;
  for (byte i = 0; i < ledCount; i++) {
    /* SoftPWMSet(ledPins[i], value); */
  }
  fadeValue++;
  if (fadeValue >= 64) {
    fadeValue = 0;
  }
}

void fadeCycle() {
  byte value;
  for (byte i = 0; i < ledCount; i++) {
    value = MAX_LED_BRIGHTNESS * constrain((cos((2 * PI * (fadeValue + i*10))/60)+1)/2, 0.0, 1.0);
    /* SoftPWMSet(ledPins[i], value); */
  }
  fadeValue++;
  if (fadeValue >= 60) {
    fadeValue = 0;
  }
}

void twinkleFade() {
  for (byte i = 0; i < ledCount; i++) {
    /* SoftPWMSet(ledPins[i], 0); */
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

void readAccelData(int *destination) {
  byte rawData[6];  // x/y/z accel register data stored here

  /* readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array */

  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
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
