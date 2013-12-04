/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
static unsigned int ledCount = 4;
static unsigned int ledPins[12] = { 10, 11, 12, 13 };

bool toggle = false;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  for (int i = 0; i < ledCount; i++) {
    pinMode(ledPins[i], OUTPUT);     
  }
}

// the loop routine runs over and over again forever:
void loop() {
  for (int i = 0; i < ledCount; i++) {
    digitalWrite(ledPins[i], toggle);
    toggle = !toggle;
  }
  toggle = !toggle;
  
  delay(500);               // wait for a second
}
