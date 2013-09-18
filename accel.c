/*
 MMA8452Q Basic Example Code
 Nathan Seidle
 SparkFun Electronics
 November 5, 2012

 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This example code shows how to read the X/Y/Z accelerations and basic functions of the MMA5842. It leaves out
 all the neat features this IC is capable of (tap, orientation, and inerrupts) and just displays X/Y/Z. See
 the advanced example code to see more features.

 Hardware setup:
 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA -------^^(330)^^------- A4
 SCL -------^^(330)^^------- A5
 GND ---------------------- GND

 The MMA8452 is 3.3V so we recommend using 330 or 1k resistors between a 5V Arduino and the MMA8452 breakout.

 The MMA8452 has built in pull-up resistors for I2C so you do not need additional pull-ups.
 */

#define F_CPU 8000000UL // 8 MHz

#include <util/delay.h>
#include <i2cmaster.h>

// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A

#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.


unsigned long lastTime;
unsigned int timeSinceLastCheck = 0;
float currentAcc[3] = {0.0, 0.0, 0.0};

static float ledsX[3] = { -0.5,  0.5, 0.0 };
static float ledsY[3] = { -0.5, -0.5, 1.0 };
static unsigned int ledCount = 3;


// Read uint8_tsToRead sequentially, starting at addressToRead into the dest uint8_t array
void readRegisters(uint8_t addressToRead, int uint8_tsToRead, uint8_t * dest)
{
  /*TinyWireM.beginTransmission(MMA8452_ADDRESS);*/
  /*TinyWireM.send(addressToRead);*/
  /*TinyWireM.endTransmission(); //endTransmission but keep the connection active*/

  /*TinyWireM.requestFrom(MMA8452_ADDRESS, uint8_tsToRead); //Ask for uint8_ts, once done, bus is released by default*/

  /*while(TinyWireM.available() < uint8_tsToRead); //Hang out until we get the # of uint8_ts we expect*/

  /*for(int x = 0 ; x < uint8_tsToRead ; x++)*/
  /*  dest[x] = TinyWireM.receive();*/

  i2c_start_wait(MMA8452_ADDRESS+I2C_WRITE);     // set device address and write mode

  i2c_write(addressToRead);                        // write address
  i2c_rep_start(MMA8452_ADDRESS+I2C_READ);       // set device address and read mode

  for(int x = 0 ; x < uint8_tsToRead-1 ; x++)
    dest[x] = i2c_read(1);                    // read one uint8_t and request more

  dest[uint8_tsToRead-1] = i2c_read(0);                    // read one uint8_t and stop
}

// Read a single uint8_t from addressToRead and return it as a uint8_t
uint8_t readRegister(uint8_t addressToRead)
{
  /*TinyWireM.beginTransmission(MMA8452_ADDRESS);*/
  /*TinyWireM.send(addressToRead);*/
  /*TinyWireM.endTransmission(); //endTransmission but keep the connection active*/

  /*TinyWireM.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 uint8_t, once done, bus is released by default*/

  /*while(!TinyWireM.available()) ; //Wait for the data to come back*/
  /*return TinyWireM.receive(); //Return this one uint8_t*/

  i2c_start_wait(MMA8452_ADDRESS+I2C_WRITE);     // set device address and write mode

  i2c_write(addressToRead);                        // write address
  i2c_rep_start(MMA8452_ADDRESS+I2C_READ);       // set device address and read mode

  uint8_t value = i2c_readNak();                    // read one uint8_t from EEPROM
  i2c_stop();

  return value;
}

// Writes a single uint8_t (dataToWrite) into addressToWrite
void writeRegister(uint8_t addressToWrite, uint8_t dataToWrite)
{
  /*TinyWireM.beginTransmission(MMA8452_ADDRESS);*/
  /*TinyWireM.send(addressToWrite);*/
  /*TinyWireM.send(dataToWrite);*/
  /*TinyWireM.endTransmission(); //Stop transmitting*/

  i2c_start_wait(MMA8452_ADDRESS+I2C_WRITE);     // set device address and write mode
  i2c_write(addressToWrite);                        // write address
  i2c_write(dataToWrite);                        // write value
  i2c_stop();                             // set stop conditon = release bus
}

// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby(void)
{
  uint8_t c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active(void)
{
  uint8_t c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}


void readAccelData(int *destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

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

// Initialize the MMA8452 registers
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452(void)
{
  _delay_ms(500); // give MMA8452 a chance to boot

  i2c_init();

  uint8_t ret = i2c_start(MMA8452_ADDRESS+I2C_WRITE);       // set device address and write mode

  if ( ret ) {
      /* failed to issue start condition, possibly no device found */
      i2c_stop();
    while(1) {
      PORTA ^= 1 << PA0;
      _delay_ms(50);
    } // Loop forever if communication doesn't happen
  }

  uint8_t c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {
    /*Serial.println("MMA8452Q is online...");*/
    PORTA |= 1 << PA0;
  }
  else
  {
    /*Serial.print("Could not connect to MMA8452Q: 0x");*/
    /*Serial.println(c, HEX);*/
    while(1) {
      PORTA ^= 1 << PA0;
      _delay_ms(500);
    } // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  uint8_t fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);

  //The default data rate is 800Hz and we don't modify it in this example code

  MMA8452Active();  // Set to active to start reading
}

unsigned long millis(void) {
  return 0;
}

void setup(void)
{
  /*Serial.begin(57600);*/
  /*Serial.println("MMA8452 Basic Example");*/
  lastTime = millis();

  /* pinMode(0, OUTPUT); */
  /* pinMode(1, OUTPUT); */
  /* pinMode(2, OUTPUT); */
  // set pins 0, 1 and 2 to output
  DDRA = (1<<DDA0) | (1<<DDA1) | (1<<DDA2);

  initMMA8452(); //Test and intialize the MMA8452
}

void loop(void)
{
  int i, output;
  float dot;
  int accelCount[3];  // Stores the 12-bit signed value

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  timeSinceLastCheck += elapsedTime;

  if (timeSinceLastCheck > 100) {

    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    float accelG[3];  // Stores the real accel value in g's
    for (i = 0; i < 3; i++) {
      accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set

      // use a rolling filter
      currentAcc[i] = 0.95 * accelG[i] + currentAcc[i] * 0.05;
    }

    for (i = 0; i < ledCount; i++) {
      dot = currentAcc[0] * ledsX[i] + currentAcc[1] * ledsY[i];

      // invert dot so the up pointing led is lit
      /* output = (int) (255 * constrain(-dot, 0, 1)); */

      if (-dot > 0.5) {
        PORTA |= (1<<PA0);
      } else {
        PORTA &= ~(0<<PA0); // clear PA0
      }

      /*Serial.print(accelG[i], 2);  // Print g values*/
      /*Serial.print("/");*/
      /*Serial.print(acc, 2);*/
      /*Serial.print(output, HEX);  // Print color values*/
      /*Serial.print("\t");  // tabs in between axes*/

      /*Serial.println();*/
      timeSinceLastCheck = 0;
    }
  }
}

int main(void) {
  setup();
  while(1) {
    /* loop(); */
  }
}
