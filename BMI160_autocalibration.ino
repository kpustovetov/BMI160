// Libraries addition
#include <TroykaI2CHub.h>
#include <BMI160Gen.h>

// We have 8 channels, but this may be extended by ading extra I2C hubs
#define channels 8

// BMI160 can have just 2 possible addresses
const int add1 = 0x68;
const int add2 = 0x69;

// Not needed in new version of library for this project but we are still using
//Hanyazou version (newer version is EmotiBit)
const int select_pin = 10;

// Array for keeping channels not participating in the aquisition
// Basically, for speed optimization
int voidChan[channels] = {9, 9, 9, 9, 9, 9, 9, 9};

// Flag for skipping channels not participating in the aquisition
bool flag = 0;

// Sensor object creation. This option is unavailable in the 
//original hanyazou library, it's our modification
BMI160GenClass sensor1;
BMI160GenClass sensor2;

// Object for I2C hub
// default address is 0x70. For more hubs refer the datasheet or
// manufacturer's website:
//https://amperka.ru/product/troyka-i2c-hub?utm_source=man&utm_campaign=troyka-pull-up&utm_medium=wiki
TroykaI2CHub splitter;

void setup() {
  // Initialize UART at max speed
  Serial.begin(2000000);
  
  // Waiting for UART to start. Commented out version is for default
  // Arduino core. Uncommented one is for GyverCore
  
  //  while (!Serial) {
  //  }
  Serial.flush();

  // Initialize I2C hub
  splitter.begin();

  // Scanning through the channels to check which ones are connected
  // in case one of the sensors in a unit is dead code will show a 
  // warning and the unit will not be started
  for (int i = 0; i < channels; i++) {

    // Variable for counting operating IMUs on a single channel
    byte devCount = 0;

    // Toggling channels
    splitter.setBusChannel(i);

    // Scanning the channel for operating IMUs
    devCount = startScanerI2C(i);

    switch (devCount) {
      // None found (not connected or both broken)
      case 0:
        voidChan[i] = i;
        continue;
        break;

      // One found (one IMU is broken)
      case 1:
        // Don't see the point of starting just one sensor as the displacement
        // code will rely on 2 sensors
        Serial.print("Something's wrong with one of the sensors on Channel ");
        Serial.println(i + 1);
        voidChan[i] = i;
        continue;
        break;

      // Two found (all good)
      case 2:
        // Sensor initialization. Hanyazou version.
        sensor1.begin(BMI160GenClass::I2C_MODE, add1, select_pin);
        sensor2.begin(BMI160GenClass::I2C_MODE, add2, select_pin);
        break;
    }

    //calibration
    // Before you run the code, place the IMU on a leveled flat surface
    // After you have started running the code, sit quitely on the chair
    // and try not to breathe. Sensors being extremely sensitive
    sensor1.setAccelOffsetEnabled(1);
    sensor2.setAccelOffsetEnabled(1);

    sensor1.autoCalibrateXAccelOffset(0);
    sensor1.autoCalibrateYAccelOffset(0);
    sensor1.autoCalibrateZAccelOffset(1);

    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.println(" calibration: \t");

    // Verifying that offsets are set successfilly

    Serial.print("Sensor 1 Accel offsets: \t");

    Serial.print(sensor1.getXAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor1.getYAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor1.getZAccelOffset());
    Serial.println("\t\t");

    sensor2.autoCalibrateXAccelOffset(0);
    sensor2.autoCalibrateYAccelOffset(0);
    sensor2.autoCalibrateZAccelOffset(1);

    // Verifying that offsets are set successfilly

    Serial.print("Sensor 2 Accel offsets: \t");

    Serial.print(sensor2.getXAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor2.getYAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor2.getZAccelOffset());
    Serial.println("\t\t");

    sensor1.setGyroOffsetEnabled(1);
    sensor2.setGyroOffsetEnabled(1);

    sensor1.autoCalibrateGyroOffset();
    sensor2.autoCalibrateGyroOffset();

    // Verifying that offsets are set successfilly

    Serial.print("Sensor 1 Gyro offsets: \t");

    Serial.print(sensor1.getXGyroOffset());
    Serial.print("\t\t");
    Serial.print(sensor1.getYGyroOffset());
    Serial.print("\t\t");
    Serial.print(sensor1.getZGyroOffset());
    Serial.println("\t\t");

    Serial.print("Sensor 2 Gyro offsets: \t");

    Serial.print(sensor2.getXGyroOffset());
    Serial.print("\t\t");
    Serial.print(sensor2.getYGyroOffset());
    Serial.print("\t\t");
    Serial.print(sensor2.getZGyroOffset());
    Serial.println("\t\t");


    // Wait for one second
    delay(1000);
  }
}
void loop() {
  exit(0);
}

// Function for checking number of operating IMUs on a single channel
// Modified version of code provided by the I2C hub manufacturer (Amperka Robotics)
byte startScanerI2C(int chan)
{
  // State variable
  byte state;
  // Current address variable
  byte address;
  // Variable for counting number of operating IMUs
  byte countDevices = 0;

  // Scanning around the addresses. Changed Amperka version.
  // Less universal but faster, because we know BMI160 can be either
  // 0x68 or 0x69, which is 104 and 105 in decimal system respectively,
  // so could make the list of scanned addresses even smaller
  for (address = 100; address < 110; address++ ) {
    // Transmit data on current address
    Wire.beginTransmission(address);
    // End transmission, should get zero
    state = Wire.endTransmission();
    // If the received byte is zero
    if (state == 0) {
      countDevices++;
    }
  }
  // If no devices found => print a message
  if (countDevices == 0) {
    Serial.println("No I²C devices found");
    voidChan[chan] = chan + 1;
  } else if (countDevices == 1) {
    Serial.print("Something's wrong with one of the sensors on Channel ");
    Serial.println(chan + 1);
    voidChan[chan] = chan + 1;

  } else {
    // If everything is okay, print the message
    Serial.println("All good with device on Channel ");
    Serial.println(chan + 1);
  }

  return countDevices;
}
