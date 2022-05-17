// Libraries addition
#include <TroykaI2CHub.h>
#include <MadgwickAHRS.h>
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

// Number of channels not participating in the aquisition
int numVoidChan = 0;

// Flag for skipping channels not participating in the aquisition
bool flag = 0;

// Variable for Madgwick sampling rate (initially 100)
float sampleRate1 = 100;
float sampleRate2 = 100;

// Sensor object creation. This option is unavailable in the 
//original hanyazou library, it's our modification
BMI160GenClass sensor1;
BMI160GenClass sensor2;

// Object for I2C hub
// default address is 0x70. For more hubs refer the datasheet or
// manufacturer's website:
//https://amperka.ru/product/troyka-i2c-hub?utm_source=man&utm_campaign=troyka-pull-up&utm_medium=wiki
TroykaI2CHub splitter;

//Objects for Madgwick filter
Madgwick filter1;
Madgwick filter2;

void setup() {
  // Initialize UART at max speed
  Serial.begin(2000000);
  
  // Waiting for UART to start. Commented out version is for default
  // Arduino core. Uncommented one is for GyverCore
  
  //  while (!Serial) {
  //  }
  Serial.flush();

  // Initialize I2C hub and Madgwick filters
  splitter.begin();
  filter1.begin();
  filter2.begin();

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
    // Offsets are listed for a single unit as we used just one
    // for the displacement calculation tests. Offsets calculation code 
    // can be seen in BMI_Autocalibration.ino file 
    sensor1.setAccelOffsetEnabled(1);
    sensor2.setAccelOffsetEnabled(1);

    sensor1.setXAccelOffset(-4);
    sensor1.setYAccelOffset(30);
    sensor1.setZAccelOffset(-12);

    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.println(" calibration:");

    // Verifying that offsets are set successfilly

    Serial.print("Sensor 1 Accel offsets: \t");

    Serial.print(sensor1.getXAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor1.getYAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor1.getZAccelOffset());
    Serial.println("\t\t");

    sensor2.setXAccelOffset(-41);
    sensor2.setYAccelOffset(-7);
    sensor2.setZAccelOffset(-20);

    Serial.print("Sensor 2 Accel offsets: \t");

    Serial.print(sensor2.getXAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor2.getYAccelOffset());
    Serial.print("\t\t");
    Serial.print(sensor2.getZAccelOffset());
    Serial.println("\t\t");

    sensor1.setGyroOffsetEnabled(1);
    sensor2.setGyroOffsetEnabled(1);

    sensor1.setXGyroOffset(1);
    sensor1.setYGyroOffset(2);
    sensor1.setZGyroOffset(3);
    sensor2.setXGyroOffset(-7);
    sensor2.setYGyroOffset(-5);
    sensor2.setZGyroOffset(5);

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

  }

  // Counting numbers of channels not participating in the aquisition

  for (int i = 0; i < channels; i++) {
    if (voidChan[i] != 9) {
      numVoidChan += 1;
    }
  }

  // Wait for 1 second just in case
  delay(1000);
}

void loop() {
 
  int gxRaw1[channels], gyRaw1[channels], gzRaw1[channels];         // raw gyro values
  int axRaw1[channels], ayRaw1[channels], azRaw1[channels];         // raw accel values

  float gx1[channels], gy1[channels], gz1[channels];         // final gyro values
  float ax1[channels], ay1[channels], az1[channels];         // final accel values

  int gxRaw2[channels], gyRaw2[channels], gzRaw2[channels];         // raw gyro values
  int axRaw2[channels], ayRaw2[channels], azRaw2[channels];         // raw accel values

  float gx2[channels], gy2[channels], gz2[channels];         // final gyro values
  float ax2[channels], ay2[channels], az2[channels];         // final accel values



  // Variables for Yaw, Pitch and Roll for both sensors
  float yaw1[channels], pitch1[channels], roll1[channels];
  float yaw2[channels], pitch2[channels], roll2[channels];


  // Reading values over each single channel
  for (int i = 0; i < channels; i++) {

    // Loop for skipping channels not participating in the aquisition
    flag = 0;
    for (int j = 0; j < channels; j++) {
      if (i == voidChan[j]) {
        flag = 1;
        continue;
      }
    }
    if (flag == 1) {
      continue;
    }

    // // Write down current time for correct fs Madgwick calculation
    unsigned long startMillis = millis();
    //    unsigned long startMillis1 = millis();

    // Toggling channels
    splitter.setBusChannel(i);

    // read raw gyro measurements from device
    sensor1.readGyro(gxRaw1[i], gyRaw1[i], gzRaw1[i]);
    sensor2.readGyro(gxRaw2[i], gyRaw2[i], gzRaw2[i]);

    // read raw accel measurements from device
    sensor1.readAccelerometer(axRaw1[i], ayRaw1[i], azRaw1[i]);
    sensor2.readAccelerometer(axRaw2[i], ayRaw2[i], azRaw2[i]);

    // calculate the gyro and accel
    ax1[i] = convertRawAccel(axRaw1[i]);
    ay1[i] = convertRawAccel(ayRaw1[i]);
    az1[i] = convertRawAccel(azRaw1[i]);

    gx1[i] = convertRawGyro(gxRaw1[i]);
    gy1[i] = convertRawGyro(gyRaw1[i]);
    gz1[i] = convertRawGyro(gzRaw1[i]);

    ax2[i] = convertRawAccel(axRaw2[i]);
    ay2[i] = convertRawAccel(ayRaw2[i]);
    az2[i] = convertRawAccel(azRaw2[i]);

    gx2[i] = convertRawGyro(gxRaw2[i]);
    gy2[i] = convertRawGyro(gyRaw2[i]);
    gz2[i] = convertRawGyro(gzRaw2[i]);

    
    // Exponential moving average filtration. Commented out the the
    // sake of processing time. Can be included if needed. Same function
    // implemented on the displacement calculation code on python
    //    ax1[i] = expRunningAverageAdaptive(ax1[i]);
    //    ay1[i] = expRunningAverageAdaptive(ay1[i]);
    //    az1[i] = expRunningAverageAdaptive(az1[i]);
    //
    //    gx1[i] = expRunningAverageAdaptive(gxRaw1[i]);
    //    gy1[i] = expRunningAverageAdaptive(gyRaw1[i]);
    //    gz1[i] = expRunningAverageAdaptive(gzRaw1[i]);
    //
    //    ax2[i] = expRunningAverageAdaptive(axRaw2[i]);
    //    ay2[i] = expRunningAverageAdaptive(ayRaw2[i]);
    //    az2[i] = expRunningAverageAdaptive(azRaw2[i]);
    //
    //    gx2[i] = expRunningAverageAdaptive(gxRaw2[i]);
    //    gy2[i] = expRunningAverageAdaptive(gyRaw2[i]);
    //    gz2[i] = expRunningAverageAdaptive(gzRaw2[i]);

    // Set sampling frequency for Magdwick filters
    filter1.setFrequency(sampleRate1);
    filter2.setFrequency(sampleRate2);
    
    // Refresh input data for the Madgwick filters
    filter1.update(gx1[i], gy1[i], gz1[i], ax1[i], ay1[i], az1[i]);
    filter2.update(gx2[i], gy2[i], gz2[i], ax2[i], ay2[i], az2[i]);

    // Calculating Yaw, Roll and Pitch
    yaw1[i] = filter1.getYawDeg();
    pitch1[i] = filter1.getPitchDeg();
    roll1[i] = filter1.getRollDeg();
    yaw2[i] = filter2.getYawDeg();
    pitch2[i] = filter2.getPitchDeg();
    roll2[i] = filter2.getRollDeg();

    // Not specifying channel number for the sake of transmission speed
    //    Serial.print("Channel ");
    //    Serial.print(i + 1);
    //    Serial.print(": ");

    // Output angles and linear accelerations
    Serial.print(yaw1[i]);
    Serial.print("\t");
    Serial.print(pitch1[i]);
    Serial.print("\t");
    Serial.print(roll1[i]);
    Serial.print("\t");
    Serial.print(ax1[i]);
    Serial.print("\t");
    Serial.print(ay1[i]);
    Serial.print("\t");
    Serial.print(az1[i]);
    Serial.print("\t");
    Serial.print(yaw2[i]);
    Serial.print("\t");
    Serial.print(pitch2[i]);
    Serial.print("\t");
    Serial.print(roll2[i]);
    Serial.print("\t");
    Serial.print(ax2[i]);
    Serial.print("\t");
    Serial.print(ay2[i]);
    Serial.print("\t");
    Serial.print(az2[i]);
    Serial.print("\t");

    // Compute time required for data aquisition and processing
    unsigned long deltaMillis = millis() - startMillis;
    
    // Refresh Madgwick sampling rates
    sampleRate1 = 1000 / deltaMillis;
    sampleRate2 = 1000 / deltaMillis;

  }

  // In the end output time to calculate Δt for the displacement
  // (done in python displacement code)
//  Serial.println(micros());
  Serial.println();
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

// Function for raw gyroscope values conversion
float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

// Function for raw accelerometer values conversion
float convertRawAccel(int aRaw) {
  // since we are using +/-2g range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 19620.0) / 32768.0;  //mm/s^2

  return a;
}

// Exponential moving average filter. Not used in current version
// but can be added if needed
// Beware: requires coefficients adjustment
float expRunningAverageAdaptive(float newVal) {
  static float filVal = 0;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 1.5) k = 0.9;
  else k = 0.03;

  filVal += (newVal - filVal) * k;
  return filVal;
}
