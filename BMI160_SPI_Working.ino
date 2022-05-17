#include <BMI160Gen.h>
#include <SPI.h>

#define timerDelay 2500

const int select_pin = 10;
const int i2c_addr_1 = 0x68;    //SDO pulled down to GND, 0x69 if pulled up to
const int i2c_addr_2 = 0x69;    //SDO pulled down to GND, 0x69 if pulled up to

BMI160GenClass sensor1;
BMI160GenClass sensor2;

void setup() {
  Serial.begin(2000000); // initialize Serial communication
  
  // Comment this out for GyverCore
  //  while (!Serial) {
  //  }

  //pinMode(4, OUTPUT);

  // For GyverCore
  Serial.clear();    // wait for the serial port to open

  // initialize device
  sensor1.begin(BMI160GenClass::I2C_MODE, i2c_addr_1, select_pin);
  sensor2.begin(BMI160GenClass::I2C_MODE, i2c_addr_2, select_pin);

  //set LPF settings
  sensor1.setGyroDLPFMode(0);
  sensor1.setAccelDLPFMode(0);

  //set accel range
  sensor1.setFullScaleAccelRange(3); //for 2g

  sensor2.setFullScaleAccelRange(3); //for 2g

  uint8_t accel_rate = 10;

  sensor1.setAccelRate(accel_rate);
  sensor2.setAccelRate(accel_rate);

  //calibration
  sensor1.setAccelOffsetEnabled(1);
  sensor2.setAccelOffsetEnabled(1);

  sensor1.setXAccelOffset(-4);
  sensor1.setYAccelOffset(30);
  sensor1.setZAccelOffset(-12);
  sensor2.setXAccelOffset(-41);
  sensor2.setYAccelOffset(-7);
  sensor2.setZAccelOffset(-20);

  sensor1.setGyroOffsetEnabled(1);
  sensor2.setGyroOffsetEnabled(1);

  sensor1.setXGyroOffset(1);
  sensor1.setYGyroOffset(2);
  sensor1.setZGyroOffset(3);
  sensor2.setXGyroOffset(-7);
  sensor2.setYGyroOffset(-5);
  sensor2.setZGyroOffset(5);



  //  Serial.println("Initiation done");
  // Устанавливаем вывод MISO в режим выхода
  pinMode(MISO, OUTPUT);

  // Устанавливаем режим ведомого в контрольном регистре SPI (SPI Control Register)
  SPCR |= _BV(SPE);

  Serial.flush();

  Serial.println("All the settings done");

}

void loop() {

  int gxRaw[2], gyRaw[2], gzRaw[2];         // raw gyro values
  int axRaw[2], ayRaw[2], azRaw[2];         // raw accel values

  float gx[2], gy[2], gz[2];         // final gyro values
  float ax[2], ay[2], az[2];         // final accel values

  // read raw gyro measurements from device
  sensor1.readGyro(gxRaw[0], gyRaw[0], gzRaw[0]);
  sensor2.readGyro(gxRaw[1], gyRaw[1], gzRaw[1]);

  // read raw accel measurements from device
  sensor1.readAccelerometer(axRaw[0], ayRaw[0], azRaw[0]);
  sensor2.readAccelerometer(axRaw[1], ayRaw[1], azRaw[1]);

  // calculate the gyro and accel
  ax[0] = convertRawAccel(axRaw[0]);
  ay[0] = convertRawAccel(ayRaw[0]);
  az[0] = convertRawAccel(azRaw[0]);

  ax[1] = convertRawAccel(axRaw[1]);
  ay[1] = convertRawAccel(ayRaw[1]);
  az[1] = convertRawAccel(azRaw[1]);

  gx[0] = convertRawGyro(gxRaw[0]);
  gy[0] = convertRawGyro(gyRaw[0]);
  gz[0] = convertRawGyro(gzRaw[0]);

  gx[1] = convertRawGyro(gxRaw[1]);
  gy[1] = convertRawGyro(gyRaw[1]);
  gz[1] = convertRawGyro(gzRaw[1]);
  
  // Try replacing it with spi.close() on RaspPi
  long delayTimer = micros();

  while (micros() - delayTimer < timerDelay);

  // Starting byte
  SPDR = 0xCB;
  while (!(SPSR & (1 << SPIF)));

  // LSB is sent first
  for (int i = 0; i < (sizeof(gx) / 2); i++) {
    if (*ptr_gx == 0) {
      *ptr_gx = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_gx);    // send the gx (sens1) byte by byte
    Serial.print(*ptr_gx);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_gx++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = 0; i < (sizeof(gy) / 2); i++) {
    if (*ptr_gy == 0) {
      *ptr_gy = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_gy);    // send the gy (sens1) byte by byte
    Serial.print(*ptr_gy);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_gy++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = 0; i < (sizeof(gz) / 2); i++) {
    if (*ptr_gz == 0) {
      *ptr_gz = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_gz);    // send the gz (sens1) byte by byte
    Serial.print(*ptr_gz);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_gz++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = 0; i < (sizeof(ax) / 2); i++) {
    if (*ptr_ax == 0) {
      *ptr_ax = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_ax);    // send the ax (sens1) byte by byte
    Serial.print(*ptr_ax);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_ax++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = 0; i < (sizeof(ay) / 2); i++) {
    if (*ptr_ay == 0) {
      *ptr_ay = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_ay);    // send the ay (sens1) byte by byte
    Serial.print(*ptr_ay);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_ay++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = 0; i < (sizeof(az) / 2); i++) {
    if (*ptr_az == 0) {
      *ptr_az = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_az);    // send the az (sens1) byte by byte
    Serial.print(*ptr_az);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_az++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  for (int i = (sizeof(gx) / 2); i < (sizeof(gx)); i++) {
    if (*ptr_gx == 0) {
      *ptr_gx = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_gx);    // send the gx (sens2) byte by byte
    Serial.print(*ptr_gx);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_gx++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = (sizeof(gy) / 2); i < (sizeof(gy)); i++) {
    if (*ptr_gy == 0) {
      *ptr_gy = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_gy);    // send the gy (sens2) byte by byte
    Serial.print(*ptr_gy);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_gy++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = (sizeof(gz) / 2); i < (sizeof(gz)); i++) {
    if (*ptr_gz == 0) {
      *ptr_gz = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_gz);    // send the gz (sens2) byte by byte
    Serial.print(*ptr_gz);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_gz++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = (sizeof(ax) / 2); i < (sizeof(ax)); i++) {
    if (*ptr_ax == 0) {
      *ptr_ax = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_ax);    // send the ax (sens2) byte by byte
    Serial.print(*ptr_ax);    // for debugging, print out each byte
        Serial.print(", ");
    *ptr_ax++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = (sizeof(ay) / 2); i < (sizeof(ay)); i++) {
    if (*ptr_ay == 0) {
      *ptr_ay = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_ay);    // send the ay (sens2) byte by byte
    Serial.print(*ptr_ay);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_ay++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // LSB is sent first
  for (int i = (sizeof(az) / 2); i < (sizeof(az)); i++) {
    if (*ptr_az == 0) {
      *ptr_az = 90;
    }
    while (!(SPSR & (1 << SPIF)));
    SPI.transfer(*ptr_az);    // send the az (sens2) byte by byte
    Serial.print(*ptr_az);    // for debugging, print out each byte
    Serial.print(", ");
    *ptr_az++;

    while (!(SPSR & (1 << SPIF)));

    delayTimer = micros();
    while (micros() - delayTimer < timerDelay);
  }
  Serial.println();

  // Stopping byte
  SPDR = 0xCE;
  while (!(SPSR & (1 << SPIF)));



}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

float convertRawAccel(int aRaw) {
  // since we are using +/-2g range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 19620.0) / 32768.0;  //mm/s^2

  return a;
}
