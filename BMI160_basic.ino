#include <BMI160Gen.h>
#include <Wire.h>

const int select_pin = 10;
const int i2c_addr = 0x68;    //SDO pulled down to GND, 0x69 if pulled up to

BMI160GenClass sens;

float timeStep = 0;

int i = 1;

void setup() {
  Serial.begin(9600); // initialize Serial communication
  //  while (!Serial);    // wait for the serial port to open
  Serial.flush();

  // initialize device
  //  BMI160.begin(BMI160GenClass::SPI_MODE, select_pin);
  sens.begin(BMI160GenClass::I2C_MODE, i2c_addr, select_pin);
  //  Wire.begin();
  //  Wire.beginTransmission(i2c_addr);
  //  if ( Wire.endTransmission() != 0 ){
  //    Serial.println("BMI160BME::i2c_init(): I2C failed.");
  //  } else {
  //    Serial.println("BMI160BME::i2c_init(): I2C successful.");
  //  }


  //set LPF settings
  //  Serial.println("Set LPF settings...");
  sens.setGyroDLPFMode(0);
  sens.setAccelDLPFMode(0);

  //set offsets
  sens.setXAccelOffset(-2);
  sens.setYAccelOffset(-1);
  sens.setZAccelOffset(-10);
  sens.setXGyroOffset(-4);
  sens.setYGyroOffset(7);
  sens.setZGyroOffset(-3);

  //set accel range
  sens.setFullScaleAccelRange(3); //for 2g
  sens.setFullScaleGyroRange(4);

  //accel and gyro rate
  //13 = 3200Hz
  sens.setGyroRate(13);
  sens.setAccelRate(12);

}

void loop() {

  //  float arr[1000];

  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  int axRaw, ayRaw, azRaw;         // raw accel values

  float gx, gy, gz;         // final gyro values
  float ax, ay, az;         // final accel values

  float filtered_gx, filtered_gy, filtered_gz;
  float filtered_ax, filtered_ay, filtered_az;

  int final_ax;

  // read raw gyro measurements from device
  sens.readGyro(gxRaw, gyRaw, gzRaw);

  // read raw accel measurements from device
  sens.readAccelerometer(axRaw, ayRaw, azRaw);

  // calculate the gyro and accel
  ax = convertRawAccel(axRaw);
  ay = convertRawAccel(ayRaw);
  az = convertRawAccel(azRaw);

  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  //  filtered_ax = expRunningAverageAdaptive(ax);
  //  filtered_ay = expRunningAverageAdaptive(ay);
  //  filtered_az = expRunningAverageAdaptive(az);
  //
  //  final_ax = findMedianN_optim(filtered_ax);

  // display tab-separated gyro x/y/z values
  //  Serial.print("Initial values:\t");
  //  Serial.print("g:\t");
  //  Serial.print(gxRaw);
  //  Serial.print("\t");
  //  Serial.print(gyRaw);
  //  Serial.print("\t");
  //  Serial.print(gzRaw);
  //  Serial.print("\t");
  //  Serial.print("a:\t");
  //  Serial.print(axRaw);
  //  Serial.print("\t");
  //  Serial.print(ayRaw);
  //  Serial.print("\t");
  //  Serial.print(azRaw);
  //  Serial.println();
  //  Serial.print(i);
  //  Serial.print(',');

  //  //Serial.print("Final values:\t");
  //  //Serial.print("g:\t");
  //  for (int i = 0; i < 18; i++) {
  //    Serial.print(gx);
  //    Serial.print("\t");
  //    Serial.print(gy);
  //    Serial.print("\t");
  //    Serial.print(gz);
  //    Serial.print("\t");
  //  //Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  //    Serial.print("\t");
  //  Serial.print(ax);
  //  Serial.print("\t");
  //  Serial.print(final_ax);
  //  }
  //  Serial.println(micros() - timeStep);
  //     Serial.println(micros());
  Serial.println();
  //  //  delay(50);

  //  arr[0*i] = gx;
  //  arr[1*i] = gy;
  //  arr[2*i] = gz;
  //  arr[3*i] = ax;
  //  arr[4*i] = ay;
  //  arr[5*i] = az;
  //  arr[6*i] = micros()-timeStep;

  //  i = i+1;
  //
  //  Serial.println(i);
  //
  //  if (i == 140) {
  //    for (int j = 0; j <= i; j++) {
  //      Serial.println(arr[6*j]);
  //    }
  //    exit(0);
  //  }
  i = i + 1;
  timeStep = micros();
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

  float a = (aRaw * 19620.0) / (32768.0 * 9810.0); //mm/s^2

  return a;
}


// бегущее среднее с адаптивным коэффициентом
float expRunningAverageAdaptive(float newVal) {
  static float filVal = 0;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 1.5) k = 0.9;
  else k = 0.03;

  filVal += (newVal - filVal) * k;
  return filVal;
}

// облегчённый вариант медианы для N значений
// предложен Виталием Емельяновым, доработан AlexGyver
// возвращает медиану по последним NUM_READ вызовам
// НАВЕРНОЕ ЛУЧШИЙ ВАРИАНТ!
#define NUM_READ 10  // порядок медианы
// медиана на N значений со своим буфером, ускоренный вариант
float findMedianN_optim(float newVal) {
  static float buffer[NUM_READ];  // статический буфер
  static byte count = 0;
  buffer[count] = newVal;
  if ((count < NUM_READ - 1) and (buffer[count] > buffer[count + 1])) {
    for (int i = count; i < NUM_READ - 1; i++) {
      if (buffer[i] > buffer[i + 1]) {
        float buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  } else {
    if ((count > 0) and (buffer[count - 1] > buffer[count])) {
      for (int i = count; i > 0; i--) {
        if (buffer[i] < buffer[i - 1]) {
          float buff = buffer[i];
          buffer[i] = buffer[i - 1];
          buffer[i - 1] = buff;
        }
      }
    }
  }
  if (++count >= NUM_READ) count = 0;
  return buffer[(int)NUM_READ / 2];
}
