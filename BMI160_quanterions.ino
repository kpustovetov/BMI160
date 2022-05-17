// Библиотека для работы с модулями IMU
#include <TroykaIMU.h>
#include <BMI160Gen.h>

// Создаём объект для фильтра Madgwick
Madgwick filter;
// Создаём объект для работы с гироскопом
Gyroscope gyroscope;
// Создаём объект для работы с акселерометром
Accelerometer accelerometer;

BMI160GenClass sens;

// Переменные для данных с гироскопа и акселерометра
float gx, gy, gz, ax, ay, az;

int gxRaw, gyRaw, gzRaw;         // raw gyro values
int axRaw, ayRaw, azRaw;         // raw accel values

// Переменные для хранения самолётных углов ориентации
float yaw, pitch, roll;

// Переменная для хранения частоты выборок фильтра
float sampleRate = 100;

const int select_pin = 10;
const int i2c_addr = 0x68;    //SDO pulled down to GND, 0x69 if pulled up to


void setup() {
    // Открываем последовательный порт
    Serial.begin(9600);
    // Выводим сообщение о начале инициализации
    //Serial.println("IMU Begin");
    // Инициализируем гироскоп
    gyroscope.begin();
    // Инициализируем акселерометр
    accelerometer.begin();
    // Инициализируем фильтр
    filter.begin();
    //Initialize BMI160
    sens.begin(BMI160GenClass::I2C_MODE, i2c_addr, select_pin);
    // Выводим сообщение об удачной инициализации
    //Serial.println("Initialization completed");

      //set offsets
  sens.setXAccelOffset(-41);
  sens.setYAccelOffset(-7);
  sens.setZAccelOffset(-20);
  sens.setXGyroOffset(-7);
  sens.setYGyroOffset(-5);
  sens.setZGyroOffset(5);
}

void loop() {
    // Запоминаем текущее время
    unsigned long startMillis = millis();
    // Считываем данные с акселерометра в единицах G
    sens.readAccelerometer(axRaw, ayRaw, azRaw);

    ax = convertRawAccel(axRaw);
    ay = convertRawAccel(ayRaw);
    az = convertRawAccel(azRaw);
    
    // Считываем данные с гироскопа в радианах в секунду
    sens.readGyro(gxRaw, gyRaw, gzRaw);

    gx = convertRawGyro(gxRaw);
    gy = convertRawGyro(gyRaw);
    gz = convertRawGyro(gzRaw);
    
    // Устанавливаем частоту фильтра
    filter.setFrequency(sampleRate);
    // Обновляем входные данные в фильтр
    filter.update(gx, gy, gz, ax, ay, az);

    if (Serial.available() > 0) {
      int val = Serial.read();
        //Если пришёл символ 's'
        if (val == 's') {
            float q0, q1, q2, q3;
            filter.readQuaternion(q0, q1, q2, q3);
            
            // Выводим кватернион в serial-порт
            Serial.print(q0);
            Serial.print(",");
            Serial.print(q1);
            Serial.print(",");
            Serial.print(q2);
            Serial.print(",");
            Serial.println(q3);
        }
    }

    // Вычисляем затраченное время на обработку данных
    unsigned long deltaMillis = millis() - startMillis;
    // Вычисляем частоту обработки фильтра
    sampleRate = 1000 / deltaMillis;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (float)(gRaw * 250.0 * PI) / (32768.0 * 180);

  return g;
}

float convertRawAccel(int aRaw) {
  // since we are using +/-2g range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

//  float a = (aRaw * 19620.0) / 32768.0;  //mm/s^2
  float a = (float)(aRaw) / 16384.0;  // in g

  return a;
}
