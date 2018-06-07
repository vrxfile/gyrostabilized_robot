/*
  Маленький робот на S4A с Bluetooth и гироскопом
  Created by Rostislav Varzar
*/

#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <SoftwareSerial.h>

LSM9DS1 imu;
#define LSM9DS1_M   0x1E
#define LSM9DS1_AG  0x6B
float gyro_zero = 0;

#define PWMA 5
#define DIRA 10
#define PWMB 6
#define DIRB 11

// Мощности моторов
float MPWRA = 0;
float MPWRB = 0;

// Переменные для управления роботом
char command = 'S';
char prevCommand = 'A';
int velocity = 0;
unsigned long timer0 = 2000;
unsigned long timer1 = 0;
char lastCommand = 'A';

// Программный UART для Bluetooth
SoftwareSerial BT(A0, 0);

void setup()
{
  // Инициализация последовательного порта
  Serial.begin(9600);

  // Инициализация последовательного порта для Bluetooth
  BT.begin(9600);

  // Инициализация датчика положения
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1) ;
  }
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }

  // Ноулевое значение гироскопа по оси Z
  gyro_zero = imu.calcGyro(imu.gz);

  // Инициализация выходов для управления моторами
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void loop()
{
  // Тестирование датчиков
  //  while (true)
  //  {
  //    if (imu.gyroAvailable()) {
  //      imu.readGyro();
  //    }
  //    Serial.print(imu.calcGyro(imu.gx));
  //    Serial.print("\t");
  //    delay(5);
  //    Serial.print(imu.calcGyro(imu.gy));
  //    Serial.print("\t");
  //    delay(5);
  //    Serial.print(imu.calcGyro(imu.gz));
  //    Serial.print("\t");
  //    delay(5);
  //    Serial.println("");
  //  }

  // Езда с управлением по Bluetooth
  Serial.println("Started BT going program");
  while (true)
  {
    // Bluetooth
    if (BT.available() > 0) {
      timer1 = millis();
      prevCommand = command;
      command = BT.read();
      Serial.print("Command: ");
      Serial.println(command);
      if (command != prevCommand) {
        lastCommand = command;
        switch (command) {
          case 'F':
            MPWRA = -velocity;
            MPWRB = -velocity;
            break;
          case 'B':
            MPWRA = velocity;
            MPWRB = velocity;
            break;
          case 'L':
            MPWRA = velocity;
            MPWRB = -velocity;
            break;
          case 'R':
            MPWRA = -velocity;
            MPWRB = velocity;
            break;
          case 'S':
            MPWRA = 0;
            MPWRB = 0;
            break;
          case 'I':
            MPWRA = -velocity;
            MPWRB = -velocity / 2;
            break;
          case 'J':
            MPWRA = velocity;
            MPWRB = velocity / 2;
            break;
          case 'G':
            MPWRA = -velocity / 2;
            MPWRB = -velocity;
            break;
          case 'H':
            MPWRA = velocity / 2;
            MPWRB = velocity;
            break;
          case 'D':
            MPWRA = 0;
            MPWRB = 0;
            break;
          default:  //Get velocity
            if (command == 'q') {
              velocity = 50;
            }
            else {
              if ((command >= 48) && (command <= 57)) {
                velocity = (command - 48) * 10;
              }
            }
        }
      }
    }
    else {
      timer0 = millis();
      if ((timer0 - timer1) > 500) {
        MPWRA = 0;
        MPWRB = 0;
      }
    }
    if (imu.gyroAvailable()) {
      imu.readGyro();
    }
    float gyr_z = (imu.calcGyro(imu.gz) - gyro_zero) / 5.0;
    Serial.println("GYR " + String(gyr_z, 2));
    if ((lastCommand == 'q') || (lastCommand == 'A') || (lastCommand == 'D')  || (lastCommand == 'S') ||
        (lastCommand == 'B') || (lastCommand == 'F')) {
      motorA_setpower(MPWRA - gyr_z, false);
      motorB_setpower(MPWRB + gyr_z, false);
    } else {
      motorA_setpower(MPWRA, false);
      motorB_setpower(MPWRB, false);
    }
  }
}

// Мощность мотора "A" от -100% до +100% (от знака зависит направление вращения)
void motorA_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  // Установка направления
  if (pwr < 0)
  {
    digitalWrite(DIRA, LOW);
  }
  else
  {
    digitalWrite(DIRA, HIGH);
  }
  // Установка мощности
  int pwmvalue = fabs(pwr) * 2.55;
  analogWrite(PWMA, pwmvalue);
}

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  // Установка направления
  if (pwr < 0)
  {
    digitalWrite(DIRB, LOW);
  }
  else
  {
    digitalWrite(DIRB, HIGH);
  }
  // Установка мощности
  int pwmvalue = fabs(pwr) * 2.55;
  analogWrite(PWMB, pwmvalue);
}

