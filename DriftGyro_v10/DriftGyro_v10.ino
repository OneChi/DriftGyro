#include <Ticker.h>
#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Объявляем переменные и константы
const int throttlePin = 4;
const int steeringPin = 2;
const int auxPin = 3;
const int servoPin = 9;
const int brakePin = 8;

#define MIN_STEERING 1000     //1000
#define MAX_STEERING 2000     //2000
#define NEUTRAL      1500     //1500

const byte throttleFlag = 1;
const byte steeringFlag = 2;
const byte auxFlag = 4;

volatile int throttleIn = NEUTRAL;
volatile int steeringIn = NEUTRAL;
volatile int auxIn = NEUTRAL;
volatile unsigned long throttleInStart;
volatile unsigned long steeringInStart;
volatile unsigned long auxInStart;
volatile uint8_t UpdateFlags = 0;
int steeringInDifference = 0;
int steeringInOld = NEUTRAL;

Servo steeringServo;
#define SERVO_SPEED 80 // Geschwindigkeit in 0..100%

int gyroVal;
int gain = 50;
int gainNew = 50;
MPU6050 mpu;         // Sensor Invensense MPU6050
int16_t gx, gy, gz;

double q = 4; // процессный шум
double r = 16; // измерительный шум
double p = 2000; // оценка ошибки
double k = 0; // усиление Калмана
int16_t x = 1500; // значение
int16_t kalmanUpdate(int16_t measurement);



void calcThrottle() {
    if(digitalRead(throttlePin)) {
      throttleInStart = micros();
    } else {
      throttleIn = (int)(micros() - throttleInStart);
      UpdateFlags |= throttleFlag;
    }
  
}

void calcSteering() {
    if(digitalRead(steeringPin)) {
      steeringInStart = micros();
    } else {
      steeringIn = (int)(micros() - steeringInStart);
      UpdateFlags |= steeringFlag;
    }
  
}

void calcAux() {
    if(digitalRead(auxPin)) {
      auxInStart = micros();
    } else {
      auxIn = (int)(micros() - auxInStart);
      UpdateFlags |= auxFlag;
    }
  
}


Ticker trottle(calcThrottle, 10); // Создаем экземпляр класса Ticker
Ticker steering(calcSteering, 10); // Создаем экземпляр класса Ticker
Ticker aux(calcAux, 0.005f); // Создаем экземпляр класса Ticker

void setup() {
  Serial.begin(115200);
  Serial.println("Дрифт Гироскоп");

  pinMode(brakePin, OUTPUT);
  steeringServo.attach(servoPin, MIN_STEERING, MAX_STEERING);

  // Используем Ticker для обработки прерываний от аналоговых входов
  trottle.start();
  steering.start();
  aux.start();
  
  mpu.initialize();
  mpu.setFullScaleGyroRange(0);
  mpu.setDLPFMode(6);
}

void loop() {

  trottle.update();
  steering.update();
  aux.update();


  if (throttleFlag || steeringFlag || auxFlag) {
    if (throttleFlag) {
      // Обработка прерывания от ThrottleIn
      // Управление акселератором
      if(throttleIn < (NEUTRAL-50)) { // Bremse
        digitalWrite(brakePin, HIGH);
      } else {
        digitalWrite(brakePin, LOW);
      }
    }
    if (steeringFlag) {
      // Обработка прерывания от steeringIn
      Serial.print("after - ");
      Serial.println(steeringIn);
      gy = mpu.getRotationY(); // Y-Gyroskop-Daten einlesen
      gyroVal = map(-gy, -32768, 32767, -10*gain, 10*gain); // Umrechnen in µs außerhalb Neutralposition
      steeringIn += gyroVal;    // Servo Signal korrigieren
      steeringIn = kalmanUpdate(steeringIn); // Kalman-Filter


      steeringInDifference = steeringIn - steeringInOld; // max 200µs je Iteration für Servo mit 0,05s Stellzeit
      steeringIn = steeringInOld + ((steeringInDifference*SERVO_SPEED)/100);
  
      // Begrenzungen
      if(steeringIn<MIN_STEERING){
        steeringIn=MIN_STEERING;}
      if(steeringIn>MAX_STEERING){
        steeringIn=MAX_STEERING;}
        
      steeringServo.writeMicroseconds(steeringIn); // Servo ansteuern mit korrigiertem Signal
  
      steeringInOld = steeringIn; // aktuellen Wert zwischenspeichern
    }
    if (auxFlag) {
    
      gainNew = map(auxIn, 1000, 2000, 0, 100); //Umrechnen in %
    
      // Begrenzungen
      if(gainNew>100){
        gainNew=100;}
      if(gainNew<0){
        gainNew=0;}
    
      gain = gainNew;   //gain mit neuem gain-Wert überschreiben
    
    }
  }

}


// Функция Калмана
int16_t kalmanUpdate(int16_t measurement) {
  p = p + q;
  k = p / (p + r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;
  return x;
}
