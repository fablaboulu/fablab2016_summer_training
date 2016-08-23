//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.

//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include <Scheduler.h>
#include<Wire.h>
const int MPU_addr = 0x69;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

long randomi;

#include <Servo.h>

Servo myservo;


int pos = 0;

int ledPin = 4;
int speakerOut = 5;

int n = 0;



int timeUpDown[] = {3822, 3606, 3404, 3214, 3032, 2862,
                    2702, 2550, 2406, 2272, 2144, 2024,
                    1911, 1803, 1702, 1607, 1516, 1431,
                    1351, 1275, 1203, 1136, 1072, 1012
                   };


byte song[] = {12, 12, 12, 14, 16, 16, 14, 14, 12, 16, 14, 14, 12, 12, 12, 12,
               14, 14, 14, 14, 9, 9, 9, 9, 14, 12, 11, 9, 7, 7, 7, 7
              };

byte beat = 0;
int MAXCOUNT = 32;
float TEMPO_SECONDS = 0.2;
byte statePin = LOW;
byte period = 0;
int i, timeUp;

Scheduler motorScheduler = Scheduler();
Scheduler buzzerScheduler = Scheduler();
//Scheduler acceScheduler=Scheduler();
Scheduler servoScheduler = Scheduler();

void setup() {

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  myservo.attach(13);
  motorScheduler.schedule(motor, 500);
  buzzerScheduler.schedule(buzzer, 1400);
  //acceScheduler.schedule(acce, 10);
  servoScheduler.schedule(servo, 10);
  randomSeed(A0);

  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(10, OUTPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(speakerOut, OUTPUT);

}

void loop() {
  if (n < 20) {
    //acceScheduler.update();
    motorScheduler.update();
    buzzerScheduler.update();
    servoScheduler.update();

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
    if (AcX > 0 || AcX < -20000) {
      n++;
    }
    else {
      digitalWrite(6, HIGH);
      digitalWrite(9, HIGH);
      digitalWrite(2, HIGH);
      digitalWrite(10, HIGH);


    }


  }

}
void motor() {
  Serial.println("RUNNING MOTOR");

  randomi = random(3);
  if (randomi == 0) {
    digitalWrite(6, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(2, HIGH);
    digitalWrite(10, LOW);

  }
  if (randomi == 1) {
    digitalWrite(6, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(2, HIGH);
    digitalWrite(10, LOW);
  }
  if (randomi == 2) {
    digitalWrite(6, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(2, LOW);
    digitalWrite(10, HIGH);
  }


  motorScheduler.schedule(motor, 50);
}
//int toneFrequency = 400;
int  toneFrequency [4] = {200, 400, 500, 700};
//int toneDuration = 700;
int toneDuration [4] = {400, 200, 300, 200};
int buzzerPos = 0;
void buzzer() {
  Serial.println("RUNNING BUZZER");
  tone(speakerOut, toneFrequency[buzzerPos], toneDuration[buzzerPos]);
  buzzerPos++;
  if (buzzerPos >= 4)
    buzzerPos = 0;

  buzzerScheduler.schedule(buzzer, 100);
}

void acce() {

  //acceScheduler.schedule(acce, 1);

}
int servoIncrement = -7;
void servo() {
  Serial.println("RUNNING SERVO");
  if (pos >= 90 || pos <= 0)
    servoIncrement = -servoIncrement;
  pos = pos + servoIncrement;
  myservo.write(pos);
  /*
      for (pos = 0; pos <= 180; pos +=90){
        myservo.write(pos);

    }
    for(pos = 180; pos >=0; pos -=90){
    myservo.write(pos);

    }
  */
  servoScheduler.schedule(servo, 100);
}

