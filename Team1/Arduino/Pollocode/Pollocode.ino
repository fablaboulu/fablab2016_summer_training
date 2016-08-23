//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <Servo.h>
#include<Wire.h>
int pos = 0;
int seconds = 0; //set the time in seconds here
int minutes = 0; //set the time in minutes here
int hours = 0; // set the time in hours here

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
const int MPU_addr = 0x69; // I2C address of the MPU-6050
int jotain = 1;
const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;
boolean toimi = true;
int aika = 5;
const int buzzerPin = 11; // Digital Pin 8
const int ledPin1 = 6;  // Digital Pin 12
const int ledPin2 = 3;  // Digital Pin 13 Built In Led can Change it if you want

int counter = 0;
int sensori = 0;
Servo myservo;

boolean serialInputReady = false;
String inputString = "";




void firstSection()
{
  if (toimi) {
    beep(a, 500);
    beep(a, 500);
    beep(a, 500);
    beep(f, 350);
    beep(cH, 150);
    beep(a, 500);
    beep(f, 350);
    beep(cH, 150);
    beep(a, 650);

    delay(500);

    beep(eH, 500);
    beep(eH, 500);
    beep(eH, 500);
    beep(fH, 350);
    beep(cH, 150);
    beep(gS, 500);
    beep(f, 350);
    beep(cH, 150);
    beep(a, 650);

    delay(500);
  }
}
void secondSection()
{
  if (toimi) {
    beep(aH, 500);
    beep(a, 300);
    beep(a, 150);
    beep(aH, 500);
    beep(gSH, 325);
    beep(gH, 175);
    beep(fSH, 125);
    beep(fH, 125);
    beep(fSH, 250);

    delay(325);

    beep(aS, 250);
    beep(dSH, 500);
    beep(dH, 325);
    beep(cSH, 175);
    beep(cH, 125);
    beep(b, 125);
    beep(cH, 250);

    delay(350);
  }
}
void beep(int note, int duration)
{
  if (toimi) {
    tone(buzzerPin, note, duration);

    //Play different LED depending on value of 'counter'
    if (counter % 2 == 0)
    {
      digitalWrite(ledPin1, HIGH);
      delay(duration);
      digitalWrite(ledPin1, LOW);
    }
    else
    {
      digitalWrite(ledPin2, HIGH);
      delay(duration);
      digitalWrite(ledPin2, LOW);
    }

    //Stop tone on buzzerPin
    noTone(buzzerPin);

    delay(50);

    //Increment counter
    counter++;
  }
}
void servo() {
  if (toimi) {

    digitalWrite(5, HIGH);
    digitalWrite(9, LOW);

    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(2);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(2);                       // waits 15ms for the servo to reach the position
    }
  }
}
void staph() {

  if (GyY > 2000 || GyY < -1000) {
    toimi = true;
    digitalWrite(12, LOW);
    digitalWrite(7, LOW);
    digitalWrite(5, LOW);
    digitalWrite(9, LOW);


  }

  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);

}


void setup() {

  aika = seconds * 1000 + minutes * 60000 + hours * 3600000;

  delay(10000);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(7, OUTPUT);
  myservo.attach(4);


}
void checkSensorData() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();
  staph();
}
void loop() {

  checkSensorData();
  if (toimi) {

    digitalWrite(12, LOW);
    digitalWrite(5, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(9, HIGH);
  }
  else {
    digitalWrite(12, LOW);
    digitalWrite(5, LOW);
    digitalWrite(7, LOW);
    digitalWrite(9, LOW);
  }
  firstSection();
  checkSensorData();
  delay(100);
  servo();
  delay(100);
  checkSensorData();

  secondSection();
  checkSensorData();
  if (toimi) {
    beep(f, 250);
    beep(gS, 500);
    beep(f, 350);
    beep(a, 125);
    beep(cH, 500);
    beep(a, 375);
    beep(cH, 125);
    beep(eH, 650);
  }

  checkSensorData();
  delay(500);

  checkSensorData();

  secondSection();
  checkSensorData();
  delay(100);

  checkSensorData();
  if (toimi) {
    beep(f, 250);
    beep(gS, 500);
    beep(f, 375);
    beep(cH, 125);
    beep(a, 500);
    beep(f, 375);
    beep(cH, 125);
    beep(a, 650);
    checkSensorData();
    delay(650);
  }
}


