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

#include<Wire.h>          // Accelerometer
const int MPU_addr = 0x69; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int n = 0;


class Flasher1      //LEDit
{
    int ledPin;      // the number of the LED pin
    long OnTime;
    long OffTime;    // milliseconds of interval

    // These maintain the current state
    int ledState;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    Flasher1(int pin, long on, long off)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = on;
      OffTime = off;

      ledState = LOW;
      previousMillis = 0;
    }

    void Update()
    {
      unsigned long currentMillis = millis();

      if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = LOW;  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = HIGH;  // turn it on
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(ledPin, ledState);   // Update the actual LED
      }
    }
};

Flasher1 led1(12, 500, 600);

class Piezo1        //ääni
{
    // Class Member Variables
    // These are initialized at startup
    int piezo1Pin;      // the number of the LED pin
    long OnTime;
    long OffTime;    // milliseconds of interval
    int toneState;

    // These maintain the current state
    int piezo1State;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    Piezo1 (int pin, long on, long off)
    {
      piezo1Pin = pin;
      pinMode(piezo1Pin, OUTPUT);

      OnTime = on;
      OffTime = off;

      piezo1State = HIGH;
      previousMillis = 0;
      tone(8, 400);
      tone(9, 400);
      toneState = LOW;
    }

    void Update()
    {
      unsigned long currentMillis = millis();

      if  ((toneState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        tone(9, 1000);  // Turn it off
        tone(8, 1000);
        toneState = LOW;
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(piezo1Pin, piezo1State);  // Update the actual LED
      }
      else if ((toneState == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        tone(9, 400); // turn it on
        tone(8, 400);
        toneState = HIGH;
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(piezo1Pin, piezo1State);   // Update the actual LED
      }
    }
};

Piezo1 piezo1(9, 500, 1000);
Piezo1 piezo2(8, 500, 1000);



class Motor1        //moottorit
{
    // Class Member Variables
    // These are initialized at startup
    int motor1Pin;      // the number of the motor pin
    long OnTime;    // milliseconds of interval
    long OffTime;
    // These maintain the current state
    int motor1State;                 // motorState used to set the motor
    unsigned long previousMillis;   // will store last time LED was updated

  public:
    Motor1(int pin, long on, long off)
    {
      motor1Pin = pin;
      pinMode(motor1Pin, OUTPUT);

      OnTime = on;
      OffTime = off;

      motor1State = LOW;
      previousMillis = 0;
    }

    void Update()
    {
      // check to see if it's time to change the state of the LED
      unsigned long currentMillis = millis();

      if ((motor1State == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        motor1State = LOW;  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(motor1Pin, motor1State);  // Update the actual LED
      }
      else if ((motor1State == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        motor1State = HIGH;  // turn it on
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(motor1Pin, motor1State);   // Update the actual LED
      }
    }
};


Motor1 motor1(2, 2489, 2489);
Motor1 motor2(5, 5987, 5987);


class Motor2          //moottori
{
    // Class Member Variables
    // These are initialized at startup
    int motor2Pin;      // the number of the motor pin
    long OnTime;    // milliseconds of interval
    long OffTime;
    // These maintain the current state
    int motor2State;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    Motor2(int pin, long on, long off)
    {
      motor2Pin = pin;
      pinMode(motor2Pin, OUTPUT);

      OnTime = on;
      OffTime = off;

      motor2State = HIGH;
      previousMillis = 0;
    }

    void Update()
    {
      // check to see if it's time to change the state of the LED
      unsigned long currentMillis = millis();

      if ((motor2State == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        motor2State = LOW;  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(motor2Pin, motor2State);  // Update the actual LED
      }
      else if ((motor2State == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        motor2State = HIGH;  // turn it on
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(motor2Pin, motor2State);   // Update the actual LED
      }
    }
};


Motor2 motor3(3, 2489, 2489);
Motor2 motor4(6, 5987, 5987);


#include <Servo.h>

class Sweeper             //servo
{
    Servo servo;              // the servo
    int pos;              // current servo position
    int increment;        // increment to move for each interval
    int  updateInterval;      // interval between updates
    unsigned long lastUpdate; // last update of position

  public:
    Sweeper(int pin, int interval)
    {
      updateInterval = interval;
      increment = 2;
    }

    void Attach(int pin)
    {
      servo.attach(pin);
    }

    void Detach()
    {
      servo.detach();
    }

    void Update()
    {
      if ((millis() - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        pos += increment;
        servo.write(pos);
        Serial.println(pos);
        if ((pos >= 180) || (pos <= 0)) // end of sweep
        {
          // reverse direction
          increment = -increment;
        }
      }
    }
};

Sweeper sweeper1(11, 10);

class Flasher2      //led
{
    // Class Member Variables
    // These are initialized at startup
    int ledPin;      // the number of the LED pin
    long OnTime;
    long OffTime;    // milliseconds of interval

    // These maintain the current state
    int ledState;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    Flasher2(int pin, long on, long off)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = on;
      OffTime = off;

      ledState = HIGH;
      previousMillis = 0;
    }

    void Update()
    {
      unsigned long currentMillis = millis();

      if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = LOW;  // Turn it off
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = HIGH;  // turn it on
        previousMillis = currentMillis;   // Remember the time
        digitalWrite(ledPin, ledState);   // Update the actual LED
      }
    }
};

Flasher2 led2(13, 650, 450);



void setup() {

  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  sweeper1.Attach(11);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis < 30000) {      //10s tauko ennen alkua
    noTone(8);
    noTone(9);
  }

  if (n > 8) {                      //pysäytys ravistuksesta
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    noTone(9);
    noTone(8);

  }
  if ((n < 8) && (currentMillis > 30000)) {   //pysäytys ravistuksesta
    led1.Update();                              //ja 10s tauko
    led2.Update();
    sweeper1.Update();
    motor1.Update();
    motor2.Update();
    motor3.Update();
    motor4.Update();
    piezo1.Update();
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

  if (AcY >= 5000) {                        //ravistus
    n++;
  }
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
}
