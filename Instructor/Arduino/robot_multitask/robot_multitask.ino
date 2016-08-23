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

//AUTHOR: Iván Sánchez Milara
#include <Scheduler.h>
#include<Wire.h>
#include <Servo.h>

Scheduler motorTaskScheduler=Scheduler();
Scheduler buzzerTaskScheduler=Scheduler();
Scheduler ledTaskScheduler=Scheduler();
Scheduler servoTaskScheduler=Scheduler();
Scheduler accTaskScheduler = Scheduler();

const long DELAY = 5000;

const int MOTOR_INTERVAL=2000;
const int BUZZER_INTERVAL=1300;
const int LED_INTERVAL = 500;
const int SERVO_INTERVAL = 5;
const int ACC_INTERVAL=5;

const int MOTOR1_PINA=2;
const int MOTOR1_PINB=8;
const int MOTOR2_PINA=5;
const int MOTOR2_PINB=6;
const int SERVO_PIN=12;
const int BUZZER_PINA=10;
const int BUZZER_PINB=11;
const int LED1_PIN=9;
const int LED2_PIN=13;

//Accelerometer address
const int MPU_addr=0x69;

//SERVO STATE VARIABLES
Servo servo;

//MOTOR STATE VARIABLES
/*First states moves the motors forward for a while
 * Second states moves the motors randomly
 */
 int motorState = LOW;

 //BEEP VARIABLES AND CONSTANTS 
const int NOTE_DURATION = 1000; 
const int NOTE_FREQUENCY = 440;

//LED VARIABLES AND CONSTANTS
int led1State = HIGH;
int led2State = LOW;

//SERVO VARIABLES AND CONSTANTS
int servoIncrement=1; 
int servoPos=0;

//ACC VARIABLES AND CONSTANTS;
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;


// Used to indicate the times that the accTaskelerometer Y accTaskess is over the threshold. When this number is 5 we stop the robot. 
int n = 0;

//Used to indicate when the robot should start moving.
boolean isWaiting = true; 


void setup() {
  motorTaskScheduler.schedule(motorTask, MOTOR_INTERVAL);
  buzzerTaskScheduler.schedule(buzzerTask, BUZZER_INTERVAL);
  ledTaskScheduler.schedule(ledTask, LED_INTERVAL);
  servoTaskScheduler.schedule(servoTask, SERVO_INTERVAL);
  accTaskScheduler.schedule(accTask, ACC_INTERVAL);
  pinMode (LED1_PIN, OUTPUT);
  pinMode (LED2_PIN, OUTPUT);
  pinMode(BUZZER_PINB, OUTPUT);
  pinMode(MOTOR1_PINA, OUTPUT);
  pinMode(MOTOR1_PINB, OUTPUT);
  pinMode(MOTOR2_PINA, OUTPUT);
  pinMode(MOTOR2_PINB, OUTPUT);
  
  digitalWrite(BUZZER_PINB, LOW);

  //SETUP ACCELEROMETER
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);

  //SERIAL
  Serial.begin(9600);
}

void loop() {
    if (isWaiting){
      String delayMessage="Waiting " + String(DELAY) + " seconds";
      Serial.println(delayMessage);
      delay(DELAY);
      isWaiting =false;
      initRobot();
    }
    else {
      if (n < 5){
         motorTaskScheduler.update();
         buzzerTaskScheduler.update();
         ledTaskScheduler.update();
         servoTaskScheduler.update();
         accTaskScheduler.update();
      }

      else{
        stopRobot();
      }
    }
}

void initRobot() {
  Serial.println("Starting the robot");
  //Setup the servo pin
  servo.attach(SERVO_PIN);
  // Turn on led1 and off led2
  digitalWrite(LED1_PIN, led1State);
  digitalWrite(LED2_PIN, led2State);
}

void stopRobot() {
  //STOP motorTaskS; ledTask; servoTask; accTask; buzzerTask;
  servo.detach();
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  digitalWrite(MOTOR1_PINA, LOW);
  digitalWrite(MOTOR1_PINB, LOW);
  digitalWrite(MOTOR2_PINA, LOW);
  digitalWrite(MOTOR2_PINB, LOW);

  Serial.println("Robot is sleeping. ZZZZZZ ZZZZZZ");
  //Wait for other DELAY; remove data from the acc.
  isWaiting = true;
  n=0;
  
}

 
void motorTask() {
  Serial.println("MOTOR RUNNING");
  if(motorState == HIGH){
    motorState = LOW;
    
    digitalWrite(MOTOR1_PINA, LOW);
    digitalWrite(MOTOR1_PINB, HIGH); 
    digitalWrite(MOTOR2_PINA, LOW);
    digitalWrite(MOTOR2_PINB, HIGH); 
  }
   else if (motorState == LOW)  {
      motorState = HIGH;  // turn it on
      moveRandom(MOTOR1_PINA, MOTOR1_PINB, 3);
      moveRandom(MOTOR2_PINA, MOTOR2_PINB, 4);
    }
  //Execute here the code to change the state of the motorTask: go left, go right, go straight, go backward.
  motorTaskScheduler.schedule(motorTask, MOTOR_INTERVAL);
}
void moveRandom(int motorTerminal1, int motorTerminal2, long value){
  long randomi = random(value);
  
  if (randomi == 0){
    digitalWrite(motorTerminal1, LOW); 
    digitalWrite(motorTerminal2, HIGH);
  }

  else if (randomi == 1){
    digitalWrite(motorTerminal1, HIGH); 
    digitalWrite(motorTerminal2, LOW);
  }

  else if (randomi == 2){
    digitalWrite(motorTerminal1, HIGH); 
    digitalWrite(motorTerminal2, LOW);
  }
  
}


void buzzerTask(){
  //Execute here the code to execute to play one note in the buzzerTask
  tone(BUZZER_PINA, NOTE_FREQUENCY, NOTE_DURATION);
  //Reschedule
  buzzerTaskScheduler.schedule(buzzerTask, BUZZER_INTERVAL);
}


void ledTask(){
  //Execute here the code to switch on/off the led
  if(led1State == HIGH){
    led1State = LOW; 
  }
  else {
    led1State = HIGH;
  }

  if (led2State == HIGH){
    led2State = HIGH;
  }
  else {
    led2State = LOW;
  }
  digitalWrite(LED1_PIN, led1State);
  digitalWrite(LED2_PIN, led2State);
  //Schedule again the task
  ledTaskScheduler.schedule(ledTask, LED_INTERVAL);
}


void servoTask(){
  //Execute here the code to turn the servoTask SERVO_INCREMENT.
  servoPos += servoIncrement;
  servo.write(servoPos);
  if ((servoPos >= 180) || (servoPos <= 0)) // end of sweep
  {
    // reverse direction
    servoIncrement = -servoIncrement;
  }
  //Schedule the task for the servo
  servoTaskScheduler.schedule(servoTask, SERVO_INTERVAL);
}

void accTask() {
    //Execute here the code to check accTaskelerometer movement.
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    if (AcX > 5000){
      n++;
    }
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    /*Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);*/
      
  //Run the scheduler
  accTaskScheduler.schedule(accTask,ACC_INTERVAL);
}






