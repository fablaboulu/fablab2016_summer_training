#include <Scheduler.h>
#include <Servo.h>

int rightmotor = 6, leftmotor = 5, buzzer = 10;

int pos = 0;


Servo lhand;
Servo rhand;
Scheduler motorScheduler=Scheduler();
Scheduler servoScheduler=Scheduler();
Scheduler buzzerScheduler=Scheduler();
void setup() {
  pinMode(rightmotor, OUTPUT);
  pinMode(leftmotor, OUTPUT);
  pinMode(buzzer, OUTPUT);
  lhand.attach(13);
  rhand.attach(12);
  servoScheduler.schedule(servo, 50);
  motorScheduler.schedule(motors, 10000);
  buzzerScheduler.schedule(sound, 1000);
}

void loop() {
motorScheduler.update();
servoScheduler.update();
buzzerScheduler.update();
}
  void servo(){
  for (pos = 0; pos <= 45; pos += 15) {
    lhand.write(pos);
    rhand.write(pos);
    delay(50);
  }
  for (pos = 45; pos >= 0; pos -= 15) {
  lhand.write(pos);
  rhand.write(pos);
  delay(50);
  }
    servoScheduler.schedule(servo, 1000 );
  }

  void motors() {

  digitalWrite(leftmotor, HIGH);
  digitalWrite(rightmotor, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(8, LOW);
  
  }

  void sound() {

    tone (11, 500);
    
    
  }

  
