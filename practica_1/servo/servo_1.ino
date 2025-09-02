#include <Servo.h>

#define SERVO_PIN 6

Servo servo;

void setup() {

  servo.attach(SERVO_PIN);

}

void loop() {

  servo.writeMicroseconds(1000);

  delay(3000);

  servo.writeMicroseconds(1500);

  delay(3000);
  
  servo.writeMicroseconds(2000);

  delay(3000);


}
