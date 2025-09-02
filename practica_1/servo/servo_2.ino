#include <Servo.h>

#define SERVO_PIN 6

Servo servo;

void setup() {

  servo.attach(SERVO_PIN);

}

void loop() {

  servo.writeMicroseconds(2380);

  delay(3000);

  servo.writeMicroseconds(1470);

  delay(3000);

  servo.writeMicroseconds(560);

  delay(3000);


}//560 - 2380
