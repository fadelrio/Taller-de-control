#include <Servo.h>

#define SERVO_PIN 6
#define PERIODO_MS 1000

int pin_pote = A0;

Servo servo;

void setup() {

  Serial.begin(9600);

  servo.attach(SERVO_PIN);

}

void loop() {

  
  unsigned long t_start = micros();

  int medida = analogRead(pin_pote);

  int valor_servo = medida*((2380-560)/1024.0) + 560;

  Serial.println(valor_servo);

  servo.writeMicroseconds(valor_servo);


  unsigned long t_finish = micros();

  unsigned long delay_ms = PERIODO_MS - ((t_finish - t_start)/1000);

  delay(delay_ms);
  


}//560 - 2380
