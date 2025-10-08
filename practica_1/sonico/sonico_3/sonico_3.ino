#include <NewPing.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 40
#define PERIODO_MS 20


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 


void setup() {
  Serial.begin(9600);
}

void loop() {
  
  unsigned long t_start = micros();

  int round_time_us = sonar.ping(MAX_DISTANCE);

  float distance = round_time_us/59.974;

  
  Serial.print(distance);

  unsigned long t_finish = micros();

  
  int delay_us = PERIODO_MS*1000 - (t_finish - t_start);

  
  delayMicroseconds(delay_us);


  t_finish = micros();
  
  Serial.print("\nTardo : ");
  Serial.print(t_finish - t_start);
  Serial.print(" us");
  Serial.print("\n");
  
}//tarda como máx (con 50cm = max dist) 5ms (200hz) y como minimo 2.5ms (400hz)
