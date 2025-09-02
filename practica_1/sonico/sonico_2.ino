#include <NewPing.h>

#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 50


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 


void setup() {
  Serial.begin(9600);
}

void loop() {
  
  unsigned long t_start = micros();

  int round_time_us = sonar.ping(MAX_DISTANCE);

  float distance = round_time_us/59.974;

  unsigned long t_finish = micros();

  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.print(" cm\n");
  
  
  Serial.print("Tardo : ");
  Serial.print(t_finish - t_start);
  Serial.print(" us");
  Serial.print("\n");
  
}//tarda como máx (con 50cm = max dist) 5ms (200hz) y como minimo 2.5ms (400hz)
