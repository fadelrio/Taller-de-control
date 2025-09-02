#include <NewPing.h>

#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 50


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 


void setup() {
  Serial.begin(9600);
}

void loop() {

  int round_time_us = sonar.ping(MAX_DISTANCE);

  float distance = round_time_us/59.974;

  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.print(" cm\n");
  
}
