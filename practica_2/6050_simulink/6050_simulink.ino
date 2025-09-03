

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PERIODO_MS 20
#define MAX_DELAY_US 16383

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("No se encontró el 6050 gil");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

void loop() {
  unsigned long t_start = micros();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float data[6] = {a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z};

  mandar_al_simulinkardo(data);
  unsigned long t_finish = micros();
  
  unsigned long delay_us = PERIODO_MS*1000 - (t_finish - t_start);

  if (delay_us > MAX_DELAY_US){
    delayMicroseconds(MAX_DELAY_US);
    delayMicroseconds(delay_us - MAX_DELAY_US);
    }else{
      delayMicroseconds(delay_us);
      }
}

void mandar_al_simulinkardo(float data[]){
  Serial.write("coma");
  byte * b;
  for(int i = 0;i < 6; i++){
    b = (byte *) &data[i];
    Serial.write(b,4);
  }
}