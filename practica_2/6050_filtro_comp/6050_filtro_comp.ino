

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PERIODO_MS 20
#define MAX_DELAY_US 16383
#define CALIBRATION_STEPS 500
#define ALFA .8

float titag = 0;
float titaa = 0;
float tita =0;

int calibration_steps = CALIBRATION_STEPS;

float offset_gyro_x = 0;

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

  if(calibration_steps > 0){
    
    mpu.getEvent(&a, &g, &temp);

    offset_gyro_x = offset_gyro_x + g.gyro.x/CALIBRATION_STEPS;

    calibration_steps--;

  }else{
    mpu.getEvent(&a, &g, &temp);

    titag = tita + (g.gyro.x-offset_gyro_x)*PERIODO_MS/1000.0;
    
    titaa = atan2(a.acceleration.y,a.acceleration.z);

    tita = ALFA*titag+(1-ALFA)*titaa;

    mandar_al_simulinkardo(tita*180/3.1416,titag*180/3.1416,titaa*180/3.1416);

  }
  
  
  
  unsigned long t_finish = micros();
  
  unsigned long delay_us = PERIODO_MS*1000 - (t_finish - t_start);

  if (delay_us > MAX_DELAY_US){
    delayMicroseconds(MAX_DELAY_US);
    delayMicroseconds(delay_us - MAX_DELAY_US);
    }else{
      delayMicroseconds(delay_us);
      }
}

void mandar_al_simulinkardo(float tita,float titag,float titaa){
  Serial.write("coma");
  byte *b = (byte *) &tita;
  Serial.write(b,4);
  b = (byte *) &titag;
  Serial.write(b,4);
  b = (byte *) &titaa;
  Serial.write(b,4);
}
