#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SERVO_PIN 6
#define PERIODO_MS 20
#define MAX_DELAY_US 16383
#define CALIBRATION_STEPS 500
#define ALFA .8
#define M_SERVO 10.111111111111
#define B_SERVO 1470


float titag = 0;
float titaa = 0;
float tita =0;

float ang_servo = 0;

int grados[] = {40,-40,30,-30,20,-20,10,-10};
int i = 0;

int iteraciones_2seg = 0;

int calibration_steps = CALIBRATION_STEPS;

float offset_gyro_x = 0;

Adafruit_MPU6050 mpu;

Servo servo;

void setup() {
  //BEGIN SETUP ARDUINO  
  Serial.begin(115200);
  //END SETUP ARDUINO

  
  //BEGIN SETUP IMU
  if (!mpu.begin()) {
    Serial.println("No se encontró el 6050 gil");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  //END SETUP IMU

  //BEGIN SETUP SERVO
  servo.attach(SERVO_PIN);
  writeAnguloServo(servo, ang_servo);
  //END SETUP SERVO 

}

void loop() {

  //BEGIN FREQ BLOCK 1
  unsigned long t_start = micros();
  //END FREQ BLOCK 1

  //BEGIN CALI IMU
  sensors_event_t a, g, temp;

  if(calibration_steps > 0){
    
    mpu.getEvent(&a, &g, &temp);

    offset_gyro_x = offset_gyro_x + g.gyro.x/CALIBRATION_STEPS;

    calibration_steps--;

      unsigned long t_finish = micros();
  
    unsigned long delay_us = PERIODO_MS*1000 - (t_finish - t_start);

    if (delay_us > MAX_DELAY_US){
     delayMicroseconds(MAX_DELAY_US);
      delayMicroseconds(delay_us - MAX_DELAY_US);
      }else{
        delayMicroseconds(delay_us);
        }
   return;
  }
  //END CALI IMU

  //BEGIN DATA AQ IMU

  mpu.getEvent(&a, &g, &temp);

  titag = tita + (g.gyro.x-offset_gyro_x)*PERIODO_MS/1000.0;
    
  titaa = atan2(a.acceleration.y,a.acceleration.z);

  tita = ALFA*titag+(1-ALFA)*titaa;

  mandar_al_simulinkardo((tita*180/3.1416) - 1.1, ang_servo);

  //END DATA AQ IMU

  //BEGIN SERVO STEPS
  
  if(iteraciones_2seg == 100){
    ang_servo = grados[i];
    writeAnguloServo(servo, ang_servo);
    if(i < 7)
      i++;
  }
  
  //END SERVO STEPS

  //BEGIN ITER

  if(iteraciones_2seg < 100){
    iteraciones_2seg++;
  }else{
    iteraciones_2seg = 0;
  }

  //END ITER
  
  //BEGIN FREQ BLOCK 2

  unsigned long t_finish = micros();
  
  unsigned long delay_us = PERIODO_MS*1000 - (t_finish - t_start);

  if (delay_us > MAX_DELAY_US){
    delayMicroseconds(MAX_DELAY_US);
    delayMicroseconds(delay_us - MAX_DELAY_US);
    }else{
      delayMicroseconds(delay_us);
      }
  
  //END FREQ BLOCK 2
}

void mandar_al_simulinkardo(float tita, float ang_servo){
  Serial.write("coma");
  byte *b = (byte *) &tita;
  Serial.write(b,4);
  b = (byte *) &ang_servo;
  Serial.write(b,4);
}

void writeAnguloServo(Servo servo, float angulo){
  if (-90<angulo<90)
    servo.writeMicroseconds(angulo*M_SERVO + B_SERVO);
}
