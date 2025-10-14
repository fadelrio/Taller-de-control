#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NewPing.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 40
#define SERVO_PIN 6
#define PERIODO_MS 20
#define MAX_DELAY_US 16383
#define CALIBRATION_STEPS_GYRO 100
#define CALIBRATION_STEPS_TITA 100
#define ALFA .8
#define M_SERVO 10.111111111111
#define B_SERVO 1470
#define PI 3.14159265359
#define OFFSET_CARRITO 14.5
//segundo controlador, con bilinear//primer controlador, con backward//primer controlador, con bilinear

#define KP 1.7


float titag = 0;
float titaa = 0;
float titaf = 0;
float tita = 0;

float ang_servo = 0;

float ang_servo_ant = ang_servo;

float referencia = 5;

float error = 0;


int iteraciones_5seg = 0;

int calibration_steps_gyro = CALIBRATION_STEPS_GYRO;
int calibration_steps_tita = CALIBRATION_STEPS_TITA;

float offset_gyro_x = 0;

unsigned long t_finish;

unsigned long delay_us;

float offset_tita = 0;

Adafruit_MPU6050 mpu;

Servo servo;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

void setup() {
  //BEGIN SETUP ARDUINO  
  Serial.begin(115200);
  //END SETUP ARDUINO

  
  //BEGIN SETUP IMU
  if (!mpu.begin()) {
    Serial.println("No se encontró el 6050");
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

  if(calibration_steps_gyro > 0){
    
    mpu.getEvent(&a, &g, &temp);

    offset_gyro_x = offset_gyro_x + g.gyro.x/CALIBRATION_STEPS_GYRO;

    calibration_steps_gyro--;

    t_finish = micros();
  
    delay_us = PERIODO_MS*1000 - (t_finish - t_start);

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

  titag = titaf + (g.gyro.x-offset_gyro_x)*PERIODO_MS/1000.0;
    
  titaa = atan2(a.acceleration.y,a.acceleration.z);

  titaf = ALFA*titag+(1-ALFA)*titaa;

  //END DATA AQ IMU

  //BEGIN CALI TITA
  if (calibration_steps_tita > 0){
    
    offset_tita = offset_tita + titaf/CALIBRATION_STEPS_TITA;

    calibration_steps_tita--;

    t_finish = micros();
  
    delay_us = PERIODO_MS*1000 - (t_finish - t_start);

    if (delay_us > MAX_DELAY_US){
     delayMicroseconds(MAX_DELAY_US);
      delayMicroseconds(delay_us - MAX_DELAY_US);
      }else{
        delayMicroseconds(delay_us);
        }
   return;
  }

  tita = titaf - offset_tita;

  //END CALI TITA

  //BEGIN DATA SONAR
  int round_time_us = sonar.ping(MAX_DISTANCE);

  float distancia = round_time_us/59.974 - OFFSET_CARRITO;
  //END DATA SONAR

  //BEGIN CONTROL
  error = referencia - distancia;
  
  ang_servo = -KP*error;

  writeAnguloServo(servo, ang_servo);

  mandar_al_simulink((tita*180/PI), ang_servo, distancia, referencia);
  
  //END CONTROL

  //BEGIN ITER

  if(iteraciones_5seg < 250){
    iteraciones_5seg++;
  }else{
    iteraciones_5seg = 0;
    referencia = referencia*-1;
  }

  //END ITER
  
  //BEGIN FREQ BLOCK 2

  t_finish = micros();
  
  delay_us = PERIODO_MS*1000 - (t_finish - t_start);

  if (delay_us > MAX_DELAY_US){
    delayMicroseconds(MAX_DELAY_US);
    delayMicroseconds(delay_us - MAX_DELAY_US);
    }else{
      delayMicroseconds(delay_us);
      }
  
  //END FREQ BLOCK 2
}

void mandar_al_simulink(float titam, float ang_servo, float distancia, float referencia){
  Serial.write("coma");
  byte *b = (byte *) &titam;
  Serial.write(b,4);
  b = (byte *) &ang_servo;
  Serial.write(b,4);
  b = (byte *) &distancia;
  Serial.write(b,4);
  b = (byte *) &referencia;
  Serial.write(b,4);
}

void writeAnguloServo(Servo servo, float angulo){
  if (-40<angulo<40)
    servo.writeMicroseconds(angulo*M_SERVO + B_SERVO);
}
