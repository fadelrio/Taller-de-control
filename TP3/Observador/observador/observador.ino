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
#define ALFA .95
#define M_SERVO 11
#define B_SERVO 1435
#define PI 3.14159265359
#define OFFSET_CARRITO 14.5
//segundo controlador, con bilinear//primer controlador, con backward//primer controlador, con bilinear
#define L11 1.3243
#define L12 -0.0001
#define L21 15.8964
#define L22 -0.5027
#define L31 0
#define L32 0.8765
#define L41 0
#define L42 -5.8917
#define KP 1.5


float titag = 0;
float titaa = 0;
float titaf = 0;
float tita = 0;

float error = 0;

float referencia = 0;

float sen_control = 0;

float x1_h = 0;
float x1_h_ant = 0;
float x2_h = 0;
float x2_h_ant = 0;
float x3_h = 0;
float x3_h_ant = 0;
float x4_h = 0;
float x4_h_ant = 0;

float distancia_ant = 0;

float velocidad = 0;

float err_1 = 0;

float err_2 = 0;

int refes[] = {-8,0,8,0};
int i = 0;


int iteraciones_2seg = 0;

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
  writeAnguloServo(servo, referencia);
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

  //BEGIN ANGLE STEPS
  if(iteraciones_2seg == 120){
    referencia = refes[i];
    if(i < 3)
      i++;
  }
  //END ANGLE STEPS

  //BEGIN CONTROL
  error = referencia - distancia;

  sen_control = -KP*error;
  
  writeAnguloServo(servo, sen_control);
  //END CONTROL

  //BEGIN OBSERVADOR

  velocidad = (distancia - distancia_ant)/0.02;
  distancia_ant = distancia;

  err_1 = distancia - x1_h_ant;
  err_2 = tita*180/PI - x3_h_ant;

  x1_h = x1_h_ant + 0.02*x2_h_ant + L11*err_1 + L12*err_2;
  x2_h = 0.906*x2_h_ant - 0.5*x3_h_ant + L21*err_1 + L22*err_2;
  x3_h = x3_h_ant + 0.02*x4_h_ant + L31*err_1 + L32*err_2;
  x4_h = -5.2414*x3_h_ant + 0.209*x4_h_ant + L41*err_1 + L42*err_2 + 2.2878*sen_control;

  x1_h_ant = x1_h;
  x2_h_ant = x2_h;
  x3_h_ant = x3_h;
  x4_h_ant = x4_h;

  //END OBSERVADOR

  //BEGIN SEND DATA

  mandar_al_simulink(distancia, x1_h, tita*180/PI, x3_h, (g.gyro.x-offset_gyro_x)*180/PI, x4_h, velocidad, x2_h);
  
  //END SEND DATA

  //BEGIN ITER

  if(iteraciones_2seg < 120){
    iteraciones_2seg++;
  }else{
    iteraciones_2seg = 0;
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

void mandar_al_simulink(float distancia, float x1_h, float tita_barra, float x3_h, float vel_ang, float x4_h,float velocidad, float x2_h){
  Serial.write("coma");
  byte *b = (byte *) &distancia;
  Serial.write(b,4);
  b = (byte *) &x1_h;
  Serial.write(b,4);
  b = (byte *) &tita_barra;
  Serial.write(b,4);
  b = (byte *) &x3_h;
  Serial.write(b,4);
  b = (byte *) &vel_ang;
  Serial.write(b,4);
  b = (byte *) &x4_h;
  Serial.write(b,4);
  b = (byte *) &velocidad;
  Serial.write(b,4);
  b = (byte *) &x2_h;
  Serial.write(b,4);
}

void writeAnguloServo(Servo servo, float angulo){
  if (-30<angulo<30)
    servo.writeMicroseconds(angulo*M_SERVO + B_SERVO);
}
