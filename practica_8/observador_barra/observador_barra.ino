#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SERVO_PIN 6
#define PERIODO_MS 20
#define PERIODO 0.02
#define MAX_DELAY_US 16383
#define CALIBRATION_STEPS_GYRO 100
#define CALIBRATION_STEPS_TITA 100
#define ALFA .8
#define M_SERVO 11
#define B_SERVO 1435
#define P1P2 146.6233
#define P1MP2 -35.8400
#define K 57.195
#define L11 0.7174
#define L12 0.0325
#define L21  -2.8313
#define L22 1.5589
#define L31  -0.0277
#define L32 -0.0007

float titag = 0;
float titaa = 0;
float titaf = 0;
float tita =0;

float omega_m = 0;

float err_tita = 0;

float err_titap = 0;

float tita_gorro_ant = 0;
float tita_punto_gorro_ant = 0;
float tita_gorro = 0;
float tita_punto_gorro = 0;
float b_gorro = 0;

float ang_servo = 0;

int grados[] = {20,-20};
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

  
  //BEGIN BIAS OMEGA
  omega_m = ((g.gyro.x-offset_gyro_x)*180/3.1416) + 10;
  //END BIAS OMEGA

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


  //BEGIN SERVO STEPS
  
  if(iteraciones_2seg == 100){
    ang_servo = grados[i];
    writeAnguloServo(servo, ang_servo);
    if(i < 3){
      i++;
    }else{
      i = 0;
    }

  }

  err_tita = tita*180/3.1416 - tita_gorro;
  err_titap = omega_m - (tita_punto_gorro + b_gorro);

  tita_gorro_ant = tita_gorro;
  tita_punto_gorro_ant = tita_punto_gorro;

  tita_gorro = tita_gorro_ant + tita_punto_gorro_ant*PERIODO + L11*err_tita + L12*err_titap;

  tita_punto_gorro = -tita_gorro_ant*P1P2*PERIODO + tita_punto_gorro_ant*(1-P1MP2*PERIODO) + (K*PERIODO)*ang_servo + L21*err_tita + L22*err_titap;

  b_gorro = b_gorro + L31*err_tita + L32*err_titap;

  mandar_al_simulink((tita*180/3.1416), ang_servo, tita_gorro, tita_punto_gorro, omega_m, b_gorro);
  
  //END SERVO STEPS

  //BEGIN ITER

  if(iteraciones_2seg < 100){
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

void mandar_al_simulink(float titam, float ang_servo, float tita_gorro, float tita_punto_gorro, float gyro_x, float b_gorro){
  Serial.write("coma");
  byte *b = (byte *) &titam;
  Serial.write(b,4);
  b = (byte *) &ang_servo;
  Serial.write(b,4);
  b = (byte *) &tita_gorro;
  Serial.write(b,4);
  b = (byte *) &tita_punto_gorro;
  Serial.write(b,4);
  b = (byte *) &gyro_x;
  Serial.write(b,4);
  b = (byte *) &b_gorro;
  Serial.write(b,4);
}

void writeAnguloServo(Servo servo, float angulo){
  if (-90<angulo<90)
    servo.writeMicroseconds(angulo*M_SERVO + B_SERVO);
}
