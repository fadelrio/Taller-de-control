
int pin_pote = A0;

#define AMPLITUD_ANGULAR_POTE 270.0
#define DEF_ADC 1024
#define PERIODO_MS 20
#define MAX_DELAY_US 16383

void setup() {

  Serial.begin(9600);

}

void loop() {
  unsigned long t_start = micros();

  int medida = analogRead(pin_pote);
  
  float angulo = -medida*(AMPLITUD_ANGULAR_POTE/DEF_ADC) + 135;


  Serial.print(angulo);

  unsigned long t_finish = micros();

  unsigned long delay_us = PERIODO_MS*1000 - (t_finish - t_start);

  if (delay_us > MAX_DELAY_US){
    delayMicroseconds(MAX_DELAY_US);
    delayMicroseconds(delay_us - MAX_DELAY_US);
    }else{
      delayMicroseconds(delay_us);
      }

  t_finish = micros();
  
  Serial.print("\nTardo : ");
  Serial.print(t_finish - t_start);
  Serial.print(" us\n");
}
