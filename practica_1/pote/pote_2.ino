
int pin_pote = A0;

#define AMPLITUD_ANGULAR_POTE 270.0
#define DEF_ADC 1024


void setup() {

  Serial.begin(9600);

}

void loop() {
  int t_start = micros();

  int medida = analogRead(pin_pote);
  
  float angulo = -medida*(AMPLITUD_ANGULAR_POTE/DEF_ADC) + 135;
  
  int t_finish = micros();

  Serial.print("El angulo es de ");
  Serial.print(angulo);
  Serial.print(" grados\n");

  Serial.print("Tardo : ");
  Serial.print(t_finish - t_start);
  Serial.print(" us\n");
}
