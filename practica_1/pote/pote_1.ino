
int pin_pote = A0;

void setup() {

  Serial.begin(9600);

}

void loop() {
  int t_start = micros();

  int medida = analogRead(pin_pote);

  int t_finish = micros();

  Serial.println(medida);

  Serial.print("Tardo : ");
  Serial.print(t_finish - t_start);
  Serial.print(" us");
  Serial.print("\n");
}
//Tarda aproximadamente 116us, lo que da una frecuencia de 8.6kHz
