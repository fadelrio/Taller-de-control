
void setup(void) {
  Serial.begin(115200);
  delay(100);
}

void loop() {
  
  unsigned long t_start = micros();
  
  sendn_floats(1);
  
  unsigned long t_finish = micros();
  
  Serial.print("\n");
  Serial.print("Tardo : ");
  Serial.print(t_finish - t_start);
  Serial.print(" us");
  Serial.print("\n");
}

//envia n floats por serial
void sendn_floats(int n){
  float f = 1.5;
  byte * b;
  for (int i = 0; i < n;i++){
    f = f+i;
    b = (byte *) &f;
    Serial.write(b,4);
  }
}
