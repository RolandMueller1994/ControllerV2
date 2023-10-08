#include <Arduino.h>

int sends[] = {30, 32, 34, 36, 38, 40, 42, 44, 46, 24, 52};
int returns[] = {31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51};

void setup() {

  for(int i=0; i<11; i++) {
    pinMode(sends[i], INPUT);
    pinMode(returns[i], OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  for(int retNo=0; retNo<11; retNo++) {
    Serial.print("\nReturn No: ");
    Serial.print(retNo);
    Serial.print("\n");
    for(int i=0; i<11; i++) {
      if(i==retNo)
        digitalWrite(returns[i], HIGH);
      else
        digitalWrite(returns[i], LOW);
    }
    for(int sendNo=0; sendNo<11; sendNo++) {
      if(digitalRead(sends[sendNo])) {
        Serial.print("Conn :");
        Serial.print(sendNo);
        Serial.print("\n");
      }
    }
  }
  delay(3000);

}
