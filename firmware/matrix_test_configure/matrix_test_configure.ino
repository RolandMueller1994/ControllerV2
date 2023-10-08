#include <Arduino.h>

#include <ShiftRegister74HC595.h>

ShiftRegister74HC595<10> sr(35, 36, 38);

void manualShiftOut(int clockPin, int latchPin, int dataPin, byte 
data) {

     digitalWrite(clockPin, LOW);
     digitalWrite(latchPin, LOW);

     for(int i=0; i<8; i++) {
        digitalWrite(dataPin, (data << i) & 0x80);
        digitalWrite(clockPin, HIGH);
        delay(1);
        digitalWrite(clockPin, LOW);
        delay(1);
      }

     digitalWrite(latchPin, HIGH);
}

uint8_t values[10];

void setup() {
  // put your setup code here, to run once:
  // Output Enable
  uint8_t reg = 1;
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  pinMode(34, OUTPUT);
  digitalWrite(34, LOW);
  pinMode(37, OUTPUT);
  digitalWrite(37, HIGH);

  delay(500);

  for(uint8_t i=0; i<8; i++)
    values[i] = 0xCC;
  values[8] = 0b01010011;
  values[9] = 0x00;
  //sr.setAll(values);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //sr.setAllHigh();
  //manualShiftOut(46, 44, 47, 0x11);

  values[5] = 0xEE;
  values[6] = 0xA9;
  values[9] = 0b00111111;
  /*
  uint8_t inNo = 9;
  uint8_t outNo = 11;

  uint8_t regNo = outNo/2;
  uint8_t regVal = 0;

  if(outNo%2 > 0) {
    regVal = (inNo << 4) & 0xF0;
    values[regNo] = 0x0C | regVal;
  } else {
    regVal = inNo & 0x0F;
    values[regNo] = 0xC0 | regVal;
  }*/

  
  
  
  sr.setAll(values);
  while(true) 
    delay(1000);
  
  
  //values[0] = 0x00;
  for(uint8_t i=0; i<11; i++) {
    for(uint8_t j=0; j<11; j++) {
      for(int a=0; a<10; a++)
        values[a] = 0xCC;
      uint8_t regNo = j/2;
      uint8_t regVal = 0;

      if(j%2 > 0) {
        regVal = (i << 4) & 0xF0;
        values[regNo] = 0x0C | regVal;
      } else {
        regVal = i & 0x0F;
        values[regNo] = 0xC0 | regVal;
      }
      
      sr.setAll(values);
      delay(3000);
    }
  }
  
}
