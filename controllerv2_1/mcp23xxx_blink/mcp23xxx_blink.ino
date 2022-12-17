// Blinks an LED attached to a MCP23XXX pin.

// ok to include only the one needed
// both included here to make things simple for example
#include "Adafruit_MCP23X08.h"
#include "Adafruit_MCP23X17.h"

#define LED_PIN 0     // MCP23XXX pin LED is attached to

// only used for SPI
#define CS_PIN 6

// uncomment appropriate line
Adafruit_MCP23X08 mcp;
//Adafruit_MCP23X17 mcp;

void setup() {
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(250);
  digitalWrite(12, HIGH);
  delay(250);
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("MCP23xxx Blink Test!");
  
  // uncomment appropriate mcp.begin
  if (!mcp.begin_I2C(0x23)) {
  //if (!mcp.begin_SPI(CS_PIN)) {
    Serial.println("Error.");
    while (1);
  }

  // configure pin for output
  Serial.println(mcp.pinMode(LED_PIN, OUTPUT));
  Serial.println(mcp.readIoDir());

  Serial.println("Looping...");
}

void loop() {
  
  Serial.println(mcp.digitalWrite(LED_PIN, 1));
  delay(500);
  Serial.println(mcp.digitalWrite(LED_PIN, 0));
  delay(500);
}
