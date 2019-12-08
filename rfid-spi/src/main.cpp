#include <Arduino.h>

#include <SPI.h>
#include <MFRC522.h>


MFRC522 mfrc522(PA0, PA1);   // Create MFRC522 instance.

MFRC522::MIFARE_Key key;

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}