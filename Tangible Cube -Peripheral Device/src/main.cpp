/*
 * This is the peripheral device
 * BT Address: 34b1f7d3beeb
 * Service UUID: 00001800-0000-1000-8000-00805f9b34fb
 */

#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  while(!Serial);

  do{
    Serial.write('Q');
    delay(100);
  } while(Serial.readString().length() < 7);

  Serial.write('Q');
  // Serial.print("peripheral device");
}

void loop() {
  if(Serial.available()){
    String msgQ = Serial.readStringUntil('\n');
    String msgq = "";
    String msgP = "";

    if(msgQ.length() > 4){
      Serial.write('q');

      do{
        msgq = Serial.readStringUntil('\n');
      }
      while(msgq.length() < 4);
      Serial.write('P');

      do{
        msgP = Serial.readStringUntil('\n');
      }
      while(msgP.length() < 4);
      Serial.println("");

      Serial.write('Q');
    }

  }
}