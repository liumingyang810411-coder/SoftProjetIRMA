#include <Servo.h>

Servo shutter;

void setup() {
  shutter.attach(5);     

void loop() {
  shutter.write(0);      
  delay(1000);
  shutter.write(90);     
  delay(1000);
  shutter.write(180);    
  delay(1000);
}