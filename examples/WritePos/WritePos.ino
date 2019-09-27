#include <LSServo.h>

LSServo SERVO;

#define ID 8

void setup()
{
  Serial1.begin(115200);
  SERVO.pSerial = &Serial1;
  delay(500);
  //SERVO.EnableTorque(ID, 1);
}

void loop()
{
  SERVO.SetPos(ID, 500, 1800);
  delay(2000); 
  SERVO.SetPos(ID, 900, 1800);
  delay(2000);
  SERVO.SetPos(ID, 500, 1800);
  delay(2000);
  SERVO.SetPos(ID, 100, 1800);
  delay(10000);
}
