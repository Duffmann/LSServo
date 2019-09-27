# LSServo
Arduino library to make all features of the smart serial [LX-16A servos by LewanSoul](http://www.lewansoul.com/product/detail-17.html) easily accessible to your code.
![Pic of LX-16A](http://www.lewansoul.com/uploads/picture/20180717/7d9310879aef7005136cc5792dc827ea.jpg)
## Usage
* Simply download the zip or clone this repo and move it into your `Arduino/libraries` directory.
* Add `#include <LSServo.h>` into your Arduino code
* Create an instance of the LSServo (`LSServo SERVO;`)
* `setup()` section:  make sure you open and assign the Serial port where you connected the LX-16A Servo(s)
* `main()` section: use any/all of the functions you find in `src/LSSProtocol.h` (massively commented)

## Example
Here is a simple example how to control a single servo using the lib (servo has to be programmed to ID "8" before):

```
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
```

## What for?
...and here is my low-cost QuadPed for which I created this lib. It was made only from a few 3D printed partes, 12 LX-16A servos and some metal servo brackets. By now, it walk's really smoothly thanks to the fantastic tutorial on inverse kinementics by [Oscar Liang](https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/). It is controlled by a small, cheap yet very powerful Cortex ARM M4 based STM Nucleo32 board as inverse kinematics and S-Curve acceleration control requires tons of float arithmetics.
![QuadPed](https://github.com/Duffmann/LSServo/blob/master/LX16A_QuadPed.jpg) 

*Note*: There are still some todo's (see code) and for sure also many, many coding issues with this library. This is my first endeavour to create an Arduino Library and also my first-ever public repo - so please do not expect this to be without flaws.
