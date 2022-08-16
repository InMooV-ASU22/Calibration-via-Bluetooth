/* ================================================================= //
//  Ain Shams University - Faculty of Engineering                    //
//  Ain Shams Virtal Hospital                                        //
//  HCM - Human Centered Mechatronics Lab.                           //
//  InMoov Robot Project                                             //
//                                                                   //
//  This is a unique work for InMoov Robot                           //
//  at Human Centered Mechatronics (HCM) Lab - Ain Shams University  //
//  by:                                                              //
//  ☑ Aly Mostafa Hafez                                              //
//  ☑ Hossam Nasr Elghareeb                                          //
//                                                                   //
//  Date: August 2022 (Latest version)                               //
//                                                                   //
//  Hardware used:                                                   //
//  · Arduino Mega                                                   //
//  · 2 PCA9685 I2C Servo Drivers                                    //
//  · Servomotors (to be calibrated)                                 //
//  · HC-05 Bluetooth Module                                         //
// ================================================================  */

#include  <Wire.h>
#include  <Adafruit_PWMServoDriver.h>
#include  <SoftwareSerial.h>

//SoftwareSerial MyBlue(17, 16); // RX | TX

/*  Insert here the pin number of the required servo to be calibrated
    via Right I2C Servo driver   */
int  Right_pin_number = 0;

/*  Insert here the pin number of the required servo to be calibrated
    via Left I2C Servo driver    */
int  Left_pin_number  = 0;

int  flag         = 2 ;
int  counter      = 300 ;
int  DutyToDegree = 0 ;
char z            = 0 ;
Adafruit_PWMServoDriver RightI2C = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver LeftI2C  = Adafruit_PWMServoDriver(0x41);

#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  //  MyBlue.begin(9600);
  Serial.println("8 channel Servo test!");

  RightI2C.begin();
  LeftI2C.begin();

  RightI2C.setOscillatorFrequency(27000000);
  RightI2C.setPWMFreq(SERVO_FREQ);
  delay(10);

  LeftI2C.setOscillatorFrequency(27000000);
  LeftI2C.setPWMFreq(SERVO_FREQ);

  LeftI2C.setPWM(0, 0, counter);
  RightI2C.setPWM(0, 0, counter);
  delay(10);
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  RightI2C.setPWM(n, 0, pulse);
  LeftI2C.setPWM(n, 0, pulse);
}

void loop()
{
  //Blue() ;

  if (Serial.available()) {
    z = Serial.read();
// ==================== Right I2C ==================== //
    if (z == 'a') {
      if (counter < SERVOMAX) {
        counter += 5;
        RightI2C.setPWM(Right_pin_number, 0, counter);
      }
    }
    else if (z == 'b') {
      if (counter > SERVOMIN) {
        counter -= 5;
        RightI2C.setPWM(Right_pin_number, 0, counter);
      }
    }
// ==================== Left I2C ==================== //
    else if (z == 'c') {
      if (counter < SERVOMAX) {
        counter += 5;
        LeftI2C.setPWM(Left_pin_number, 0, counter);
      }
    }
    else if (z == 'd') {
      if (counter > SERVOMIN) {
        counter -= 5;
        LeftI2C.setPWM(Left_pin_number, 0, counter);
      }
    }
    else {
      z = 'z';
    }
  }
  Serial.println(counter);
}
