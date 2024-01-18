
// Packages
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = motorShield.getMotor(1); 
Adafruit_DCMotor *motorR = motorShield.getMotor(3); 

void setup()
{   
  motorShield.begin();
  motorL->run(BRAKE);
  motorR->run(BRAKE);
  motorL->setSpeed(150);
  motorR->setSpeed(150);
  delay(5000);
}

void loop()
{
  
}

// Motor testing
/*
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  delay(5000);
  motorL->setSpeed(0);
  motorR->setSpeed(0);
*/