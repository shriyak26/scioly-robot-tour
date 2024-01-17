
// Packages
#include "bsp_api.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = motorShield.getMotor(1); 
Adafruit_DCMotor *motorRight = motorShield.getMotor(3); 

void setup()
{
    
    motorShield.begin();
    motorLeft->setSpeed(255);
}

void loop()
{
    motorLeft->run(FORWARD);
}