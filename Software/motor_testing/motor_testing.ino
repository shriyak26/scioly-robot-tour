#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <ArduinoQueue.h>
#include <Wire.h>
#include <MD_REncoder.h>

// Pins for the encoder
#define encoderLPinA 6
#define encoderLPinB 7
#define encoderRPinA 8
#define encoderRPinB 9

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = motorShield.getMotor(1);
Adafruit_DCMotor *motorR = motorShield.getMotor(2);

MD_REncoder encoderL = MD_REncoder(6,7);

// Variables to store the encoder values
volatile int encoderLValue = 0;

void setup() {
  motorShield.begin();
  motorL->setSpeed(200);
  encoderL.begin();
  

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if the encoder value has changed
  motorL->run(FORWARD);
  for(int i = 0; i < 1000; i++)
  {
    uint8_t x = encoderL.read();
    Serial.println(x);
    delay(1);
  }

  motorL->setSpeed(0);
  while(true)
  {
    delay(100);
  }
}