#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <ArduinoQueue.h>
#include <Wire.h>

#define encoderLPinA 6
#define encoderLPinB 7
#define encoderRPinA 8
#define encoderRPinB 9

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = motorShield.getMotor(1);
Adafruit_DCMotor *motorR = motorShield.getMotor(2);

volatile int lastEncodedL = 0, lastEncodedR = 0;
volatile long encoderValueL = 0, encoderValueR = 0;

void setup()
{
  motorShield.begin();
  motorL->setSpeed(200);
  Serial.begin(9600);

  // calculateSpeed();
  // delay(5000);

  // initialize encoder
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderLPinB, INPUT_PULLUP);
  pinMode(encoderRPinA, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);

  digitalWrite(encoderLPinA, HIGH);
  digitalWrite(encoderLPinB, HIGH);
  digitalWrite(encoderRPinA, HIGH);
  digitalWrite(encoderRPinB, HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderLPinA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLPinB), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPinA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPinB), updateEncoderR, CHANGE);
}

void loop()
{
  motorL->run(FORWARD);
  for(int i = 0; i < 5000; i++)
  {
    Serial.println(encoderValueL);
    delay(1);
  }

  motorL->setSpeed(0);
  while(true)
  {
    delay(100);
  }
}

void updateEncoderL() {
  int MSB = digitalRead(encoderLPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderLPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum =
      (lastEncodedL << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValueL--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValueL++;

  lastEncodedL = encoded; // store this value for next time
}

void updateEncoderR() {
  int MSB = digitalRead(encoderRPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderRPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum =
      (lastEncodedR << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValueR--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValueR++;

  lastEncodedR = encoded; // store this value for next time
}