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
volatile long encodeTest = 0;

// for PID control
long previousTimeL = 0;
float ePreviousL = 0;
float eIntegralL = 0;
long previousTimeR = 0;
float ePreviousR = 0;
float eIntegralR = 0;

// PID gains
float kp = 2.0;
float kd = 0.05;
float ki = 0;

int param = -10000;

void setup()
{
  motorShield.begin();
  motorL->setSpeed(255);
  motorR->setSpeed(255);
  Serial.begin(115200);

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
  if (fabs(encoderValueL - param) > 1 || fabs(encoderValueR - param) > 1)
  {
    pidPositionMatchLeft(param);
    pidPositionMatchRight(param);
    Serial.print(param);
    Serial.print(", ");
    Serial.println(encoderValueL);
    Serial.print(", ");
    Serial.println(encoderValueR);
  }
  else
  {
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    delay(10);
  }
}

float pidPositionMatchCalculateLeft(int target, float kp, float kd, float ki) {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeL)) / 1.0e6;

  // compute error, derivative, integral
  int e = encoderValueL - target;
  float eDerivative = (e - ePreviousL) / deltaT;
  eIntegralL = eIntegralL + e * deltaT;

  // compute PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegralL);

  previousTimeL = currentTime;
  ePreviousL = e;

  return u;
}

float pidPositionMatchCalculateRight(int target, float kp, float kd, float ki) {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeR)) / 1.0e6;

  // compute error, derivative, integral
  int e = encoderValueR - target;
  float eDerivative = (e - ePreviousR) / deltaT;
  eIntegralR = eIntegralR + e * deltaT;

  // compute PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegralR);

  previousTimeR = currentTime;
  ePreviousR = e;

  return u;
}

void pidPositionMatchLeft(int target) {
  float u = pidPositionMatchCalculateLeft(target, kp, kd, ki);
  float speed = fabs(u);

  if (speed > 100) {
    speed = 100;
  }
  else if (speed < 20) {
    speed = 0;
  }

  motorL->setSpeed(speed);

  if (u < 0) {
    
    motorL->run(BACKWARD);
  }
  else {
    motorL->run(FORWARD);
  }
}

void pidPositionMatchRight(int target) {
  float u = pidPositionMatchCalculateRight(encoderValueL, kp, kd, ki);
  float speed = fabs(u);

  if (speed > 150) {
    speed = 150;
  }
  else if (speed < 20) {
    speed = 0;
  }

  motorR->setSpeed(speed);

  if (u > 0) {
    
    motorR->run(BACKWARD);
  }
  else {
    motorR->run(FORWARD);
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