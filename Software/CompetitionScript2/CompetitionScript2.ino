#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <queue>
#include <math.h>

#define MPU6050_ADDR 0x68

#define buttonPin       4

// motor 1 = left, motor 2 = right
#define motor1Pin1      5
#define motor1Pin2      6
#define motor2Pin1      7
#define motor2Pin2      8

#define motor1PWMPin    9
#define motor2PWMPin    10

#define sensorTrigPin   12
#define sensorEchoPin   11