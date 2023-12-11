#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <QList.h>
#include <math.h>
#include <MPU6050.h>

// max and min speed of motors
const int maxSpeed = 255;
const int minSpeed = 150;

// equilibrium speed and target angle values
int equilibriumSpeed = 200;
float targetAngle = 0;
float angle = 0;            // for MPU mounting orientation

// immediate speed values for algorithm to tune
int leftSpeed;
int rightSpeed;

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// start button pin
#define buttonPin       4

// motor pins (motor 1 = left, motor 2 = right)
#define motor1Pin1      5
#define motor1Pin2      6
#define motor2Pin1      7
#define motor2Pin2      8

// motor PWM pins (speed control)
#define motor1PWMPin    9
#define motor2PWMPin    10

// ultrasonic distance sensor pins
#define sensorTrigPin   12
#define sensorEchoPin   11

float AccX, AccY, AccZ;                                         // accelerometer values (linear acceleration)
float GyroX, GyroY, GyroZ;                                      // gyroscope values (angular velocity)
float AccAngleX, AccAngleY;                                     // accelerometer angles
float GyroAngleX, GyroAngleY, GyroAngleZ;                       // gyroscope angles
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; // error values
float elapsedTime, currentTime, previousTime;                   // timer variables
int c = 0;

void setup()
{
    // initialize MPU6050
    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);   // begin transmission to MPU6050
    Wire.write(0x6B);                       // PWR_MGMT_1 register
    Wire.write(0x00);                       // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    // initialize start button
    pinMode(buttonPin, INPUT);

    // initialize motor pins
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    // initialize motor PWM pins
    pinMode(motor1PWMPin, OUTPUT);
    pinMode(motor2PWMPin, OUTPUT);

    // initialize ultrasonic distance sensor pins
    pinMode(sensorTrigPin, OUTPUT);
    pinMode(sensorEchoPin, INPUT);
}

void loop()
{

}

void moveDelay(int delay)
{
    int deltaAngle = round(targetAngle - angle);
    
    // move forward
    motorForward();

    

    // stop
    motorStop();
}

/**
 * Calculates the average error of the MPU. Vehicle must be stationary.
*/
void calculateError()
{
    // Read accelerometer values 200 times
    for (c = 0; c < 200; c++) {
        readAcceleration();
        // Sum all readings
        AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
        AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
        c++;
    }
    // Divide the sum by 200 to get the averaged error value, since expected value of reading is zero
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;

    // Read gyro values 200 times
    for (c = 0; c < 200; c++) {
        readGyro();
        // Sum all readings
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
        c++;
    }
    // Divide the sum by 200 to get the averaged error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
}

void motorForward()
{
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void motorBackward()
{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void motorTurnLeft()
{
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void motorTurnRight()
{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void motorStop()
{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
}

/**
 * Updates linear acceleration values
*/
void readAcceleration() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value
}

/**
 * Updates angular velocity values
*/
void readGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}