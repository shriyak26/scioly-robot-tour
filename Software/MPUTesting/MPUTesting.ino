#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
}

/*
#include <TinyMPU6050.h>

#include <Arduino.h>

#include <Wire.h>

#define MPU6050_ADDR 0x68

MPU6050 mpu(Wire);

float AccX, AccY, AccZ;                                         // accelerometer values (linear acceleration)
float GyroX, GyroY, GyroZ;                                      // gyroscope values (angular velocity)

void setup() {
    mpu.Initialize();
    
    Serial.begin(9600);
    Serial.println("Calibrating!");
    mpu.Calibrate();
    Serial.println("Calibrated!");
    Wire.begin();
}

void loop() {
    mpu.Execute();
    Serial.print("Gyro X: ");
    Serial.print(mpu.GetAngX());
    Serial.print(" Gyro Y: ");
    Serial.print(mpu.GetAngY());
    Serial.print(" Gyro Z: ");
    Serial.println(mpu.GetAngZ());
    delay(500);
}
/**/