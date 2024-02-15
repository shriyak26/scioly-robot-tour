#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <ArduinoQueue.h>
#include <Wire.h>

// Define the pins for the encoder
#define encoderLPinA 6
#define encoderLPinB 7
#define encoderRPinA 8
#define encoderRPinB 9

// Variables to store the previous state of the encoder pins
volatile int previousEncoderLPinA = LOW;
volatile int previousEncoderLPinB = LOW;

void setup() {
    // Set the encoder pins as inputs
    pinMode(encoderLPinA, INPUT);
    pinMode(encoderLPinB, INPUT);

    // Enable internal pull-up resistors for the encoder pins
    digitalWrite(encoderLPinA, HIGH);
    digitalWrite(encoderLPinB, HIGH);

    attachInterrupt(digitalPinToInterrupt(encoderLPinA), update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderLPinB), update, CHANGE);

    // Initialize the serial communication
    Serial.begin(9600);
}

void loop() {

}

void update() {
  int currentEncoderLPinA = digitalRead(encoderLPinA);
    int currentEncoderLPinB = digitalRead(encoderLPinB);

    if (currentEncoderLPinA != previousEncoderLPinA || currentEncoderLPinB != previousEncoderLPinB) {
        // Print the current state of the encoder pins
        Serial.print("Encoder L Pin A: ");
        Serial.print(currentEncoderLPinA);
        Serial.print(" | Encoder L Pin B: ");
        Serial.println(currentEncoderLPinB);

        // Update the previous state of the encoder pins
        previousEncoderLPinA = currentEncoderLPinA;
        previousEncoderLPinB = currentEncoderLPinB;
}
}