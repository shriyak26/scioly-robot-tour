
// Packages
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = motorShield.getMotor(1); 
Adafruit_DCMotor *motorR = motorShield.getMotor(2); 


#define WAIT            0       // wait (not actually used)
#define START           1       // start button pressed (also not used)
#define FORWARD         10      // move forward
#define BACKWARD        11      // move backward
#define FORWARD_TIL     12      // move forward until distance
#define BACKWARD_TIL    13      // move backward until distance
#define RIGHT           20      // turn right
#define LEFT            21      // turn left
#define STOP            22      // end of program
#define ABORT           99      // abort program



bool status;
int statusAngle;


// command queues
QList<int> commandQueue;
QList<unsigned long> paramQueue;

void setup()
{   
  motorShield.begin();
  motorL->run(BRAKE);
  motorR->run(BRAKE);
  motorL->setSpeed(200);
  motorR->setSpeed(200);
  // delay(5000);

  // initialize signal pin
  //...

  // initialize start button
  //...

  // initialize ultrasonic distance sensor pins
  //...

  //blink when ready to start after calibration?


  status = false;
  target angle = 0;


  //--------------------------------
  //add commands HERE

  //add must be first!
  add(START);



  // must be last!
  add(STOP);



}

void loop()
{
  checkButton();


  if (status)
    {
        int currentCmd = commandQueue.front();
        unsigned long currentParam = paramQueue.front();
        
        switch (currentCmd)
        {
            case WAIT:
                // wait
                break;
            case START:
                // start
                commandQueue.pop_front();
                paramQueue.pop_front();
                newCommand = true;
                delay(2000);
                break;
            case FORWARD:
                // move forward
                if (newCommand)
                {
                    
                    motorForward();
                    newCommand = false;
                    startTime = millis();
                }
                else
                {
                    if (millis() < (startTime + currentParam))
                    {
                        courseCorrect();
                    }
                    else
                    {
                        motorStop();
                        commandQueue.pop_front();
                        paramQueue.pop_front();
                        newCommand = true;
                        delay(2000);
                    }
                }
                break;
            case BACKWARD:
                // move backward
                if (newCommand)
                {
                    motorBackward();
                    newCommand = false;
                    startTime = millis();
                }
                else
                {
                    if (millis() < (startTime + currentParam))
                    {
                        courseCorrectBackward();
                    }
                    else
                    {
                        motorStop();
                        commandQueue.pop_front();
                        paramQueue.pop_front();
                        newCommand = true;
                        delay(2000);
                    }
                }
                break;
            case FORWARD_TIL:
                // move forward
                if (newCommand)
                {
                    motorForward();
                    newCommand = false;
                }
                else
                {
                    if (getDistance() - currentParam > DISTANCE_TOLERANCE)
                    {
                        courseCorrect();
                    }
                    else
                    {
                        motorStop();
                        commandQueue.pop_front();
                        paramQueue.pop_front();
                        newCommand = true;
                        delay(2000);
                    }
                }
                break;
            case BACKWARD_TIL:
                // move backward
                if (newCommand)
                {
                    motorBackward();
                    newCommand = false;
                }
                else
                {
                    if (getDistance() - currentParam < -DISTANCE_TOLERANCE)
                    {
                        courseCorrectBackward();
                    }
                    else
                    {
                        motorStop();
                        commandQueue.pop_front();
                        paramQueue.pop_front();
                        newCommand = true;
                        delay(2000);
                    }
                }
                break;
            case RIGHT:
                // turn right
                if (newCommand)
                {
                    speedLeft = DEFAULT_SPEED_TURN;
                    speedRight = DEFAULT_SPEED_TURN;
                    updateSpeed();
                    
                    targetAngle += 90;

                    turnRight();
                    newCommand = false;
                }
                else
                {
                    Serial.println(MPU.getAngleZ());
                    if (MPU.getAngleZ() - targetAngle + TURN_CALIBRATE > ANGLE_TOLERANCE)
                    {
                        turnLeft();
                        MPU.update();
                    }
                    else if (MPU.getAngleZ() - targetAngle + TURN_CALIBRATE < -ANGLE_TOLERANCE)
                    {
                        turnRight();
                        MPU.update();
                    }
                    else
                    {
                        motorStop();
                        commandQueue.pop_front();
                        paramQueue.pop_front();
                        newCommand = true;
                        targetAngle -= ANGLE_CALIBRATE;
                        delay(2000);
                    }
                }
                break;
            case LEFT:
                // turn left
                if (newCommand)
                {
                    speedLeft = DEFAULT_SPEED_TURN;
                    speedRight = DEFAULT_SPEED_TURN;
                    updateSpeed();
                    
                    targetAngle -= 90;

                    turnLeft();
                    newCommand = false;
                }
                else
                {
                    if (MPU.getAngleZ() - targetAngle - TURN_CALIBRATE > ANGLE_TOLERANCE)
                    {
                        turnLeft();
                        MPU.update();
                    }
                    else if (MPU.getAngleZ() - targetAngle - TURN_CALIBRATE < -ANGLE_TOLERANCE)
                    {
                        turnRight();
                        MPU.update();
                    }
                    else
                    {
                        motorStop();
                        commandQueue.pop_front();
                        paramQueue.pop_front();
                        newCommand = true;
                        targetAngle += ANGLE_CALIBRATE;
                        delay(2000);
                    }
                }
                break;
            case STOP:
                // stop
                motorStop();
                break;
        }
    }

}



void add(int cmd)
{
    add(cmd, 0);
}

void add(int cmd, float param)
{
    commandQueue.push_back(cmd);
    paramQueue.push_back(param);
}

void checkButton()
{
    if (digitalRead(buttonPin) == LOW)
    {
        status = !status;
    }
    
}

void courseCorrect()
{
    MPU.update();
    if (MPU.getAngleZ() - targetAngle > 0)
        {
            speedLeft = DEFAULT_SPEED_STRAIGHT;
            speedRight = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
        }
        else
        {
            speedLeft = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
            speedRight = DEFAULT_SPEED_STRAIGHT;
        }

    updateSpeed();
}

void courseCorrectBackward()
{
    MPU.update();
    if (MPU.getAngleZ() - targetAngle < 0)
    {
        speedLeft = DEFAULT_SPEED_STRAIGHT;
        speedRight = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
    }
    else
    {
        speedLeft = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
        speedRight = DEFAULT_SPEED_STRAIGHT;
    }
    updateSpeed();
}

void updateSpeed()
{
    
}

void motorForward()
{
    
}

void motorBackward()
{
    
}


void turnRight()
{
    
}

void turnLeft()
{
    
}

void motorStop()
{
    
}





/**
 * Calculates distance to object in front using TWO front ultrasonic sensor
 * @return distance in mm
*/
float getDistance()
{
    //lol implement it
}














// Motor testing
/*
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  delay(5000);
  motorL->setSpeed(0);
  motorR->setSpeed(0);
*/