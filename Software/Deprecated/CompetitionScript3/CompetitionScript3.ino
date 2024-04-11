#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <QList.h>
#include <math.h>

// default: 1450
const unsigned long ONE_UNIT_DELAY = 1450;

// untested
const unsigned long TWO_UNIT_DELAY = 2700;
const unsigned long HALF_UNIT_DELAY = 750;
const unsigned long SPECIAL_DELAY = 400;

// default: 200
const int ONE_UNIT_DISTANCE = 200;
// default: 650
const int TWO_UNIT_DISTANCE = 650;

const int DEFAULT_SPEED_STRAIGHT = 100;
const int DEFAULT_SPEED_TURN = 80;

const int CORRECTION_MULTIPLIER = 2;
// Changes what angle robot moves forward after turning. Higher value = less turn. default: 20
const int ANGLE_CALIBRATE = 20;
// Changes how much the robot turns per turn. Higher value = less turn. default: 15
const int TURN_CALIBRATE = 15;

const int DISTANCE_TOLERANCE = 10;
const int ANGLE_TOLERANCE = 5;

int speedLeft, speedRight;
int targetAngle;
unsigned long startTime;
bool status;
bool newCommand;

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

// command presets
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

// command queues
QList<int> commandQueue;
QList<unsigned long> paramQueue;

MPU6050 MPU(Wire);

void setup()
{
    // for troubleshooting
    // Serial.begin(9600);
    
    // initialize signal pin
    pinMode(LED_BUILTIN, OUTPUT);
    
    // initialize start button
    pinMode(buttonPin, INPUT_PULLUP);

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

    delay(500);

    Wire.begin();
    MPU.begin();
    MPU.calcGyroOffsets();

    // delay(10000);

    /**/
    // blink LED 3 times to indicate calibration complete
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(300);
        digitalWrite(LED_BUILTIN, LOW);
        delay(300);
    }
    /**/

    status = false;
    newCommand = true;
    targetAngle = 0;

    // must be first!
    add(START);

    // add commands here
    // add(BACKWARD_TIL,TWO_UNIT_DISTANCE);
    /**/
    // add(FORWARD,ONE_UNIT_DELAY);
    // add(FORWARD,ONE_UNIT_DELAY);
    // add(RIGHT);
    add(LEFT);
    add(FORWARD,ONE_UNIT_DELAY);
    /*
    add(FORWARD,ONE_UNIT_DELAY);
    add(LEFT);
    add(LEFT);
    add(FORWARD,ONE_UNIT_DELAY);
    add(FORWARD,ONE_UNIT_DELAY);
    add(BACKWARD,ONE_UNIT_DELAY);
    add(BACKWARD,ONE_UNIT_DELAY);
    /**/
    /*
    add(FORWARD_TIL,ONE_UNIT_DISTANCE);
    add(LEFT);
    add(FORWARD,ONE_UNIT_DELAY);
    add(RIGHT);
    add(FORWARD_TIL,ONE_UNIT_DISTANCE);
    add(RIGHT);
    add(FORWARD_TIL,ONE_UNIT_DISTANCE);
    add(LEFT); 
    add(FORWARD_TIL,ONE_UNIT_DISTANCE);
    add(LEFT);
    add(FORWARD,ONE_UNIT_DELAY);
    add(LEFT);
    add(BACKWARD_TIL,TWO_UNIT_DISTANCE); 
    add(LEFT);
    add(FORWARD_TIL,ONE_UNIT_DISTANCE + 30);
    /**/

    // must be last!
    add(STOP);

    /*
    motorForward();
    delay(1000);
    motorStop();
    */

}

void loop()
{
    checkButton();

    MPU.update();

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
    
    /*
    if (targetAngle == 0)
    {
        if (MPU.getAngleZ() > 0)
        {
            speedLeft = DEFAULT_SPEED_STRAIGHT;
            speedRight = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
        }
        else
        {
            speedLeft = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
            speedRight = DEFAULT_SPEED_STRAIGHT;
        }
    }
    else if (targetAngle == 90 || targetAngle == -90)
    {
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
    }
    else
    {
        if (MPU.getAngleZ() < 0)
        {
            speedLeft = DEFAULT_SPEED_STRAIGHT;
            speedRight = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
        }
        else
        {
            speedLeft = DEFAULT_SPEED_STRAIGHT + fabs(MPU.getAngleZ() - targetAngle) * CORRECTION_MULTIPLIER;
            speedRight = DEFAULT_SPEED_STRAIGHT;
        }
    }
    */

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
    analogWrite(motor1PWMPin, speedLeft);
    analogWrite(motor2PWMPin, speedRight);
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


void turnRight()
{
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void turnLeft()
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
 * Calculates distance to object in front using front ultrasonic sensor
 * @return distance in mm
*/
float getDistance()
{
    digitalWrite(sensorTrigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(sensorTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensorTrigPin, LOW);
    
    float duration = pulseIn(sensorEchoPin, HIGH);

    // 0.343: speed of sound in mm/microsecond
    // 2: round trip of sound
    return duration * 0.343 / 2;;
}