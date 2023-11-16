#define STRAIGHT_DELAY  3000
#define TURN_DELAY      600



#include <QList.h>
#include <math.h>

using namespace std;

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

// pressing button changes the status, true = program runs
bool status = false;

QList<int> cmdQueue;
QList<float> paramQueue;

// int references for each command
#define START_WAIT  0
#define FORWARD     10 
#define TURN_LEFT   11
#define TURN_RIGHT  13
#define MOVE_TIL    14
#define BACKWARD    15
#define TEST        16
#define STOP        1

// PWM values for straight and turning, 0 = min, 255 = max
#define START_PWM       50
#define STRAIGHT_PWM    100
#define TURN_PWM        65

#define ACCEL_DELAY     10000
#define ACCEL_DELAY_2   1000
#define ACCEL_STOP      50

// placeholder values, we need to mess with them to get accurate distances
// NOT IN USE
// milimeters per milisecond
float linearVelocity = 0.5;
// degrees per milisecond
float rotationalVelocity = 5;

void setup()
{
    pinMode(buttonPin, INPUT_PULLUP);
    
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    pinMode(motor1PWMPin, OUTPUT);
    pinMode(motor2PWMPin, OUTPUT);

    // motor speed control: 0 = min, 255 = max
    analogWrite(motor1PWMPin, 100);
    analogWrite(motor2PWMPin, 100);

    pinMode(sensorTrigPin, OUTPUT);
    pinMode(sensorEchoPin, INPUT);

    cmdQueue.clear();
    paramQueue.clear();

    // must be first!
    add(START_WAIT);

    // BEGIN SETUP
    // commands go here
    // add(FORWARD);
    add(TURN_RIGHT);

    // must be last!
    add(STOP);
}

void loop()
{
    checkButton();

    if(status)
    {
        cmdQueue.pop_front();
        paramQueue.pop_front();
    }

    int currentCmd = cmdQueue.front();
    float currentParam = paramQueue.front();

    switch (currentCmd)
    {
        case START_WAIT :
            break;
        case FORWARD :
            moveForward(currentParam);
            break;
        case TURN_LEFT :
            turnLeft(currentParam);
            break;
        case TURN_RIGHT :
            turnRight(currentParam);
            break;
        case MOVE_TIL :
            moveTil(currentParam);
            break;
        case BACKWARD :
            moveBackward(currentParam);
            break;
        case TEST :
            // distanceTester();
            break;
        case STOP :
            status = false;
            setup();
        default :
            status = false;
    }
}

void add(int cmd)
{
    cmdQueue.push_back(cmd);
    paramQueue.push_back(0);
}

void add(int cmd, float param)
{
    cmdQueue.push_back(cmd);
    paramQueue.push_back(param);
}

// checks if button has been pressed
void checkButton()
{
    // if pressed, inverts status variable
    if(digitalRead(buttonPin) == LOW)
    {
        status = !status;
    }
}

void setSpeed(int speed)
{
    analogWrite(motor1PWMPin, speed);
    analogWrite(motor2PWMPin, speed);
}
// moves forward for distance (mm)
void moveForward(float distance)
{
    int speed = STRAIGHT_PWM;
    
    motorForward();
    
    // float time = distance / linearVelocity;
    delay(STRAIGHT_DELAY);

    // deceleration/braking algorithm
    while(speed > ACCEL_STOP)
    {
        delayMicroseconds(ACCEL_DELAY);
        setSpeed(--speed);
    }

    motorStop();
}

// moves backward for distance (mm)
void moveBackward(float distance)
{
    int speed = STRAIGHT_PWM;
    
    motorBackward();
    
    // float time = distance / linearVelocity;
    delay(STRAIGHT_DELAY);

    // deceleration/braking algorithm
    while(speed > ACCEL_STOP)
    {
        delayMicroseconds(ACCEL_DELAY);
        setSpeed(--speed);
    }

    motorStop();
}

// turns left for degrees
void turnLeft(float degrees)
{
    int speed = TURN_PWM;
    
    setSpeed(speed);

    // float time = degrees / rotationalVelocity;
    motorLeftForward();
    motorRightBackward();
    delay(TURN_DELAY);

    // disabled because deceleration takes too long
    /*
    while(speed > ACCEL_STOP)
    {
        delayMicroseconds(ACCEL_DELAY_2);
        setSpeed(--speed);
    }
    */

    motorStop();
}

// turns right for degrees
void turnRight(float degrees)
{
    int speed = TURN_PWM;
    
    setSpeed(speed);
    
    setSpeed(TURN_PWM);
    // float time = degrees / rotationalVelocity;
    motorLeftBackward();
    motorRightForward();
    delay(TURN_DELAY);

    /*
    while(speed > ACCEL_STOP)
    {
        delayMicroseconds(ACCEL_DELAY_2);
        setSpeed(--speed);
    }
    */

    motorStop();
}

// turns both motors on forward
void motorForward()
{
    motorLeftForward();
    motorRightForward();
}

// turns both motors on backward
void motorBackward()
{
    motorLeftBackward();
    motorRightBackward();
}

void motorRightForward()
{
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
}

void motorLeftForward()
{
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void motorRightBackward()
{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
}

void motorLeftBackward()
{
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

// turns both motors off
void motorStop()
{
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
}

// move until distance from wall (mm)
void moveTil(float distance)
{
    int speed = STRAIGHT_PWM;
    setSpeed(speed);
    
    // start motors in correct direction
    if(getDistance() > distance)
    {
        motorForward();
    }
    else
    {
        motorBackward();
    }
    
    // stop motors when within margin of error (2 mm)
    // to allow for deceleration
    while((fabs(getDistance() - distance) > 2))
    {
        delayMicroseconds(1);
    }

    while(speed > ACCEL_STOP)
    {
        delayMicroseconds(ACCEL_DELAY);
        setSpeed(--speed);
    }

    motorStop();
}

float getDistance()
{
    // ensure trigger is low
    digitalWrite(sensorTrigPin, LOW);
    delayMicroseconds(2);

    // send pulse
    digitalWrite(sensorTrigPin, HIGH);
    delayMicroseconds(10);

    // turn off pulse
    digitalWrite(sensorTrigPin, LOW);

    // get duration of return
    float duration = pulseIn(sensorEchoPin, HIGH);

    // convert to distance in mm (speed of sound = 343 m/s)
    // duration in microseconds * mm per microsecond / 2 (round trip)
    return duration * 0.343 / 2;
}

