// Packages
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <ArduinoQueue.h>
#include <Wire.h>

int targetTime = 50; // in seconds
int DIS1 = 3700; // 3700
int DIS2 = 7400;
int TURN90 = 750;
int turnTime = 5000; // do not change unless u need to ig
int startDelay = 2000;
int movementDelay = 100;

// Pins
#define sensorRTrigPin 2
#define sensorREchoPin 3
#define sensorLTrigPin 4
#define sensorLEchoPin 5
#define encoderLPinA 6
#define encoderLPinB 7
#define encoderRPinA 8
#define encoderRPinB 9
#define buttonPin 10

// Motor
Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = motorShield.getMotor(1);
Adafruit_DCMotor *motorR = motorShield.getMotor(2);

#define WAIT 0   // wait (not actually used)
#define START 1  // start button pressed (also not used)
#define FD 10    // move forward
#define BD 11    // move backward
#define FDT 12   // move forward until distance
#define BDT 13   // move backward until distance
#define RT 20    // turn right
#define LT 21    // turn left
#define RTE 35   // turn right but with encoders
#define LTE 36   // turn left but with encoders
#define STOP 22  // end of program
#define ABORT 99 // abort program

bool status, newCommand;
int targetAngle;
int defaultSpeed = 250, defaultSpeedTurn = 150;
int speedL, speedR;
int distanceTolerance = 10;
volatile int lastEncodedL = 0, lastEncodedR = 0;
volatile long encoderValueL = 0, encoderValueR = 0;
int startEncoderValue = 0;
int startEncoderValueL = 0;
int startEncoderValueR = 0;
bool motorLStopped, motorRStopped;

// command queues
ArduinoQueue<int> commandQueue(50);
ArduinoQueue<int> paramQueue(50);

void setup() {
  motorShield.begin();
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

  // initialize start button
  pinMode(buttonPin, INPUT_PULLUP);

  // initialize ultrasonic distance sensor pins
  //...

  // blink when ready to start after calibration?

  motorLStopped = false;
  motorRStopped = false;

  newCommand = true;

  status = false;
  targetAngle = 0;

  //--------------------------------
  // add commands HERE
  //--------------------------------

  // must be first!
  add(START);

  add(FD,DIS1);
  // add(RTE,TURN90);

  // must be last!
  add(STOP);

  pinMode(LED_BUILTIN, OUTPUT);
  
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void loop() {
  checkButton();

  /**/
  if (status) {
    int currentCmd = commandQueue.getHead();
    int currentParam = paramQueue.getHead();

    switch (currentCmd) {
    case WAIT:
      // wait
      break;
    case START:
      // start
      commandQueue.dequeue();
      paramQueue.dequeue();
      newCommand = true;
      /*
      delay(100);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN,LOW);
      /**/
      delay(startDelay);
      /**/
      break;
    case FD:
      // move forward
      if (newCommand) {
        speedL = defaultSpeed;
        speedR = defaultSpeed;
        updateSpeed();
        startEncoderValue = encoderValueL;
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorForward();
        newCommand = false;
      } else {
        if (encoderValueL > (startEncoderValue - currentParam)) {
          // courseCorrect();
          delay(1);
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case BD:
      // move backward
      if (newCommand) {
        speedL = defaultSpeed;
        speedR = defaultSpeed;
        updateSpeed();
        startEncoderValue = encoderValueL;
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorBackward();
        newCommand = false;
      } else {
        if (encoderValueL > (startEncoderValue + currentParam)) {
          courseCorrectBackward();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case FDT:
      // move forward
      if (newCommand) {
        speedL = defaultSpeed;
        speedR = defaultSpeed;
        updateSpeed();
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorForward();
        newCommand = false;
      } else {
        // for now just use one sensor
        if (getDistance() - currentParam > distanceTolerance) {
          courseCorrect();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case BDT:
      // move backward
      if (newCommand) {
        speedL = defaultSpeed;
        speedR = defaultSpeed;
        updateSpeed();
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorBackward();
        newCommand = false;
      } else {
        if (getDistance() - currentParam < -distanceTolerance) {
          courseCorrectBackward();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    /*
    case RT:
      // turn right
      if (newCommand) {
        speedL = defaultSpeedTurn;
        speedRight = defaultSpeedTurn;
        updateSpeed();

        targetAngle += 90;

        turnRight();
        newCommand = false;
      } else {
        // Serial.println(MPU.getAngleZ());
        if (MPU.getAngleZ() - targetAngle + TURN_CALIBRATE > ANGLE_TOLERANCE) {
          turnLeft();
          MPU.update();
        } else if (MPU.getAngleZ() - targetAngle + TURN_CALIBRATE <
                   -ANGLE_TOLERANCE) {
          turnRight();
          MPU.update();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          targetAngle -= ANGLE_CALIBRATE;
          delay(movementDelay);
        }
      }
      break;
    case LT:
      // turn left
      if (newCommand) {
        speedL = defaultSpeedTurn;
        speedRight = defaultSpeedTurn;
        updateSpeed();

        targetAngle -= 90;

        turnLeft();
        newCommand = false;
      } else {
        if (MPU.getAngleZ() - targetAngle - TURN_CALIBRATE > ANGLE_TOLERANCE) {
          turnLeft();
          MPU.update();
        } else if (MPU.getAngleZ() - targetAngle - TURN_CALIBRATE <
                   -ANGLE_TOLERANCE) {
          turnRight();
          MPU.update();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          targetAngle += ANGLE_CALIBRATE;
          delay(movementDelay);
        }
      }
      break; */
      /**/
    case RTE:
      // turn right with encoders
      if (newCommand) {
        speedL = defaultSpeedTurn;
        speedR = defaultSpeedTurn;
        updateSpeed();
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorRight();
        newCommand = false;
      } else {

        if (encoderValueL <= (startEncoderValue - currentParam)) {
          motorStopL();
          motorLStopped = true;
        }
        if (encoderValueR <= (startEncoderValue - currentParam)) {
          motorStopR();
          motorRStopped = true;
        }
        if (motorLStopped && motorRStopped) {
          commandQueue.dequeue();
          paramQueue.dequeue();
        motorLStopped = false;
        motorRStopped = false;
        newCommand = true;
        delay(movementDelay);
        }
      }
    break;
    case LTE:
      // turn left with encoders
      if (newCommand) {
        speedL = defaultSpeedTurn;
        speedR = defaultSpeedTurn;
        updateSpeed();
        startEncoderValue = encoderValueL;
        motorLeft();
        newCommand = false;
      } else {
        if (encoderValueL >= (startEncoderValue + currentParam)) {
          motorStopL();
          motorLStopped = true;
        }
        if (encoderValueR >= (startEncoderValue + currentParam)) {
          motorStopR();
          motorRStopped = true;
        }
        if (motorLStopped && motorRStopped) {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case STOP:
      // stop
      motorStop();
      break;
    }
  }/**/
}

void add(int cmd) { add(cmd, 0); }

void add(int cmd, int param) {
  commandQueue.enqueue(cmd);
  paramQueue.enqueue(param);
}

void checkButton() {
  if (digitalRead(buttonPin) == LOW)
  {
    status = !status;
  }
}

//--------------------------------
// courseCorrect stuff:
// left encoder decr with forward, incr backwards
// right encoder incr with forward, decr backwards
//--------------------------------

// course correct forward
// currently for encoders, compare encoders see if match
void courseCorrect() {
  // drifting right
  if (startEncoderValueL - encoderValueL > encoderValueR - startEncoderValueR) {
    // speed up right motor
    speedR = defaultSpeed + ((startEncoderValueL - encoderValueL) -
             (encoderValueR - startEncoderValueR)) / 100;
    updateSpeed();
  }
  // drifting left
  else if (startEncoderValueL - encoderValueL <
           encoderValueR - startEncoderValueR) {
    // slow down right motor
    speedR = defaultSpeed - ((encoderValueR - startEncoderValueR) - (startEncoderValueL - encoderValueL)) / 100;
    updateSpeed();
  }
}

void courseCorrectBackward() {
  // drifting right
  if (encoderValueL - startEncoderValueL > startEncoderValueR - encoderValueR) {
    // increase right speed
    speedR = defaultSpeed + (encoderValueL - startEncoderValueL) -
             (startEncoderValueR - encoderValueR);
    updateSpeed();
  }
  // drifting left
  if (encoderValueL - startEncoderValueL < startEncoderValueR - encoderValueR) {
    // decrease right speed
    speedR = defaultSpeed - ((encoderValueR - startEncoderValueR) - (startEncoderValueL - encoderValueL));
    updateSpeed();
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

void updateSpeed() {
  motorL->setSpeed(speedL);
  motorR->setSpeed(speedR);
}

void motorForward() {
  updateSpeed();
  motorL->run(FORWARD);
  motorR->run(FORWARD);
}

void motorBackward() {
  updateSpeed();
  motorL->run(BACKWARD);
  motorR->run(BACKWARD);
}

void motorRight() {
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
}

void motorLeft() {
  motorL->run(BACKWARD);
  motorR->run(FORWARD);
}

void motorStop() {
  motorL->setSpeed(0);
  motorR->setSpeed(0);
}

void motorStopL() { motorL->setSpeed(0); }
void motorStopR() { motorR->setSpeed(0); }

/**
 * Calculates distance to object in front using TWO front ultrasonic sensor
 * currently only using left sensor lol
 * @return distance in mm
 */
float getDistance() {
  digitalWrite(sensorLTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sensorLTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorLTrigPin, LOW);

  float duration = pulseIn(sensorLEchoPin, HIGH);

  // 0.343: speed of sound in mm/microsecond
  // 2: round trip of sound
  return duration * 0.343 / 2;
  ;
}

/*
void calculateSpeed() {
  // hmmm how do?
  // take into account:
  /*
  time of turning
  how much pausing lol
  acceleration/decceleration
  general movement forward backward speed
  */
// turnsNum = the number of times robot turns either right or left
/*
  turnsNum = 0;
  for (int i = 0; i < commandQueue.size(); i++) {
    if (commandQueue.get(i) == RIGHT || commandQueue.get(i) == LEFT) {
      turnsNum++;
    }
  }

  // targetTime - turnTime * turnsNum - ...
}
*/

// Motor testing
/*
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  delay(5000);
  motorL->setSpeed(0);
  motorR->setSpeed(0);
*/