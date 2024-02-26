/*
  Pre-Run Checklist
  1. New batteries
  2. Cables
  
  Timing Reference Sheet
  Straight:
  150- 2 seconds

  Turn: 1 second
*/

// Packages
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <ArduinoQueue.h>
#include "ICM_20948.h"
#include <Wire.h>

int targetTime = 50; // in seconds
int DIS1 = 7000;     // 3700
int DIS2 = 7400;
int TIL1 = 120;
int TURN90 = 4650;
int turnTime = 4800; // do not change unless u need to ig
int startDelay = 2000;
int movementDelay = 500;

// temporary fix
int rightTrim = 0;
int turnCount = 0;

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

// IMU
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

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
int targetAngle = 0, orientation = 0;
int defaultSpeedL = 155, defaultSpeedR = 150, defaultSpeedTurnL = 50,
    defaultSpeedTurnR = 50;
int speedL, speedR;
int turnSpeedLimit = 100;
int distanceTolerance = 1;
int encoderTolerance = 1;
int directionTolerance = 1;
volatile int lastEncodedL = 0, lastEncodedR = 0;
volatile long encoderValueL = 0, encoderValueR = 0;
long encoderValueLAbs = 0, encoderValueRAbs = 0;
int startEncoderValue = 0;
long startEncoderValueL = 0;
long startEncoderValueR = 0;
bool motorLStopped, motorRStopped;
bool accelerate;
bool deccelerate;
int accelerateRate = 1;
int targetSpeedL = defaultSpeedL;
int targetSpeedR = defaultSpeedR;
int startSpeedL = 80;
int startSpeedR = 80;

// for PID control
long previousTimeL = 0;
float ePreviousL = 0;
float eIntegralL = 0;
long previousTimeR = 0;
float ePreviousR = 0;
float eIntegralR = 0;

// PID gains
float kpL = 2.0;
float kdL = 0.05;
float kiL = 0.0;

float kpR = 3.0;
float kdR = 0.05;
float kiR = 0.01;

// timing test
int startTime;
int endTime;

// command queues
ArduinoQueue<int> commandQueue(50);
ArduinoQueue<int> paramQueue(50);

void setup() {
  motorShield.begin();
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

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize IMU communication
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    }
    else {
      initialized = true;
    }
  }

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (!success) {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  // initialize start button
  pinMode(buttonPin, INPUT_PULLUP);

  // initialize ultrasonic distance sensor pins
  pinMode(sensorLTrigPin, OUTPUT);
  pinMode(sensorLEchoPin, INPUT);

  // blink when ready to start after calibration?

  motorLStopped = false;
  motorRStopped = false;

  newCommand = true;

  status = false;

  //--------------------------------
  // add commands HERE
  //--------------------------------

  // must be first!
  add(START);

  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);

  // add(FDT,TIL1);
  /*
  add(FD, DIS1);
  add(RTE, TURN90);
  add(RTE, TURN90);
  add(FD, DIS1);
  add(LTE, TURN90);
  add(LTE, TURN90);
  /**/

  // must be last!
  add(STOP);

  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  Serial.println(targetAngle);
}

void loop() {
  checkButton();

  /**/
  if (status) {
    int currentCmd = commandQueue.getHead();
    int currentParam = paramQueue.getHead();

    // 
    updateIMU();

    switch (currentCmd) {
    case WAIT:
      // wait
      break;
    case START:
      // start
      Serial.println("hi");
      commandQueue.dequeue();
      paramQueue.dequeue();
      newCommand = true;
      /*
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      /**/
      delay(startDelay);
      /**/
      break;
    case FD:
      // move forward
      if (newCommand) {
        speedL = targetSpeedL;
        speedR = targetSpeedR;
        updateSpeed();
        startEncoderValue = encoderValueL;
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorForward();
        newCommand = false;
        // startTime = millis();
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
        speedL = defaultSpeedL;
        speedR = defaultSpeedR;

        updateSpeed();
        startEncoderValue = encoderValueL;
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorBackward();
        newCommand = false;
      } else {
        if (encoderValueL > (startEncoderValue + currentParam)) {
          // courseCorrectBackward();
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
        speedL = defaultSpeedL;
        speedR = defaultSpeedR;
        updateSpeed();
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorForward();
        newCommand = false;
      } else {
        // for now just use one sensor
        float distance = getDistance();
        if (distance - currentParam > distanceTolerance) {
          // courseCorrect();
          Serial.println(distance);
        } else {
          Serial.println(distance);
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
        speedL = defaultSpeedL;
        speedR = defaultSpeedR;
        updateSpeed();
        startEncoderValueL = encoderValueL;
        startEncoderValueR = encoderValueR;
        motorBackward();
        newCommand = false;
      } else {
        if (getDistance() - currentParam < -distanceTolerance) {
          // courseCorrectBackward();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
      /**/
      case RT:
        // turn right
        if (newCommand) {
          // Serial.println(targetAngle);
          targetAngle -= 90;
          // Serial.println(targetAngle);
          /**/
          if (targetAngle == -270) {
            targetAngle = 90;
          }
          /**/
          motorL->setSpeed(turnSpeedLimit);
          motorL->run(FORWARD);
          newCommand = false;
          // Serial.println(targetAngle);
        } else {
          if ((targetAngle == 90 && orientation < 0) || (targetAngle != -180 && fabs(orientation - targetAngle) > directionTolerance) || (targetAngle == -180 && orientation < 0)) {
            pidPositionMatchRight(encoderValueRAbs - fabs(encoderValueL - encoderValueLAbs));
            /*
            Serial.print(targetAngle);
            Serial.print(", ");
            Serial.print(orientation - targetAngle);
            Serial.print(", ");
            Serial.println(orientation);
            /**/
          } else {
            motorStop();
            commandQueue.dequeue();
            paramQueue.dequeue();
            encoderValueLAbs = encoderValueL;
            encoderValueRAbs = encoderValueR;
            /*
            if (targetAngle == -179) {
              targetAngle = -180;
            }
            /**/
            newCommand = true;
            delay(movementDelay);
          }
        }
        break;
      case LT:
        // turn left
        if (newCommand) {
          // Serial.println(targetAngle);
          targetAngle += 90;
          // Serial.println(targetAngle);
          /**/
          if (targetAngle == 270) {
            targetAngle = -90;
          }
          /**/
          motorL->setSpeed(turnSpeedLimit);
          motorL->run(BACKWARD);
          newCommand = false;
          // Serial.println(targetAngle);
        } else {
          if ((targetAngle == -90 && orientation > 0) || (targetAngle != 180 && fabs(orientation - targetAngle) > directionTolerance) || (targetAngle == 180 && orientation > 0)) {
            pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
            /*
            Serial.print(targetAngle);
            Serial.print(", ");
            Serial.print(orientation - targetAngle);
            Serial.print(", ");
            Serial.println(orientation);
            /**/
          } else {
            motorStop();
            commandQueue.dequeue();
            paramQueue.dequeue();
            encoderValueLAbs = encoderValueL;
            encoderValueRAbs = encoderValueR;
            newCommand = true;
            delay(movementDelay);
          }
        }
        break;
      /**/
    case RTE:
      // turn right with encoders
      if (newCommand) {
        newCommand = false;
      } else {
        if (fabs(encoderValueL - (encoderValueLAbs - currentParam)) > encoderTolerance || fabs(encoderValueR - (encoderValueRAbs - currentParam)) > encoderTolerance) {
          pidPositionMatchLeft(encoderValueLAbs - currentParam);
          pidPositionMatchRight(encoderValueRAbs - fabs(encoderValueL - encoderValueLAbs));
        }
        else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs-= currentParam;
          encoderValueRAbs-= currentParam;
          newCommand = true;
          Serial.println(encoderValueR);
          delay(movementDelay);
        }
      }
      break;
    case LTE:
      // turn left with encoders
      if (newCommand) {
        newCommand = false;
      } else {
        if (fabs(encoderValueL - (encoderValueLAbs + currentParam)) > encoderTolerance || fabs(encoderValueR - (encoderValueRAbs + currentParam)) > encoderTolerance) {
          pidPositionMatchLeft(encoderValueLAbs + currentParam);
          pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
        }
        else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs+= currentParam;
          encoderValueRAbs+= currentParam;
          newCommand = true;
          Serial.println(encoderValueR);
          delay(movementDelay);
        }
      }
      break;
    case STOP:
      // stop
      motorStop();
      break;
    }
  } /**/
}

void add(int cmd) { add(cmd, 0); }

void add(int cmd, int param) {
  commandQueue.enqueue(cmd);
  paramQueue.enqueue(param);
}

void checkButton() {
  if (digitalRead(buttonPin) == LOW) {
    status = !status;
  }
}

void updateIMU() {
  // Read any DMP data waiting in the FIFO
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) { // Was valid data available?
    if ((data.header & DMP_header_bitmap_Quat6) > 0) { // We have asked for GRV data so we should receive Quat6
      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      orientation = (int) yaw;
    }
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

void pidPositionMatchLeft(int target) {
  float u = pidPositionMatchCalculateLeft(target, kpL, kdL, kiL);
  float speed = fabs(u);

  if (speed > turnSpeedLimit) {
    speed = turnSpeedLimit;
  }
  else if (speed < 20) {
    speed = 20;
  }

  motorL->setSpeed(speed);

  if (u < 0) {
    motorL->run(BACKWARD);
  }
  else {
    motorL->run(FORWARD);
  }
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

void pidPositionMatchRight(int target) {
  float u = pidPositionMatchCalculateRight(target, kpR, kdR, kiR);
  float speed = fabs(u);

  if (speed > 255) {
    speed = 255;
  }
  else if (speed < 20) {
    speed = 20;
  }

  motorR->setSpeed(speed);

  if (u > 0) {
    motorR->run(BACKWARD);
  }
  else {
    motorR->run(FORWARD);
  }
}

//--------------------------------
// courseCorrect stuff:
// left encoder decr with forward, incr backwards
// right encoder incr with forward, decr backwards
//--------------------------------

// course correct forward
// currently for encoders, compare encoders see if match
/*
void courseCorrect() {
  // drifting right
  if (startEncoderValueL - encoderValueL > encoderValueR - startEncoderValueR) {
    // speed up right motor
    speedR = defaultSpeed + ((startEncoderValueL - encoderValueL) -
                             (encoderValueR - startEncoderValueR)) /
                                100;
    updateSpeed();
  }
  // drifting left
  else if (startEncoderValueL - encoderValueL <
           encoderValueR - startEncoderValueR) {
    // slow down right motor
    speedR = defaultSpeed - ((encoderValueR - startEncoderValueR) -
                             (startEncoderValueL - encoderValueL)) /
                                100;
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
    speedR = defaultSpeed - ((encoderValueR - startEncoderValueR) -
                             (startEncoderValueL - encoderValueL));
    updateSpeed();
  }
}
/**/

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
  motorL->run(FORWARD);
  motorR->run(FORWARD);
}

void motorBackward() {
  motorL->run(BACKWARD);
  motorR->run(BACKWARD);
}

void motorRight() {
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
  motorLStopped = false;
  motorRStopped = false;
}

void motorLeft() {
  motorL->run(BACKWARD);
  motorR->run(FORWARD);
  motorLStopped = false;
  motorRStopped = false;
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
  float distance = duration * 0.343 / 2;
  // 0.343: speed of sound in mm/microsecond
  // 2: round trip of sound
  return distance;
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

/*
case FD:
  // move forward
  if (newCommand) {
    speedL = startSpeedL;
    speedR = startSpeedR;
    accelerate = true;
    updateSpeed();
    startEncoderValue = encoderValueL;
    startEncoderValueL = encoderValueL;
    startEncoderValueR = encoderValueR;
    motorForward();
    newCommand = false;
  } else {
    if (encoderValueL > (startEncoderValue - currentParam)) {
      // courseCorrect();
      if (accelerate) {
        speedL += accelerateRate;
        speedR += accelerateRate;
        updateSpeed();
        if (speedL >= targetSpeedL) {
          accelerate = false;
        }
      } else if (encoderValueL - encoderTolerance >= (startEncoderValue - currentParam)) {
        deccelerate = true;
      }
      if (deccelerate) {
        speedL -= accelerateRate;
        speedR -= accelerateRate;
        updateSpeed();
        if (speedL <= startSpeedL) {
          deccelerate = false;
        }
      }
    } else {
      motorStop();
      commandQueue.dequeue();
      paramQueue.dequeue();
      newCommand = true;
      delay(movementDelay);
    }
  }
  break;
*/