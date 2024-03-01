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
int DIS1 = 22000;     // 3700
int DIS2 = 7400;
int TIL1 = 120;
int TURN90 = 4650;
int turnTime = 4800; // do not change unless u need to ig
int startDelay = 2000;
int movementDelay = 250;

int linearSpeedLimit = 200;

// temporary fix
int rightTrim = 5;

// Pins
#define sensorLTrigPin 2
#define sensorLEchoPin 3
#define sensorRTrigPin 4
#define sensorREchoPin 5
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
#define DM 15    // distance match
#define RT 20    // turn right
#define LT 21    // turn left
#define RTE 35   // turn right but with encoders
#define LTE 36   // turn left but with encoders
#define STOP 22  // end of program
#define ABORT 99 // abort program

bool status, newCommand;
int targetAngle = 0;
float orientation = 0.0;
int defaultSpeedL = 155, defaultSpeedR = 150;
int turnSpeedLimit = 100;

int distanceTolerance = 1;
int encoderTolerance = 1;
int directionTolerance = 1;

volatile int lastEncodedL = 0, lastEncodedR = 0;
volatile long encoderValueL = 0, encoderValueR = 0;
long encoderValueLAbs = 0, encoderValueRAbs = 0;
bool accelerate;
bool deccelerate;
int accelerateRate = 1;
int targetSpeedL = defaultSpeedL;
int targetSpeedR = defaultSpeedR;
int startSpeedL = 80;
int startSpeedR = 80;
float disL = 0, disR = 0;

// for PID control
long previousTimeL = 0;
float ePreviousL = 0;
float eIntegralL = 0;

long previousTimeR = 0;
float ePreviousR = 0;
float eIntegralR = 0;

long previousTimeL2 = 0;
float ePreviousL2 = 0;
float eIntegralL2 = 0;

long previousTimeR2 = 0;
float ePreviousR2 = 0;
float eIntegralR2 = 0;

// PID gains
float kpL = 2.0;
float kdL = 0.05;
float kiL = 0.0;

float kpR = 5.0;
float kdR = 0.05;
float kiR = 0.01;

float kpLinear = 4.0;

float kpL2 = 5.0;
float kdL2 = 0.05;
float kiL2 = 0.0;

float kpR2 = 5.0;
float kdR2 = 0.05;
float kiR2 = 0.0;

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

  // initialize distance sensors
  pinMode(sensorLTrigPin, OUTPUT);
  pinMode(sensorLEchoPin, INPUT);
  pinMode(sensorRTrigPin, OUTPUT);
  pinMode(sensorREchoPin, INPUT);

  // initialize start button
  pinMode(buttonPin, INPUT_PULLUP);

  // initialize ultrasonic distance sensor pins
  pinMode(sensorLTrigPin, OUTPUT);
  pinMode(sensorLEchoPin, INPUT);

  newCommand = true;

  status = false;

  //--------------------------------
  // add commands HERE
  //--------------------------------

  // must be first!
  add(START);

  // add(FD, 50000);

  /*
  add(FD, DIS1);
  add(RT);
  add(FD, DIS1);
  add(RT);
  add(FD, DIS1);
  add(RT);
  add(FD, DIS1);
  add(RT);
  /**/
  /*
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
  add(LT);
  add(LT);
  /**/

  add(FDT,TIL1);
  /*
  /**/

  // must be last!
  add(STOP);

  // blink for ready to begin
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  // Serial.println(targetAngle);
}

void loop() {
  checkButton();

  /**/
  if (status) {
    digitalWrite(LED_BUILTIN, LOW);

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
        newCommand = false;
        // startTime = millis();
        /**/
        motorL->setSpeed(linearSpeedLimit);
        motorR->setSpeed(linearSpeedLimit + rightTrim);
        motorL->run(FORWARD);
        motorR->run(FORWARD);
        /**/
      } else {
        if (encoderValueL - (encoderValueLAbs - currentParam) > encoderTolerance) {
          // pidPositionMatchLeft(encoderValueLAbs - currentParam, linearSpeedLimit);
          // pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
          courseCorrect();
          // Serial.println(encoderValueL);
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
    case BD:
      // move backward
      
      break;
    case FDT:
        if (newCommand) {
          newCommand = false;
          motorR->setSpeed(linearSpeedLimit + rightTrim);
          motorR->run(FORWARD);
        } else {
          getDistance();
          if (fabs(disL - currentParam) > distanceTolerance) {
            pidDistanceMatchLeft(currentParam, linearSpeedLimit);
            courseCorrect();
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
    case DM:
      // move forward til distance from wall
      if (newCommand) {
        newCommand = false;
      } else {
        checkDistance();
        if (fabs(disL - currentParam) > distanceTolerance || fabs(disR - currentParam) > distanceTolerance) {
          pidDistanceMatchLeft(currentParam, linearSpeedLimit);
          pidDistanceMatchRight(disL);
          /*
          Serial.print(disL);
          Serial.print(", ");
          Serial.println(disR);
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
    case BDT:
      // move backward til distance from wall
      
      break;
      /**/
      case RT:
        // turn right
        if (newCommand) {
          // Serial.println(targetAngle);
          targetAngle -= 90;
          // Serial.println(targetAngle);
          /**/
          if (targetAngle == -180) {
            targetAngle = 180;
          }
          /**/
          motorL->setSpeed(turnSpeedLimit);
          motorL->run(FORWARD);
          newCommand = false;
          // Serial.println(targetAngle);
        } else {
          if ((targetAngle == 90 && orientation < 0) || (targetAngle != 180 && fabs(orientation - targetAngle) > directionTolerance) || (targetAngle == 180 && orientation < 0)) {
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
          pidPositionMatchLeft(encoderValueLAbs - currentParam, turnSpeedLimit);
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
          pidPositionMatchLeft(encoderValueLAbs + currentParam, turnSpeedLimit);
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

void checkDistance()
{
  digitalWrite(sensorLTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorLTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorLTrigPin, LOW);
  float duration = pulseIn(sensorLEchoPin, HIGH);
  disL = duration * 0.343 / 2;
  delay(60);
  digitalWrite(sensorRTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorRTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorRTrigPin, LOW);
  duration = pulseIn(sensorREchoPin, HIGH);
  disR = duration * 0.343 / 2;
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

      float temp = (int) (yaw * 100 + .5);
      orientation = (float) temp / 100;
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

void pidPositionMatchLeft(int target, int targetSpeed) {
  float u = pidPositionMatchCalculateLeft(target, kpL, kdL, kiL);
  float speed = fabs(u);

  if (speed > targetSpeed) {
    speed = targetSpeed;
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

float pidDistanceMatchCalculateLeft(int target, float kp, float kd, float ki) {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeL2)) / 1.0e6;

  // compute error, derivative, integral
  int e = disL - target;
  float eDerivative = (e - ePreviousL2) / deltaT;
  eIntegralL2 = eIntegralL2 + e * deltaT;

  // compute PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegralL2);

  previousTimeL2 = currentTime;
  ePreviousL2 = e;

  return u;
}

void pidDistanceMatchLeft(int target, int targetSpeed) {
  float u = pidDistanceMatchCalculateLeft(target, kpL2, kdL2, kiL2);
  float speed = fabs(u);

  if (speed > targetSpeed) {
    speed = targetSpeed;
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

float pidDistanceMatchCalculateRight(int target, float kp, float kd, float ki) {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeR2)) / 1.0e6;

  // compute error, derivative, integral
  int e = disR - target;
  float eDerivative = (e - ePreviousR2) / deltaT;
  // Serial.println(e);
  eIntegralR2 = eIntegralR2 + e * deltaT;
  

  // compute PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegralR2);

  previousTimeR2 = currentTime;
  ePreviousR2 = e;
  
  // Serial.println(ePreviousR2);
  return u;
}

void pidDistanceMatchRight(int target) {
  float u = pidDistanceMatchCalculateRight(target, kpR2, kdR2, kiR2);
  float speed = fabs(u);

  if (speed > 255) {
    speed = 255;
  }
  else if (speed < 20) {
    speed = 20;
  }

  motorR->setSpeed(speed);

  if (u < 0) {
    motorR->run(BACKWARD);
  }
  else {
    motorR->run(FORWARD);
  }
}

void courseCorrect() {
  int speed;
  int difference = round(kpLinear * (fabs((float) targetAngle) - fabs(orientation)));
  
  if (targetAngle == 180) {
    if (orientation > 0) {
      speed = linearSpeedLimit + difference;
    }
    else {
      speed = linearSpeedLimit - difference;
    }
  } else if (targetAngle == 0) {
    if (orientation > targetAngle) {
      speed = linearSpeedLimit + difference;
    } else {
      speed = linearSpeedLimit - difference;
    }
  } else if (targetAngle == -90) {
    speed = linearSpeedLimit - difference;
  } else {
    speed = linearSpeedLimit + difference;
  }

  if (speed > 255) {
    speed = 255;
  }

  /*
  Serial.print(targetAngle);
  Serial.print(", ");
  Serial.print(orientation);
  Serial.print(", ");
  Serial.print(difference);
  Serial.print(", ");
  Serial.println(speed);
  /**/

  motorR->setSpeed(speed);
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

void motorStop() {
  motorL->setSpeed(0);
  motorR->setSpeed(0);
}

/**
 * Calculates distance to object in front using TWO front ultrasonic sensor
 * currently only using left sensor lol
 * @return distance in mm
 */
void getDistance() {
  digitalWrite(sensorLTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sensorLTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorLTrigPin, LOW);

  float duration = pulseIn(sensorLEchoPin, HIGH);
  float distance = duration * 0.343 / 2;
  // 0.343: speed of sound in mm/microsecond
  // 2: round trip of sound
  disL = distance;
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