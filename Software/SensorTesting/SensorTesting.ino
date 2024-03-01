#define sensorLTrigPin 2
#define sensorLEchoPin 3
#define sensorRTrigPin 4
#define sensorREchoPin 5

long trigTime;
bool respondedL = true, respondedR = true;
bool trigTurn = true;
bool trigWaiting = false;
float disL = 0, disR = 0;
long trigDelay = 0;
bool trigInterval = false;

void setup() {
    pinMode(sensorLTrigPin, OUTPUT);
    pinMode(sensorLEchoPin, INPUT);
    pinMode(sensorRTrigPin, OUTPUT);
    pinMode(sensorREchoPin, INPUT);
    Serial.begin(115200);

    //attachInterrupt(digitalPinToInterrupt(sensorLEchoPin), getDistanceL, RISING);
    //attachInterrupt(digitalPinToInterrupt(sensorREchoPin), getDistanceR, RISING);
}

void loop() {
    checkDistance();
    Serial.print(disL);
    Serial.print(", ");
    Serial.println(disR);
    /*
    if (!trigWaiting) {
      if (trigTurn) {
        digitalWrite(sensorLTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(sensorLTrigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sensorLTrigPin, LOW);
      } else {
        digitalWrite(sensorRTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(sensorRTrigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sensorRTrigPin, LOW);
      }
      trigTime = micros();
      trigTurn = !trigTurn;
      trigWaiting = true;
    } else {
      if (digitalRead(sensorLEchoPin) == HIGH) {
        long currentTime = micros();
        disL = (currentTime - trigTime) * 0.343 / 2;
        trigDelay = millis();
        trigInterval = true;
        Serial.print(disL);
        Serial.print(", ");
        Serial.println(disR);
      }
      if (digitalRead(sensorREchoPin) == HIGH) {
        long currentTime = micros();
        disL = (currentTime - trigTime) * 0.343 / 2;
        trigDelay = millis();
        trigInterval = true;
        Serial.print(disL);
        Serial.print(", ");
        Serial.println(disR);
      }
      if (trigInterval && millis() - trigDelay > 60) {
        trigWaiting = false;
        trigInterval = false;
      }
    }

    /**/
}

float getDistance()
{
    digitalWrite(sensorLTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensorLTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensorLTrigPin, LOW);
    float duration = pulseIn(sensorLEchoPin, HIGH);
    float distance = duration * 0.343 / 2;
    return distance;
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

void getDistanceL() {
  long currentTime = micros();
  disL = (currentTime - trigTime) * 0.343 / 2;
  trigDelay = millis();
  trigInterval = true;
  Serial.print(disL);
  Serial.print(", ");
  Serial.println(disR);
}

void getDistanceR() {
  long currentTime = micros();
  disL = (currentTime - trigTime) * 0.343 / 2;
  trigDelay = millis();
  trigInterval = true;
  Serial.print(disL);
  Serial.print(", ");
  Serial.println(disR);
}

/*
    float distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    delay(1000);
*/