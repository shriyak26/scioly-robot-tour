#define sensorTrigPin   12
#define sensorEchoPin   11

void setup() {
    pinMode(sensorTrigPin, OUTPUT);
    pinMode(sensorEchoPin, INPUT);
    Serial.begin(9600);
}

void loop() {
    float distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    delay(1000);
}

float getDistance()
{
    digitalWrite(sensorTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensorTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensorTrigPin, LOW);
    float duration = pulseIn(sensorEchoPin, HIGH);
    float distance = duration * 0.0343 / 2;
    return distance;
}