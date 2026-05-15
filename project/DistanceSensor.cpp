#include "DistanceSensor.h"

const float DistanceSensor::MAX_DISTANCE_CM = 200.0f;

DistanceSensor::DistanceSensor(const uint8_t trigPin, const uint8_t echoPin)
  : PIN_TRIG(trigPin),
    PIN_ECHO(echoPin),
    distance(-1.0f),
    lastValid(0),
    lastTask(0)
{
}

void DistanceSensor::init(void) {
  pinMode(this->PIN_TRIG, OUTPUT);
  pinMode(this->PIN_ECHO, INPUT);
  digitalWrite(this->PIN_TRIG, LOW);
}

void DistanceSensor::task() {
  const unsigned long now = millis();

  // rate-limit interno (mesmo que CarApp chame, isso evita spam)
  if (now - this->lastTask < MIN_INTERVAL_MS) {
    return;
  }
  this->lastTask = now;

  // Trigger HC-SR04
  digitalWrite(this->PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(this->PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->PIN_TRIG, LOW);

  // FIX principal: timeout evita travar loop
  const unsigned long duration = pulseIn(this->PIN_ECHO, HIGH, SENSOR_TIMEOUT_US);

  // Se timeout/sem eco -> inválido
  if (duration == 0) {
    this->distance = -1.0f;
    return;
  }

  // cm = (us * 0.0343) / 2
  const float d = (duration * 0.0343f) * 0.5f;

  if (d <= 0.0f || d > MAX_DISTANCE_CM) {
    this->distance = -1.0f;
    return;
  }

  this->distance = d;
  this->lastValid = now;
}

float DistanceSensor::getDistance(void) const {
  return this->distance;
}

bool DistanceSensor::isValid(void) const {
  return this->distance > 0.0f;
}

unsigned long DistanceSensor::lastValidMs(void) const {
  return this->lastValid;
}
