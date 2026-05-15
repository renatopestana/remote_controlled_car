#pragma once
#include <Arduino.h>

class DistanceSensor {
public:
  DistanceSensor(const uint8_t trigPin, const uint8_t echoPin);

  void init(void);
  void task(void);

  float getDistance(void) const;     // cm; -1 = inválido
  bool isValid(void) const;
  unsigned long lastValidMs(void) const;

private:
  const uint8_t PIN_TRIG;
  const uint8_t PIN_ECHO;

  float distance;
  unsigned long lastValid;
  unsigned long lastTask;

  static const unsigned long SENSOR_TIMEOUT_US = 30000UL; // 30ms
  static const unsigned long MIN_INTERVAL_MS   = 50UL;    // evita leituras rápidas demais
  static const float MAX_DISTANCE_CM;
};
