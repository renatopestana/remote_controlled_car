#include "CarApp.h"
#include <Arduino.h>

static const float PROXIMITY_LIMIT = 8.0f;
static const uint8_t MAX_SPEED = 255;
static const uint8_t NEUTRAL_FACTOR = 40;

static const uint8_t X_MID_POINT = 126;
static const uint8_t Y_MID_POINT = 121;

static const uint8_t X_MID_UPPER = (X_MID_POINT + NEUTRAL_FACTOR);
static const uint8_t X_MID_LOWER = (X_MID_POINT - NEUTRAL_FACTOR);
static const uint8_t Y_MID_UPPER = (Y_MID_POINT + NEUTRAL_FACTOR);
static const uint8_t Y_MID_LOWER = (Y_MID_POINT - NEUTRAL_FACTOR);

static const uint8_t BUTTON_PRESSED = 1;

static const unsigned long RADIO_TASK_MS  = 15;
static const unsigned long SENSOR_TASK_MS = 50;

// fail-safe: se sensor ficar inválido por muito tempo, bloqueia FWD
static const unsigned long SENSOR_INVALID_FAILSAFE_MS = 300;

CarApp::CarApp(const CartAppInitStruct_t *iff) :
  motorRight(iff->motorRight),
  motorLeft(iff->motorLeft),
  radio(iff->radio),
  distSensorFront(iff->distSensorFront),
  lights(iff->lights)
{
}

void CarApp::processRemoteControlBuffer(uint16_t buffer[RADIO_BUFFER_SIZE])
{
  this->remoteCtrlData.x = map(buffer[0], 0, 1023, 0, MAX_SPEED);
  this->remoteCtrlData.y = map(buffer[1], 0, 1023, 0, MAX_SPEED);

  this->remoteCtrlData.bt_1 = buffer[2];
  this->remoteCtrlData.bt_2 = buffer[3];
  this->remoteCtrlData.bt_3 = buffer[4];
  this->remoteCtrlData.bt_4 = buffer[5];
  this->remoteCtrlData.bt_5 = buffer[6];
  this->remoteCtrlData.bt_6 = buffer[7];
  this->remoteCtrlData.bt_7 = buffer[8];
}

bool CarApp::isXActive(const uint8_t x)
{
  return (x >= X_MID_UPPER) || (x <= X_MID_LOWER);
}

bool CarApp::isYActive(const uint8_t y)
{
  return (y >= Y_MID_UPPER) || (y <= Y_MID_LOWER);
}

DRIVE_DIRECTION_t CarApp::getDriveDirection(const uint8_t x, const uint8_t y)
{
  if (isYActive(y))
  {
    if (y >= Y_MID_UPPER) return DRV_FWD;
    if (y <= Y_MID_LOWER) return DRV_BWD;
  }

  if (isXActive(x))
  {
    if (x >= X_MID_UPPER) return DRV_RIGHT;
    if (x <= X_MID_LOWER) return DRV_LEFT;
  }

  return DRV_COAST;
}

void CarApp::driveMotors(const DRIVE_DIRECTION_t direction)
{
  switch (direction)
  {
    case DRV_COAST: this->driveCoast(); break;
    case DRV_BRAKE: this->driveMotorStop(); break;
    case DRV_FWD:   this->driveFWD(MAX_SPEED); break;
    case DRV_BWD:   this->driveBWD(MAX_SPEED); break;
    case DRV_LEFT:  this->driveLFT(MAX_SPEED); break;
    case DRV_RIGHT: this->driveRGT(MAX_SPEED); break;
    default:        this->driveCoast(); break;
  }
}

void CarApp::driveFWD(const uint8_t speed)
{
  this->motorRight->rotateForward(speed);
  this->motorLeft->rotateForward(speed);
}

void CarApp::driveBWD(const uint8_t speed)
{
  this->motorRight->rotateBackward(speed);
  this->motorLeft->rotateBackward(speed);
}

void CarApp::driveLFT(const uint8_t speed)
{
  this->motorLeft->rotateBackward(speed);
  this->motorRight->rotateForward(speed);
}

void CarApp::driveRGT(const uint8_t speed)
{
  this->motorLeft->rotateForward(speed);
  this->motorRight->rotateBackward(speed);
}

void CarApp::driveMotorStop()
{
  this->motorRight->stop();
  this->motorLeft->stop();
}

void CarApp::driveCoast()
{
  this->motorRight->coast();
  this->motorLeft->coast();
}

void CarApp::init()
{
  this->radio->init();
  this->motorRight->init();
  this->motorLeft->init();
  this->lights->init();
  this->distSensorFront->init();

  this->lights->brakeLightOn();
  this->driveMotorStop();

  uint16_t *buffer = this->radio->getBufferPtr();
  buffer[0] = 512;
  buffer[1] = 512;

  this->processRemoteControlBuffer(buffer);
}

void CarApp::task()
{
  const unsigned long now = millis();

  // ========== RADIO ==========
  static unsigned long lastRadioTask = 0;
  if (now - lastRadioTask >= RADIO_TASK_MS)
  {
    this->radio->task();
    lastRadioTask = now;
  }

  // Se rádio caiu: para sempre (fail-safe)
  if (!this->radio->isConnected())
  {
    this->lights->brakeLightOn();
    this->driveMotorStop();
    return;
  }

  uint16_t *buffer = this->radio->getBufferPtr();
  this->processRemoteControlBuffer(buffer);

  // ========== EMERGÊNCIA ==========
  if (this->remoteCtrlData.bt_4 == BUTTON_PRESSED)
  {
    this->lights->brakeLightOn();
    this->driveMotorStop();
    return;
  }
  else
  {
    this->lights->brakeLightOff();
  }

  // ========== SENSOR ==========
  static unsigned long lastSensorTask = 0;
  if (now - lastSensorTask >= SENSOR_TASK_MS)
  {
    this->distSensorFront->task();
    lastSensorTask = now;
  }

  const float front = this->distSensorFront->getDistance();
  const DRIVE_DIRECTION_t dir = this->getDriveDirection(this->remoteCtrlData.x, this->remoteCtrlData.y);

  // ========== DEBUG ==========
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500)
  {
    Serial.print("X: "); Serial.print(this->remoteCtrlData.x);
    Serial.print(" Y: "); Serial.print(this->remoteCtrlData.y);
    Serial.print(" Front: "); Serial.println(front);
    lastPrint = now;
  }

  // ========== INTEGRAÇÃO SENSOR + RÁDIO (FAIL-SAFE) ==========
  // Se sensor inválido por muito tempo e comando é FWD, para por segurança
  static unsigned long invalidSince = 0;
  if (front <= 0.0f)
  {
    if (invalidSince == 0) invalidSince = now;

    if ((now - invalidSince) > SENSOR_INVALID_FAILSAFE_MS && dir == DRV_FWD)
    {
      this->lights->brakeLightOn();
      this->driveMotorStop();
      return;
    }
  }
  else
  {
    invalidSince = 0;
  }

  // Se leitura válida e muito perto: bloqueia avanço, mas deixa manobra
  if (front > 0.0f && front <= PROXIMITY_LIMIT)
  {
    if (dir == DRV_BWD || dir == DRV_LEFT || dir == DRV_RIGHT)
      this->driveMotors(dir);
    else
    {
      this->lights->brakeLightOn();
      this->driveMotorStop();
    }
    return;
  }

  // Caso normal
  this->driveMotors(dir);
}
