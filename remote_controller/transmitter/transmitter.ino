#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10

#define X_AXIS A0
#define Y_AXIS A1

#define BTN1 8
#define BTN2 2
#define BTN3 3
#define BTN4 4
#define BTN5 5
#define BTN6 6
#define BTN7 7

const uint64_t pipe = 0xE8E8F0F0E1LL;

RF24 radio(CE_PIN, CSN_PIN);
uint16_t data[9];

// ================= SETUP =================
void setup()
{
  Serial.begin(9600);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  // ✅ MELHOR: com ACK (mais robusto)
  radio.setAutoAck(true);
  radio.setRetries(3, 5);

  radio.openWritingPipe(pipe);
  radio.stopListening();

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);
  pinMode(BTN5, INPUT_PULLUP);
  pinMode(BTN6, INPUT_PULLUP);
  pinMode(BTN7, INPUT_PULLUP);
}

// ================= READ =================
void readControls()
{
  data[0] = analogRead(X_AXIS);
  data[1] = analogRead(Y_AXIS);

  data[2] = !digitalRead(BTN1);
  data[3] = !digitalRead(BTN2);
  data[4] = !digitalRead(BTN3);
  data[5] = !digitalRead(BTN4);
  data[6] = !digitalRead(BTN5);
  data[7] = !digitalRead(BTN6);
  data[8] = !digitalRead(BTN7);
}

// ================= LOOP =================
unsigned long lastTask = 0;
const unsigned long interval = 15; // alinhado com RX

uint8_t failCount = 0;

void loop()
{
  if (millis() - lastTask >= interval)
  {
    lastTask = millis();

    // ✅ leitura única (consistente)
    readControls();

    // transmissão
    bool ok = radio.write(data, sizeof(data));

    if (!ok)
    {
      failCount++;
      Serial.println("TX fail");

      // ✅ recovery mais seguro (sem loop pesado)
      if (failCount > 5)
      {
        Serial.println("Reinit RF");

        radio.powerDown();
        delay(5);
        radio.powerUp();

        failCount = 0;
      }
    }
    else
    {
      failCount = 0;
    }
  }
}
