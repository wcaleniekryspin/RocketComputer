#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

constexpr uint8_t LORA_CS          = 8;
constexpr uint8_t LORA_SCK         = 9;
constexpr uint8_t LORA_MOSI        = 10;
constexpr uint8_t LORA_MISO        = 11;
constexpr uint8_t LORA_RST         = 12;
constexpr uint8_t LORA_BUSY        = 13;
constexpr uint8_t LORA_DIO1        = 14;

constexpr float FREQUENCY          = 868.0f;
constexpr float BANDWIDTH          = 125.0f;
constexpr uint8_t SF               = 9;
constexpr uint8_t CODING_RATE      = 5;
constexpr uint8_t POWER            = 20;
constexpr uint8_t PREAMBLE_LEN     = 10;
constexpr uint16_t TX_TIMEOUT      = 500;


SPIClass spiLoRa;
Module loraModule(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, spiLoRa);
SX1262 radio(&loraModule);

byte rxBuffer[256];
volatile bool receivedFlag = false;
char cmdBuffer[64];
uint8_t idx = 0;

void setReceivedFlag(void) { receivedFlag = true; }

void setup()
{
  Serial.begin(115200);
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  int state = radio.begin(FREQUENCY, BANDWIDTH, SF, CODING_RATE, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POWER, PREAMBLE_LEN);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("[LoRa] Init failed, code: ");
    Serial.println(state);
    while (true) delay(100);
  }

  radio.setDio1Action(setReceivedFlag);
  radio.startReceive();
  Serial.println("[LoRa] Ready");
}

void loop()
{
  if (receivedFlag)
  {
    receivedFlag = false;
    int16_t state = radio.readData(rxBuffer, sizeof(rxBuffer) - 1U);
    if (state == RADIOLIB_ERR_NONE)
    {
      size_t len = radio.getPacketLength();
      rxBuffer[len] = '\0';
      Serial.write(rxBuffer, len);
      Serial.print(" [RSSI=");
      Serial.print(radio.getRSSI());
      Serial.print("dBm SNR=");
      Serial.print(radio.getSNR());
      Serial.println("dB]");
    }
    radio.startReceive();
  }

  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (idx > 0)
      {
        cmdBuffer[idx] = '\0';

        uint8_t txBuf[64];
        memcpy(txBuf, cmdBuffer, idx);

        radio.clearDio1Action();
        radio.transmit(txBuf, idx);
        radio.setDio1Action(setReceivedFlag);
        radio.startReceive();

        idx = 0;
      }
      else if (idx < sizeof(cmdBuffer) - 1U)
      {
        cmdBuffer[idx++] = c;
      }
    }
  }
}





