#ifndef CONFIG_H
#define CONFIG_H

#define SERIAL_DEBUG    1
#if SERIAL_DEBUG == 1
  #define debugInit(x)  Serial.begin(x)
  #define debug(x)      Serial.print(x)
  #define debugln(x)    Serial.println(x)
  #define debugBin(x)   Serial.print(x, BIN)
  #define debugHex(x)   Serial.print(x, HEX)
  #define debugf(...)   Serial.printf(__VA_ARGS__)
#else
  #define debugInit(x)
  #define debug(x)
  #define debugln(x)
  #define debugBin(x)
  #define debugHex(x)
  #define debugf(...)
#endif

#define BV16(x)         (uint16_t(1u) << (x))



// ============================================================
// PINY GPIO
// ============================================================
constexpr int LED_1             = PE_5;                      // info
constexpr int LED_2             = PE_6;                      // do errorów
constexpr int BUZZER            = PE_15;
constexpr int SOLENOID_1        = PE_7;
constexpr int SOLENOID_2        = PB_1;
constexpr int BATTERY           = PC_0;

// Piny DIP switch (wejścia)
constexpr int MODE1             = PD_13;
constexpr int MODE2             = PD_12;
constexpr int MODE3             = PD_11;
constexpr int MODE4             = PD_10;

// ============================================================
// PINY SPI
// ============================================================

// Definicje pinów SPI4
constexpr int SPI4_SCK          = PE_12;
constexpr int SPI4_MISO         = PE_13;
constexpr int SPI4_MOSI         = PE_14;

// Piny CS dla urządzeń na SPI4
constexpr int CS_BMP2           = PE_8;
constexpr int CS_BMP1           = PE_9;
constexpr int CS_ADXL           = PE_10;
constexpr int CS_LSM            = PE_11;

// SPI1 (Flash)
constexpr int CS_FLASH          = PA_4;
constexpr int SPI1_SCK          = PA_5;
constexpr int SPI1_MISO         = PA_6;
constexpr int SPI1_MOSI         = PA_7;

// SPI3 (LoRa)
constexpr int SPI_LORA_SCK      = PC_10;
constexpr int SPI_LORA_MISO     = PC_11;
constexpr int SPI_LORA_MOSI     = PC_12;
constexpr int BUSY_LORA         = PD_0;
constexpr int CS_LORA           = PD_3;
constexpr int RST_LORA          = -1;      // brak podłączenia
constexpr int DIO1_LORA         = -1;      // brak podłączenia

// ============================================================
// UART (GPS)
// ============================================================
constexpr int RX_MAX            = PB_10;
constexpr int TX_MAX            = PB_11;

// ============================================================
// ERROR CODES
// ============================================================
constexpr LORA_ERROR                         = BV16(0);
constexpr LSM_ERROR                          = BV16(1);
constexpr BMP1_ERROR                         = BV16(2);
constexpr BMP2_ERROR                         = BV16(3);
constexpr ADXL_ERROR                         = BV16(4);
constexpr GPS_ERROR                          = BV16(5);
constexpr FLASH_ERROR                        = BV16(6);
constexpr FLASH_FILE_ERROR                   = BV16(7);
constexpr FLASH_NULL_MSG_ERROR               = BV16(8);
constexpr MSG_TOO_LONG_ERROR                 = BV16(9);

// ============================================================
// KONFIGURACJA LORA
// ============================================================
constexpr float FREQUENCY                  = 868.0f;   // MHz
constexpr float BANDWIDTH                  = 125.0f;   // kHz
constexpr uint8_t SF                       = 9;        // 6-12
constexpr uint8_t CODING_RATE              = 5;        // 5-8
constexpr uint8_t POWER                    = 20;       // dBm (do 17-22 dBm)
constexpr uint8_t PREAMBLE_LENGTH          = 10;       // 6-30 symbols, the longer the symbols, the better the synchronization and range?, but the slower the transmission.
constexpr uint16_t LORA_TX_TIMEOUT         = 500;      // ms

// ============================================================
// KONFIGURACJA CZUJNIKÓW I OBLICZEŃ
// ============================================================
constexpr uint16_t SOLENOID_DELAY                  = 100;    // ms

constexpr float FUSION_ALPHA                       = 0.8f;
constexpr float ADXL375_MG2G_MULTIPLIER            = 0.049f;  // trzeba sprawdzić czy to dodać → / ADXL343_MG2G_MULTIPLIER)
constexpr float REFERENCE_PRESSURE_HPA             = 1013.25f;

constexpr uint8_t OFFSETS_SENSORS_READ             = 50;
constexpr uint8_t OFFSETS_GPS_READ                 = 10;

constexpr float BATTERY_FULL_VOLTAGE               = 4.2f;      // V
constexpr float BATTERY_MAX_READ                   = 4095.0f;

constexpr uint32_t INTERVAL_ERROR_CHECK            = 1000;      // ms
constexpr float MAX_ACCEL_LSM                      = 150.0f;   // m/s²
constexpr float MAX_ACCEL_ADXL                     = 150.0f;   // m/s²
constexpr float MAX_GYRO                           = 2000.0f;  // dps

constexpr float MIN_PRESSURE                       = 500.0f;
constexpr float MAX_PRESSURE                       = 1100.0f;
constexpr float MIN_ALTITUDE                       = -100.0f;
constexpr float MAX_ALTITUDE                       = 5000.0f;
constexpr float MIN_SPEED                          = -2000.0f;
constexpr float MAX_SPEED                          = 2000.0f;
constexpr float MIN_TEMP                           = -20.0f;
constexpr float MAX_TEMP                           = 60.0f;






// ============================================================
// DETEKCJA FAZ LOTU (wartości progowe)
// ============================================================
constexpr float LAUNCH_ACCEL_THRESHOLD            = 30.0f;     // >3G
constexpr uint32_t LAUNCH_DEBOUNCE_MS             = 75;        // ms

constexpr float BURNOUT_ACCEL_THRESHOLD           = 5.0f;      // <0.5G
constexpr uint32_t BURNOUT_DEBOUNCE_MS            = 300;       // ms

constexpr float APOGEE_VELOCITY_THRESHOLD         = 1.0f;      // m/s
constexpr uint32_t APOGEE_DEBOUNCE_MS             = 500;       // ms

constexpr float LANDING_VELOCITY_THRESHOLD        = 1.0f;      // m/s
constexpr uint32_t LANDING_DEBOUNCE_MS            = 10000;     // ms

// ============================================================
// SPADOCHRONY – warunki otwarcia
// ============================================================

// Drogue
constexpr float DEPLOY_DROGUE_MAX_SPEED           = 15.0f;     // m/s
constexpr float DEPLOY_DROGUE_MIN_ALTITUDE        = 400.0f;    // metry
constexpr uint32_t DROGUE_PARASHUTE_TIMEOUT       = 20000;     // ms

// Main
constexpr float DEPLOY_MAIN_MAX_SPEED             = 30.0f;     // m/s
constexpr float DEPLOY_MAIN_MIN_ALTITUDE          = 200.0f;    // metry
constexpr uint32_t MAIN_PARASHUTE_TIMEOUT         = 30000;     // ms

// Czas otwarcia elektrozaworu
constexpr uint32_t SOLENOID_DELAY                 = 300;       //ms

// ============================================================
// INTERWAŁY CZASOWE
// ============================================================

// Current mode
constexpr uint32_t INTERVAL_DEBUG                 = 2500;      // ms
constexpr uint32_t INTERVAL_FLIGHT                = 50;        // ms
constexpr uint32_t INTERVAL_DUMP                  = 500;       // ms
constexpr uint32_t INTERVAL_SLEEP                 = 5000;      // ms

// Flight Status
constexpr uint32_t INTERVAL_IDLE                      = 1000;
constexpr uint32_t INTERVAL_BURN                      = 50;
constexpr uint32_t INTERVAL_RISING                    = 50;
constexpr uint32_t INTERVAL_FALLING                   = 100;
constexpr uint32_t INTERVAL_TOUCHDOWN                 = 10000;

// ============================================================
// KONFIGURACJA FLASH (W25Q128)
// ============================================================
constexpr uint16_t FLUSH_AFTER_WRITES     = 10;

constexpr uint16_t GPS_BAUNDRATE          = 9600;
constexpr uint16_t MAX_FILE_NUMBER        = 9999;






static_assert(SF >= 6 && SF <= 12, "LoRa Spreading Factor must be between 6 and 12");
static_assert(CODING_RATE >= 5 && CODING_RATE <= 8, "LoRa Coding Rate must be 5,6,7,8 (4/5 to 4/8)");
static_assert(POWER <= 22, "LoRa power cannot exceed 22 dBm for SX1262");
static_assert(FLUSH_AFTER_WRITES > 0, "FLUSH_AFTER_WRITES must be positive");







// DATA LEN AND POSITIONS
// HEADER for 16 bits
constexpr uint16_t timePos                            = 16;
constexpr uint16_t timeLen                            = 22;
constexpr uint16_t packetPos                          = (timePos + timeLen);
constexpr uint16_t packetLen                          = 16;
constexpr uint16_t errorPos                           = (packetPos + packetLen);
constexpr uint16_t errorLen                           = 16;
constexpr uint16_t statusPos                          = (errorPos + errorLen);
constexpr uint16_t statusLen                          = 4;
constexpr uint16_t handlePos                          = (statusPos + statusLen);
constexpr uint16_t handleLen                          = 1;

constexpr uint16_t gpsLatPos                          = (handlePos + handleLen);
constexpr uint16_t gpsLatLen                          = (17+1);
constexpr uint16_t gpsLngPos                          = (gpsLatPos + gpsLatLen);
constexpr uint16_t gpsLngLen                          = (17+1);
constexpr uint16_t gpsAltiPos                         = (gpsLngPos + gpsLngLen);
constexpr uint16_t gpsAltiLen                         = 14;
constexpr uint16_t gpsHourPos                         = (gpsAltiPos + gpsAltiLen);
constexpr uint16_t gpsHourLen                         = 5;
constexpr uint16_t gpsMinPos                          = (gpsHourPos + gpsHourLen);
constexpr uint16_t gpsMinLen                          = 6;
constexpr uint16_t gpsSecPos                          = (gpsMinPos + gpsMinLen);
constexpr uint16_t gpsSecLen                          = 6;
constexpr uint16_t gpsCentisecPos                     = (gpsSecPos + gpsSecLen);
constexpr uint16_t gpsCentisecLen                     = 7;
constexpr uint16_t gpsSpeedPos                        = (gpsCentisecPos + gpsCentisecLen);
constexpr uint16_t gpsSpeedLen                        = (14+1);
constexpr uint16_t gpsCoursePos                       = (gpsSpeedPos + gpsSpeedLen);
constexpr uint16_t gpsCourseLen                       = 9;
constexpr uint16_t gpsSatNumPos                       = (gpsCoursePos + gpsCourseLen);
constexpr uint16_t gpsSatNumLen                       = 3;
constexpr uint16_t gpsHdopPos                         = (gpsSatNumPos + gpsSatNumLen);
constexpr uint16_t gpsHdopLen                         = 5;

constexpr uint16_t lsmAccelXPos                       = (gpsHdopPos + gpsHdopLen);
constexpr uint16_t lsmAccelXLen                       = (14+1);
constexpr uint16_t lsmAccelYPos                       = (lsmAccelXPos + lsmAccelXLen);
constexpr uint16_t lsmAccelYLen                       = (14+1);
constexpr uint16_t lsmAccelZPos                       = (lsmAccelYPos + lsmAccelYLen);
constexpr uint16_t lsmAccelZLen                       = (14+1);
constexpr uint16_t lsmGyroXPos                        = (lsmAccelZPos + lsmAccelZLen);
constexpr uint16_t lsmGyroXLen                        = (14+1);
constexpr uint16_t lsmGyroYPos                        = (lsmGyroXPos + lsmGyroXLen);
constexpr uint16_t lsmGyroYLen                        = (14+1);
constexpr uint16_t lsmGyroZPos                        = (lsmGyroYPos + lsmGyroYLen);
constexpr uint16_t lsmGyroZLen                        = (14+1);
constexpr uint16_t lsmTempPos                         = (lsmGyroZPos + lsmGyroZLen);
constexpr uint16_t lsmTempLen                         = (9+1);
constexpr uint16_t lsmSpeedPos                        = (lsmTempPos + lsmTempLen);
constexpr uint16_t lsmSpeedLen                        = (10+1);

constexpr uint16_t adxlAccelXPos                      = (lsmSpeedPos + lsmSpeedLen);
constexpr uint16_t adxlAccelXLen                      = (15+1);
constexpr uint16_t adxlAccelYPos                      = (adxlAccelXPos + adxlAccelXLen);
constexpr uint16_t adxlAccelYLen                      = (15+1);
constexpr uint16_t adxlAccelZPos                      = (adxlAccelYPos + adxlAccelYLen);
constexpr uint16_t adxlAccelZLen                      = (15+1);
constexpr uint16_t adxlSpeedPos                       = (adxlAccelZPos + adxlAccelZLen);
constexpr uint16_t adxlSpeedLen                       = (14+1);

constexpr uint16_t bmp1TempPos                         (adxlSpeedPos + adxlSpeedLen);
constexpr uint16_t bmp1TempLen                        = (9+1);
constexpr uint16_t bmp1PressPos                       = (bmp1TempPos + bmp1TempLen);
constexpr uint16_t bmp1PressLen                       = 14;
constexpr uint16_t bmp1AltiPos                        = (bmp1PressPos + bmp1PressLen);
constexpr uint16_t bmp1AltiLen                        = 14;
constexpr uint16_t bmp1SpeedPos                       = (bmp1AltiPos + bmp1AltiLen);
constexpr uint16_t bmp1SpeedLen                       = (14+1);

constexpr uint16_t bmp2TempPos                        = (bmp1SpeedPos + bmp1SpeedLen);
constexpr uint16_t bmp2TempLen                        = (9+1);
constexpr uint16_t bmp2PressPos                       = (bmp2TempPos + bmp2TempLen);
constexpr uint16_t bmp2PressLen                       = 14;
constexpr uint16_t bmp2AltiPos                        = (bmp2PressPos + bmp2PressLen);
constexpr uint16_t bmp2AltiLen                        = 14;
constexpr uint16_t bmp2SpeedPos                       = (bmp2AltiPos + bmp2AltiLen);
constexpr uint16_t bmp2SpeedLen                       = (14+1);

constexpr uint16_t batteryPos                         = (bmp2SpeedPos + bmp2SpeedLen);
constexpr uint16_t batteryLen                         = 7;

// CHECKSUM for 8 bits

#endif  // CONFIG_H