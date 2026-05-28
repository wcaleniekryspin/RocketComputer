#ifndef RAKIETA_H
#define RAKIETA_H

#include <sys/_stdint.h>
#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_FlashTransport.h>

#include <BitStorage.h>
#include "config.h"


class Rakieta
{
private:
  // === Zmienne stanu ===
  enum class SystemMode {
    DEBUG,
    FLIGHT,
    DUMP,
    SLEEP
  } currentMode;

  enum class FlightState {
    IDLE,
    BOOST,
    COAST,
    APOGEE,
    DESCENT,
    LANDED
  } currentFlightState;

  enum class ParachuteType { DROGUE, MAIN };

  struct {
    struct {
      uint8_t valid;
      float lat;
      float lng;
      float alti;
    } gps;
    struct {
      uint8_t valid;
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
    } lsm;
    struct {
      uint8_t valid;
      float ax;
      float ay;
      float az;
    } adxl;
    struct {
      uint8_t valid;
      float alti;
    } bmp1, bmp2;
  } offsets;

  struct {
    struct {
      float lat;
      float lng;
      float alti;
      uint8_t h;
      uint8_t m;
      uint8_t s;
      uint8_t centi;
      float speed;
      float course;
      uint8_t satNum;
      uint8_t hdop;
      float maxAlti;
    } gps;
    struct {
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
      float temp;
      float lastTotalAlti;
      float lastTotalSpeed;
      float lastTotalAccel;
      float lastTotalRotation;
      uint32_t lastTime;
    } lsm;
    struct {
      float ax;
      float ay;
      float az;
      float lastTotalSpeed;
      float lastTotalAccel;
      uint32_t lastTime;
    } adxl;
    struct {
      float pressure;
      float altitude;
      float temp;
      float lastAltitude;
      float lastVerticalSpeed;
      float maxAltitude;
      uint32_t lastTime;
    } bmp1, bmp2;
    struct {
      float voltage;
    } battery;
    struct {
      float roll;
      float pitch;
    } orientation;
  } data;

  // === Operacyjne ===
  bool offsetsSet;
  bool drogueDeployed;
  bool mainDeployed;
  uint16_t errorFlags;

  // === Urządzenia On/Off ===
  bool led1IsOn;
  uint32_t led1OffTime;
  bool led2IsOn;
  uint32_t led2OffTime;
  bool buzzerIsOn;
  uint32_t buzzerOffTime;
  bool solenoid1IsOn;
  uint32_t solenoid1OffTime;
  bool solenoid2IsOn;
  uint32_t solenoid2OffTime;

  // === Dane przefiltrowane ===
  float filteredAccelX;
  float filteredAccelY;
  float filteredAccelZ;
  float filteredGyroX;
  float filteredGyroY;
  float filteredGyroZ;
  float fusedAltitude;
  float prevFusedAltitude;

  // Czasy dla detekcji
  bool inFlight;
  uint32_t launchDetectTime;
  uint32_t burnoutDetectTime;
  uint32_t apogeeDetectTime;
  uint32_t descentDetectTime;
  uint32_t landedDetectTime;

  // === Obiekty SPI ===
  SPIClass spiFlash;  // SPI1: W25Q128
  SPIClass spiLora;   // SPI3: SX1262
  SPIClass spiFast;   // SPI4: LSM, ADXL375, BMP388

  // === Czujniki i peryferia ===
  Adafruit_LSM6DS lsm;
  Adafruit_BMP3XX bmp1;
  Adafruit_BMP3XX bmp2;
  Adafruit_ADXL343 adxl;
  TinyGPSPlus gps;
  HardwareSerial gpsSerial;

  // === LoRa ===
  static volatile bool operationDone;
  uint32_t packet;
  uint32_t loraMsgStartTime;
  SPISettings loraSettings;
  SX1262 lora;
  BitStorage message;

  // === Flash ===
  uint16_t flashWriteCount;
  uint32_t dumpAddress;
  // String currentFileName;
  File32 flashDataFile;
  File32 dumpFile;
  Adafruit_FlashTransport_SPI flashTransport;
  Adafruit_SPIFlash flash;
  FatVolume fatfs;

  // === Zarządzanie czasem ===
  uint32_t lastErrorCheckTime;
  uint32_t lastFlightLoop;
  uint32_t lastDebugPrint;
  uint32_t lastDumpProgress;
  uint32_t lastSleepCheck;
  uint32_t lastFlightModeLoop;


  void initGPIO();
  void initSPI();
  bool initLSM();
  bool initADXL();
  bool initBMP1();
  bool initBMP2();
  bool initGPS();

  void updateLeds(const uint32_t now);
  void updateBuzzer(const uint32_t now);
  void updateSolenoid(const uint32_t now);

  void systemReset();
  void systemSleep(const uint32_t time);
  void setSystemMode();
  void printSystemMode();
  void printFlightMode();  /// prawdopodobnie do usunięcia bo nigdzie nie będzie używane

  void handleBattery();  // tzreba sprawdzić czy odpowiednie są przeliczniki
  void handleLsm();
  void handleAdxl();
  void handleBmp();
  void handleGPS();
  void readSensorsData();
  void printData();
  void setOffsets();
  void prepareOffsetsMsg(char* buffer, size_t bufferSize);
  void prepareDataLineMsg(char* buffer, size_t bufferSize);
  void filterAcceleration();  /// nigdzie nie używane
  void filterGyro();  /// nigdzie nie używane
  void calculateOrientation();  /// nigdzie nie używane
  void fuseBMPAndIMU();  /// nigdzie nie używane

  bool initLora();
  static void setOperationFlag();
  void prepareLoraStatusMsg(char* buffer, size_t bufferSize);
  void preparePacket();
  void sendPacket();
  void transmit(const uint8_t* msg, const size_t len);
  void transmit(const char* msg, size_t len);
  void startListening();
  void handleCommand(const char* command);
  void checkRadio();
  void sendGpsOffset();

  bool initFlash();
  bool flashFindNextFileNumber(char* fileName, size_t bufferSize);
  bool flashOpenNewFile();
  void flashWriteString(const char* msg);
  void flashFlushBuffer();
  void flashCloseFile();
  bool flashRecoverAfterReset();
  void flashDumpFileList();
  void flashDumpFileData(const uint16_t fileNumber);
  void flashDumpLastFile();

  bool detectLaunch();
  bool detectBurnout();
  bool detectApogee();
  bool detectLanding();
  bool checkDeploymentConditions(const ParachuteType type);

  void sendFlightSummary();
  void drogueParashuteOpen();
  void mainParashuteOpen();
  void updateFlightState();

  void initWatchdog();
  void watchdog();
  void reinitComponent(bool (*initFunc)());  // helper do reinicjalizacji
  void handleErrors();                           // główna funkcja obsługi błędów

  void handleDebugMode();  /// do zmieniania w locie
  void handleFlightMode();
  void handleDumpMode();  /// nie wiem co tu się dzieje obecnie
  void handleSleepMode();  /// chyba będzie ok
  void handleModes(const uint32_t now);


public:
  Rakieta();
  ~Rakieta();
  
  void init();
  void loop();
};


#endif  // RAKIETA_H
