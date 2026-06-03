#ifndef RAKIETA_H
#define RAKIETA_H

#include <sys/_stdint.h>
#include <stdlib.h>
#include <cstring>
#include <cstdio>
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

/*
  https://github.com/wcaleniekryspin/OnBoardComputerV2/blob/main/OnBoardComputerv2/OnBoardComputerv2.ino
  https://github.com/wcaleniekryspin/OnBoardComputerV2/blob/main/OnBoardComputerv2/Rakieta.h
  https://github.com/wcaleniekryspin/OnBoardComputerV2/blob/main/OnBoardComputerv2/Rakieta.cpp
  https://github.com/wcaleniekryspin/OnBoardComputerV2/blob/main/OnBoardComputerv2/config.h
*/



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
      float lastTotalAlti;
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
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
      float accel;
      float speed;
      float alti;
      float maxAlti;
      float roll;
      float pitch;
    } filtered;
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

  // Czasy dla detekcji
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
  SPISettings loraSettings;
  Module loraModule;
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
  uint32_t lastFlightModeLoop;


  void initGPIO();
  void initSPI();
  bool initLSM();
  bool initADXL();
  bool initBMP1();
  bool initBMP2();
  bool initGPS();

  void updateLeds(const uint32_t);
  void updateBuzzer(const uint32_t);
  void updateSolenoid(const uint32_t);

  void systemReset();
  void systemSleep(const uint32_t);
  void setSystemMode();
  void printSystemMode() const;
  void printFlightMode() const;

  void handleBattery();  // tzreba sprawdzić czy odpowiednie są przeliczniki
  void handleLsm();
  void handleAdxl();
  void handleBmp1();
  void handleBmp2();
  void handleGPS();
  void readSensorsData();
  void printData() const;
  void setOffsets();
  void prepareOffsetsMsg(char*, size_t);
  void prepareDataLineMsg(char*, size_t);
  void filterGyro();
  void calculateOrientation();
  void filterAcceleration();
  void filterSpeed();
  void filterAlti();

  bool initLora();
  static void setOperationFlag();
  void prepareLoraStatusMsg(char*, size_t);
  void preparePacket();
  void sendPacket();
  void transmit(const uint8_t*, const size_t);
  void transmit(const char*, size_t);
  void startListening();
  void handleCommand(const char* command);
  void checkRadio();
  void sendGpsOffset();

  bool initFlash();
  bool flashFindNextFileNumber(char*, size_t);
  bool flashOpenNewFile();
  void flashWriteString(const char*);
  void flashFlushBuffer();
  void flashCloseFile();
  bool flashRecoverAfterReset();
  void flashDumpFileList();
  void flashDumpFileData(const uint16_t);
  void flashDumpLastFile();

  bool detectLaunch();
  bool detectBurnout();
  bool detectApogee();
  bool detectLanding();
  bool checkDeploymentConditions(const ParachuteType);

  void sendFlightSummary();
  void drogueParashuteOpen();
  void mainParashuteOpen();
  void updateFlightState();

  void initWatchdog();
  void watchdog();
  void reinitComponent(bool (Rakieta::*initFunc)());
  void handleErrors();

  void handleDebugMode();  /// do zmieniania w locie
  void handleFlightMode();
  void handleDumpMode();  /// nie wiem co tu się dzieje obecnie
  void handleSleepMode();  /// chyba będzie ok
  void handleMode(const uint32_t);


public:
  Rakieta();
  ~Rakieta();
  
  void init();
  void loop();
};


#endif  // RAKIETA_H