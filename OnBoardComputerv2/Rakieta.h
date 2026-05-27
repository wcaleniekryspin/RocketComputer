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


/* kolejność:
  setLed                          w
  buzzerOn / buzzerOff            w initGpio, handleDebugMode, handleFlightMode

  init: GPIO, dip, spio, gps, lsm, adxl, bmp

  XSolenoidOn / XSolenoidOff      w updateSolenoid
  systemReset                     w loraHandleCommand
  systemSleep                     w loraHandleCommand, handleSleepMode
  setSystemMode                   w init
  printSystemMode                 w 
  printFlightMode                 w 

  readBatteryVoltage              w readSensorData
  readGpsData                     w readSensorData
  readLsmData                     w readSensorData
  readAdxlData                    w readSensorData
  readBmpData                     w readSensorData
  readSensorData                  w superloop
  filterAcceleration              po readSensorData
  filterGyro                      po readSensorData
  filterLowPass  | float filterLowPass(float raw, float prev, float alpha) { return alpha * raw + (1 - alpha) * prev; }

  calculateAltitude               po readSensorData
  calculateOrientation            po readSensorData
  calculateVerticalVelocity       po calculateAltitude
  fuseBMPAndIMU                   po filtrach    | fusedAltitude = alpha * (bmpAltitude) + (1 - alpha) * (prevFused + verticalAccel * dt)
  kalmanFilter                    po fuseBMPAndIMU

  loraInit                        w setup
  loraPrintStatus                 w setup
  loraPrepareMsg                  w loraSendPacket
  loraSendString                  w 
  loraSendPacket                  w handleFlightMode
  loraStartListening              w superloop
  loraSendGpsOffset               w loraHandleCommand
  loraHandleCommand               w loraCheckRadio
  loraCheckRadio                  w superloop

  flashInit                       w init
  flashFindNextFileNumber         po flashInit
  flashOpenNewFile                w handleFlightMode
  flashWriteData                  w superloop
  flashWriteRocketData            w superloop
  flashFlushBuffer                przed resetem i co jakiś czas
  flashCloseFile                  po  =======  taczdałn
  flashRecoverAfterReset          podczas boot
  flashDumpFileList               w handleDumpMode
  flashDumpFileData               w handleDumpMode
  flashDumpLastFile               w handleDumpMode

  ? checkDeploymentConditions    w detectApogee
  drogueParashuteOpen             w detectApogee
  mainParashuteOpen               w descent
  detectLaunch                    w updateFlightState
  detectBurnout                   w updateFlightState
  detectApogee                    w updateFlightState
  detectLanding                   w updateFlightState
  updateFlightState               w superloop

  watchdog                        w superloop
  updateBuzzer                    w superloop
  updateStatus                    w superloop
  updateSolenoid                  w superloop
  setOffsets                      w init
  prepareOffsetsMsg               w loraHandleComand

  handleDebugMode                 w superloop
  handleFlightMode                w superloop
  handleDumpMode                  w superloop
  handleSleepMode                 w superloop

  systemInit
  systemLoop
*/


// It must be here for the RadioLib module to function properly
inline volatile bool operationDone = false;
inline void setOperationFlag(void) { operationDone = true; }


class Rakieta
{
private:
  // === Zmienne stanu ===
  enum SystemMode {
    DEBUG,
    FLIGHT,
    DUMP,
    SLEEP
  } currentMode;

  enum FlightState {
    IDLE,
    BOOST,
    COAST,
    APOGEE,
    DESCENT,
    LANDED
  } currentFlightState;

  enum ParachuteType { DROGUE, MAIN };

  struct {
    struct {
      uint8_t valid = 0;
      float lat = 0;
      float lng = 0;
      float alti = 0;
    } gps;
    struct {
      uint8_t valid = 0;
      float ax = 0;
      float ay = 0;
      float az = 0;
      float gx = 0;
      float gy = 0;
      float gz = 0;
    } lsm;
    struct {
      uint8_t valid = 0;
      float ax = 0;
      float ay = 0;
      float az = 0;
    } adxl;
    struct {
      uint8_t valid = 0;
      float alti = 0;
    } bmp1, bmp2;
  } offsets;

  struct {
    struct {
      float lat = 0;
      float lng = 0;
      float alti = 0;
      uint8_t h = 0;
      uint8_t m = 0;
      uint8_t s = 0;
      uint8_t centi = 0;
      float speed = 0;
      float course = 0;
      uint8_t satNum = 0;
      uint8_t hdop = 0;
      float maxAlti = 0;
    } gps;
    struct {
      float ax = 0;
      float ay = 0;
      float az = 0;
      float gx = 0;
      float gy = 0;
      float gz = 0;
      float temp = 0;
      float lastTotalAlti = 0;
      float lastTotalSpeed = 0;
      float lastTotalAccel = 0;
      float lastTotalRotation = 0;
      uint32_t lastTime = 0;
    } lsm;
    struct {
      float ax = 0;
      float ay = 0;
      float az = 0;
      float lastTotalSpeed = 0;
      float lastTotalAccel = 0;
      uint32_t lastTime = 0;
    } adxl;
    struct {
      float pressure = 0;
      float altitude = 0;
      float temp = 0;
      float lastAltitude = 0;
      float lastVerticalSpeed = 0;
      float maxAltitude = 0;
      uint32_t lastTime = 0;
    } bmp1, bmp2;
    struct {
      float voltage = 0;
    } battery;
    struct {
      float roll = 0;
      float pitch = 0;
    } orientation;
  } data;

  // === Operacyjne ===
  bool offsetsSet = false;
  bool drogueDeployed = false;
  bool mainDeployed = false;
  uint16_t errorFlags = 0;

  // === Urządzenia On/Off ===
  bool led1IsOn = false;
  uint32_t led1OffTime = 0;
  bool led2IsOn = false;
  uint32_t led2OffTime = 0;
  bool buzzerIsOn = false;
  uint32_t buzzerOffTime = 0;
  bool solenoid1IsOn = false;
  uint32_t solenoid1OffTime = 0;
  bool solenoid2IsOn = false;
  uint32_t solenoid2OffTime = 0;

  // === Dane przefiltrowane ===
  float filteredAccelX = 0;
  float filteredAccelY = 0;
  float filteredAccelZ = 0;
  float filteredGyroX = 0;
  float filteredGyroY = 0;
  float filteredGyroZ = 0;
  float fusedAltitude = 0;
  float prevFusedAltitude = 0;

  // Czasy dla detekcji
  bool inFlight = false;
  uint32_t launchDetectTime = 0;
  uint32_t burnoutDetectTime = 0;
  uint32_t apogeeDetectTime = 0;
  uint32_t descentDetectTime = 0;
  uint32_t landedDetectTime = 0;

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
  uint32_t packet = 0;
  uint32_t loraMsgStartTime = 0;
  SPISettings loraSettings;
  SX1262 lora;
  BitStorage message;

  // === Flash ===
  uint16_t flashWriteCount = 0;
  uint32_t dumpAddress = 0;
  // String currentFileName;
  File32 flashDataFile;
  File32 dumpFile;
  Adafruit_FlashTransport_SPI flashTransport;
  Adafruit_SPIFlash flash;
  FatVolume fatfs;

  // === Zarządzanie czasem ===
  uint32_t lastWatchdogTime = 0;
  uint32_t lastFlightLoop = 0;
  uint32_t lastDebugPrint = 0;
  uint32_t lastDumpProgress = 0;
  uint32_t lastSleepCheck = 0;
  uint32_t lastFlightModeLoop = 0;


  void initGPIO();
  void initSPI();
  bool initLSM();
  bool initADXL();
  bool initBMP();
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
  String prepareOffsetsMsg();
  String prepareDataLineMsg();
  void filterAcceleration();  /// nigdzie nie używane
  void filterGyro();  /// nigdzie nie używane
  void calculateOrientation();  /// nigdzie nie używane
  void fuseBMPAndIMU();  /// nigdzie nie używane
  bool initLora();
  String prepareLoraStatusMsg();
  void preparePacket();
  void sendPacket();
  void transmit(String& msg);
  void transmit(const uint8_t *msg, const size_t len);
  void startListening();
  void handleCommand(const String& command);  /// trzeba pododawać komendy
  void checkRadio();
  void sendGpsOffset();
  bool initFlash();
  bool flashFindNextFileNumber();
  bool flashOpenNewFile();
  void flashWriteString(const String& msg);
  void flashFlushBuffer();
  void flashCloseFile();
  bool flashRecoverAfterReset();  /// wydaje mi się, że trzeba by było zmienić bo te funkcje istnieją ?
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
  void handleDebugMode();  /// do zmieniania w locie
  void handleFlightMode();
  void handleDumpMode();  /// nie wiem co tu się dzieje obecnie
  void handleSleepMode();  /// chyba będzie ok
  void handleModes(const uint32_t now);

  
/*
  // === Metody prywatne ===
  void readModeFromSwitch();
  void printSystemMode();
  void printData();
  void systemReset();
  void updateBuzzer() {};
  void watchdog() {};
  
  // Inicjalizacja komponentów
  bool initSPI();
  bool initLSM();
  bool initADXL375();
  bool initBMP388();
  bool initFlash();
  bool initLoRa();
  bool initGPS();
  bool initGPIO();
  bool initDIPSwitch();

  // Odczyt danych
  void setOffsets();
  void handleLsm();
  void handleBmp();
  void handleAdxl();
  void handleGPS();
  void handleBattery();

  // Obsługa radia
  void startListening();
  void printRadioStatus();
  void handleCommand(String);
  void checkRadio();
  void transmit(String) {};
  void transmit(uint8_t*, size_t) {};
  void prepareMsg() {};
  void sendMsg() {};
  void sendGpsOffset() {};

  String prepareOffsetsMsg() {};
  String prepareDataLineMsg() {};
  bool flashFindNextFileNumber() {};
  bool flashOpenNewFile() {};
  bool flashWriteData(const String&) {};
  void writeRocketData() {};
  
  // Metody dla poszczególnych trybów
  void handleDebugMode();
  void handleFlightMode();
  void handleDumpMode();
  void handleSleepMode();

*/
  
public:
  Rakieta();
  ~Rakieta();
  
  void init();
  void loop();
};

/*
    void parashuteOpen();                      // Initiates the parachute deployment procedure (activates solenoid)
    void activateSolenoid(uint8_t, uint32_t);  // Activates a solenoid valve with the specified number of pulses and pulse timing
    void updateSolenoid();                     // Updates solenoid state machine and pulse timing (call frequently to manage pulses)
    void updateStatus();                       // Updates the rocket's flight status/state based on current sensor data

    void setFlightMode(bool);                  // Sets the inFlight flag (true = flight mode enabled, false = ground mode)
*/

#endif  // RAKIETA_H